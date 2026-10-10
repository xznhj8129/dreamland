"""One server-owned scheduled flight run, incremental output/log reading, Abort and shutdown."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from libs.project import Project
    from webapp.state import AppState

REPO_DIR = Path(__file__).resolve().parent.parent  # simulation runner and its cwd
SIM_FLIGHT = REPO_DIR / "sim_flight.py"

SHUTDOWN_WAIT_S = 5.0  # after SIGINT, for the runner to stop its SITL and write its log, before it is killed

# The retired Tk view's labels (sim_view.py START_LABELS), keyed like libs.simrun START_OPTIONS and in its order.
START_LABELS = {
    "alt": "Altitude (m)",
    "speed": "Speed (m/s)",
    "heading": "Heading (deg)",
    "lat": "Latitude",
    "lon": "Longitude",
    "pitch": "Pitch (deg)",
    "roll": "Roll (deg)",
    "wind-from": "Wind from (deg)",
    "wind-speed": "Wind (m/s)",
    "launch-pitch": "Launcher pitch (deg)",
    "launch-length": "Launcher length (m)",
    "launch-speed": "Catapult speed (m/s)",
    "launch-height": "Launcher height (m)",
}


class _Refused(Exception):
    """A start request the SIM panel refuses before saving or launching anything."""


def resource_path(state: AppState, kind: str, name: str, controller: str) -> Path:
    from libs.scenario import CONTROLLERS
    if kind == "scenario":
        if controller not in CONTROLLERS:
            raise ValueError(f"Choose a controller from {CONTROLLERS}")
        directory, suffix = state.project.scenarios_dir / controller, ".py"
    elif kind == "diff":
        directory, suffix = state.project.inav_dir, ".txt"
    else:
        raise ValueError("kind must be scenario or diff")
    if not name or Path(name).name != name or not name.endswith(suffix):
        raise ValueError(f"Choose a plain filename ending in {suffix}")
    path = directory / name
    if path.resolve().parent != directory.resolve():
        raise ValueError("File must be inside the project's directory")
    return path


def read_resource(state: AppState, kind: str, name: str, controller: str) -> dict:
    if kind == "diff" and name == "generated":
        path = state.project.generated_inav_dir / "design.txt"
        if not path.is_file():
            raise ValueError("Run once to generate the design diff, or choose a project diff")
        return {"source": path.read_text(), "readonly": True, "initial": {}, "scenario": None,
                "note": "Last generated design diff. A run regenerates it; save a copy to edit and fly it as written."}
    path = resource_path(state, kind, name, controller)
    readonly = False
    if kind == "scenario":
        from libs.scenario import scenario_path
        path = scenario_path(state.project.scenarios_dir, controller, name.removesuffix(".py"))
        readonly = path.parent != state.project.scenarios_dir / controller
    source = path.read_text()
    initial, spec, error = {}, None, None
    if kind == "scenario":
        from libs.scenario import ScenarioSpec, starting_conditions
        try:
            initial = starting_conditions(source)
            spec = ScenarioSpec.from_source(source, controller).payload()
        except (ValueError, SyntaxError) as problem:
            error = str(problem)
    note = "Shared scenario. Save a project copy to edit." if readonly else path.relative_to(state.project.root).as_posix()
    return {"source": source, "readonly": readonly, "initial": initial, "scenario": spec, "error": error, "note": note}


def save_resource(state: AppState, kind: str, name: str, source: str, controller: str, create: bool = False) -> dict:
    with _RUN._lock:
        if _RUN.process is not None:
            raise ValueError("Abort the active simulation before saving SIM files")
    path = resource_path(state, kind, name, controller)
    if not isinstance(source, str):
        raise ValueError("source must be text")
    if kind == "scenario":
        from libs.scenario import ScenarioSpec, starting_conditions
        compile(source, name, "exec")
        starting_conditions(source)
        ScenarioSpec.from_source(source, controller)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x" if create else "w") as file:
        file.write(source)
    return read_resource(state, kind, name, controller)


def options(state: AppState) -> dict[str, Any]:
    from controllers.inav import DEFAULT_SITL_PATH, is_real_fc_diff
    from libs.scenario import CONTROLLERS, DEFAULT_LIMIT_S, scenario_names
    from libs.simrun import HOME, START_ALT_M, START_OPTIONS, launcher_defaults, start_reasons
    from libs.jsbsim import AERO_PROVENANCE
    project, resolved = state.project, state.resolved
    airframe = resolved.airframe if resolved is not None else None
    defaults = {"alt": START_ALT_M, "heading": 0.0, "lat": HOME[0], "lon": HOME[1], "pitch": 0.0,
                "roll": 0.0, "wind-from": 0.0, "wind-speed": 0.0}
    if airframe is not None:
        defaults["speed"] = airframe.operating_point.airspeed_m_s
    return {
        "controllers": list(CONTROLLERS),
        "scenarios": {controller: scenario_names(project.scenarios_dir, controller) if airframe is not None else []
                      for controller in CONTROLLERS},
        "scenario_templates": {controller: read_resource(state, "scenario", "stability.py" if controller == "mcu" else "experiment.py", controller)
                               for controller in CONTROLLERS},
        "aero_model": AERO_PROVENANCE,
        "firmware": DEFAULT_SITL_PATH.relative_to(REPO_DIR.parent).as_posix(),
        "condition_defaults": defaults,
        "inav_configs": [{"file": path.name, "label": ("real FC diff: " if is_real_fc_diff(path) else "SITL diff: ") + path.name}
                         for path in sorted(project.inav_dir.glob("*.txt"))],
        "starts": [start for start, reason in start_reasons(airframe).items() if reason is None] if airframe is not None else [],
        "start_options": [{"key": key, "label": START_LABELS[key], "air_only": spec.air_only,
                           "launcher_only": spec.launcher_only, "catapult_only": spec.catapult_only}
                          for key, spec in START_OPTIONS.items()],
        "launcher_defaults": launcher_defaults(),
        "default_duration_s": DEFAULT_LIMIT_S,
    }


def _plan(state: AppState, body: dict[str, Any]) -> tuple[list[str], str, str, Path]:
    from libs.scenario import CONTROLLERS, DEFAULT_LIMIT_S, ScenarioSpec, scenario_names, scenario_path, starting_conditions
    from libs.flightlog import flight_log_path
    from libs.simrun import START_OPTIONS, start_reasons
    if state.resolved is None:
        raise _Refused("No valid airframe; fix the definition first.")
    project, airframe = state.project, state.resolved.airframe
    controller, scenario = body.get("controller"), body.get("scenario")
    if controller not in CONTROLLERS:
        raise _Refused(f"Choose a controller from {CONTROLLERS}")
    if scenario not in scenario_names(project.scenarios_dir, controller):
        raise _Refused(f"Choose a saved {controller} scenario")
    path = scenario_path(project.scenarios_dir, controller, scenario)
    try:
        source = path.read_text()
        initial = starting_conditions(source)
        ScenarioSpec.from_source(source, controller)
    except (ValueError, SyntaxError, OSError) as problem:
        raise _Refused(str(problem)) from problem
    start = body.get("start", initial.get("start", "air"))
    starts = [name for name, reason in start_reasons(airframe).items() if reason is None]
    if start not in starts:
        raise _Refused(f"start must be one of {starts}")
    command = [sys.executable, "-u", str(SIM_FLIGHT), str(project.root), scenario, "--controller", controller, "--start", start]
    clock = body.get("clock", "realtime")
    if clock not in ("realtime", "stepped", "stepped-faster"):
        raise _Refused("clock must be realtime, stepped or stepped-faster")
    command += ["--clock", clock]
    for field, default, label in (("rate", 200.0, "Tick rate"), ("duration", DEFAULT_LIMIT_S, "Time limit")):
        value = body.get(field, default)
        try:
            number = float(value)
        except (ValueError, TypeError):
            raise _Refused(f"{label} must be a number") from None
        if not math.isfinite(number) or number <= 0:
            raise _Refused(f"{label} must be finite and positive")
        command += [f"--{field}", f"{number:g}"]
    conditions = body.get("conditions") or {}
    if not isinstance(conditions, dict):
        raise _Refused("conditions must be an object of start conditions")
    given = {str(key).replace("_", "-"): value for key, value in conditions.items()}
    unknown = given.keys() - START_OPTIONS.keys()
    if unknown:
        raise _Refused(f"unknown start condition: {sorted(unknown)}")
    for option, spec in START_OPTIONS.items():
        if option not in given or not spec.applies_to(start):
            continue
        text = str(given[option]).strip()
        if not text:
            raise _Refused(f"{START_LABELS[option]}: enter a value")
        try:
            number = float(text)
        except ValueError:
            raise _Refused(f"{START_LABELS[option]}: {text!r} is not a number") from None
        if not math.isfinite(number):
            raise _Refused(f"{START_LABELS[option]}: must be finite")
        command += [f"--{option}", f"{number:g}"]
    config = body.get("inav_config")
    if controller == "inav":
        if config not in (None, "", "generated"):
            configs = {path.name for path in project.inav_dir.glob("*.txt")}
            if config not in configs:
                raise _Refused("Choose an INAV .txt diff, or generated")
            command += ["--inav-config", str(project.inav_dir / config)]
        for protocol in ("msp", "mavlink"):
            enabled = body.get(protocol, True)
            if not isinstance(enabled, bool):
                raise _Refused(f"{protocol} must be true or false")
            if not enabled:
                command += [f"--no-{protocol}"]
    elif config not in (None, "", "generated") or "msp" in body or "mavlink" in body:
        raise _Refused("INAV settings and external links require the INAV controller")
    shown = ["python", *(os.path.relpath(part, REPO_DIR) if os.path.isabs(part) else part for part in command[2:])]
    return command, f"Running {controller}: {scenario}...", " ".join(shown), flight_log_path(project, controller, scenario, start)


def _project_file(project: Project, path: Path) -> str:
    """path as a project-relative POSIX string, the form the /files route and the UI links use."""
    try:
        return path.resolve().relative_to(project.root.resolve()).as_posix()
    except ValueError:
        return str(path)


def _read_log(log_path: Path | None, run_started: float, offset: int) -> tuple[list[str] | None, list[list[float | None]], int]:
    """The complete CSV lines since byte offset, ported from the Tk view's read_log (sim_view.py:352-371).

    A log older than this run is the previous run's (the runners remove or rewrite it at start). The header is the
    first line, so it comes back only when reading from 0; rows are the parsed numbers after it. A non-finite cell
    (the runner writes "nan" for a log() column before its first value) is kept as None: Python's json would write
    NaN, which browser JSON.parse rejects, and a null keeps the row aligned with the header for the plots.
    """
    offset = max(offset, 0)
    if log_path is None:
        return None, [], offset
    try:
        if log_path.stat().st_mtime < run_started:
            return None, [], offset
        with open(log_path, "rb") as file:
            file.seek(offset)
            chunk = file.read()
    except FileNotFoundError:
        return None, [], offset
    end = chunk.rfind(b"\n") + 1
    if end == 0:  # no complete line yet
        return None, [], offset
    lines = chunk[:end].decode().splitlines()
    header = lines.pop(0).split(",") if offset == 0 and lines else None
    rows: list[list[float | None]] = []
    for line in lines:
        row: list[float | None] = []
        for value in line.split(","):
            number = float(value)
            row.append(number if math.isfinite(number) else None)
        rows.append(row)
    return header, rows, offset + end


class SimRun:
    """The one live run and its reader thread; every field is guarded by _lock (the loop and the thread share it)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.process: subprocess.Popen[str] | None = None
        self.run_id = 0
        self.status = "Idle"
        self.command = ""
        self.exit_code: int | None = None
        self.stdout_lines: list[str] = []
        self.aborting = False
        self.log_path: Path | None = None
        self.log_file: str | None = None
        self.run_started = 0.0

    def start_run(self, state: AppState, body: dict[str, Any]) -> tuple[bool, str]:
        """Validate, save the definition, then start the runner; the client polls state_payload() for progress."""
        with self._lock:
            if self.process is not None:
                return False, "A simulation run is already active; abort it first."
        try:
            command, status, command_line, log_path = _plan(state, body)
        except _Refused as error:
            return False, str(error)
        try:
            state.save()  # what flies is the file on disk
        except OSError as error:
            return False, f"could not write {state.project.build_path}: {error}"
        started = time.time()
        try:
            process = subprocess.Popen(command, cwd=REPO_DIR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        except OSError as error:
            return False, f"could not start {command_line}: {error}"
        with self._lock:
            self.run_id += 1
            self.process = process
            self.status = status
            self.command = command_line
            self.exit_code = None
            self.stdout_lines = [command_line]  # the summary's first line is the equivalent command
            self.aborting = False
            self.log_path = log_path
            self.log_file = _project_file(state.project, log_path)
            self.run_started = started
        threading.Thread(target=self._read_output, args=(process,), name="sim-stdout", daemon=True).start()
        return True, status

    def _read_output(self, process: subprocess.Popen[str]) -> None:
        """The runner's stdout until EOF, then its exit code and the final status."""
        for line in process.stdout:
            with self._lock:
                self.stdout_lines.append(line.rstrip("\n"))
        code = process.wait()
        with self._lock:
            if self.process is process:
                self.process = None
                self.exit_code = code
                self.status = "Aborted" if self.aborting else f"Finished (exit code {code})"

    def abort_run(self) -> bool:
        """SIGINT as for closing the window; the runner writes its log and the controller closes its transport."""
        with self._lock:
            process = self.process
            if process is None or self.aborting:
                return False
            self.aborting = True
            self.status = "Aborting..."
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:  # it exited between the check and the signal
            return True
        timer = threading.Timer(SHUTDOWN_WAIT_S, _kill_if_alive, args=(process,))
        timer.daemon = True
        timer.start()
        return True

    def state_payload(self, stdout_offset: int = 0, log_offset: int = 0, run_id: int | None = None) -> dict[str, Any]:
        """The run's status, stdout lines and flight-log rows since the client's offsets; only reads state and the log.

        A run_id that differs from the current one (the client's first poll, or a new run) resets both streams: the
        stdout comes from its first line and the log resets to the header and every row. log_offset is a byte offset
        into the CSV after the last complete line returned, so a poll with the returned offsets is incremental.
        """
        with self._lock:
            current_run_id = self.run_id
            reset = run_id != current_run_id
            stdout_total = len(self.stdout_lines)
            start = 0 if reset else min(max(stdout_offset, 0), stdout_total)
            payload: dict[str, Any] = {
                "run_id": current_run_id,
                "running": self.process is not None,
                "status": self.status,
                "exit_code": self.exit_code,
                "command": self.command,
                "stdout": {"offset": stdout_total, "lines": list(self.stdout_lines[start:])},
            }
            log_path, log_file, run_started = self.log_path, self.log_file, self.run_started
        header, rows, next_offset = _read_log(log_path, run_started, 0 if reset else log_offset)
        payload["log"] = {"file": log_file, "reset": reset, "offset": next_offset, "header": header, "rows": rows}
        payload["telemetry_file"] = None
        if log_path is not None:
            from libs.flightlog import telemetry_path
            path = telemetry_path(log_path)
            if path.is_file() and path.stat().st_mtime >= run_started:
                payload["telemetry_file"] = str(Path(log_file).with_suffix(".telemetry.csv"))
        return payload

    def shutdown(self) -> None:
        """Stop a live run so it cannot outlive the server: SIGINT, wait, then kill."""
        with self._lock:
            process = self.process
            if process is None:
                return
            self.aborting = True
            self.status = "Aborting..."
        try:
            process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=SHUTDOWN_WAIT_S)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def _kill_if_alive(process: subprocess.Popen[str]) -> None:
    """The 5 s abort timer: a runner that ignored SIGINT is killed."""
    if process.poll() is None:
        process.kill()


_RUN = SimRun()


def start_run(state: AppState, body: dict[str, Any]) -> tuple[bool, str]:
    """(ok, message): validate and start one run (webapp/sim.SimRun.start_run)."""
    return _RUN.start_run(state, body)


def abort_run() -> bool:
    """True when a live run was signalled; False when there was none (or one was already aborting)."""
    return _RUN.abort_run()


def state_payload(stdout_offset: int = 0, log_offset: int = 0, run_id: int | None = None) -> dict[str, Any]:
    """The polling payload for /api/sim/state (webapp/sim.SimRun.state_payload)."""
    return _RUN.state_payload(stdout_offset, log_offset, run_id)


def shutdown() -> None:
    """Abort any live run; called on server exit (webbuilder.serve) so no runner outlives it."""
    _RUN.shutdown()
