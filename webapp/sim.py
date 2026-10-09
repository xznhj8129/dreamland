"""The web server's SIM run backend: one INAV SITL or controller-script run at a time, ported from the retired Tk
SIM view (libs/views/sim_view.py; user-visible behavior in docs/ui/sim_view.md).

options() is the panel's option lists, start_run() saves the definition and starts the same subprocess the Tk view
started (sim_inav.py or sim_script.py), the stdout reader thread fills the summary lines, and state_payload() tails
the CSV flight log through client-held offsets. Only one run exists at a time (the SITL bench uses fixed TCP ports)
and it is SIGINT-ed on Abort or server shutdown, then killed if it outlives SHUTDOWN_WAIT_S.

sim_inav.py / sim_script.py / libs.simrun load lazily inside the functions that need them: they pull in inavbench,
matplotlib and AeroSandbox, which server startup must not pay for (webapp.cad keeps its imports lazy the same way).
"""

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

REPO_DIR = Path(__file__).resolve().parent.parent  # the runners and their cwd, as sim_inav.REPO_DIR
SIM_INAV = REPO_DIR / "sim_inav.py"
SIM_SCRIPT = REPO_DIR / "sim_script.py"

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


def options(state: AppState) -> dict[str, Any]:
    """What the SIM panel offers for the current project (sim_view.SimMainView.refresh).

    The starts are the forward-flight ones start_reasons() allows (air, then ground or a launcher); hover is INAV-only
    and always starts on the ground, which start_run() enforces by omitting --start exactly as the Tk view did.
    start_options carries each condition's flags (air_only, launcher_only, catapult_only) and launcher_defaults the
    placeholder values each launch start's launcher fields show.
    """
    from libs.controller import controller_names
    from libs.simrun import START_OPTIONS, launcher_defaults, start_reasons
    from sim_inav import is_real_fc_diff, tests_for
    from sim_script import DEFAULT_DURATION_S

    project = state.project
    resolved = state.resolved
    airframe = resolved.airframe if resolved is not None else None
    return {
        "controllers": controller_names(project.controllers_dir),
        "inav_configs": [
            {
                "file": path.name,
                "label": ("real FC diff: " if is_real_fc_diff(path) else "SITL diff: ") + path.name,
            }
            for path in sorted(project.inav_dir.glob("*.txt"))
        ],
        "scenarios": tests_for(airframe) if airframe is not None else [],
        "starts": [start for start, reason in start_reasons(airframe).items() if reason is None] if airframe is not None else [],
        "start_options": [
            {"key": key, "label": START_LABELS[key], "air_only": spec.air_only,
             "launcher_only": spec.launcher_only, "catapult_only": spec.catapult_only}
            for key, spec in START_OPTIONS.items()
        ],
        "launcher_defaults": launcher_defaults(),
        "default_duration_s": DEFAULT_DURATION_S,
    }


def _plan(state: AppState, body: dict[str, Any]) -> tuple[list[str], str, str, Path]:
    """Validate one run request and build (argv, status, shown command line, log path), as Tk start_run did.

    The body names the controller kind ("inav" or "script"); an INAV run carries "scenario" and an optional
    "inav_config" file name (absent/"generated" = generated from the design), a script run carries "script".
    Both carry "start" and "conditions" (a number per START_OPTIONS key; blank = the runner's default).
    """
    from libs.controller import controller_names
    from libs.simrun import START_OPTIONS, start_reasons
    from sim_inav import flight_log_path, tests_for
    from sim_script import script_log_path

    if state.resolved is None:
        raise _Refused("No valid airframe; fix the definition first.")
    airframe = state.resolved.airframe
    project = state.project
    controller = body.get("controller", "inav")
    if controller not in ("inav", "script"):
        raise _Refused(f"controller must be 'inav' or 'script', not {controller!r}")

    given: dict[str, str] = {}
    conditions = body.get("conditions") or {}
    if not isinstance(conditions, dict):
        raise _Refused("conditions must be an object of start conditions")
    for key, value in conditions.items():
        option = str(key).replace("_", "-")  # wind_from is accepted as wind-from
        if option not in START_OPTIONS:
            raise _Refused(f"unknown start condition: {key!r}")
        given[option] = (value if isinstance(value, str) else str(value)).strip()

    if controller == "script":
        script = body.get("script")
        if script not in controller_names(project.controllers_dir):
            raise _Refused(f"no controller script {script!r} in {project.controllers_dir}")
        starts = [start for start, reason in start_reasons(airframe).items() if reason is None]
        start = body.get("start", "air")
        if start not in starts:
            raise _Refused(f"start must be one of {starts}")
        scenario = None
    else:
        tests = tests_for(airframe)
        scenario = body.get("scenario")
        if scenario not in tests:
            raise _Refused(f"scenario must be one of {tests}")
        if scenario == "hover":
            start = "ground"  # hover always stands on the ground (sim_inav forces it too)
        else:
            starts = [s for s, reason in start_reasons(airframe).items() if reason is None]
            start = body.get("start", "air")
            if start not in starts:
                raise _Refused(f"start must be one of {starts}")

    # The conditions in START_OPTIONS order, %g and blank values skipped, exactly as the Tk entries built them; an
    # air-only condition is skipped whenever the start is not air, a launcher condition whenever the start is not a
    # launch, and launch-speed on a rail (the runner refuses one passed by hand anyway).
    args: list[str] = []
    for option, spec in START_OPTIONS.items():
        text = given.get(option, "")
        if not text:
            continue
        if spec.air_only and start != "air":
            continue
        if spec.launcher_only and start not in ("catapult", "rail"):
            continue
        if spec.catapult_only and start != "catapult":
            continue
        try:
            number = float(text)
        except ValueError:
            raise _Refused(f"{START_LABELS[option]}: {text!r} is not a number") from None
        args += [f"--{option}", f"{number:g}"]

    if controller == "script":
        duration = body.get("duration", "")
        duration = "" if duration is None else (duration if isinstance(duration, str) else str(duration))
        duration = duration.strip()
        if duration:
            try:
                seconds = float(duration)
            except ValueError:
                raise _Refused(f"Duration (s): {duration!r} is not a number") from None
            args += ["--duration", f"{seconds:g}"]
        command = [sys.executable, "-u", str(SIM_SCRIPT), str(project.root), script, "--start", start, *args]
        status = f"Running script: {script}..."
        log_path = script_log_path(project, script)
    else:
        config = body.get("inav_config")
        if config in (None, "", "generated", "generated from the design"):
            config = None
        else:
            configs = {path.name for path in project.inav_dir.glob("*.txt")}
            if not isinstance(config, str) or config not in configs:
                raise _Refused(f"inav_config must be a .txt diff in {project.inav_dir}, or 'generated'")
        command = [sys.executable, "-u", str(SIM_INAV), scenario, str(project.root)]
        if scenario != "hover":
            command += ["--start", start]
        command += args
        if config is not None:
            command += ["--inav-config", str(project.inav_dir / config)]
        status = f"Running {scenario}..."
        log_path = flight_log_path(project, scenario, start)

    shown = ["python", *(os.path.relpath(part, REPO_DIR) if os.path.isabs(part) else part for part in command[2:])]
    return command, status, " ".join(shown), log_path


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
        """SIGINT as for closing the window; the runner writes its log and sim_inav stops its SITL."""
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
