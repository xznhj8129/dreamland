"""Headless assertions for the SIM run backend (webapp/sim.py, the /api/sim routes).

Run: uv run python -m webapp.test_sim

Flies the real aircraft/example project, so its runs land in its logs/ and an export-code change rebuilds its own model
once. The suite's own files (two INAV diffs, an mcu launch_check scenario) are written for the run and removed after,
never over existing ones. Checks: the exact argv (including both
launch starts and their log suffixes), the launcher start-condition defaults, flags and refusals, the options payload,
one finished script run with incremental stdout/log deltas and offset resume, catapult and rail script runs checked
for launch mechanics only (the catapult with overridden launcher values), a nan/inf
log cell that must reach the JSON as null, a refused bad condition, a refused second start while a run is live, a SIGINT
abort (exit code 130) and the server-exit shutdown path.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import io
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

from libs.jsbsim import DEFAULT_WORKERS, MAX_WORKERS  # noqa: E402
from libs.project import Project  # noqa: E402
from libs.simrun import KNOT_M_S, add_start_arguments, apply_start_defaults, resolve_start, start_line  # noqa: E402
from webapp import sim  # noqa: E402
from webapp.server import create_app  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"
RUN_TIMEOUT_S = 600.0  # a run after an export-code change first rebuilds the example's model
SITL_DIFF, REAL_DIFF = "test-sitl.txt", "test-real.txt"  # the suite's INAV diffs, removed after the run
LAUNCH_CHECK = "launch_check"  # the suite's mcu launch-mechanics scenario, removed after the run
POLL_S = 0.25


def suite_files() -> dict[Path, str]:
    """The files the suite adds to the example for its run; none may exist already."""
    return {
        EXAMPLE / "inav" / SITL_DIFF: "set nav_fw_cruise_thr 1400\n",
        EXAMPLE / "inav" / REAL_DIFF: "# INAV/MATEKF405 7.0.0\nset nav_fw_cruise_thr 1500\n",
        EXAMPLE / "scenarios" / "mcu" / f"{LAUNCH_CHECK}.py": (
            "from controllers import mcu as controller\n\n"
            "def run(flight):\n"
            "    controller.esc[:] = [2000] * len(controller.esc)\n"
            "    controller.arm()\n"
            "    controller.release()\n"
            "    while True:\n"
            "        flight.wait(0.01)\n"
        ),
    }


def make_state() -> AppState:
    """The real example project, aero off."""
    state = AppState(Project(root=EXAMPLE))
    original = state.recompute
    state.recompute = lambda run_aero=True: original(run_aero=False)
    state.recompute()
    assert state.resolved is not None, "the example airframe did not resolve"
    return state


async def get_state(client: TestClient, stdout_offset: int = 0, log_offset: int = 0, run_id: int | None = None) -> dict:
    params = {"stdout_offset": str(stdout_offset), "log_offset": str(log_offset)}
    if run_id is not None:
        params["run_id"] = str(run_id)
    response = await client.get("/api/sim/state", params=params)
    assert response.status == 200, response.status
    return await response.json()


def check_argv(state: AppState) -> None:
    """Both controllers select scripts through the same CLI and scheduler arguments."""
    command, status, shown, log_path = sim._plan(state, {
        "controller": "mcu", "scenario": "level", "clock": "stepped", "start": "air",
        "conditions": {"alt": "100", "roll": "5"}, "duration": "2", "workers": 1,
    })
    assert command == [
        sys.executable, "-u", str(ROOT / "sim_flight.py"), str(state.project.root), "level",
        "--controller", "mcu", "--start", "air", "--clock", "stepped", "--rate", "200", "--duration", "2", "--workers", "1",
        "--alt", "100", "--roll", "5",
    ], command
    assert status == "Running mcu: level..." and log_path == state.project.logs_dir / "mcu_level_air.csv"
    assert shown == f"python sim_flight.py {os.path.relpath(state.project.root, ROOT)} level --controller mcu --start air --clock stepped --rate 200 --duration 2 --workers 1 --alt 100 --roll 5", shown

    command, _, _, _ = sim._plan(state, {"controller": "mcu", "scenario": "level", "start": "air"})
    assert command[command.index("--workers") + 1] == str(DEFAULT_WORKERS), command
    command, _, _, _ = sim._plan(state, {"controller": "mcu", "scenario": "level", "start": "air", "workers": MAX_WORKERS + 1})
    assert command[command.index("--workers") + 1] == str(MAX_WORKERS), command  # more than the cap is clamped to it
    try:
        sim._plan(state, {"controller": "mcu", "scenario": "level", "start": "air", "workers": 0})
        raise AssertionError("workers 0 was accepted")
    except sim._Refused as refused:
        assert "at least 1" in str(refused), refused

    command, status, shown, log_path = sim._plan(state, {
        "controller": "inav", "scenario": "efficiency", "start": "air",
        "conditions": {"alt": "100", "wind_from": "270"}, "inav_config": SITL_DIFF,
    })
    assert command == [
        sys.executable, "-u", str(ROOT / "sim_flight.py"), str(state.project.root), "efficiency",
        "--controller", "inav", "--start", "air", "--clock", "realtime", "--rate", "200", "--duration", "600",
        "--workers", str(DEFAULT_WORKERS),
        "--alt", "100", "--wind-from", "270", "--inav-config", str(state.project.inav_dir / SITL_DIFF),
    ], command
    assert status == "Running inav: efficiency..." and log_path == state.project.logs_dir / "inav_efficiency_air.csv"
    print("ok   argv: both controllers use sim_flight.py and common scheduling arguments")


def check_launch_plan(state: AppState) -> None:
    """A launch start plans --start <start> and its own flight log; launcher conditions resolve, skip or are refused."""
    for start in ("catapult", "rail"):
        command, status, shown, log_path = sim._plan(state, {
            "controller": "inav", "scenario": "controls", "start": start, "conditions": {},
        })
        assert command == [
            sys.executable, "-u", str(ROOT / "sim_flight.py"), str(state.project.root), "controls", "--controller", "inav", "--start", start, "--clock", "realtime", "--rate", "200", "--duration", "600",
            "--workers", str(DEFAULT_WORKERS),
        ], command
        assert status == "Running inav: controls..." and log_path == state.project.logs_dir / f"inav_controls_{start}.csv", (status, log_path)
        assert shown == f"python sim_flight.py {os.path.relpath(state.project.root, ROOT)} controls --controller inav --start {start} --clock realtime --rate 200 --duration 600 --workers {DEFAULT_WORKERS}", shown

    # launcher overrides pass through in START_OPTIONS order
    command, *_ = sim._plan(state, {
        "controller": "inav", "scenario": "controls", "start": "catapult",
        "conditions": {"launch-speed": "20", "launch-pitch": "12"},
    })
    assert command == [
        sys.executable, "-u", str(ROOT / "sim_flight.py"), str(state.project.root), "controls", "--controller", "inav", "--start", "catapult", "--clock", "realtime", "--rate", "200", "--duration", "600",
        "--workers", str(DEFAULT_WORKERS),
        "--launch-pitch", "12", "--launch-speed", "20",
    ], command

    # a launcher condition off a launch start, and launch-speed on the unpowered rail, are skipped (the runner refuses)
    for start, conditions in (("air", {"launch-pitch": "12"}), ("rail", {"launch-speed": "20"})):
        command, *_ = sim._plan(state, {"controller": "inav", "scenario": "controls", "start": start, "conditions": conditions})
        assert command == [
            sys.executable, "-u", str(ROOT / "sim_flight.py"), str(state.project.root), "controls", "--controller", "inav", "--start", start, "--clock", "realtime", "--rate", "200", "--duration", "600",
            "--workers", str(DEFAULT_WORKERS),
        ], (start, command)

    parser = argparse.ArgumentParser()
    add_start_arguments(parser)

    def resolve(option_args: list[str], start: str):
        args = parser.parse_args(["--start", start, *option_args])
        return resolve_start(parser, args, start, 150.0, 15.0)

    def refused(option_args: list[str], start: str, message: str) -> None:
        args = parser.parse_args(["--start", start, *option_args])
        with contextlib.redirect_stderr(io.StringIO()) as stderr:
            try:
                resolve_start(parser, args, start, 150.0, 15.0)
            except SystemExit as exit_code:
                assert exit_code.code == 2, exit_code
            else:
                raise AssertionError(f"start {start} accepted {option_args}")
        assert message in stderr.getvalue(), stderr.getvalue()

    # the runner itself refuses an air-only option for a launch start, naming the launcher, before anything flies
    log = state.project.logs_dir / "inav_controls_catapult.csv"
    before = log.stat().st_mtime_ns if log.exists() else None
    refused(["--alt", "100"], "catapult", "--alt: a launcher start sits on the rail, at rest")
    assert (log.stat().st_mtime_ns if log.exists() else None) == before, "a refused start must not fly"

    # the supplied overrides land on the resolved launcher, and the start line prints them
    catapult = resolve(["--launch-pitch", "12", "--launch-length", "2.5", "--launch-speed", "20", "--launch-height", "1.8"], "catapult")
    assert (catapult.launcher.pitch_deg, catapult.launcher.length_m, catapult.launcher.speed_ms, catapult.launcher.height_m) == (12.0, 2.5, 20.0, 1.8), catapult.launcher
    assert start_line("catapult", catapult) == "start: catapult, 12 deg, 2.5 m rail, 20.0 m/s, 1.8 m above ground", start_line("catapult", catapult)
    defaults = resolve([], "catapult")
    assert (defaults.launcher.pitch_deg, defaults.launcher.length_m, defaults.launcher.speed_ms, defaults.launcher.height_m) == (10.0, 2.0, 15.0, 1.5), defaults.launcher
    rail = resolve(["--launch-pitch", "25"], "rail")
    assert (rail.launcher.pitch_deg, rail.launcher.length_m, rail.launcher.height_m) == (25.0, 3.0, 1.0), rail.launcher
    assert start_line("rail", resolve([], "rail")) == "start: rail, 20 deg, 3.0 m long, 1.0 m above ground"
    assert resolve([], "air").launcher is None

    # a launcher option on air/ground, launch-speed on the rail, and non-positive values are refused with the option named
    refused(["--launch-pitch", "12"], "air", "--launch-pitch: only a catapult or rail start uses it")
    refused(["--launch-height", "2"], "ground", "--launch-height: only a catapult or rail start uses it")
    refused(["--launch-speed", "20"], "rail", "--launch-speed: the rail is unpowered")
    refused(["--launch-speed", "-1"], "catapult", "--launch-speed: must be positive, got -1")
    refused(["--launch-length", "0"], "rail", "--launch-length: must be positive, got 0")
    print("ok   launch plan: --start catapult/rail with their log suffixes; launcher overrides resolve in order and print; "
          "air-only, off-launcher, rail-speed and non-positive values refused")


async def check_options(client: TestClient) -> None:
    response = await client.get("/api/sim/options")
    assert response.status == 200, response.status
    payload = await response.json()
    assert payload["controllers"] == ["inav", "mcu"], payload["controllers"]
    configs = {item["file"]: item["label"] for item in payload["inav_configs"]}
    assert "generated" not in configs, "generated is the default, not an offered file"
    assert configs[SITL_DIFF] == f"SITL diff: {SITL_DIFF}", configs
    assert configs[REAL_DIFF] == f"real FC diff: {REAL_DIFF}", configs
    assert {"efficiency", "top-speed", "controls", "glide", "experiment"} <= set(payload["scenarios"]["inav"]), payload["scenarios"]
    assert {"launch", "level", "stability", LAUNCH_CHECK} <= set(payload["scenarios"]["mcu"]), payload["scenarios"]
    assert payload["starts"] == ["air", "catapult", "rail"], payload["starts"]
    options_list = payload["start_options"]
    assert [item["key"] for item in options_list] == [
        "alt", "speed", "heading", "lat", "lon", "pitch", "roll", "wind-from", "wind-speed",
        "launch-pitch", "launch-length", "launch-speed", "launch-height",
    ], options_list
    labels = {item["key"]: item["label"] for item in options_list}
    assert labels["alt"] == "Altitude (m)" and labels["wind-from"] == "Wind from (deg)"
    assert labels["launch-pitch"] == "Launcher pitch (deg)" and labels["launch-height"] == "Launcher height (m)"
    assert labels["launch-length"] == "Launcher length (m)" and labels["launch-speed"] == "Catapult speed (m/s)"
    assert {item["key"] for item in options_list if item["air_only"]} == {"alt", "speed", "pitch", "roll"}
    assert {item["key"] for item in options_list if item["launcher_only"]} == {"launch-pitch", "launch-length", "launch-speed", "launch-height"}
    assert {item["key"] for item in options_list if item["catapult_only"]} == {"launch-speed"}
    assert payload["launcher_defaults"] == {
        "catapult": {"launch-pitch": 10.0, "launch-length": 2.0, "launch-speed": 15.0, "launch-height": 1.5},
        "rail": {"launch-pitch": 20.0, "launch-length": 3.0, "launch-height": 1.0},
    }, payload["launcher_defaults"]
    assert payload["default_duration_s"] == 600.0
    print("ok   options: controllers, labelled diffs (no generated), scenarios, starts, nine start conditions, "
          "four labelled launcher fields and their per-start defaults")


async def check_bad_condition(client: TestClient) -> None:
    before = await get_state(client)
    response = await client.post("/api/sim/run", json={
        "controller": "mcu", "clock": "stepped", "scenario": "level", "start": "air", "conditions": {"alt": "abc"}, "duration": "2",
    })
    assert response.status == 400, response.status
    payload = await response.json()
    assert payload == {"ok": False, "message": "Altitude (m): 'abc' is not a number"}, payload
    for bad in (
        {"controller": "mcu", "clock": "stepped", "scenario": "nope"},
        {"controller": "inav", "scenario": "nope"},
        {"controller": "mcu", "clock": "stepped", "scenario": "level", "start": "ground"},  # the example cannot take off yet
        {"controller": "mcu", "clock": "stepped", "scenario": "level", "conditions": {"bogus": "1"}},
        {"controller": "mcu", "clock": "stepped", "scenario": "level", "duration": "abc"},
        {"controller": "inav", "scenario": "efficiency", "inav_config": f"../{SITL_DIFF}"},
    ):
        response = await client.post("/api/sim/run", json=bad)
        assert response.status == 400, (bad, response.status)
    after = await get_state(client)
    assert after["running"] is False and after["run_id"] == before["run_id"], "a refused start must not start a run"
    print("ok   bad condition: refused with the labelled message; bad script/scenario/start/key/config refused too, no run started")


async def check_script_run(client: TestClient) -> None:
    response = await client.post("/api/sim/run", json={
        "controller": "mcu", "clock": "stepped", "scenario": "level", "start": "air",
        "conditions": {"alt": "100", "wind_from": "270"}, "duration": "2",
    })
    assert response.status == 200, response.status
    assert await response.json() == {"ok": True}

    lines: list[str] = []
    rows: list[list[float]] = []
    header: list[str] | None = None
    stdout_offset = log_offset = 0
    run_id: int | None = None
    first_lines = None
    deadline = time.monotonic() + RUN_TIMEOUT_S
    while True:
        state = await get_state(client, stdout_offset, log_offset, run_id)
        run_id = state["run_id"]
        lines.extend(state["stdout"]["lines"])
        if first_lines is None:
            first_lines = len(state["stdout"]["lines"])
        stdout_offset = state["stdout"]["offset"]
        if state["log"]["header"] is not None:
            header = state["log"]["header"]
        rows.extend(state["log"]["rows"])
        log_offset = state["log"]["offset"]
        if not state["running"]:
            final = state
            break
        assert time.monotonic() < deadline, "the script run did not finish"
        await asyncio.sleep(POLL_S)

    assert final["status"] == "Finished (exit code 0)" and final["exit_code"] == 0, (final["status"], final["exit_code"])
    assert first_lines == 1, f"the first poll should carry only the command line, saw {first_lines}"
    assert lines[0].startswith("python ") and lines[0] == final["command"], lines[0]
    assert "sim_flight.py" in lines[0] and "--duration 2" in lines[0], lines[0]
    assert any("Scenario stopped: 2 s simulated time limit" in line for line in lines), lines[-5:]
    assert len(lines) > first_lines, "no stdout delta arrived after the command line"
    assert final["log"]["file"] == "logs/mcu_level_air.csv", final["log"]["file"]
    assert header is not None and "t" in header and "agl_m" in header, header
    assert rows and all(len(row) == len(header) for row in rows), (len(rows), len(header) if header else None)
    assert all(isinstance(value, float) for value in rows[0]), "rows must be JSON numbers"

    quiet = await get_state(client, stdout_offset, log_offset, run_id)
    assert quiet["log"]["reset"] is False
    assert quiet["stdout"]["lines"] == [] and quiet["stdout"]["offset"] == stdout_offset, quiet["stdout"]
    assert quiet["log"]["rows"] == [] and quiet["log"]["offset"] == log_offset, (quiet["log"]["offset"], log_offset)
    print(f"ok   script run: command line first, {len(lines)} stdout lines and {len(rows)} log rows arrived in deltas, offsets resume")


async def run_to_end(client: TestClient, body: dict) -> tuple[dict, list[str], list[str] | None, list[list[float | None]]]:
    """POST one run and poll until it ends; returns (final state, stdout lines, log header, log rows)."""
    response = await client.post("/api/sim/run", json=body)
    assert response.status == 200, (response.status, await response.text())
    assert await response.json() == {"ok": True}
    lines: list[str] = []
    rows: list[list[float | None]] = []
    header: list[str] | None = None
    stdout_offset = log_offset = 0
    run_id: int | None = None
    deadline = time.monotonic() + RUN_TIMEOUT_S
    while True:
        state = await get_state(client, stdout_offset, log_offset, run_id)
        run_id = state["run_id"]
        lines.extend(state["stdout"]["lines"])
        stdout_offset = state["stdout"]["offset"]
        if state["log"]["header"] is not None:
            header = state["log"]["header"]
        rows.extend(state["log"]["rows"])
        log_offset = state["log"]["offset"]
        if not state["running"]:
            return state, lines, header, rows
        assert time.monotonic() < deadline, "the run did not finish"
        await asyncio.sleep(POLL_S)


async def check_launch_runs(client: TestClient, state: AppState) -> None:
    """Real script runs from both launchers, launch mechanics only (the dumb full-PWM, neutral-servo script
    makes no attempt to fly): the catapult fires at the hold, and either launcher is left behind (speed and height
    gained). No stability or flight assertion."""
    for start in ("catapult", "rail"):
        # the catapult run overrides its launcher (the start line must print the resolved values); the rail keeps the default
        conditions = {"launch-pitch": "12", "launch-length": "2.5", "launch-speed": "18", "launch-height": "1.8"} if start == "catapult" else {}
        final, lines, header, rows = await run_to_end(client, {
            "controller": "mcu", "clock": "stepped", "scenario": LAUNCH_CHECK, "start": start, "conditions": conditions, "duration": "5",
        })
        assert final["status"] == "Finished (exit code 0)", (start, final["status"], lines[-5:])
        if start == "catapult":
            assert "catapult fired at 1.0 s simulated" in lines, (start, lines[-5:])  # fires at the hold
            assert "start: catapult, 12 deg, 2.5 m rail, 18.0 m/s, 1.8 m above ground" in lines, lines[:20]
        else:
            assert "start: rail, 20 deg, 3.0 m long, 1.0 m above ground" in lines, lines[:20]
        assert header is not None and {"t", "agl_m", "ias_kts", "motor1"} <= set(header), (start, header)
        columns = {name: index for index, name in enumerate(header)}
        heights = [row[columns["agl_m"]] for row in rows]
        speeds = [row[columns["ias_kts"]] * KNOT_M_S for row in rows]
        full_throttle = all(row[columns["motor1"]] == 2000 for row in rows)
        assert full_throttle, (start, {row[columns["motor1"]] for row in rows})  # the motor runs up under command
        assert max(speeds) > speeds[0] + 5.0, (start, speeds[0], max(speeds))  # moving down / pushed up the rail
        assert max(heights) > heights[0] + 0.5, (start, heights[0], max(heights))  # off the rail
        print(f"ok   {start} launch run: exit 0, off the rail (max {max(heights):.1f} m agl, {max(speeds):.1f} m/s), {len(rows)} CSV rows")


def _reject_json_constant(name: str) -> None:
    raise AssertionError(f"non-JSON constant {name!r} reached the payload")


async def check_nan_rows(client: TestClient, state: AppState) -> None:
    """A nan/inf log cell (a log() column before its first value) must serialize as null, never as NaN/Infinity.

    Appends one such row to the finished script run's log and reads it back through the real route: browser
    JSON.parse rejects the NaN token Python's json writes for a non-finite float.
    """
    log_path = state.project.logs_dir / "mcu_level_air.csv"
    assert log_path.exists(), log_path
    with open(log_path, "a") as file:
        file.write("nan,1.5,inf,-inf,2.5\n")
    response = await client.get("/api/sim/state")
    assert response.status == 200, response.status
    text = await response.text()
    assert "NaN" not in text and "Infinity" not in text, "a non-finite JSON constant reached the browser"
    payload = json.loads(text, parse_constant=_reject_json_constant)
    assert payload["log"]["rows"][-1] == [None, 1.5, None, None, 2.5], payload["log"]["rows"][-1]
    print("ok   nan rows: nan/inf log cells are null in the JSON; no NaN/Infinity token reaches JSON.parse")


async def check_abort_and_refusal(client: TestClient) -> None:
    body = {"controller": "mcu", "clock": "realtime", "scenario": "level", "start": "air", "conditions": {"alt": "100"}, "duration": "3600"}
    response = await client.post("/api/sim/run", json=body)
    assert response.status == 200 and await response.json() == {"ok": True}

    lines: list[str] = []
    run_id: int | None = None
    stdout_offset = log_offset = 0
    deadline = time.monotonic() + 120.0
    while not any(line.startswith("start:") for line in lines):  # wait for the child's imports/export reuse to finish
        assert time.monotonic() < deadline, "the long run never started flying"
        state = await get_state(client, stdout_offset, log_offset, run_id)
        run_id = state["run_id"]
        lines.extend(state["stdout"]["lines"])
        stdout_offset = state["stdout"]["offset"]
        log_offset = state["log"]["offset"]
        assert state["running"], "the long run ended before it was aborted"
        await asyncio.sleep(POLL_S)

    response = await client.post("/api/sim/run", json=body)
    assert response.status == 400, response.status
    message = (await response.json())["message"]
    assert "already active" in message, message

    response = await client.post("/api/sim/abort")
    assert response.status == 200 and await response.json() == {"ok": True}
    assert (await get_state(client, stdout_offset, log_offset, run_id))["status"] == "Aborting...", "abort must show immediately"

    deadline = time.monotonic() + 120.0
    final = None
    while time.monotonic() < deadline:
        state = await get_state(client, stdout_offset, log_offset, run_id)
        run_id = state["run_id"]
        lines.extend(state["stdout"]["lines"])
        stdout_offset = state["stdout"]["offset"]
        log_offset = state["log"]["offset"]
        if not state["running"]:
            final = state
            break
        await asyncio.sleep(POLL_S)
    assert final is not None, "the aborted run did not exit"
    assert final["status"] == "Aborted" and final["exit_code"] == 130, (final["status"], final["exit_code"])
    assert any(line.strip() == "Aborted." for line in lines), lines[-5:]
    print("ok   abort: live run refused a second start, SIGINT ended it as Aborted (exit code 130)")


async def check_shutdown(client: TestClient) -> None:
    """The server-exit path: shutdown() SIGINTs a live run and does not return before it is gone."""
    body = {"controller": "mcu", "clock": "realtime", "scenario": "level", "start": "air", "conditions": {"alt": "100"}, "duration": "3600"}
    response = await client.post("/api/sim/run", json=body)
    assert response.status == 200 and await response.json() == {"ok": True}
    run_id: int | None = None
    stdout_offset = log_offset = 0
    lines: list[str] = []
    deadline = time.monotonic() + 120.0
    while not any(line.startswith("start:") for line in lines):
        assert time.monotonic() < deadline, "the shutdown run never started flying"
        state = await get_state(client, stdout_offset, log_offset, run_id)
        run_id = state["run_id"]
        lines.extend(state["stdout"]["lines"])
        stdout_offset = state["stdout"]["offset"]
        log_offset = state["log"]["offset"]
        await asyncio.sleep(POLL_S)

    sim.shutdown()  # called from webbuilder.serve's finally, immediately before runner.cleanup()
    deadline = time.monotonic() + 30.0
    while True:
        final = await get_state(client, stdout_offset, log_offset, run_id)
        if not final["running"]:
            break
        assert time.monotonic() < deadline, "the run survived sim.shutdown()"
        await asyncio.sleep(POLL_S)
    assert final["status"] == "Aborted", final["status"]
    print("ok   shutdown: a live run is gone before the server cleanup returns")


def check_start_ownership(state: AppState) -> None:
    """Start conditions: a scenario's optional START supplies defaults; the toolbar and CLI override them.

    acro_climb declares no START, so planning it without a start falls back to air. Explicit toolbar start values are
    passed through to sim_flight.py unchanged. A START fills only what the CLI leaves out; a bad START is refused.
    """
    from libs.scenario import ScenarioSpec

    def start_value(command: list[str], option: str) -> str:
        return command[command.index(option) + 1]

    # 1. the acro_climb resource and the options scenario templates carry no initial-conditions payload, and all parse
    acro = sim.read_resource(state, "scenario", "acro_climb.py", "inav")
    assert "initial" not in acro, sorted(acro)
    assert acro["error"] is None, acro["error"]
    ScenarioSpec.from_source(acro["source"], "inav")
    templates = sim.options(state)["scenario_templates"]
    for controller in ("inav", "mcu"):
        payload = templates[controller]
        assert "initial" not in payload, (controller, sorted(payload))
        assert payload["error"] is None, (controller, payload["error"])
        ScenarioSpec.from_source(payload["source"], controller)

    # 2. acro_climb, which used to declare a catapult start, now defaults to air with no start supplied
    command, status, _shown, log_path = sim._plan(state, {"controller": "inav", "scenario": "acro_climb"})
    assert start_value(command, "--start") == "air", command
    assert status == "Running inav: acro_climb..." and log_path.name == "inav_acro_climb_air.csv", (status, log_path)

    # 3. explicit toolbar values reach sim_flight.py and resolve to the real launcher unchanged
    command, *_ = sim._plan(state, {"controller": "inav", "scenario": "experiment", "start": "catapult",
                                    "conditions": {"launch-pitch": "30", "launch-speed": "20"}})
    assert start_value(command, "--start") == "catapult", command
    assert start_value(command, "--launch-pitch") == "30" and start_value(command, "--launch-speed") == "20", command
    parser = argparse.ArgumentParser()
    add_start_arguments(parser)
    args = parser.parse_args(["--start", start_value(command, "--start"),
                              "--launch-pitch", start_value(command, "--launch-pitch"),
                              "--launch-speed", start_value(command, "--launch-speed")])
    resolved = resolve_start(parser, args, args.start, 150.0, 15.0)
    assert (resolved.launcher.pitch_deg, resolved.launcher.speed_ms) == (30.0, 20.0), resolved.launcher

    # 4. the CLI parser with no --start defaults to air
    bare = parser.parse_args([])
    apply_start_defaults(bare, {})
    assert bare.start == "air", bare.start

    # 5. a scenario's START fills what the CLI leaves out, the CLI wins, and the outline carries it
    declared = 'START = {"start": "rail", "launch-pitch": 85}\n\ndef run(flight):\n    pass\n'
    spec = ScenarioSpec.from_source(declared, "mcu")
    assert spec.payload()["start"] == {"start": "rail", "launch-pitch": 85}, spec.payload()
    args = parser.parse_args([])
    apply_start_defaults(args, spec.start)
    assert (args.start, args.launch_pitch) == ("rail", 85.0), args
    args = parser.parse_args(["--launch-pitch", "30"])
    apply_start_defaults(args, spec.start)
    assert (args.start, args.launch_pitch) == ("rail", 30.0), args
    args = parser.parse_args(["--start", "air"])
    apply_start_defaults(args, spec.start)
    assert (args.start, args.launch_pitch) == ("air", None), args
    for bad, message in (('{"altitude": 5}', "unknown"), ('{"start": "rail", "alt": 100}', "does not apply"),
                         ('{"start": "orbit"}', "start must be one of"), ('{"wind-speed": "3"}', "finite number")):
        try:
            ScenarioSpec.from_source(f"START = {bad}\n\ndef run(flight):\n    pass\n", "inav")
        except ValueError as problem:
            assert message in str(problem), (bad, problem)
        else:
            raise AssertionError(f"START = {bad} was accepted")

    print("ok   start ownership: the acro_climb resource and the options scenario templates carry no initial conditions; "
          "acro_climb plans air without a start; explicit catapult launch-pitch/speed pass through and resolve; "
          "CLI defaults to air; a scenario START fills the CLI's gaps, the CLI wins, bad STARTs are refused")


async def check_routes(state: AppState) -> None:
    app = create_app(state, ROOT / "web" / "dist")
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
        await check_options(client)
        await check_bad_condition(client)
        await check_script_run(client)
        await check_nan_rows(client, state)  # appends to the mcu_level_air.csv the last finished run tracks
        await check_launch_runs(client, state)
        await check_abort_and_refusal(client)
        await check_shutdown(client)
    finally:
        sim.shutdown()  # a failed check must not leave the runner alive after the check
        await client.close()


def main() -> None:
    files = suite_files()
    existing = [str(path) for path in files if path.exists()]
    if existing:
        raise SystemExit(f"the suite would overwrite {existing}; move them first")
    try:
        for path, text in files.items():
            path.write_text(text)
        state = make_state()
        check_argv(state)
        check_launch_plan(state)
        check_start_ownership(state)
        try:
            asyncio.run(check_routes(state))
        finally:
            sim.shutdown()
    finally:
        for path in files:
            path.unlink(missing_ok=True)
    print("sim suite OK")


if __name__ == "__main__":
    main()
