"""
Usage:
python sim_script.py aircraft/simpleplane level
python sim_script.py aircraft/simpleplane level --start catapult       # or rail; a fixed wing leaves its launcher
python sim_script.py aircraft/simpleplane level --duration 120 --alt 300 --heading 90 --wind-from 270 --wind-speed 5

Flies an aircraft project's JSBSim export (libs/jsbsim.py, reused when nothing it depends on changed) with a flight
controller script from the project's controllers/ (libs/controller.py), in lockstep simulated time: the script's
loop() runs rate_hz times per simulated second, as fast as the computer allows. Start conditions as for sim_inav.py
(libs/simrun.py). The run ends after --duration simulated seconds, when the script calls stop(), or on the ground.
The flight log (logs/script_<name>.csv) has the same truth, motor and servo columns as an INAV run, the electrical
model, what the script's hardware read and the script's log() columns; the plots go next to it.
"""

import argparse
import csv
import sys
import time
from pathlib import Path

from inavbench.bench import LOG_HZ
from inavbench.vehicles import Plant

from libs.airframe import PlacedPart, load_airframe, load_parts_data
from libs.controller import load_controller
from libs.geometry import resolve_airframe
from libs.jsbsim import reuse_or_export
from libs.project import Project, open_project
from libs.simrun import (
    FT, START_ALT_M, add_start_arguments, build_vehicle, electrical_columns, ground_hit_text, hit_ground, resolve_start, start_line, start_reasons, write_plots,
)

DEFAULT_DURATION_S = 60.0
ABORTED_EXIT_CODE = 130  # stopped by SIGINT, as a shell reports Ctrl-C
LAUNCH_HOLD_S = 1.0  # the bench's default wait_s: a latched catapult fires only after its motor has run up


def script_log_path(project: Project, name: str) -> Path:
    """The flight log (CSV) of a script's run."""
    return project.logs_dir / f"script_{name}.csv"


def imu_mount(fc: PlacedPart, cg_m) -> tuple:
    """The FC's offset from the CG and its axes, in the plant's body axes (x forward, y left, z up; the geometry's x aft,
    y right, z up: both frames flip x and y, so the rotation is D R D with D = diag(-1, -1, 1))."""
    flip = (-1.0, -1.0, 1.0)
    offset = tuple(f * (p - c) for f, p, c in zip(flip, fc.position_m, cg_m))
    rotation = tuple(tuple(flip[i] * value * flip[j] for j, value in enumerate(row)) for i, row in enumerate(fc.rotation()))
    return offset, rotation


def main() -> None:
    parser = argparse.ArgumentParser(description="Fly an aircraft project with a flight controller script (libs/controller.py) in simulated time.")
    parser.add_argument("project", type=Path, help="aircraft project directory (aircraft/<name>)")
    parser.add_argument("controller", help="script name in the project's controllers/ (without .py)")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help=f"simulated seconds (default {DEFAULT_DURATION_S:g})")
    parser.add_argument("--pack-voltage", type=float, default=None, help="battery voltage for the motors (default: nominal)")
    add_start_arguments(parser)
    args = parser.parse_args()

    project = open_project(args.project)
    controller_class = load_controller(project.controllers_dir, args.controller)
    parts_data = load_parts_data()
    resolved, _problems = resolve_airframe(load_airframe(project.build_path, parts_data), parts_data)
    airframe = resolved.airframe
    start = args.start or "air"
    reason = start_reasons(airframe)[start]
    if reason is not None:
        parser.error(f"{args.project}: start {start}: {reason}")
    state = resolve_start(parser, args, start, START_ALT_M, airframe.operating_point.airspeed_m_s)
    export, reused = reuse_or_export(resolved, parts_data, project.jsbsim_dir, project.name, args.pack_voltage)
    motor_count, servo_count = len(export.motor_properties), len(export.servo_properties)
    print(f"{airframe.name}: {motor_count} motors (esc[0..{motor_count - 1}]), {servo_count} servos (servo[0..{servo_count - 1}]), "
          f"JSBSim model {'reused' if reused else 'exported'}; script {args.controller} ({controller_class.__name__}) at {controller_class.rate_hz:g} Hz")
    print(start_line(start, state))

    plant = Plant(build_vehicle(export, "", start, state, airframe.tailsitter), heading_deg=state.heading_deg,
                  home=(state.lat_deg, state.lon_deg), launcher=state.launcher)
    plant.hover_frame = False  # the script's IMU is fixed to the body, whatever the aircraft
    plant.set_wind(*state.wind_ned_m_s())
    controller = controller_class(motor_count, servo_count)
    fc = next((placed for placed in airframe.parts if placed.part.split(":")[0] == "fc"), None)
    if fc is not None:
        controller.imu._mount(*imu_mount(fc, resolved.cg_m))
        print(f"IMU on {fc.name} ({fc.part}), {[round(v * 1000.0, 1) for v in fc.position_m]} mm, yaw/pitch/roll {list(fc.orientation_deg)} deg")
    else:
        print("IMU at the CG, body axes (no fc part placed)")
    run_log = script_log_path(project, args.controller)
    run_log.parent.mkdir(parents=True, exist_ok=True)
    run_log.unlink(missing_ok=True)
    rows = []
    started = time.monotonic()
    try:
        fly(plant, controller, export, motor_count, args.duration, rows, start)
    finally:
        if rows:
            write_log(run_log, rows)
            print(f"{rows[-1]['t']:.1f} s simulated in {time.monotonic() - started:.1f} s; flight log: {run_log}")
            write_plots(run_log, resolved, f"{airframe.name}: script {args.controller}")


def fly(plant: Plant, controller, export, motor_count: int, duration_s: float, rows: list[dict], start: str) -> None:
    """The lockstep loop: sensors, the script's loop(), outputs to the plant, one step of 1 / rate_hz; a log row at
    LOG_HZ. Appends the rows as it goes, so a run that ends in an error keeps its log. A catapult start is latched
    until it fires at LAUNCH_HOLD_S simulated seconds (the bench's own wait_s): the script runs the motor up while
    held. A launch rail needs no fire: the vehicle's own thrust runs it up the rail (Plant.advance / Rail)."""
    step_us = round(1e6 / controller.rate_hz)
    log_every_us = round(1e6 / LOG_HZ)
    time_us, next_log_us, highest_agl_m = 0, 0, 0.0
    fired = start != "catapult"
    fdm = plant.fdm
    controller._sample(plant, time_us)
    controller.setup()
    while time_us < duration_s * 1e6 and not controller._stopped:
        controller._sample(plant, time_us)
        try:
            controller.loop()
        except Exception as error:
            error.add_note(f"in the script's loop() at {time_us / 1e6:.3f} s simulated")
            raise
        esc_us, servo_us = controller._outputs_us()
        plant.set_outputs(esc_us, servo_us)
        if time_us >= next_log_us:
            next_log_us += log_every_us
            rows.append(log_row(plant, controller, export, motor_count, time_us, esc_us, servo_us))
        if not fired and time_us >= LAUNCH_HOLD_S * 1e6:
            plant.fire()
            fired = True
            print(f"catapult fired at {time_us / 1e6:.1f} s simulated")
        plant.advance(step_us / 1e6)
        time_us += step_us
        highest_agl_m = max(highest_agl_m, fdm["position/h-agl-ft"] * FT)
        if hit_ground(fdm, highest_agl_m):
            print(f"Hit the ground {ground_hit_text(time_us / 1e6, fdm)}")
            return
    print("Stopped by the script." if controller._stopped else f"Ran the full {duration_s:g} s.")


def log_row(plant: Plant, controller, export, motor_count: int, time_us: int, esc_us: list[int], servo_us: list[int]) -> dict:
    """Truth, outputs, electrical model, the script's hardware readings and its log() values, as one row."""
    row = {"t": time_us / 1e6, "armed": 1} | plant.truth()
    row |= {f"motor{index + 1}": value for index, value in enumerate(esc_us)} | {f"servo{index}": value for index, value in enumerate(servo_us)}
    if export.motors:
        row |= electrical_columns(plant.fdm, export, motor_count)
    return row | controller._sensor_columns() | controller._logged


def write_log(path: Path, rows: list[dict]) -> None:
    """The rows as CSV; a log() column that started late reads nan before its first value."""
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(columns)
        for row in rows:
            writer.writerow([f"{row[key]:.7f}" if key in ("lat", "lon") else row.get(key, "nan") for key in columns])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:  # the SIM view's Abort (SIGINT): the log and plots are already written
        print("Aborted.")
        sys.exit(ABORTED_EXIT_CODE)
