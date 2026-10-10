"""Run one scheduled flight script with the selected controller.

python sim_flight.py aircraft/m62v1 experiment --controller inav
python sim_flight.py aircraft/example level --controller mcu --clock stepped
"""

import argparse
import importlib
import math
import sys
import time
from pathlib import Path

from libs.airframe import load_airframe, load_parts_data
from libs.flightlog import flight_log_path, flight_snapshot
from libs.geometry import resolve_airframe
from libs.jsbsim import reuse_or_export
from libs.project import open_project
from libs.scenario import CONTROLLERS, DEFAULT_LIMIT_S, Flight, ScenarioSpec, ScenarioStopped, apply_start_defaults, load_scenario, scenario_path
from libs.sensors import Hardware
from libs.simrun import GroundHit, START_ALT_M, add_start_arguments, resolve_start, start_line, start_reasons, write_plots


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    parser.add_argument("scenario", help="script name in scenarios/<controller>/")
    parser.add_argument("--controller", choices=CONTROLLERS, default="inav")
    parser.add_argument("--clock", choices=("realtime", "stepped", "stepped-faster"), default="realtime")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--duration", type=float, default=DEFAULT_LIMIT_S, help="maximum simulated seconds")
    parser.add_argument("--pack-voltage", type=float)
    parser.add_argument("--sitl", type=Path)
    parser.add_argument("--inav-config", type=Path)
    parser.add_argument("--no-msp", action="store_true")
    parser.add_argument("--no-mavlink", action="store_true")
    add_start_arguments(parser)
    args = parser.parse_args()
    for name in ("rate", "duration"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name} must be finite and positive")
    if args.controller != "inav" and (args.sitl or args.inav_config or args.no_msp or args.no_mavlink):
        parser.error("SITL, INAV settings, MSP and MAVLink options require the INAV controller")
    project = open_project(args.project)
    path = scenario_path(project.scenarios_dir, args.controller, args.scenario)
    source = path.read_text()
    spec = ScenarioSpec.from_source(source, args.controller)
    apply_start_defaults(args, source)
    parts = load_parts_data()
    resolved, _ = resolve_airframe(load_airframe(project.build_path, parts), parts)
    airframe = resolved.airframe
    reason = start_reasons(airframe)[args.start]
    if reason is not None:
        parser.error(f"start {args.start}: {reason}")
    state = resolve_start(parser, args, args.start, START_ALT_M, airframe.operating_point.airspeed_m_s)
    export, reused = reuse_or_export(resolved, parts, project.jsbsim_dir, project.name, args.pack_voltage)
    print(f"{airframe.name}: {len(export.motor_properties)} motors, {len(export.servo_properties)} servo outputs; model {'reused' if reused else 'exported'}")
    print(f"Flight aero: lifting line through ±{export.forward_alpha_limit_deg:.2f} deg AoA; "
          f"fades to AeroBuildup by ±{export.envelope_alpha_end_deg:.2f} deg")
    print(start_line(args.start, state))
    print(f"Controller: {args.controller}; scenario: {path.relative_to(project.root) if path.is_relative_to(project.root) else path.relative_to(Path(__file__).resolve().parent)}")
    print(f"Clock: {args.clock}, {args.rate:g} Hz")
    hardware = Hardware(len(export.motor_properties), len(export.servo_properties))
    hardware._logged = dict.fromkeys(spec.log_fields, math.nan)
    fc = next((placed for placed in airframe.parts if placed.part.split(":")[0] == "fc"), None)
    if fc is not None:
        flip = (-1.0, -1.0, 1.0)
        offset = tuple(f * (p - c) for f, p, c in zip(flip, fc.position_m, resolved.cg_m))
        rotation = tuple(tuple(flip[i] * value * flip[j] for j, value in enumerate(row)) for i, row in enumerate(fc.rotation()))
        hardware.imu._mount(offset, rotation)
    log_path = flight_log_path(project, args.controller, args.scenario, args.start)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.unlink(missing_ok=True)
    measurements = log_path.with_suffix(".measurements.json")
    measurements.unlink(missing_ok=True)
    log_path.with_suffix(".telemetry.csv").unlink(missing_ok=True)
    controller = importlib.import_module(f"controllers.{args.controller}")
    started = time.monotonic()
    try:
        with controller.connect(project, resolved, export, args, state, hardware, log_path) as engine:
            engine.plant.set_wind(*state.wind_ned_m_s())
            hardware._sample(engine.plant, 0)
            flight = Flight(engine, args.start, state, measurements, airframe=airframe, export=export,
                            control_count=len(resolved.controls), duration_s=args.duration)
            reserved = flight_snapshot(engine, export).keys() | hardware._sensor_columns().keys()
            overlap = set(spec.log_fields) & reserved
            if overlap:
                raise ValueError(f"LOG_FIELDS overlap simulator/sensor columns: {sorted(overlap)}")
            controller.bind(flight)
            flight.run(load_scenario(path), spec)
    except ScenarioStopped as stopped:
        print(f"Scenario stopped: {stopped}")
    except GroundHit as hit:
        print(f"Hit the ground {hit}; the run ends.")
    finally:
        print(f"Run wall time: {time.monotonic() - started:.2f} s")
        if log_path.exists():
            write_plots(log_path, resolved, f"{airframe.name}: {args.controller} / {args.scenario}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Aborted.")
        sys.exit(130)
