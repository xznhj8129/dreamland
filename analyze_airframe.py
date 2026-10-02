"""
Usage:
python analyze_airframe.py aircraft/m62v2.json
python analyze_airframe.py aircraft/m62v2.json --aoa 5 --airspeed 20
python analyze_airframe.py aircraft/m62v2.json --sweep
python analyze_airframe.py aircraft/m62v2.json --export-step out.step --step-mode fused --export-stl out.stl

Plane-first analysis of an airframe/2 file: geometry, mass and CG (with ranges), volume checks,
aerodynamics and static stability, optional STEP/STL export. --aoa/--airspeed/--altitude override the file's operating
point.
"""

import argparse
import dataclasses
import json
from pathlib import Path

from libs.aero import analyze_aero
from libs.airframe import PARTS_PATH, load_airframe
from libs.cad import export_step, export_stl
from libs.geometry import resolve_airframe
from libs.report import build_report_text


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a plane-first airframe definition.")
    parser.add_argument("airframe", type=Path, help="airframe/2 JSON file")
    parser.add_argument("--aoa", type=float, default=None, help="Override operating point AoA (deg)")
    parser.add_argument("--airspeed", type=float, default=None, help="Override operating point airspeed (m/s)")
    parser.add_argument("--altitude", type=float, default=None, help="Override operating point altitude (m)")
    parser.add_argument("--sweep", action="store_true", help="Print the full AoA sweep table")
    parser.add_argument("--export-step", type=Path, default=None, help="Write the airframe as STEP")
    parser.add_argument("--step-mode", choices=["assembly", "fused"], default="assembly", help="STEP: named parts per section, or one fused outer shape")
    parser.add_argument("--export-stl", type=Path, default=None, help="Write the airframe as one STL mesh")
    parser.add_argument("--export-prop-discs", action="store_true", help="Include thin propeller discs in CAD exports (actuator-disc CFD)")
    args = parser.parse_args()

    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    airframe = load_airframe(args.airframe, parts_data)
    overrides = {key: value for key, value in (("aoa_deg", args.aoa), ("airspeed_m_s", args.airspeed), ("altitude_m", args.altitude)) if value is not None}
    if overrides:
        airframe = dataclasses.replace(airframe, operating_point=dataclasses.replace(airframe.operating_point, **overrides))

    resolved, problems = resolve_airframe(airframe, parts_data)
    aero = analyze_aero(resolved)
    print(f"Source: {args.airframe}")
    print(build_report_text(resolved, aero, problems, args.sweep))
    if args.export_step is not None:
        written_mode = export_step(resolved, args.export_step, mode=args.step_mode, include_prop_discs=args.export_prop_discs)
        print(f"STEP written ({written_mode}): {args.export_step}")
    if args.export_stl is not None:
        written_mode = export_stl(resolved, args.export_stl, include_prop_discs=args.export_prop_discs)
        print(f"STL written ({written_mode}): {args.export_stl}")


if __name__ == "__main__":
    main()
