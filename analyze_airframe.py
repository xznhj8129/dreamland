"""
Usage:
python analyze_airframe.py aircraft/m62v2.json
python analyze_airframe.py aircraft/m62v2.json --aoa 5 --airspeed 20
python analyze_airframe.py aircraft/m62v2.json --sweep

Plane-first analysis of an airframe/2 file: geometry, mass and CG (with ranges), volume checks,
AeroBuildup aerodynamics and static stability. --aoa/--airspeed/--altitude override the file's operating point.
"""

import argparse
import dataclasses
import json
from pathlib import Path

from libs.aero import analyze_aero
from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe
from libs.report import build_report_text


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a plane-first airframe definition.")
    parser.add_argument("airframe", type=Path, help="airframe/2 JSON file")
    parser.add_argument("--aoa", type=float, default=None, help="Override operating point AoA (deg)")
    parser.add_argument("--airspeed", type=float, default=None, help="Override operating point airspeed (m/s)")
    parser.add_argument("--altitude", type=float, default=None, help="Override operating point altitude (m)")
    parser.add_argument("--sweep", action="store_true", help="Print the full AoA sweep table")
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


if __name__ == "__main__":
    main()
