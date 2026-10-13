"""
Usage:
python analyze_airframe.py aircraft/m62v2
python analyze_airframe.py aircraft/m62v2 --aoa 5 --airspeed 20
python analyze_airframe.py aircraft/m62v2 --sweep
python analyze_airframe.py aircraft/m62v2 --export-step out.step --step-mode fused --export-stl out.stl
python analyze_airframe.py aircraft/m62v2 --export-sections --sections-format stl  # printable sections/couplers into cad/sections/ (STEP default)
python analyze_airframe.py aircraft/m62v2 --export-varsets     # FreeCAD varsets into the project's cad/varsets/
python analyze_airframe.py aircraft/m62v2 --export-drawings    # blueprint drawings: cad/cutaway, threeview, xray (.svg and .png)

Plane-first analysis of an aircraft project's airframe/2 build file: geometry, mass and CG (with ranges), volume checks,
aerodynamics and static stability, optional STEP/STL export of the whole airframe or of its printable parts, blueprint drawings (cutaway, three-view, x-ray), FreeCAD varsets. --aoa/--airspeed/--altitude override the file's operating
point.
"""

import argparse
import dataclasses
import sys
from pathlib import Path

from libs.aero import NoLiftingSurfacesError, OperatingPointError, analyze_aero
from libs.airframe import load_airframe, load_parts_data
from libs.cad import export_sections, export_step, export_stl
from libs.drawings import export_drawings
from libs.geometry import resolve_airframe
from libs.motorlib import MotorStalled
from libs.project import open_project
from libs.propulsion import propulsion_summary
from libs.report import build_report_text
from libs.varsets import export_varsets


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze a plane-first airframe definition.")
    parser.add_argument("project", type=Path, help="aircraft project directory (aircraft/<name>)")
    parser.add_argument("--aoa", type=float, default=None, help="Override operating point AoA (deg)")
    parser.add_argument("--airspeed", type=float, default=None, help="Override operating point airspeed (m/s)")
    parser.add_argument("--altitude", type=float, default=None, help="Override operating point altitude (m)")
    parser.add_argument("--sweep", action="store_true", help="Print the full AoA sweep table")
    parser.add_argument("--export-step", type=Path, default=None, help="Write the airframe as STEP")
    parser.add_argument("--step-mode", choices=["assembly", "fused"], default="assembly", help="STEP: named parts per section, or one fused outer shape")
    parser.add_argument("--export-stl", type=Path, default=None, help="Write the airframe as one STL mesh")
    parser.add_argument("--export-sections", action="store_true", help="Write each printable section and coupler as STEP or STL (--sections-format) into the project's cad/sections/")
    parser.add_argument("--sections-format", choices=["step", "stl"], default="step", help="Format for --export-sections (default: step)")
    parser.add_argument("--export-drawings", action="store_true", help="Draw the blueprint cutaway, three-view and x-ray into the project's cad/ (.svg and .png)")
    parser.add_argument("--export-varsets", action="store_true", help="Write the FreeCAD varsets (one JSON each) into the project's cad/varsets/")
    parser.add_argument("--export-prop-discs", action="store_true", help="Include thin propeller discs in CAD exports (actuator-disc CFD)")
    args = parser.parse_args()

    parts_data = load_parts_data()
    project = open_project(args.project)
    airframe = load_airframe(project.build_path, parts_data)
    overrides = {key: value for key, value in (("aoa_deg", args.aoa), ("airspeed_m_s", args.airspeed), ("altitude_m", args.altitude)) if value is not None}
    if overrides:
        airframe = dataclasses.replace(airframe, operating_point=dataclasses.replace(airframe.operating_point, **overrides))

    resolved, problems = resolve_airframe(airframe, parts_data)
    try:
        aero = analyze_aero(resolved)
    except (OperatingPointError, NoLiftingSurfacesError) as error:
        print(f"Aero not computed: {error}", file=sys.stderr)
        aero = None
    print(f"Source: {project.build_path}")
    propulsion = None
    if resolved.motors:
        try:
            propulsion = propulsion_summary(resolved, parts_data)
        except MotorStalled as error:
            print(f"Propulsion: {error}", file=sys.stderr)
    print(build_report_text(resolved, aero, problems, propulsion, args.sweep))
    if args.export_step is not None:
        written_mode = export_step(resolved, args.export_step, mode=args.step_mode, include_prop_discs=args.export_prop_discs)
        print(f"STEP written ({written_mode}): {args.export_step}")
    if args.export_stl is not None:
        written_mode = export_stl(resolved, args.export_stl, include_prop_discs=args.export_prop_discs)
        print(f"STL written ({written_mode}): {args.export_stl}")
    if args.export_sections:
        written, notes = export_sections(resolved, project.sections_dir, args.sections_format)
        print("\n".join(written))
        for note in notes:
            print(f"Section export: {note}", file=sys.stderr)
    if args.export_drawings:
        for path in export_drawings(resolved, project.cad_dir):
            print(f"Drawing written: {path}")
    if args.export_varsets:
        for path in export_varsets(airframe, project.varsets_dir):
            print(f"Varset written: {path}")


if __name__ == "__main__":
    main()
