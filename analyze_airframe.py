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
import math
from pathlib import Path

from libs.aero import analyze_aero, stability_verdict
from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import GRAMS_FORCE_PER_NEWTON, STANDARD_GRAVITY_M_S2, resolve_airframe


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
    reference = resolved.reference
    mass_low, mass_high = resolved.mass_kg
    weight_n = (mass_low + mass_high) / 2.0 * STANDARD_GRAVITY_M_S2

    print(f"Airframe: {airframe.name}  ({args.airframe})")
    print()
    print("Mass")
    print(f"  Total            : {mass_low * 1000:.1f} g" if mass_low == mass_high else f"  Total            : {mass_low * 1000:.1f} - {mass_high * 1000:.1f} g")
    print(f"  CG               : x {resolved.cg_m[0]:.4f} m  y {resolved.cg_m[1]:.4f}  z {resolved.cg_m[2]:.4f}")
    if resolved.cg_x_range_m[0] != resolved.cg_x_range_m[1]:
        print(f"  CG x range       : {resolved.cg_x_range_m[0]:.4f} - {resolved.cg_x_range_m[1]:.4f} m (mass ranges)")
    for point in resolved.mass_points:
        mass_text = f"{point.mass_kg[0] * 1000:7.1f}" if point.mass_kg[0] == point.mass_kg[1] else f"{point.mass_kg[0] * 1000:.1f}-{point.mass_kg[1] * 1000:.1f}"
        print(f"    {point.name:<34} {mass_text:>12} g  x {point.position[0]:.4f}{'  (computed)' if point.computed else ''}")
    print()
    print("Geometry")
    print(f"  Fuselage         : {airframe.fuselage.length_m:.3f} m, section {airframe.fuselage.section.width_m * 1000:.1f} x {airframe.fuselage.section.height_m * 1000:.1f} mm")
    print(f"  Reference        : {reference.basis}  S {reference.area_m2:.4f} m^2  c {reference.chord_m:.4f} m  b {reference.span_m:.3f} m")
    for surface_set in resolved.surfaces:
        print(f"  {surface_set.name:<16} : {surface_set.definition.layout} x{len(surface_set.panels)}, area {surface_set.planform_area_m2:.4f} m^2, MAC {surface_set.mac_m:.4f} m")
    print(f"  Motors           : {len(resolved.motors)} x {airframe.propulsion.motor.brand} {airframe.propulsion.motor.model} {airframe.propulsion.motor.kv}KV, {airframe.propulsion.motor.prop}")
    if problems:
        print()
        print("Layout problems")
        for problem in problems:
            print(f"  - {problem}")
    print()
    print("Aero (AeroBuildup)")
    print(f"  Operating point  : AoA {aero.aoa_deg:.1f} deg  V {aero.airspeed_m_s:.1f} m/s  alt {aero.altitude_m:.0f} m  q {aero.dynamic_pressure_pa:.1f} Pa")
    print(f"  Lift             : {aero.lift_n:.3f} N  {aero.lift_n * GRAMS_FORCE_PER_NEWTON:.0f} g  {aero.lift_n / weight_n * 100.0:.1f} % weight")
    print(f"  Drag             : {aero.drag_n:.3f} N")
    print(f"  CL / CD / Cm     : {aero.cl:.4f} / {aero.cd:.4f} / {aero.cm:.4f}")
    for force in aero.surfaces:
        x_text = f"{force.x_lift_m:.4f}" if math.isfinite(force.x_lift_m) else "n/a"
        print(f"    {force.name:<16} lift {force.lift_n:7.3f} N  drag {force.drag_n:6.3f} N  CL {force.cl:6.3f}  x_lift {x_text}")
    print(f"    {'body + pods':<16} lift {aero.body_lift_n:7.3f} N  drag {aero.body_drag_n:6.3f} N")
    print()
    print("Static stability")
    margins = [(label, aero.x_np_m - cg_x) for label, cg_x in (("nominal CG", resolved.cg_m[0]), ("aft CG", resolved.cg_x_range_m[1]), ("forward CG", resolved.cg_x_range_m[0]))]
    print(f"  Neutral point    : {aero.x_np_m:.4f} m")
    for label, margin in margins[: 1 if resolved.cg_x_range_m[0] == resolved.cg_x_range_m[1] else 3]:
        print(f"  Margin, {label:<9}: {margin * 1000:6.1f} mm  {margin / reference.chord_m * 100.0:5.1f} % c_ref  {stability_verdict(margin, reference.chord_m)}")
    print(f"  Barrowman CP     : {aero.barrowman_cp_m:.4f} m (rocket-style cross-check)")
    if args.sweep:
        print()
        print(f"  {'AoA':>5} {'CL':>8} {'CD':>8} {'Cm':>8} {'Lift N':>8} {'Weight %':>9} {'NP m':>8}  Pitch")
        for row in aero.sweep:
            print(f"  {row.aoa_deg:5.0f} {row.cl:8.4f} {row.cd:8.4f} {row.cm:8.4f} {row.lift_n:8.3f} {row.lift_n / weight_n * 100.0:9.1f} {row.x_np_m:8.4f}  {row.pitch}")


if __name__ == "__main__":
    main()
