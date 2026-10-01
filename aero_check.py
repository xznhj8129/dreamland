"""
Usage:
python aero_check.py
python aero_check.py aircraft/m62v2.json aircraft/quadlancet62.json --aoa 0 2 5 --airspeed 20

Cross-checks the fast configuration aero (libs/aero.py, AeroBuildup) per lifting-surface set against the
AeroSandbox lifting line with body-rooted panels carried to the axis (the same wing treatment as the detailed
model: finite-wing physics, viscous low-Re camber, stall, lift carried across the body). That is the reference.

VLM is shown for information: it ignores the body (exposed panels read low) and is inviscid (under-reads camber
lift at these Reynolds numbers); "VLM thru" carries single horizontal pairs through the body.

Exit code 1 if any surface differs from the lifting-line reference by more than --tolerance.
"""

import argparse
import copy
import dataclasses
import json
from pathlib import Path

import aerosandbox as asb
import numpy as np

from libs.aero import AERO_MODEL_SIZE, LIFTING_LINE_CORE_RADIUS_M, LIFTING_LINE_SPANWISE_RESOLUTION, analyze_aero, build_asb_airplane, carried_to_axis
from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe

DEFAULT_AIRFRAMES = (Path("aircraft/m62v2.json"), Path("aircraft/quadlancet62.json"))
VLM_SPANWISE_RESOLUTION = 12
VLM_CHORDWISE_RESOLUTION = 8


def vlm_lift_n(airplane: asb.Airplane, aoa_deg: float, airspeed_m_s: float) -> float:
    vlm = asb.VortexLatticeMethod(
        airplane=airplane,
        op_point=asb.OperatingPoint(velocity=airspeed_m_s, alpha=aoa_deg),
        spanwise_resolution=VLM_SPANWISE_RESOLUTION,
        chordwise_resolution=VLM_CHORDWISE_RESOLUTION,
    )
    return float(vlm.run()["L"])


def lifting_line_lift_n(airplane: asb.Airplane, aoa_deg: float, airspeed_m_s: float) -> float:
    return float(asb.LiftingLine(
        airplane=airplane, op_point=asb.OperatingPoint(velocity=airspeed_m_s, alpha=aoa_deg), model_size=AERO_MODEL_SIZE,
        spanwise_resolution=LIFTING_LINE_SPANWISE_RESOLUTION, vortex_core_radius=LIFTING_LINE_CORE_RADIUS_M,
    ).run()["L"])


def carried_through(wing: asb.Wing) -> asb.Wing:
    """Copy of a symmetric pair wing with an extra root section on the centerline, along the panel direction."""
    root, tip = wing.xsecs[0], wing.xsecs[-1]
    dir_y = tip.xyz_le[1] - root.xyz_le[1]
    dir_z = tip.xyz_le[2] - root.xyz_le[2]
    k = root.xyz_le[1] / dir_y
    inner = copy.deepcopy(root)
    inner.xyz_le = np.array([root.xyz_le[0], 0.0, root.xyz_le[2] - k * dir_z])
    return asb.Wing(name=wing.name, symmetric=True, xsecs=[inner, *wing.xsecs])


def layout_label(surface_set) -> str:
    return "".join(sorted({"H" if abs(panel.span_dir[2]) < 1e-9 else "V" if abs(panel.span_dir[1]) < 1e-9 else "X" for panel in surface_set.panels}))


def main() -> None:
    parser = argparse.ArgumentParser(description="Cross-check AeroBuildup surface lift against VLM.")
    parser.add_argument("airframes", type=Path, nargs="*", default=list(DEFAULT_AIRFRAMES), help="airframe/2 files")
    parser.add_argument("--aoa", type=float, nargs="+", default=[2.0, 5.0], help="AoA values in degrees.")
    parser.add_argument("--airspeed", type=float, default=20.0, help="Airspeed in m/s.")
    parser.add_argument("--tolerance", type=float, default=0.15, help="Allowed relative lift difference for carried-through pairs.")
    args = parser.parse_args()

    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    failures = []
    header = f"{'airframe':<14} {'AoA':>4}  {'surface':<18} {'layout':<6} {'AeroBuildup':>11} {'LL ref':>8} {'VLM thru':>9} {'VLM exposed':>11}  status"
    print(header)
    print("-" * len(header))
    for path in args.airframes:
        airframe = load_airframe(path, parts_data)
        for aoa_deg in args.aoa:
            operating_point = dataclasses.replace(airframe.operating_point, aoa_deg=aoa_deg, airspeed_m_s=args.airspeed, altitude_m=0.0)
            resolved, _problems = resolve_airframe(dataclasses.replace(airframe, operating_point=operating_point), parts_data)
            aero = analyze_aero(resolved)
            for surface_set, force in zip(resolved.surfaces, aero.surfaces):
                airplane = build_asb_airplane(dataclasses.replace(resolved, surfaces=(surface_set,), motors=()))
                airplane.fuselages = []
                exposed_n = vlm_lift_n(airplane, aoa_deg, args.airspeed)
                body_rooted = surface_set.definition.root == "body"
                reference_plane = copy.deepcopy(airplane)
                reference_plane.wings = [carried_to_axis(wing) if body_rooted else wing for wing in airplane.wings]
                reference_n = lifting_line_lift_n(reference_plane, aoa_deg, args.airspeed)
                through_text = f"{'n/a':>9}"
                if len(surface_set.panels) == 2 and surface_set.panels[0].xz_mirror == "primary" and body_rooted:
                    through_plane = copy.deepcopy(airplane)
                    through_plane.wings = [carried_through(wing) for wing in airplane.wings]
                    through_text = f"{vlm_lift_n(through_plane, aoa_deg, args.airspeed):9.3f}"
                if abs(reference_n) < 0.05 * abs(force.lift_n) + 0.02:
                    status = "ok (near zero lift)" if abs(force.lift_n - reference_n) < 0.05 else "check (near zero lift)"
                else:
                    error = abs(force.lift_n - reference_n) / abs(reference_n)
                    status = f"{'FAIL' if error > args.tolerance else 'ok'} ({error * 100.0:.0f} %)"
                    if error > args.tolerance:
                        failures.append((path.stem, aoa_deg, surface_set.name))
                print(f"{path.stem:<14} {aoa_deg:4.1f}  {surface_set.name:<18} {layout_label(surface_set):<6} {force.lift_n:11.3f} {reference_n:8.3f} {through_text} {exposed_n:11.3f}  {status}")
            print(
                f"{path.stem:<14} {aoa_deg:4.1f}  {'(aircraft)':<18} {'':<6} {aero.lift_n:11.3f} {'':>9} {'':>11}  "
                f"x_np={aero.x_np_m:.4f} m  cg={resolved.cg_m[0]:.4f} m"
            )

    if failures:
        print(f"\n{len(failures)} surface(s) outside {args.tolerance * 100.0:.0f} % of the lifting-line reference: {failures}")
        raise SystemExit(1)
    print("\nAll surfaces within tolerance of the lifting-line reference.")


if __name__ == "__main__":
    main()
