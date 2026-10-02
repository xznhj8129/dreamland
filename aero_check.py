"""
Usage:
python aero_check.py
python aero_check.py aircraft/m62v1.json --aoa 0 5 10 --airspeed 20

Checks the live aero's lifting line (libs/aero.py lifting_line_sweep, every AoA in one batch) against
asb.LiftingLine solved per AoA on the same wings: they must agree to --tolerance (N, N m).

Then prints, per surface set and for information:
- live: its lift in the live model (downwash from the other sets, lift carried across the body)
- alone: the same set alone in the lifting line (no other sets, so no downwash from them)
- VLM exposed: AeroSandbox VLM on the exposed panels (inviscid, no body: body-mounted panels read low)
- VLM thru: single horizontal pairs carried through the body in VLM

Exit code 1 if the batched lifting line and asb.LiftingLine differ by more than --tolerance.
"""

import argparse
import copy
import dataclasses
import json
from pathlib import Path

import aerosandbox as asb
import numpy as np

from libs.aero import (
    AERO_MODEL_SIZE, LIFTING_LINE_CORE_RADIUS_M, LIFTING_LINE_SPANWISE_RESOLUTION, build_asb_airplane, build_hybrid_airplanes,
    build_lifting_line_mesh, lifting_line_sweep,
)
from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe

DEFAULT_AIRFRAMES = (Path("aircraft/m62v1.json"), Path("aircraft/m62v2.json"), Path("aircraft/quadlancer62.json"))
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
    parser = argparse.ArgumentParser(description="Check the batched lifting line against asb.LiftingLine; per-surface lift for information.")
    parser.add_argument("airframes", type=Path, nargs="*", default=list(DEFAULT_AIRFRAMES), help="airframe/2 files")
    parser.add_argument("--aoa", type=float, nargs="+", default=[0.0, 5.0, 10.0], help="AoA values in degrees")
    parser.add_argument("--airspeed", type=float, default=20.0, help="Airspeed in m/s")
    parser.add_argument("--tolerance", type=float, default=1e-9, help="Allowed difference in lift, drag (N) and pitch moment (N m)")
    args = parser.parse_args()

    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    atmosphere = asb.Atmosphere(altitude=0.0)
    aoa_values = np.array(args.aoa, dtype=float)
    failures = []
    header = f"{'airframe':<14} {'AoA':>5}  {'surface':<18} {'layout':<6} {'live N':>8} {'alone N':>8} {'VLM thru':>9} {'VLM exposed':>11}"
    print(header)
    print("-" * len(header))
    for path in args.airframes:
        airframe = load_airframe(path, parts_data)
        operating_point = dataclasses.replace(airframe.operating_point, aoa_deg=float(aoa_values[0]), airspeed_m_s=args.airspeed, altitude_m=0.0)
        resolved, _problems = resolve_airframe(dataclasses.replace(airframe, operating_point=operating_point), parts_data)
        wing_plane, _body_plane, owners = build_hybrid_airplanes(resolved)
        surface_count = len(resolved.surfaces)
        live = lifting_line_sweep(build_lifting_line_mesh(wing_plane, owners), wing_plane.xyz_ref, surface_count, atmosphere, args.airspeed, aoa_values)
        alone = []
        for index in range(surface_count):
            wings = [wing for wing, owner in zip(wing_plane.wings, owners) if owner == index]
            alone_plane = asb.Airplane(wings=wings, xyz_ref=wing_plane.xyz_ref, s_ref=wing_plane.s_ref, c_ref=wing_plane.c_ref, b_ref=wing_plane.b_ref)
            alone.append(lifting_line_sweep(build_lifting_line_mesh(alone_plane, (0,) * len(wings)), wing_plane.xyz_ref, 1, atmosphere, args.airspeed, aoa_values))
        for column, aoa_deg in enumerate(aoa_values):
            reference = asb.LiftingLine(
                airplane=wing_plane, op_point=asb.OperatingPoint(atmosphere=atmosphere, velocity=args.airspeed, alpha=float(aoa_deg)),
                model_size=AERO_MODEL_SIZE, spanwise_resolution=LIFTING_LINE_SPANWISE_RESOLUTION, vortex_core_radius=LIFTING_LINE_CORE_RADIUS_M,
            ).run()
            batched = (live.lift_n[:, column].sum(), live.drag_n[:, column].sum(), live.pitch_moment_nm[:, column].sum())
            expected = (float(reference["L"]), float(reference["D"]), float(reference["M_b"][1]))
            difference = max(abs(a - b) for a, b in zip(batched, expected))
            status = "ok" if difference <= args.tolerance else "FAIL"
            if status == "FAIL":
                failures.append((path.stem, float(aoa_deg), difference))
            print(
                f"{path.stem:<14} {aoa_deg:5.1f}  {'(all wings)':<18} {'':<6} {batched[0]:8.3f}  "
                f"asb.LiftingLine L {expected[0]:.3f} N  max diff {difference:.1e}  {status}"
            )
            for index, surface_set in enumerate(resolved.surfaces):
                exposed_plane = build_asb_airplane(dataclasses.replace(resolved, surfaces=(surface_set,), motors=()))
                exposed_plane.fuselages = []
                through_text = f"{'n/a':>9}"
                if len(surface_set.panels) == 2 and surface_set.panels[0].xz_mirror == "primary" and surface_set.definition.root == "body":
                    through_plane = copy.deepcopy(exposed_plane)
                    through_plane.wings = [carried_through(wing) for wing in exposed_plane.wings]
                    through_text = f"{vlm_lift_n(through_plane, float(aoa_deg), args.airspeed):9.3f}"
                print(
                    f"{path.stem:<14} {aoa_deg:5.1f}  {surface_set.name:<18} {layout_label(surface_set):<6} "
                    f"{live.lift_n[index, column]:8.3f} {alone[index].lift_n[0, column]:8.3f} {through_text} "
                    f"{vlm_lift_n(exposed_plane, float(aoa_deg), args.airspeed):11.3f}"
                )

    if failures:
        print(f"\n{len(failures)} case(s) where the batched lifting line differs from asb.LiftingLine by more than {args.tolerance:g}: {failures}")
        raise SystemExit(1)
    print(f"\nBatched lifting line equals asb.LiftingLine within {args.tolerance:g} for every airframe and AoA.")


if __name__ == "__main__":
    main()
