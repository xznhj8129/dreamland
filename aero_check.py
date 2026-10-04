"""
Usage:
python aero_check.py
python aero_check.py aircraft/m62v1 --aoa 0 5 10 --airspeed 20

Checks the live aero's lifting line (libs/aero.py lifting_line_sweep, every AoA in one batch) against
asb.LiftingLine solved per AoA on the same wings: they must agree to --tolerance (N, N m). Airframes with control
surfaces are also checked with every control deflected CHECK_DEFLECTION_DEG, symmetrically and anti-symmetrically
across XZ-mirror pairs. For each XZ-mirror pair with controls, AeroBuildup with each side deflected alone
(SideControlSurface, as the JSBSim control tables) must add up to both sides deflected (but for the aircraft-level
induced drag, see side_split_differences), and SideControlSurface with opposite sides must equal ASB's own
anti-symmetric control.

Then prints, per surface set and for information:
- live: its lift in the live model (downwash from the other sets, lift carried across the body)
- alone: the same set alone in the lifting line (no other sets, so no downwash from them)
- VLM exposed: AeroSandbox VLM on the exposed panels (inviscid, no body: body-mounted panels read low)
- VLM thru: single horizontal pairs carried through the body in VLM

Exit code 1 if the batched lifting line and asb.LiftingLine, or the side split, differ by more than --tolerance.
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
    build_lifting_line_mesh, lifting_line_sweep, with_side_deflections,
)
from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe
from libs.project import PROJECTS_DIR, open_project

DEFAULT_PROJECTS = tuple(PROJECTS_DIR / name for name in ("m62v1", "m62v2", "quadlancer62", "simpleplane"))
CHECK_DEFLECTION_DEG = 10.0
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


def lifting_line_difference(wing_plane: asb.Airplane, batched, column: int, aoa_deg: float, atmosphere, airspeed_m_s: float) -> tuple[float, float]:
    """(batched total lift, largest difference in lift, drag and pitch moment) against asb.LiftingLine."""
    reference = asb.LiftingLine(
        airplane=wing_plane, op_point=asb.OperatingPoint(atmosphere=atmosphere, velocity=airspeed_m_s, alpha=aoa_deg),
        model_size=AERO_MODEL_SIZE, spanwise_resolution=LIFTING_LINE_SPANWISE_RESOLUTION, vortex_core_radius=LIFTING_LINE_CORE_RADIUS_M,
    ).run()
    totals = (batched.lift_n[:, column].sum(), batched.drag_n[:, column].sum(), batched.pitch_moment_nm[:, column].sum())
    expected = (float(reference["L"]), float(reference["D"]), float(reference["M_b"][1]))
    return totals[0], max(abs(a - b) for a, b in zip(totals, expected))


def side_split_differences(airplane: asb.Airplane, wing: int, aoa_values: np.ndarray, atmosphere, airspeed_m_s: float) -> tuple[float, float]:
    """AeroBuildup body forces and moments (N, N m): largest difference of (own side alone + mirror side alone - neutral)
    against both sides deflected by ASB, and of SideControlSurface +/- against ASB's anti-symmetric control. The sums
    leave out AeroBuildup's aircraft-level induced drag, from the total lift squared: the one load where controls
    interact (2 x dL1 x dL2, a force only), which the JSBSim tables' summed increments do not carry."""
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=airspeed_m_s, alpha=aoa_values)

    def loads(plane: asb.Airplane, induced: bool = True) -> np.ndarray:
        aero = asb.AeroBuildup(airplane=plane, op_point=op_point, model_size=AERO_MODEL_SIZE).run()
        forces = np.array(aero["F_b"], dtype=float)
        if not induced:
            forces -= np.array(op_point.convert_axes(-aero["D_induced"], 0.0, 0.0, from_axes="wind", to_axes="body"), dtype=float)
        return np.array([*forces, *aero["M_b"]], dtype=float)

    def asb_deflected(symmetric: bool) -> asb.Airplane:
        plane = copy.deepcopy(airplane)
        for xsec in plane.wings[wing].xsecs:
            for surface in xsec.control_surfaces:
                surface.deflection, surface.symmetric = CHECK_DEFLECTION_DEG, symmetric
        return plane

    sides = sum(loads(with_side_deflections(airplane, wing, *deflections), induced=False) for deflections in ((CHECK_DEFLECTION_DEG, 0.0), (0.0, CHECK_DEFLECTION_DEG))) - loads(airplane, induced=False)
    opposite = loads(with_side_deflections(airplane, wing, CHECK_DEFLECTION_DEG, -CHECK_DEFLECTION_DEG))
    return float(np.max(np.abs(sides - loads(asb_deflected(True), induced=False)))), float(np.max(np.abs(opposite - loads(asb_deflected(False)))))


def layout_label(surface_set) -> str:
    return "".join(sorted({"H" if abs(panel.span_dir[2]) < 1e-9 else "V" if abs(panel.span_dir[1]) < 1e-9 else "X" for panel in surface_set.panels}))


def main() -> None:
    parser = argparse.ArgumentParser(description="Check the batched lifting line against asb.LiftingLine; per-surface lift for information.")
    parser.add_argument("projects", type=Path, nargs="*", default=list(DEFAULT_PROJECTS), help="aircraft project directories")
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
    for project in map(open_project, args.projects):
        airframe = load_airframe(project.build_path, parts_data)
        operating_point = dataclasses.replace(airframe.operating_point, aoa_deg=float(aoa_values[0]), airspeed_m_s=args.airspeed, altitude_m=0.0)
        resolved, _problems = resolve_airframe(dataclasses.replace(airframe, operating_point=operating_point), parts_data)
        hybrid = build_hybrid_airplanes(resolved)
        wing_plane = hybrid.wings
        surface_count = len(resolved.surfaces)
        mesh = build_lifting_line_mesh(wing_plane, hybrid.owners, hybrid.controls)
        live = lifting_line_sweep(mesh, wing_plane.xyz_ref, surface_count, atmosphere, args.airspeed, aoa_values)
        alone = []
        for index in range(surface_count):
            members = [(wing, controls) for wing, owner, controls in zip(wing_plane.wings, hybrid.owners, hybrid.controls) if owner == index]
            alone_plane = asb.Airplane(wings=[wing for wing, _ in members], xyz_ref=wing_plane.xyz_ref, s_ref=wing_plane.s_ref, c_ref=wing_plane.c_ref, b_ref=wing_plane.b_ref)
            alone_mesh = build_lifting_line_mesh(alone_plane, (0,) * len(members), tuple(controls for _, controls in members))
            alone.append(lifting_line_sweep(alone_mesh, wing_plane.xyz_ref, 1, atmosphere, args.airspeed, aoa_values))
        deflected_cases = []
        for symmetric in ((True, False) if resolved.controls else ()):
            deflected_plane = copy.deepcopy(wing_plane)
            for wing in deflected_plane.wings:
                for xsec in wing.xsecs:
                    for surface in xsec.control_surfaces:
                        surface.deflection, surface.symmetric = CHECK_DEFLECTION_DEG, symmetric
            deflections = {}
            for own, mirror in hybrid.controls:
                deflections[own] = CHECK_DEFLECTION_DEG
                deflections[mirror] = CHECK_DEFLECTION_DEG if symmetric else -CHECK_DEFLECTION_DEG
            deflections.pop(0, None)
            label = f"(controls {'+' if symmetric else '+/-'}{CHECK_DEFLECTION_DEG:g})"
            deflected_cases.append((label, deflected_plane, lifting_line_sweep(mesh, wing_plane.xyz_ref, surface_count, atmosphere, args.airspeed, aoa_values, deflections)))
        asb_airplane = build_asb_airplane(resolved)
        for wing_index, (own, mirror) in enumerate(hybrid.controls):
            if own and mirror:
                for label, difference in zip(("sides add up", "+/- = anti-sym"), side_split_differences(asb_airplane, wing_index, aoa_values, atmosphere, args.airspeed)):
                    status = "ok" if difference <= args.tolerance else "FAIL"
                    if status == "FAIL":
                        failures.append((project.name, f"C{own}/C{mirror} {label}", None, difference))
                    print(f"{project.name:<14} {'all':>5}  C{own}/C{mirror} {label:<16} AeroBuildup max diff {difference:.1e}  {status}")
        for column, aoa_deg in enumerate(aoa_values):
            for label, plane, batched in (("(all wings)", wing_plane, live), *deflected_cases):
                total, difference = lifting_line_difference(plane, batched, column, float(aoa_deg), atmosphere, args.airspeed)
                status = "ok" if difference <= args.tolerance else "FAIL"
                if status == "FAIL":
                    failures.append((project.name, label, float(aoa_deg), difference))
                print(f"{project.name:<14} {aoa_deg:5.1f}  {label:<18} {'':<6} {total:8.3f}  max diff to asb.LiftingLine {difference:.1e}  {status}")
            for index, surface_set in enumerate(resolved.surfaces):
                exposed_plane = build_asb_airplane(dataclasses.replace(resolved, surfaces=(surface_set,), motors=()))
                exposed_plane.fuselages = []
                through_text = f"{'n/a':>9}"
                if len(surface_set.panels) == 2 and surface_set.panels[0].xz_mirror == "primary" and surface_set.definition.root == "body":
                    through_plane = copy.deepcopy(exposed_plane)
                    through_plane.wings = [carried_through(wing) for wing in exposed_plane.wings]
                    through_text = f"{vlm_lift_n(through_plane, float(aoa_deg), args.airspeed):9.3f}"
                print(
                    f"{project.name:<14} {aoa_deg:5.1f}  {surface_set.name:<18} {layout_label(surface_set):<6} "
                    f"{live.lift_n[index, column]:8.3f} {alone[index].lift_n[0, column]:8.3f} {through_text} "
                    f"{vlm_lift_n(exposed_plane, float(aoa_deg), args.airspeed):11.3f}"
                )

    if failures:
        print(f"\n{len(failures)} case(s) over {args.tolerance:g}: {failures}")
        raise SystemExit(1)
    print(f"\nBatched lifting line equals asb.LiftingLine, and the AeroBuildup side split adds up, within {args.tolerance:g} for every airframe and AoA.")


if __name__ == "__main__":
    main()
