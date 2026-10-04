"""
Usage:
from libs.report import build_report_text
text = build_report_text(resolved, aero, problems, parts_data, sweep=True)

Plain-text reports of a resolved airframe and its aero results, shared by analyze_airframe.py and the GUI.
"""

import math

from libs.aero import AeroResult, stability_verdict
from libs.geometry import GRAMS_FORCE_PER_NEWTON, STANDARD_GRAVITY_M_S2, ResolvedAirframe
from libs.propulsion import propulsion_summary, summary_lines


def build_report_text(resolved: ResolvedAirframe, aero: AeroResult, problems: list[str], parts_data: dict, sweep: bool) -> str:
    airframe = resolved.airframe
    lines = []
    reference = resolved.reference
    mass_low, mass_high = resolved.mass_kg
    weight_n = (mass_low + mass_high) / 2.0 * STANDARD_GRAVITY_M_S2

    lines.append(f"Airframe: {airframe.name}")
    kind = "tailsitter VTOL" if airframe.tailsitter else "VTOL" if airframe.vtol else "fixed-wing" if airframe.powered else "glider (unpowered)"
    lines.append(f"Type: {kind}")
    lines.append("")
    lines.append("Mass")
    lines.append(f"  Total            : {mass_low * 1000:.1f} g" if mass_low == mass_high else f"  Total            : {mass_low * 1000:.1f} - {mass_high * 1000:.1f} g")
    lines.append(f"  CG               : x {resolved.cg_m[0]:.4f} m  y {resolved.cg_m[1]:.4f}  z {resolved.cg_m[2]:.4f}")
    if resolved.cg_x_range_m[0] != resolved.cg_x_range_m[1]:
        lines.append(f"  CG x range       : {resolved.cg_x_range_m[0]:.4f} - {resolved.cg_x_range_m[1]:.4f} m (mass ranges)")
    inertia = resolved.inertia_kg_m2
    lines.append(f"  Inertia at CG    : Ixx {inertia[0]:.5f}  Iyy {inertia[1]:.5f}  Izz {inertia[2]:.5f} kg m^2 (x aft, y right, z up)")
    lines.append(f"                     Ixy {inertia[3]:.6f}  Ixz {inertia[4]:.6f}  Iyz {inertia[5]:.6f} (tensor entries, -sum m a b)")
    for point in resolved.mass_points:
        mass_text = f"{point.mass_kg[0] * 1000:7.1f}" if point.mass_kg[0] == point.mass_kg[1] else f"{point.mass_kg[0] * 1000:.1f}-{point.mass_kg[1] * 1000:.1f}"
        note = "  (estimated)" if point.computed else f"  (estimate {point.estimate_kg * 1000:.1f} g)" if point.estimate_kg is not None else ""
        lines.append(f"    {point.name:<34} {mass_text:>12} g  x {point.position[0]:.4f}{note}")
    estimated = [point for point in resolved.mass_points if point.estimate_kg is not None]
    if any(not point.computed for point in estimated):
        given_g = sum(point.mass_kg[0] for point in estimated) * 1000.0
        estimate_g = sum(point.estimate_kg for point in estimated) * 1000.0
        lines.append(f"  Printed parts    : {given_g:.1f} g as given, {estimate_g:.1f} g estimated (hollow sections: shell wall x material density;")
        lines.append("                     solid sections: less their volumes, and surfaces: solid airfoil volume, x print.bulk_density_kg_m3)")
    lines.append("  Sections         : empty as used (estimate), complete = everything attached in it, itemised or weighed")
    for index, section in enumerate(resolved.sections):
        estimate = "-" if section.empty_estimate_kg is None else f"{section.empty_estimate_kg * 1000:.1f}"
        low, high = section.complete_itemised_kg
        itemised = f"{low * 1000:.1f}" if low == high else f"{low * 1000:.1f}-{high * 1000:.1f}"
        complete = f"itemised {itemised} g" if section.complete_kg is None else f"weighed {section.complete_kg * 1000:.1f} g (itemised {itemised} g)"
        lines.append(f"    S{index + 1} {section.x0_m * 1000:4.0f}-{section.x1_m * 1000:4.0f} mm {'solid ' if section.solid else 'hollow'}  "
                     f"empty {section.empty_kg * 1000:6.1f} g ({estimate} g)  complete {complete}")
    lines.append("")
    lines.append("Geometry")
    preset = "" if airframe.preset is None else f", preset {airframe.preset.name}"
    lines.append(f"  Fuselage         : {airframe.fuselage.length_m:.3f} m, section {airframe.fuselage.section.width_m * 1000:.1f} x {airframe.fuselage.section.height_m * 1000:.1f} mm{preset}")
    reinforcement = airframe.fuselage.reinforcement
    if reinforcement is not None:
        pattern = reinforcement.pattern
        lines.append(f"  Rods             : {pattern.polar_n} x {pattern.diameter_m * 1000:.1f} mm {pattern.material} at r {pattern.radial_position_m * 1000:.2f} mm, "
                     f"{reinforcement.from_x_m * 1000:.0f}-{reinforcement.to_x_m * 1000:.0f} mm, angles {', '.join(f'{angle:g}' for angle in pattern.angles_deg())} deg from the horizontal")
    lines.append(f"  Reference        : {reference.basis}  S {reference.area_m2:.4f} m^2  c {reference.chord_m:.4f} m  b {reference.span_m:.3f} m")
    for surface_set in resolved.surfaces:
        lines.append(f"  {surface_set.name:<16} : {surface_set.definition.layout} x{len(surface_set.panels)}, area {surface_set.planform_area_m2:.4f} m^2, MAC {surface_set.mac_m:.4f} m")
    if airframe.propulsion is not None:
        motor = airframe.propulsion.motor
        lines.append(f"  Motors           : {len(resolved.motors)} x {motor.manufacturer} {motor.name}, {airframe.propulsion.prop} (motorlib)")
        lines.append("  Prop spin        : seen from where the thrust points: " + ", ".join(f"{placement.name} {placement.spin}" for placement in resolved.motors))
        lines.append("")
        lines.append("Propulsion (static, motorlib; throttle = INAV motor output)")
        lines.extend(f"  {line}" for line in summary_lines(propulsion_summary(resolved, parts_data)))
    if problems:
        lines.append("")
        lines.append("Layout problems")
        for problem in problems:
            lines.append(f"  - {problem}")
    lines.append("")
    lines.append("Aero (lifting-line wings + AeroBuildup bodies)")
    lines.append(f"  Operating point  : AoA {aero.aoa_deg:.1f} deg  V {aero.airspeed_m_s:.1f} m/s  alt {aero.altitude_m:.0f} m  q {aero.dynamic_pressure_pa:.1f} Pa")
    lines.append(f"  Lift             : {aero.lift_n:.3f} N  {aero.lift_n * GRAMS_FORCE_PER_NEWTON:.0f} g  {aero.lift_n / weight_n * 100.0:.1f} % weight")
    lines.append(f"  Drag             : {aero.drag_n:.3f} N")
    lines.append(f"  CL / CD / Cm     : {aero.cl:.4f} / {aero.cd:.4f} / {aero.cm:.4f}")
    for force in aero.surfaces:
        x_text = f"{force.x_lift_m:.4f}" if math.isfinite(force.x_lift_m) else "n/a"
        lines.append(f"    {force.name:<16} lift {force.lift_n:7.3f} N  drag {force.drag_n:6.3f} N  CL {force.cl:6.3f}  x_lift {x_text}")
    lines.append(f"    {'body + pods':<16} lift {aero.body_lift_n:7.3f} N  drag {aero.body_drag_n:6.3f} N")
    lines.append("")
    lines.append("Static stability")
    margins = [(label, aero.x_np_m - cg_x) for label, cg_x in (("nominal CG", resolved.cg_m[0]), ("aft CG", resolved.cg_x_range_m[1]), ("forward CG", resolved.cg_x_range_m[0]))]
    lines.append(f"  Neutral point    : {aero.x_np_m:.4f} m")
    for label, margin in margins[: 1 if resolved.cg_x_range_m[0] == resolved.cg_x_range_m[1] else 3]:
        lines.append(f"  Margin, {label:<9}: {margin * 1000:6.1f} mm  {margin / reference.chord_m * 100.0:5.1f} % c_ref  {stability_verdict(margin, reference.chord_m)}")
    lines.append(f"  Barrowman CP     : {aero.barrowman_cp_m:.4f} m (rocket-style cross-check)")
    if sweep:
        lines.append("")
        lines.append(f"  {'AoA':>5} {'CL':>8} {'CD':>8} {'Cm':>8} {'Lift N':>8} {'Weight %':>9} {'NP m':>8}  Pitch")
        for row in aero.sweep:
            lines.append(f"  {row.aoa_deg:5.0f} {row.cl:8.4f} {row.cd:8.4f} {row.cm:8.4f} {row.lift_n:8.3f} {row.lift_n / weight_n * 100.0:9.1f} {row.x_np_m:8.4f}  {row.pitch}")
    return "\n".join(lines)
