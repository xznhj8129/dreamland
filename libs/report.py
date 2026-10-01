"""
Usage:
from libs.report import build_report_text, build_detailed_report_text
text = build_report_text(resolved, aero, problems, sweep=True)
text = build_detailed_report_text(resolved, aero, detailed)

Plain-text reports of a resolved airframe and its aero results, shared by analyze_airframe.py and the GUI.
"""

import math

from libs.aero import AeroResult, DetailedAeroResult, stability_verdict
from libs.geometry import GRAMS_FORCE_PER_NEWTON, STANDARD_GRAVITY_M_S2, ResolvedAirframe


def build_report_text(resolved: ResolvedAirframe, aero: AeroResult, problems: list[str], sweep: bool) -> str:
    airframe = resolved.airframe
    lines = []
    reference = resolved.reference
    mass_low, mass_high = resolved.mass_kg
    weight_n = (mass_low + mass_high) / 2.0 * STANDARD_GRAVITY_M_S2

    lines.append(f"Airframe: {airframe.name}")
    lines.append("")
    lines.append("Mass")
    lines.append(f"  Total            : {mass_low * 1000:.1f} g" if mass_low == mass_high else f"  Total            : {mass_low * 1000:.1f} - {mass_high * 1000:.1f} g")
    lines.append(f"  CG               : x {resolved.cg_m[0]:.4f} m  y {resolved.cg_m[1]:.4f}  z {resolved.cg_m[2]:.4f}")
    if resolved.cg_x_range_m[0] != resolved.cg_x_range_m[1]:
        lines.append(f"  CG x range       : {resolved.cg_x_range_m[0]:.4f} - {resolved.cg_x_range_m[1]:.4f} m (mass ranges)")
    for point in resolved.mass_points:
        mass_text = f"{point.mass_kg[0] * 1000:7.1f}" if point.mass_kg[0] == point.mass_kg[1] else f"{point.mass_kg[0] * 1000:.1f}-{point.mass_kg[1] * 1000:.1f}"
        lines.append(f"    {point.name:<34} {mass_text:>12} g  x {point.position[0]:.4f}{'  (computed)' if point.computed else ''}")
    lines.append("")
    lines.append("Geometry")
    lines.append(f"  Fuselage         : {airframe.fuselage.length_m:.3f} m, section {airframe.fuselage.section.width_m * 1000:.1f} x {airframe.fuselage.section.height_m * 1000:.1f} mm")
    lines.append(f"  Reference        : {reference.basis}  S {reference.area_m2:.4f} m^2  c {reference.chord_m:.4f} m  b {reference.span_m:.3f} m")
    for surface_set in resolved.surfaces:
        lines.append(f"  {surface_set.name:<16} : {surface_set.definition.layout} x{len(surface_set.panels)}, area {surface_set.planform_area_m2:.4f} m^2, MAC {surface_set.mac_m:.4f} m")
    lines.append(f"  Motors           : {len(resolved.motors)} x {airframe.propulsion.motor.brand} {airframe.propulsion.motor.model} {airframe.propulsion.motor.kv}KV, {airframe.propulsion.motor.prop}")
    if problems:
        lines.append("")
        lines.append("Layout problems")
        for problem in problems:
            lines.append(f"  - {problem}")
    lines.append("")
    lines.append("Aero (AeroBuildup)")
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


def build_detailed_report_text(resolved: ResolvedAirframe, aero: AeroResult, detailed: DetailedAeroResult) -> str:
    """Detailed (hybrid) aero next to the fast model at the same operating point."""
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    chord = resolved.reference.chord_m
    lines = [
        "Detailed aero (lifting-line wings + AeroBuildup bodies)",
        "  Includes downwash between surfaces, lift carried across the body, viscous camber and stall.",
        f"  Operating point  : AoA {detailed.aoa_deg:.1f} deg  V {detailed.airspeed_m_s:.1f} m/s  alt {detailed.altitude_m:.0f} m",
        f"  {'':<18} {'fast (AeroBuildup)':>20} {'detailed (hybrid)':>20}",
    ]
    rows = (
        ("Lift N", aero.lift_n, detailed.lift_n, ".3f"),
        ("Lift % weight", aero.lift_n / weight_n * 100.0, detailed.lift_n / weight_n * 100.0, ".1f"),
        ("Drag N", aero.drag_n, detailed.drag_n, ".3f"),
        ("CL", aero.cl, detailed.cl, ".4f"),
        ("CD", aero.cd, detailed.cd, ".4f"),
        ("Cm", aero.cm, detailed.cm, ".4f"),
        ("Neutral point m", aero.x_np_m, detailed.x_np_m, ".4f"),
        ("Margin mm", (aero.x_np_m - resolved.cg_m[0]) * 1000.0, (detailed.x_np_m - resolved.cg_m[0]) * 1000.0, ".1f"),
        ("Margin % c_ref", (aero.x_np_m - resolved.cg_m[0]) / chord * 100.0, (detailed.x_np_m - resolved.cg_m[0]) / chord * 100.0, ".1f"),
    )
    for label, fast_value, detailed_value, spec in rows:
        lines.append(f"  {label:<18} {format(fast_value, spec):>20} {format(detailed_value, spec):>20}")
    lines.append(f"  {'Verdict':<18} {stability_verdict(aero.x_np_m - resolved.cg_m[0], chord):>20} {stability_verdict(detailed.x_np_m - resolved.cg_m[0], chord):>20}")
    lines.append(f"  Body + pods lift (detailed): {detailed.body_lift_n:.3f} N")
    lines.append("")
    lines.append(f"  {'AoA':>5} {'CL':>8} {'CD':>8} {'Cm':>8} {'Lift N':>8} {'Weight %':>9} {'NP m':>8}  Pitch")
    for row in detailed.sweep:
        lines.append(f"  {row.aoa_deg:5.0f} {row.cl:8.4f} {row.cd:8.4f} {row.cm:8.4f} {row.lift_n:8.3f} {row.lift_n / weight_n * 100.0:9.1f} {row.x_np_m:8.4f}  {row.pitch}")
    return "\n".join(lines)

