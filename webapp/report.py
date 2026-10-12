"""Structured editor reports from the same resolved results as the text report."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from libs.airframe import BATTERY_PART_NAME
from libs.aero import AeroResult, stability_verdict
from libs.geometry import (
    GRAMS_FORCE_PER_NEWTON,
    STANDARD_GRAVITY_M_S2,
    ResolvedAirframe,
)
from libs.propulsion import MotorPoints, PropulsionSummary, summary_lines
from libs.report import VERDICT_LEVELS
from libs.reportline import ReportLine


@dataclass(frozen=True)
class ReportTarget:
    kind: str
    index: int
    label: str


@dataclass(frozen=True)
class ReportNotice:
    text: str
    level: str
    target: ReportTarget | None


def problem_target(resolved: ResolvedAirframe, problem: str) -> ReportTarget | None:
    # Geometry checks name their affected components; map those exact diagnostic prefixes to editor selections.
    for index, volume in enumerate(resolved.airframe.volumes):
        if problem.startswith(
            (f"volume {volume.name!r} ", f"volumes {volume.name!r} and ")
        ):
            return ReportTarget("volume", index, volume.name)
    for index, part in enumerate(resolved.airframe.parts):
        if problem.startswith((f"part {part.name!r} ", f"parts {part.name!r} and ")):
            return (
                ReportTarget("battery", 0, part.name)
                if part.name == BATTERY_PART_NAME
                else ReportTarget("part", index, part.name)
            )
    if problem.startswith(("part 'battery' ", "parts 'battery' and ")):
        return ReportTarget("battery", 0, "battery")
    for index, surface in enumerate(resolved.surfaces):
        if problem.startswith(f"surface {surface.name!r} "):
            return ReportTarget("surface", index, surface.name)
    for index in range(len(resolved.sections)):
        if problem.startswith(f"section {index + 1}:"):
            return ReportTarget("section", index, f"section {index + 1}")
    return None


@dataclass(frozen=True)
class ReportMetric:
    label: str
    value: str
    level: str = "info"


@dataclass(frozen=True)
class ReportRow:
    values: list[str]
    level: str = "info"


@dataclass(frozen=True)
class ReportTable:
    columns: list[str]
    rows: list[ReportRow]


@dataclass(frozen=True)
class ReportSection:
    title: str
    metrics: list[ReportMetric] = field(default_factory=list)
    notes: list[ReportLine | ReportNotice] = field(default_factory=list)
    table: ReportTable | None = None
    collapsible: bool = False
    children: list[ReportSection] = field(default_factory=list)


@dataclass(frozen=True)
class ReportPages:
    summary: list[ReportSection]
    mass: list[ReportSection]
    aero: list[ReportSection]
    propulsion: list[ReportSection]


def mass_text(mass: tuple[float, float]) -> str:
    low, high = mass
    return (
        f"{low * 1000:.1f} g"
        if low == high
        else f"{low * 1000:.1f}–{high * 1000:.1f} g"
    )


def stability_metrics(
    resolved: ResolvedAirframe, aero: AeroResult
) -> list[ReportMetric]:
    positions = [("Nominal CG", resolved.cg_m[0])]
    if resolved.cg_x_range_m[0] != resolved.cg_x_range_m[1]:
        positions.extend(
            [
                ("Aft CG", resolved.cg_x_range_m[1]),
                ("Forward CG", resolved.cg_x_range_m[0]),
            ]
        )
    metrics = []
    for label, cg in positions:
        margin = aero.x_np_m - cg
        verdict = stability_verdict(margin, resolved.reference.chord_m)
        metrics.append(
            ReportMetric(
                label,
                f"{verdict.capitalize()} · {margin * 1000:.1f} mm · {margin / resolved.reference.chord_m * 100:.1f}% c_ref",
                VERDICT_LEVELS[verdict],
            )
        )
    return metrics


def mass_sections(resolved: ResolvedAirframe) -> list[ReportSection]:
    metrics = [ReportMetric("Total mass", mass_text(resolved.mass_kg))]
    metrics.extend(
        ReportMetric(f"CG {axis}", f"{value:.4f} m")
        for axis, value in zip("xyz", resolved.cg_m)
    )
    if resolved.cg_x_range_m[0] != resolved.cg_x_range_m[1]:
        metrics.append(
            ReportMetric(
                "CG x range",
                f"{resolved.cg_x_range_m[0]:.4f}–{resolved.cg_x_range_m[1]:.4f} m",
            )
        )
    sections = [
        ReportSection(
            "Mass & balance",
            metrics,
            [
                ReportLine(
                    "CG uses the midpoint of each mass range. Axes: x aft, y right, z up."
                )
            ],
        )
    ]
    sections.append(
        ReportSection(
            "Inertia at CG",
            [
                ReportMetric(label, f"{value:.{5 if index < 3 else 6}f} kg m²")
                for index, (label, value) in enumerate(
                    zip(
                        ("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"),
                        resolved.inertia_kg_m2,
                    )
                )
            ],
            [ReportLine("Cross terms are tensor entries: −Σ m a b.")],
        )
    )
    rows = []
    for point in resolved.mass_points:
        note = (
            "Estimated"
            if point.computed
            else f"Estimate {point.estimate_kg * 1000:.1f} g"
            if point.estimate_kg is not None
            else "Given"
        )
        rows.append(
            ReportRow(
                [point.name, mass_text(point.mass_kg), f"{point.position[0]:.4f}", note]
            )
        )
    sections.append(
        ReportSection(
            "Mass breakdown",
            table=ReportTable(["Item", "Mass", "x (m)", "Source"], rows),
        )
    )
    estimated = [
        point for point in resolved.mass_points if point.estimate_kg is not None
    ]
    if any(not point.computed for point in estimated):
        sections.append(
            ReportSection(
                "Printed parts",
                [
                    ReportMetric(
                        "As given",
                        f"{sum(point.mass_kg[0] for point in estimated) * 1000:.1f} g",
                    ),
                    ReportMetric(
                        "Estimated",
                        f"{sum(point.estimate_kg for point in estimated) * 1000:.1f} g",
                    ),
                ],
                [
                    ReportLine(
                        "Hollow sections: shell wall × material density. Solid sections: less their volumes; surfaces: solid airfoil volume × print.bulk_density_kg_m3."
                    )
                ],
            )
        )
    for index, section in enumerate(resolved.sections):
        metrics = [
            ReportMetric(
                "Extent",
                f"{section.x0_m * 1000:.0f}–{section.x1_m * 1000:.0f} mm · {'solid' if section.solid else 'hollow'}",
            ),
            ReportMetric("Empty, as used", f"{section.empty_kg * 1000:.1f} g"),
            ReportMetric(
                "Empty estimate",
                "Not available"
                if section.empty_estimate_kg is None
                else f"{section.empty_estimate_kg * 1000:.1f} g",
            ),
            ReportMetric("Complete, itemised", mass_text(section.complete_itemised_kg)),
        ]
        if section.complete_kg is not None:
            metrics.append(
                ReportMetric("Complete, weighed", f"{section.complete_kg * 1000:.1f} g")
            )
        sections.append(
            ReportSection(
                f"Fuselage section {index + 1}",
                metrics,
                [
                    ReportLine(
                        "Complete includes everything attached in this section; a weighed complete mass replaces the itemised mass."
                    )
                ],
            )
        )
    return sections


def geometry_sections(resolved: ResolvedAirframe) -> list[ReportSection]:
    airframe, reference = resolved.airframe, resolved.reference
    fuselage = airframe.fuselage
    metrics = [
        ReportMetric("Length", f"{fuselage.length_m:.3f} m"),
        ReportMetric(
            "Cross section",
            f"{fuselage.section.width_m * 1000:.1f} × {fuselage.section.height_m * 1000:.1f} mm",
        ),
    ]
    for index, row in enumerate(airframe.fuselage.loft_stations):
        profile = row.section.resolve(fuselage.section)
        metrics.append(ReportMetric(f"Loft G{index + 1} at {row.x_m * 1000:.1f} mm before end taper",
                                        f"{profile.width_m * 1000:.1f} × {profile.height_m * 1000:.1f} mm · shape {profile.shape:g}"))
    if airframe.preset is not None:
        metrics.append(ReportMetric("Preset", airframe.preset.name))
    sections = [
        ReportSection("Fuselage", metrics),
        ReportSection(
            "Aerodynamic reference",
            [
                ReportMetric("Basis", reference.basis),
                ReportMetric("Area", f"{reference.area_m2:.4f} m²"),
                ReportMetric("Chord", f"{reference.chord_m:.4f} m"),
                ReportMetric("Span", f"{reference.span_m:.3f} m"),
            ],
        ),
    ]
    for surface in resolved.surfaces:
        sections.append(
            ReportSection(
                surface.name,
                [
                    ReportMetric(
                        "Layout", f"{surface.definition.layout} × {len(surface.panels)}"
                    ),
                    ReportMetric("Planform area", f"{surface.planform_area_m2:.4f} m²"),
                    ReportMetric("MAC", f"{surface.mac_m:.4f} m"),
                ],
            )
        )
    reinforcement = fuselage.reinforcement
    if reinforcement is not None:
        pattern = reinforcement.pattern
        sections.append(
            ReportSection(
                "Reinforcement",
                [
                    ReportMetric(
                        "Rods",
                        f"{pattern.polar_n} × {pattern.diameter_m * 1000:.1f} mm · {pattern.material}",
                    ),
                    ReportMetric(
                        "Radius", f"{pattern.radial_position_m * 1000:.2f} mm"
                    ),
                    ReportMetric(
                        "Extent",
                        f"{reinforcement.from_x_m * 1000:.0f}–{reinforcement.to_x_m * 1000:.0f} mm",
                    ),
                    ReportMetric(
                        "Angles",
                        f"{', '.join(f'{angle:g}' for angle in pattern.angles_deg())}° from horizontal",
                    ),
                ],
            )
        )
    return sections


def aero_sections(
    resolved: ResolvedAirframe, aero: AeroResult | None, status: str
) -> list[ReportSection]:
    if aero is None:
        return [ReportSection("Aerodynamics", notes=[ReportLine(status)])]
    weight = sum(resolved.mass_kg) / 2 * STANDARD_GRAVITY_M_S2
    sections = [
        ReportSection(
            "Static stability estimate",
            stability_metrics(resolved, aero)
            + [
                ReportMetric("Neutral point", f"{aero.x_np_m:.4f} m"),
                ReportMetric("Barrowman CP", f"{aero.barrowman_cp_m:.4f} m"),
            ],
            [
                ReportLine(
                    "Static margin = neutral point − CG, as a percentage of reference chord. Barrowman CP is a rocket-style cross-check."
                )
            ],
        )
    ]
    sections.append(
        ReportSection(
            "Operating point",
            [
                ReportMetric("Angle of attack", f"{aero.aoa_deg:.1f}°"),
                ReportMetric("Airspeed", f"{aero.airspeed_m_s:.1f} m/s"),
                ReportMetric("Altitude", f"{aero.altitude_m:.0f} m"),
                ReportMetric("Dynamic pressure", f"{aero.dynamic_pressure_pa:.1f} Pa"),
            ],
            [ReportLine("Lifting-line wings + AeroBuildup bodies.")],
        )
    )
    sections.append(
        ReportSection(
            "Total forces",
            [
                ReportMetric(
                    "Lift",
                    f"{aero.lift_n:.3f} N · {aero.lift_n * GRAMS_FORCE_PER_NEWTON:.0f} g",
                ),
                ReportMetric("Lift / weight", f"{aero.lift_n / weight * 100:.1f}%"),
                ReportMetric("Drag", f"{aero.drag_n:.3f} N"),
                ReportMetric(
                    "CL / CD / Cm", f"{aero.cl:.4f} / {aero.cd:.4f} / {aero.cm:.4f}"
                ),
            ],
        )
    )
    rows = [
        ReportRow(
            [
                force.name,
                f"{force.lift_n:.3f}",
                f"{force.drag_n:.3f}",
                f"{force.cl:.3f}",
                f"{force.x_lift_m:.4f}" if math.isfinite(force.x_lift_m) else "n/a",
            ]
        )
        for force in aero.surfaces
    ]
    rows.append(
        ReportRow(
            [
                "Body + pods",
                f"{aero.body_lift_n:.3f}",
                f"{aero.body_drag_n:.3f}",
                "—",
                "—",
            ]
        )
    )
    sections.append(
        ReportSection(
            "Forces by surface",
            table=ReportTable(
                ["Surface", "Lift (N)", "Drag (N)", "CL", "Lift x (m)"], rows
            ),
        )
    )
    rows = [
        ReportRow(
            [
                f"{row.aoa_deg:.0f}",
                f"{row.cl:.4f}",
                f"{row.cd:.4f}",
                f"{row.cm:.4f}",
                f"{row.lift_n:.3f}",
                f"{row.lift_n / weight * 100:.1f}",
                f"{row.x_np_m:.4f}",
                row.pitch,
            ]
        )
        for row in aero.sweep
    ]
    sections.append(
        ReportSection(
            "Angle-of-attack sweep",
            table=ReportTable(
                [
                    "AoA (°)",
                    "CL",
                    "CD",
                    "Cm",
                    "Lift (N)",
                    "Weight (%)",
                    "NP (m)",
                    "Pitch",
                ],
                rows,
            ),
            collapsible=True,
        )
    )
    return sections


def motor_metrics(
    points: MotorPoints, summary: PropulsionSummary
) -> list[ReportMetric]:
    count, model = summary.motor_count, summary.models[0]
    return [
        ReportMetric(
            "Total thrust",
            f"{float(points.thrust_n) * count * GRAMS_FORCE_PER_NEWTON:.0f} g",
        ),
        ReportMetric("Total current", f"{float(points.current_a) * count:.1f} A"),
        ReportMetric("Total power", f"{float(points.power_w) * count:.0f} W"),
        ReportMetric("Motor RPM", f"{float(points.rpm):.0f}"),
        ReportMetric(
            "Pitch speed", f"{float(points.rpm) / 60 * model.pitch_m * 3.6:.0f} km/h"
        ),
        ReportMetric(
            "Efficiency",
            f"{float(points.thrust_n) * GRAMS_FORCE_PER_NEWTON / float(points.power_w):.2f} g/W",
        ),
    ]


def resistance_spread(
    values: list[float], unit: str = "mΩ", scale: float = 1000, digits: int = 1
) -> str:
    low, high = min(values) * scale, max(values) * scale
    return (
        f"{low:.{digits}f} {unit}"
        if round(high - low, digits) == 0
        else f"{low:.{digits}f}–{high:.{digits}f} {unit}"
    )


def propulsion_sections(
    resolved: ResolvedAirframe, propulsion: PropulsionSummary | None
) -> list[ReportSection]:
    definition = resolved.airframe.propulsion
    if definition is None:
        return [ReportSection("Propulsion", notes=[ReportLine("Unpowered airframe.")])]
    motor_label = (
        f"{definition.motor.manufacturer} {definition.motor.name}"
        if propulsion is None
        else propulsion.models[0].label
    )
    sections = [
        ReportSection(
            "Motors & propellers",
            [
                ReportMetric("Motors", f"{len(resolved.motors)} × {motor_label}"),
                ReportMetric("Propeller", definition.prop),
            ],
            [ReportLine("Spin is seen from where the thrust points.")],
            ReportTable(
                ["Motor", "Spin"],
                [
                    ReportRow([placement.name, placement.spin])
                    for placement in resolved.motors
                ],
            ),
        )
    ]
    if propulsion is None:
        sections.append(
            ReportSection(
                "Static performance",
                notes=[
                    ReportLine(
                        "No propulsion result. See Log for calculation errors.",
                        "warning",
                    )
                ],
            )
        )
        return sections
    model, count = propulsion.models[0], propulsion.motor_count
    sections.append(
        ReportSection(
            "Battery",
            [
                ReportMetric("Configuration", f"{model.cells}S{model.parallel}P"),
                ReportMetric(
                    "Capacity", f"{model.pack_mah:.0f} mAh · {model.pack_wh:.1f} Wh"
                ),
                ReportMetric("Nominal voltage", f"{propulsion.pack_voltage_v:.1f} V"),
            ],
        )
    )
    sections.append(
        ReportSection(
            "Full throttle",
            motor_metrics(propulsion.full, propulsion)
            + [
                ReportMetric(
                    "Thrust / weight",
                    f"{propulsion.thrust_to_weight:.2f} at {propulsion.weight_n * GRAMS_FORCE_PER_NEWTON:.0f} g",
                ),
                ReportMetric(
                    "Run time", f"{propulsion.full_throttle_time_min:.1f} min"
                ),
            ],
            [
                ReportLine(
                    "Static motorlib model; throttle = INAV motor output. Thrust, current and power are totals across all motors."
                )
            ],
        )
    )
    if propulsion.hover is not None:
        sections.append(
            ReportSection(
                "Hover",
                motor_metrics(propulsion.hover, propulsion)
                + [
                    ReportMetric("Motor output", f"{float(propulsion.hover.duty):.3f}"),
                    ReportMetric("Run time", f"{propulsion.hover_time_min:.1f} min"),
                ],
            )
        )
    warnings = [line for line in summary_lines(propulsion) if line.level != "info"]
    if warnings:
        sections.append(ReportSection("Model limitations", notes=warnings))
    wirings = [item.wiring for item in propulsion.models]
    shared = wirings[0]
    sections.append(
        ReportSection(
            "Motor circuit resistance",
            [
                ReportMetric(
                    "Per motor, total",
                    resistance_spread([w.per_motor_ohm for w in wirings]),
                ),
                ReportMetric(
                    "Motor leads",
                    resistance_spread([w.motor_leads_ohm for w in wirings]),
                ),
                ReportMetric(
                    "Motor route",
                    resistance_spread([w.motor_route_m for w in wirings], "m", 1, 3),
                ),
                ReportMetric(
                    "Motor plugs",
                    resistance_spread([w.motor_plugs_ohm for w in wirings]),
                ),
                ReportMetric("ESC", f"{shared.esc_ohm * 1000:.1f} mΩ"),
                ReportMetric(
                    "Shared contribution",
                    f"{count} × {shared.shared_ohm * 1000:.1f} mΩ",
                ),
            ],
        )
    )
    sections.append(
        ReportSection(
            "Shared battery circuit",
            [
                ReportMetric("Pack", f"{shared.pack_ohm * 1000:.1f} mΩ"),
                ReportMetric("Pack leads", f"{shared.pack_leads_ohm * 1000:.1f} mΩ"),
                ReportMetric(
                    "Battery leads", f"{shared.battery_leads_ohm * 1000:.1f} mΩ"
                ),
                ReportMetric("Battery route", f"{shared.battery_route_m:.3f} m"),
                ReportMetric(
                    "Battery plugs", f"{shared.battery_plugs_ohm * 1000:.1f} mΩ"
                ),
            ],
        )
    )
    return sections


def build_report_pages(
    resolved: ResolvedAirframe,
    aero: AeroResult | None,
    problems: list[str],
    propulsion: PropulsionSummary | None,
    status: str,
) -> ReportPages:
    airframe = resolved.airframe
    kind = (
        "Tailsitter VTOL"
        if airframe.tailsitter
        else "VTOL"
        if airframe.vtol
        else "Fixed-wing"
        if airframe.powered
        else "Glider (unpowered)"
    )
    metrics = [
        ReportMetric("Type", kind),
        ReportMetric("Total mass", mass_text(resolved.mass_kg)),
        ReportMetric("CG x", f"{resolved.cg_m[0]:.4f} m"),
    ]
    if propulsion is not None:
        metrics.append(
            ReportMetric("Thrust / weight", f"{propulsion.thrust_to_weight:.2f}")
        )
    summary = [ReportSection(airframe.name, metrics)]
    if problems:
        summary.append(
            ReportSection(
                "Layout problems",
                notes=[
                    ReportNotice(problem, "problem", problem_target(resolved, problem))
                    for problem in problems
                ],
            )
        )
    if propulsion is not None:
        warnings = [line for line in summary_lines(propulsion) if line.level != "info"]
        if warnings:
            summary.append(ReportSection("Propulsion alerts", notes=warnings))
    if aero is not None:
        summary.append(
            ReportSection(
                "Static stability estimate",
                stability_metrics(resolved, aero),
                [
                    ReportLine(
                        "Static pitch estimate at the editor operating point, not a flight-response verdict. See SIM for dynamic response; Aero for the sweep."
                    )
                ],
            )
        )
    else:
        summary.append(ReportSection("Aerodynamics", notes=[ReportLine(status)]))
    summary.append(
        ReportSection(
            "Geometry details", children=geometry_sections(resolved), collapsible=True
        )
    )
    return ReportPages(
        summary,
        mass_sections(resolved),
        aero_sections(resolved, aero, status),
        propulsion_sections(resolved, propulsion),
    )
