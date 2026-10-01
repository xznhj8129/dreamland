"""
Usage:
from libs.aero import analyze_aero
result = analyze_aero(resolved)

Aerodynamics of a resolved airframe (libs/geometry.py): AeroSandbox AeroBuildup over the AoA sweep and the
operating point, neutral point from a dCm/dCL fit, and a Barrowman rocket-style cross-check.
Geometry is read from the resolved model only.
"""

import math
from dataclasses import dataclass

import aerosandbox as asb
import numpy as np

from libs.geometry import ResolvedAirframe

AERO_MODEL_SIZE = "large"
AOA_SWEEP_DEG = range(-45, 46)
STABILITY_MARGINAL_PCT = 5.0
NEUTRAL_POINT_FIT_HALF_WINDOW_DEG = 4.0
NOSE_CNA = 2.0
NOSE_CP_FRACTION = 0.466  # of nose length (tangent ogive)


@dataclass(frozen=True)
class SweepRow:
    aoa_deg: float
    cl: float
    cd: float
    cm: float
    lift_n: float
    x_np_m: float
    pitch: str


@dataclass(frozen=True)
class SurfaceForce:
    name: str
    airfoil: str
    area_m2: float
    cl: float
    cd: float
    lift_n: float
    drag_n: float
    x_lift_m: float


@dataclass(frozen=True)
class BarrowmanRow:
    component: str
    cna: float
    x_cp_m: float
    notes: str


@dataclass(frozen=True)
class AeroResult:
    aoa_deg: float
    airspeed_m_s: float
    altitude_m: float
    density_kg_m3: float
    dynamic_pressure_pa: float
    lift_n: float
    drag_n: float
    cl: float
    cd: float
    cm: float
    lift_center_x_m: float
    surfaces: tuple[SurfaceForce, ...]
    body_lift_n: float
    body_drag_n: float
    x_np_m: float
    sweep: tuple[SweepRow, ...]
    barrowman_cp_m: float
    barrowman: tuple[BarrowmanRow, ...]


def barrowman_finset(
    effective_fin_count: float,
    span: float,
    chord_root: float,
    chord_tip: float,
    sweep_len: float,
    body_radius: float,
    ref_diameter: float,
    x_root_le: float,
) -> tuple[float, float, float]:
    mid_chord_line = math.sqrt(
        span * span + (sweep_len + 0.5 * chord_tip - 0.5 * chord_root) ** 2
    )
    cna = (
        (1.0 + body_radius / (span + body_radius))
        * (4.0 * effective_fin_count * (span / ref_diameter) ** 2)
        / (1.0 + math.sqrt(1.0 + (2.0 * mid_chord_line / (chord_root + chord_tip)) ** 2))
    )
    x_cp_local = (
        sweep_len / 3.0 * ((chord_root + 2.0 * chord_tip) / (chord_root + chord_tip))
        + (1.0 / 6.0)
        * (chord_root + chord_tip - (chord_root * chord_tip) / (chord_root + chord_tip))
    )
    return cna, x_root_le + x_cp_local, mid_chord_line


def build_asb_airplane(resolved: ResolvedAirframe) -> asb.Airplane:
    body = resolved.body
    fuselages = [asb.Fuselage(name="Fuselage", xsecs=[
        asb.FuselageXSec(xyz_c=[station.x_m, 0.0, 0.0], width=station.width_m, height=station.height_m, shape=body.shape)
        for station in body.stations
    ])]
    wings = []
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            if panel.xz_mirror == "copy":
                continue
            sections = [
                asb.WingXSec(xyz_le=list(panel.root_le), chord=panel.chord_root_m, twist=panel.incidence_deg, airfoil=asb.Airfoil(panel.airfoil)),
                asb.WingXSec(xyz_le=list(panel.tip_le), chord=panel.chord_tip_m, twist=panel.incidence_deg, airfoil=asb.Airfoil(panel.airfoil)),
            ]
            # ASB's section normal for root->tip order is (0, -span_z, span_y); flip order to match the panel normal.
            asb_normal = (0.0, -panel.span_dir[2], panel.span_dir[1])
            if sum(a * b for a, b in zip(asb_normal, panel.normal)) < 0.0:
                sections.reverse()
            wings.append(asb.Wing(name=f"{panel.surface}_{panel.index + 1}", symmetric=panel.xz_mirror == "primary", xsecs=sections))
    propulsors = []
    for motor in resolved.motors:
        x, y, z = motor.position
        half = motor.bell_length_m / 2.0
        radius = motor.bell_diameter_m / 2.0
        fuselages.append(asb.Fuselage(name=f"{motor.name} bell", xsecs=[
            asb.FuselageXSec(xyz_c=[x - half, y, z], radius=radius), asb.FuselageXSec(xyz_c=[x + half, y, z], radius=radius),
        ]))
        if motor.pod_stations:
            fuselages.append(asb.Fuselage(name=f"{motor.name} pod", xsecs=[
                asb.FuselageXSec(xyz_c=[pod_x, y, z], radius=pod_radius) for pod_x, pod_radius in motor.pod_stations
            ]))
        propulsors.append(asb.Propulsor(name=motor.name, xyz_c=list(motor.prop_center), xyz_normal=list(motor.thrust_dir), radius=motor.prop_radius_m, length=0.0))
    reference = resolved.reference
    return asb.Airplane(
        name=resolved.airframe.name, xyz_ref=list(resolved.cg_m), s_ref=reference.area_m2, c_ref=reference.chord_m,
        b_ref=reference.span_m, wings=wings, fuselages=fuselages, propulsors=propulsors,
    )


def barrowman(resolved: ResolvedAirframe) -> tuple[float, tuple[BarrowmanRow, ...]]:
    nose_length = resolved.airframe.fuselage.nose.length_m
    ref_diameter = max(station.width_m for station in resolved.body.stations)
    rows = [BarrowmanRow(f"nose {resolved.airframe.fuselage.nose.profile}", NOSE_CNA, NOSE_CP_FRACTION * nose_length, f"L={nose_length:.3f} m")]
    for surface_set in resolved.surfaces:
        definition = surface_set.definition
        # Barrowman's 4N form equals N/2 single-fin slopes for evenly spaced fins. A panel's
        # pitch-plane share is dir_y^2: vertical fins add nothing, a horizontal pair counts as 4.
        effective_fin_count = 2.0 * sum(panel.span_dir[1] ** 2 for panel in surface_set.panels)
        cna, x_cp, mid_chord_line = barrowman_finset(
            effective_fin_count=effective_fin_count,
            span=definition.exposed_span_m,
            chord_root=definition.chord_root_m,
            chord_tip=definition.chord_tip_m,
            sweep_len=definition.exposed_span_m * math.tan(math.radians(definition.sweep_le_deg)),
            body_radius=surface_set.body_half_extent_m,
            ref_diameter=ref_diameter,
            x_root_le=definition.x_root_le_m,
        )
        rows.append(BarrowmanRow(surface_set.name, cna, x_cp, f"N={len(surface_set.panels)} N_pitch={effective_fin_count:.2f} MCL={mid_chord_line:.3f} m"))
    total = sum(row.cna for row in rows)
    return sum(row.cna * row.x_cp_m for row in rows) / total, tuple(rows)


def analyze_aero(resolved: ResolvedAirframe) -> AeroResult:
    operating_point = resolved.airframe.operating_point
    cg_x = resolved.cg_m[0]
    reference = resolved.reference
    airplane = build_asb_airplane(resolved)
    atmosphere = asb.Atmosphere(altitude=operating_point.altitude_m)

    # One vectorized AeroBuildup run: the sweep plus the operating point appended at eval_index.
    sweep_aoa = np.array(AOA_SWEEP_DEG, dtype=float)
    eval_index = len(sweep_aoa)
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s, alpha=np.append(sweep_aoa, operating_point.aoa_deg))
    aero = asb.AeroBuildup(airplane=airplane, op_point=op_point, model_size=AERO_MODEL_SIZE).run()
    dynamic_pressure = float(op_point.dynamic_pressure())
    cl_all = np.asarray(aero["CL"])
    cm_all = np.asarray(aero["Cm"])

    with np.errstate(divide="ignore", invalid="ignore"):
        sweep_x_np = cg_x - np.gradient(cm_all[:eval_index], sweep_aoa) / np.gradient(cl_all[:eval_index], sweep_aoa) * reference.chord_m
    sweep = []
    for index, aoa in enumerate(sweep_aoa):
        cm = float(cm_all[index])
        pitch = "nose-up" if cm > 0.0 else "nose-down" if cm < 0.0 else "trim"
        sweep.append(SweepRow(float(aoa), float(cl_all[index]), float(aero["CD"][index]), cm, float(aero["L"][index]), float(sweep_x_np[index]), pitch))

    # NeuralFoil's low-Re section curves are bumpy, so a local derivative jitters by centimetres.
    # The neutral point uses the least-squares dCm/dCL over sweep points within the window around the operating point.
    window = np.abs(sweep_aoa - operating_point.aoa_deg) <= NEUTRAL_POINT_FIT_HALF_WINDOW_DEG
    dcm_dcl = float(np.polyfit(cl_all[:eval_index][window], cm_all[:eval_index][window], 1)[0])

    # airplane.wings holds one wing per panel that is not an XZ-mirror copy, in resolved.surfaces order.
    wing_components = iter(aero["wing_aero_components"])
    surfaces = []
    for surface_set in resolved.surfaces:
        lift = drag = pitch_moment = 0.0
        for panel in surface_set.panels:
            if panel.xz_mirror == "copy":
                continue
            component = next(wing_components)
            lift += float(component.L[eval_index])
            drag += float(component.D[eval_index])
            pitch_moment += float(component.M_b[1][eval_index])
        area = surface_set.planform_area_m2
        surfaces.append(SurfaceForce(
            name=surface_set.name, airfoil=surface_set.definition.airfoil, area_m2=area,
            cl=lift / (dynamic_pressure * area), cd=drag / (dynamic_pressure * area), lift_n=lift, drag_n=drag,
            x_lift_m=cg_x - pitch_moment / lift if abs(lift) > 1e-9 else float("nan"),
        ))

    lift = float(aero["L"][eval_index])
    barrowman_cp, barrowman_rows = barrowman(resolved)
    return AeroResult(
        aoa_deg=operating_point.aoa_deg, airspeed_m_s=operating_point.airspeed_m_s, altitude_m=operating_point.altitude_m,
        density_kg_m3=float(atmosphere.density()), dynamic_pressure_pa=dynamic_pressure,
        lift_n=lift, drag_n=float(aero["D"][eval_index]), cl=float(cl_all[eval_index]), cd=float(aero["CD"][eval_index]),
        cm=float(cm_all[eval_index]),
        lift_center_x_m=cg_x - float(aero["M_b"][1][eval_index]) / lift if abs(lift) > 1e-9 else float("nan"),
        surfaces=tuple(surfaces),
        body_lift_n=sum(float(component.L[eval_index]) for component in aero["fuselage_aero_components"]),
        body_drag_n=sum(float(component.D[eval_index]) for component in aero["fuselage_aero_components"]),
        x_np_m=cg_x - dcm_dcl * reference.chord_m, sweep=tuple(sweep),
        barrowman_cp_m=barrowman_cp, barrowman=barrowman_rows,
    )


def stability_verdict(margin_m: float, reference_length_m: float) -> str:
    margin_pct = margin_m / reference_length_m * 100.0
    if margin_pct <= 0.0:
        return "unstable"
    if margin_pct < STABILITY_MARGINAL_PCT:
        return "marginal"
    return "stable"
