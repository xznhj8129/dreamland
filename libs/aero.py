"""
Usage:
from libs.aero import analyze_aero, analyze_aero_detailed
result = analyze_aero(resolved)            # fast: AeroBuildup, ~1 s
detailed = analyze_aero_detailed(resolved)  # hybrid: lifting-line wings + AeroBuildup bodies, ~30-60 s

Aerodynamics of a resolved airframe (libs/geometry.py). Geometry is read from the resolved model only.
- Fast (analyze_aero): AeroSandbox AeroBuildup over the AoA sweep and the operating point, neutral point from a
  dCm/dCL fit, Barrowman rocket-style cross-check. No downwash between surfaces, no wing-body carry-over, and its
  finite-wing correction is centred on 0 deg rather than the zero-lift angle (over-credits camber).
- Detailed (analyze_aero_detailed): AeroSandbox LiftingLine for all wings (downwash between surfaces, viscous
  NeuralFoil section polars, stall), body-rooted panels carried through the body to the axis (lift carried across
  the body), plus AeroBuildup for the bodies alone (fuselage, pods, motors). Totals only; valid to about +-30 deg.
- VLM sweep (run_vlm_sweep): wings only, inviscid; see its report header.
"""

import concurrent.futures
import math
from dataclasses import dataclass, replace

import aerosandbox as asb
import numpy as np

from libs.geometry import STANDARD_GRAVITY_M_S2, ResolvedAirframe

AERO_MODEL_SIZE = "large"
AOA_SWEEP_DEG = range(-45, 46)
STABILITY_MARGINAL_PCT = 5.0
NEUTRAL_POINT_FIT_HALF_WINDOW_DEG = 4.0
DETAILED_SWEEP_DEG = range(-30, 31, 2)  # the lifting line returns NaN beyond about +-40 deg
DETAILED_FIT_OFFSETS_DEG = (-4.0, -2.0, 0.0, 2.0, 4.0)
LIFTING_LINE_SPANWISE_RESOLUTION = 10
LIFTING_LINE_CORE_RADIUS_M = 1e-3  # finite vortex core: panels carried to the axis meet there
VLM_SETTING_KEYS = (
    "aoa_start", "aoa_end", "aoa_increment", "airspeed_start", "airspeed_end", "airspeed_increment",
    "altitude_start", "altitude_end", "altitude_increment", "vlm_threads", "vlm_span_res", "vlm_chord_res",
)
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
            span=surface_set.exposed_span_m,
            chord_root=surface_set.exposed_chord_root_m,
            chord_tip=definition.chord_tip_m,
            sweep_len=surface_set.exposed_sweep_len_m,
            body_radius=surface_set.body_half_extent_m,
            ref_diameter=ref_diameter,
            x_root_le=surface_set.exposed_x_root_le_m,
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


@dataclass(frozen=True)
class VLMRow:
    altitude_m: float
    aoa_deg: float
    airspeed_m_s: float
    lift_n: float
    drag_induced_n: float
    cl: float
    cdi: float
    cm: float
    load_pct: float
    x_np_m: float  # wings only
    static_margin_pct: float  # wings only, % of the reference chord


@dataclass(frozen=True)
class VLMResult:
    airframe_name: str
    settings: dict
    aoa_values: tuple[float, ...]
    airspeed_values: tuple[float, ...]
    altitude_values: tuple[float, ...]
    rows: tuple[VLMRow, ...]
    solutions: dict  # aoa -> solved asb.VortexLatticeMethod, for the 3D view
    report_text: str


def range_values(start: float, end: float, step: float, label: str) -> tuple[float, ...]:
    if step <= 0.0 or end < start:
        raise ValueError(f"{label}: needs step > 0 and end >= start (got start {start}, end {end}, step {step})")
    count = int(math.floor((end - start) / step + 1e-9)) + 1
    values = [start + index * step for index in range(count)]
    if abs(values[-1] - end) > 1e-9 * max(1.0, abs(end)):
        values.append(end)  # keep the requested end point when the step does not land on it
    return tuple(values)


def build_vlm_report_text(result: VLMResult) -> str:
    settings = result.settings
    lines = [
        "VLM Sweep (wings only)",
        f"Airframe              : {result.airframe_name}",
        "",
        "AeroSandbox VLM models the lifting surfaces only: the fuselage, pods and motors are left out, so",
        "body-mounted wings read low (no lift carried across the body) and NP / static margin here are for",
        "the wings alone. The Configuration summary (AeroBuildup) includes the body.",
        "",
        "Parameters",
        f"AoA                   : {settings['aoa_start']:g} to {settings['aoa_end']:g} deg step {settings['aoa_increment']:g}",
        f"Airspeed              : {settings['airspeed_start']:g} to {settings['airspeed_end']:g} m/s step {settings['airspeed_increment']:g}",
        f"Altitude              : {settings['altitude_start']:g} to {settings['altitude_end']:g} m step {settings['altitude_increment']:g}",
        f"Resolution            : {settings['vlm_span_res']} spanwise x {settings['vlm_chord_res']} chordwise",
        f"Solved AoA points     : {len(result.aoa_values)} (coefficients do not depend on airspeed or altitude; forces scale with q)",
        "",
    ]
    header = f"{'Alt m':<8} | {'AoA':<5} | {'V m/s':<7} | {'Lift N':<8} | {'Drag_i N':<9} | {'CL':<7} | {'CDi':<7} | {'Cm':<7} | {'Load %':<7} | {'NP wings m':<10} | {'SM wings %c':<11}"
    lines += [header, "-" * len(header)]
    for row in result.rows:
        lines.append(
            f"{row.altitude_m:<8.1f} | {row.aoa_deg:<5.1f} | {row.airspeed_m_s:<7.1f} | {row.lift_n:<8.2f} | {row.drag_induced_n:<9.3f} | "
            f"{row.cl:<7.4f} | {row.cdi:<7.4f} | {row.cm:<7.4f} | {row.load_pct:<7.1f} | {row.x_np_m:<10.4f} | {row.static_margin_pct:<11.1f}"
        )
    return "\n".join(lines)


def run_vlm_sweep(resolved: ResolvedAirframe, settings: dict, progress_callback=None) -> VLMResult:
    """VLM over the AoA range, solved once per AoA; airspeed/altitude rows scale the forces by dynamic pressure."""
    aoa_values = range_values(settings["aoa_start"], settings["aoa_end"], settings["aoa_increment"], "AoA")
    airspeed_values = range_values(settings["airspeed_start"], settings["airspeed_end"], settings["airspeed_increment"], "Airspeed")
    altitude_values = range_values(settings["altitude_start"], settings["altitude_end"], settings["altitude_increment"], "Altitude")
    airplane = build_asb_airplane(resolved)
    reference = resolved.reference
    cg_x = resolved.cg_m[0]
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2

    def solve(aoa_deg: float):
        vlm = asb.VortexLatticeMethod(
            airplane=airplane,
            op_point=asb.OperatingPoint(velocity=airspeed_values[0], alpha=aoa_deg),
            spanwise_resolution=int(settings["vlm_span_res"]),
            chordwise_resolution=int(settings["vlm_chord_res"]),
        )
        return aoa_deg, vlm, vlm.run_with_stability_derivatives(alpha=True, beta=False, p=False, q=False, r=False)

    solved = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, int(settings["vlm_threads"]))) as executor:
        for completed, (aoa_deg, vlm, coefficients) in enumerate(executor.map(solve, aoa_values), start=1):
            solved[aoa_deg] = (vlm, coefficients)
            if progress_callback is not None:
                progress_callback(completed, len(aoa_values), f"AoA {aoa_deg:g} deg")

    rows = []
    for altitude_m in altitude_values:
        density = float(asb.Atmosphere(altitude=altitude_m).density())
        for aoa_deg in aoa_values:
            _vlm, coefficients = solved[aoa_deg]
            cl, cdi, cm = float(coefficients["CL"]), float(coefficients["CD"]), float(coefficients["Cm"])
            x_np = float(coefficients["x_np"])
            for airspeed_m_s in airspeed_values:
                force_scale = 0.5 * density * airspeed_m_s * airspeed_m_s * reference.area_m2
                rows.append(VLMRow(
                    altitude_m=altitude_m, aoa_deg=aoa_deg, airspeed_m_s=airspeed_m_s,
                    lift_n=cl * force_scale, drag_induced_n=abs(cdi) * force_scale, cl=cl, cdi=cdi, cm=cm,
                    load_pct=cl * force_scale / weight_n * 100.0,
                    x_np_m=x_np, static_margin_pct=(x_np - cg_x) / reference.chord_m * 100.0,
                ))
    result = VLMResult(
        airframe_name=resolved.airframe.name, settings=dict(settings), aoa_values=aoa_values,
        airspeed_values=airspeed_values, altitude_values=altitude_values, rows=tuple(rows),
        solutions={aoa: vlm for aoa, (vlm, _coefficients) in solved.items()}, report_text="",
    )
    return replace(result, report_text=build_vlm_report_text(result))


@dataclass(frozen=True)
class DetailedAeroResult:
    aoa_deg: float
    airspeed_m_s: float
    altitude_m: float
    lift_n: float
    drag_n: float
    cl: float
    cd: float
    cm: float
    body_lift_n: float
    lift_center_x_m: float
    x_np_m: float
    sweep: tuple[SweepRow, ...]


def carried_to_axis(wing: asb.Wing) -> asb.Wing:
    """Copy of a body-rooted wing with an extra inner section where its span line comes closest to the fuselage axis,
    so the lifting line carries its lift across the body."""
    root, tip = wing.xsecs[0], wing.xsecs[-1]
    span = np.array(tip.xyz_le[1:], dtype=float) - np.array(root.xyz_le[1:], dtype=float)
    span /= np.linalg.norm(span)
    root_yz = np.array(root.xyz_le[1:], dtype=float)
    inner = asb.WingXSec(xyz_le=[root.xyz_le[0], *(root_yz - float(root_yz @ span) * span)], chord=root.chord, twist=root.twist, airfoil=root.airfoil)
    return asb.Wing(name=wing.name, symmetric=wing.symmetric, xsecs=[inner, *wing.xsecs])


def build_hybrid_airplanes(resolved: ResolvedAirframe) -> tuple[asb.Airplane, asb.Airplane]:
    """(wings for the lifting line, bodies for AeroBuildup), sharing references and the CG moment reference."""
    airplane = build_asb_airplane(resolved)
    owners = [surface_set for surface_set in resolved.surfaces for panel in surface_set.panels if panel.xz_mirror != "copy"]
    wings = [carried_to_axis(wing) if owner.definition.root == "body" else wing for wing, owner in zip(airplane.wings, owners)]
    references = dict(xyz_ref=airplane.xyz_ref, s_ref=airplane.s_ref, c_ref=airplane.c_ref, b_ref=airplane.b_ref)
    return (
        asb.Airplane(name=f"{airplane.name} wings", wings=wings, fuselages=[], **references),
        asb.Airplane(name=f"{airplane.name} bodies", wings=[], fuselages=airplane.fuselages, **references),
    )


def analyze_aero_detailed(resolved: ResolvedAirframe, progress_callback=None) -> DetailedAeroResult:
    operating_point = resolved.airframe.operating_point
    limit = max(abs(DETAILED_SWEEP_DEG.start), abs(DETAILED_SWEEP_DEG.stop - 1))
    if abs(operating_point.aoa_deg) + max(DETAILED_FIT_OFFSETS_DEG) > limit:
        raise ValueError(f"detailed aero covers +-{limit} deg; operating point AoA {operating_point.aoa_deg} deg is too close to that")
    aoa_values = np.array(sorted({float(a) for a in DETAILED_SWEEP_DEG} | {operating_point.aoa_deg + offset for offset in DETAILED_FIT_OFFSETS_DEG}))
    wing_plane, body_plane = build_hybrid_airplanes(resolved)
    atmosphere = asb.Atmosphere(altitude=operating_point.altitude_m)
    op_points = asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s, alpha=aoa_values)
    body = asb.AeroBuildup(airplane=body_plane, op_point=op_points, model_size=AERO_MODEL_SIZE).run()

    totals = {key: [] for key in ("CL", "CD", "Cm", "L", "D", "M")}
    for index, aoa_deg in enumerate(aoa_values):
        wings = asb.LiftingLine(
            airplane=wing_plane,
            op_point=asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s, alpha=float(aoa_deg)),
            model_size=AERO_MODEL_SIZE, spanwise_resolution=LIFTING_LINE_SPANWISE_RESOLUTION, vortex_core_radius=LIFTING_LINE_CORE_RADIUS_M,
        ).run()
        for key in ("CL", "CD", "Cm", "L", "D"):
            totals[key].append(float(wings[key]) + float(body[key][index]))
        totals["M"].append(float(wings["M_b"][1]) + float(body["M_b"][1][index]))
        if progress_callback is not None:
            progress_callback(index + 1, len(aoa_values), f"AoA {aoa_deg:g} deg")
    cl, cm = np.array(totals["CL"]), np.array(totals["Cm"])
    cg_x = resolved.cg_m[0]
    chord = resolved.reference.chord_m
    window = np.abs(aoa_values - operating_point.aoa_deg) <= NEUTRAL_POINT_FIT_HALF_WINDOW_DEG
    x_np = cg_x - float(np.polyfit(cl[window], cm[window], 1)[0]) * chord
    with np.errstate(divide="ignore", invalid="ignore"):
        local_np = cg_x - np.gradient(cm, aoa_values) / np.gradient(cl, aoa_values) * chord
    sweep = []
    for index, aoa_deg in enumerate(aoa_values):
        if float(aoa_deg) not in DETAILED_SWEEP_DEG:
            continue
        pitch = "nose-up" if cm[index] > 0.0 else "nose-down" if cm[index] < 0.0 else "trim"
        sweep.append(SweepRow(float(aoa_deg), float(cl[index]), totals["CD"][index], float(cm[index]), totals["L"][index], float(local_np[index]), pitch))
    eval_index = int(np.argmin(np.abs(aoa_values - operating_point.aoa_deg)))
    lift = totals["L"][eval_index]
    return DetailedAeroResult(
        aoa_deg=operating_point.aoa_deg, airspeed_m_s=operating_point.airspeed_m_s, altitude_m=operating_point.altitude_m,
        lift_n=lift, drag_n=totals["D"][eval_index], cl=float(cl[eval_index]), cd=totals["CD"][eval_index], cm=float(cm[eval_index]),
        body_lift_n=float(body["L"][eval_index]),
        lift_center_x_m=cg_x - totals["M"][eval_index] / lift if abs(lift) > 1e-9 else float("nan"),
        x_np_m=x_np, sweep=tuple(sweep),
    )

