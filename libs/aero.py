"""
Usage:
from libs.aero import analyze_aero
result = analyze_aero(resolved)  # ~1-2 s

Aerodynamics of a resolved airframe (libs/geometry.py). Geometry is read from the resolved model only.
- analyze_aero: AeroSandbox's lifting line for all wings (downwash between surfaces, viscous NeuralFoil section
  polars, stall), body-rooted panels carried through the body to the axis (lift carried across the body), plus
  AeroBuildup for the bodies alone (fuselage, pods, motors). Neutral point from a dCm/dCL fit, Barrowman
  rocket-style cross-check. The lifting line is asb.LiftingLine's algorithm solved for every AoA at once
  (lifting_line_sweep); aero_check.py verifies it against asb.LiftingLine. Control surfaces are asb.ControlSurface
  on the moving segment of each panel (AeroSandbox's model: an effective AoA change of deflection x (1 - hinge^2.75)
  in the section polar, plus a drag factor); lifting_line_sweep takes the deflections per control number.
- VLM sweep (run_vlm_sweep): wings only, inviscid; see its report header.
"""

import concurrent.futures
import math
from dataclasses import dataclass, replace

import aerosandbox as asb
import aerosandbox.numpy as asb_np
import numpy as np
from aerosandbox.aerodynamics.aero_3D.singularities.uniform_strength_horseshoe_singularities import calculate_induced_velocity_horseshoe

from libs.airframe import ControlSurface
from libs.geometry import STANDARD_GRAVITY_M_S2, Panel, ResolvedAirframe

AERO_MODEL_SIZE = "large"
# Beyond about +-35 deg the wind runs backwards along swept vertical fins in asb.LiftingLine's strip model
# (negative Reynolds number, NaN section polars), which poisons the whole solve.
AOA_SWEEP_DEG = range(-30, 31)
STABILITY_MARGINAL_PCT = 5.0
NEUTRAL_POINT_FIT_HALF_WINDOW_DEG = 4.0
LIFTING_LINE_SPANWISE_RESOLUTION = 10
LIFTING_LINE_CORE_RADIUS_M = 1e-3  # finite vortex core: panels carried to the axis meet there
LIFTING_LINE_TRAILING_DIRECTION = np.array([1.0, 0.0, 0.0])  # asb.LiftingLine default: trailing legs along +x
VLM_SETTING_KEYS = (
    "aoa_start", "aoa_end", "aoa_increment", "airspeed_start", "airspeed_end", "airspeed_increment",
    "altitude_start", "altitude_end", "altitude_increment", "vlm_threads", "vlm_span_res", "vlm_chord_res",
)
NOSE_CNA = 2.0
NOSE_CP_FRACTION = 0.466  # of nose length (tangent ogive)


class OperatingPointError(Exception):
    """The operating point lies outside what the aero model covers."""


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


def panel_sections(panel: Panel, control: ControlSurface | None, tip_first: bool) -> list[asb.WingXSec]:
    """Sections of one panel, split at a split control's span ends. Each moving segment carries an undeflected
    asb.ControlSurface on its first section in the final order (ASB applies a section's control surfaces to the segment
    after it); an all-moving panel is a hinge at the leading edge (effectiveness 1: the deflection adds to the AoA)."""
    fractions, hinge = [0.0, 1.0], None
    if control is not None and control.kind == "split":
        fractions, hinge = sorted({0.0, control.span_start, control.span_end, 1.0}), control.chord_start
    elif control is not None:
        hinge = 0.0
    moving = [hinge is not None and (control.kind != "split" or control.span_start <= a and b <= control.span_end) for a, b in zip(fractions, fractions[1:])]
    sections = [
        asb.WingXSec(xyz_le=[r + (t - r) * f for r, t in zip(panel.root_le, panel.tip_le)], chord=panel.chord_root_m + (panel.chord_tip_m - panel.chord_root_m) * f,
                     twist=panel.incidence_deg, airfoil=asb.Airfoil(panel.airfoil))
        for f in fractions
    ]
    if tip_first:
        sections.reverse()
        moving.reverse()
    for section, moves in zip(sections, moving):
        if moves:
            section.control_surfaces = [asb.ControlSurface(name=f"{panel.surface} {panel.index + 1}", symmetric=True, deflection=0.0, hinge_point=hinge)]
    return sections


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
            # ASB's section normal for root->tip order is (0, -span_z, span_y); flip order to match the panel normal.
            asb_normal = (0.0, -panel.span_dir[2], panel.span_dir[1])
            sections = panel_sections(panel, surface_set.definition.control, tip_first=sum(a * b for a, b in zip(asb_normal, panel.normal)) < 0.0)
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


def carried_to_axis(wing: asb.Wing) -> asb.Wing:
    """Copy of a body-rooted wing with an extra inner section where its span line comes closest to the fuselage axis,
    so the lifting line carries its lift across the body. Sections may run tip first (build_asb_airplane flips them to
    match the panel normal, e.g. a downward fin); the inner section goes on the root end either way."""
    sections = list(wing.xsecs)
    tip_first = np.linalg.norm(sections[0].xyz_le[1:]) > np.linalg.norm(sections[-1].xyz_le[1:])
    root, tip = (sections[-1], sections[0]) if tip_first else (sections[0], sections[-1])
    span = np.array(tip.xyz_le[1:], dtype=float) - np.array(root.xyz_le[1:], dtype=float)
    span /= np.linalg.norm(span)
    root_yz = np.array(root.xyz_le[1:], dtype=float)
    inner = asb.WingXSec(xyz_le=[root.xyz_le[0], *(root_yz - float(root_yz @ span) * span)], chord=root.chord, twist=root.twist, airfoil=root.airfoil)
    return asb.Wing(name=wing.name, symmetric=wing.symmetric, xsecs=[*sections, inner] if tip_first else [inner, *sections])


@dataclass(frozen=True)
class HybridAirplanes:
    """The lifting line's wings and AeroBuildup's bodies, sharing references and the CG moment reference."""
    wings: asb.Airplane
    bodies: asb.Airplane
    owners: tuple[int, ...]  # resolved.surfaces index of each wing
    controls: tuple[tuple[int, int], ...]  # per wing: control number of its panel and of its XZ-mirror copy (0 = none)


def build_hybrid_airplanes(resolved: ResolvedAirframe) -> HybridAirplanes:
    airplane = build_asb_airplane(resolved)
    numbers = {(control.surface, control.panel_index): control.number for control in resolved.controls}
    # airplane.wings holds one wing per panel that is not an XZ-mirror copy, in resolved.surfaces order; a copy mirrors
    # its primary's span direction across XZ.
    owners, controls = [], []
    for index, surface_set in enumerate(resolved.surfaces):
        for panel in surface_set.panels:
            if panel.xz_mirror == "copy":
                continue
            copy_index = None
            if panel.xz_mirror == "primary":
                copy_index = next(other.index for other in surface_set.panels if other.xz_mirror == "copy"
                                  and abs(other.span_dir[1] + panel.span_dir[1]) < 1e-9 and abs(other.span_dir[2] - panel.span_dir[2]) < 1e-9)
            owners.append(index)
            controls.append((numbers.get((surface_set.name, panel.index), 0), numbers.get((surface_set.name, copy_index), 0)))
    wings = [carried_to_axis(wing) if resolved.surfaces[owner].definition.root == "body" else wing for wing, owner in zip(airplane.wings, owners)]
    references = dict(xyz_ref=airplane.xyz_ref, s_ref=airplane.s_ref, c_ref=airplane.c_ref, b_ref=airplane.b_ref)
    return HybridAirplanes(
        wings=asb.Airplane(name=f"{airplane.name} wings", wings=wings, fuselages=[], **references),
        bodies=asb.Airplane(name=f"{airplane.name} bodies", wings=[], fuselages=airplane.fuselages, **references),
        owners=tuple(owners), controls=tuple(controls),
    )


@dataclass(frozen=True)
class LiftingLineMesh:
    """asb.LiftingLine's horseshoe panels (one chordwise panel per spanwise strip), meshed once for every AoA."""
    owners: np.ndarray  # resolved.surfaces index of each panel
    normals: np.ndarray
    areas: np.ndarray
    chords: np.ndarray
    forward: np.ndarray
    centers: np.ndarray  # bound-leg midpoints
    bound_legs: np.ndarray
    airfoil_groups: tuple[tuple[asb.Airfoil, np.ndarray], ...]  # (airfoil, indices of the panels using it)
    controls: np.ndarray  # control number of each panel's moving segment, 0 = fixed
    hinges: dict[int, float]  # hinge chord fraction per control number (0 = all-moving)
    induced: np.ndarray  # (3, panels, panels): velocity at panel i's centre per unit circulation of horseshoe j


@dataclass(frozen=True)
class SurfaceLoads:
    """Lifting-line loads per surface set (rows, resolved.surfaces order) and AoA (columns)."""
    lift_n: np.ndarray
    drag_n: np.ndarray
    pitch_moment_nm: np.ndarray  # about the moment reference (the CG), nose-up positive
    force_body_n: np.ndarray  # (3, AoA): all wings, body axes (x forward, y right, z down)
    moment_body_nm: np.ndarray  # (3, AoA): all wings, about the moment reference, body axes


def build_lifting_line_mesh(wing_plane: asb.Airplane, owners: tuple[int, ...], wing_controls: tuple[tuple[int, int], ...]) -> LiftingLineMesh:
    """The same panels, strip airfoils, control surfaces and induced-velocity kernel as asb.LiftingLine.wing_aerodynamics.
    wing_controls: per wing, the control number of its own panel and of its mirrored side (HybridAirplanes.controls)."""
    corners, airfoils, panel_owners, panel_controls, hinges = [], [], [], [], {}
    for wing, owner, (own_control, mirror_control) in zip(wing_plane.wings, owners, wing_controls):
        wing = wing.subdivide_sections(ratio=LIFTING_LINE_SPANWISE_RESOLUTION, spacing_function=asb_np.cosspace)
        points, faces = wing.mesh_thin_surface(method="quad", chordwise_resolution=1, add_camber=False)
        strip_airfoils = [a.airfoil.blend_with_another_airfoil(airfoil=b.airfoil, blend_fraction=0.5) for a, b in zip(wing.xsecs[:-1], wing.xsecs[1:])]
        strip_moving = [bool(a.control_surfaces) for a in wing.xsecs[:-1]]
        for a in wing.xsecs[:-1]:
            for surface in a.control_surfaces:
                hinges[own_control] = hinges[mirror_control] = surface.hinge_point
        strip_controls = [own_control if moves else 0 for moves in strip_moving]
        if wing.symmetric:
            strip_airfoils *= 2  # mesh_thin_surface lists the right side, then the mirrored left side
            strip_controls += [mirror_control if moves else 0 for moves in strip_moving]
        if len(strip_airfoils) != len(faces):
            raise ValueError(f"lifting line mesh of {wing.name}: {len(faces)} panels but {len(strip_airfoils)} strip airfoils")
        corners.append(points[faces])
        airfoils += strip_airfoils
        panel_owners += [owner] * len(faces)
        panel_controls += strip_controls
    front_left, back_left, back_right, front_right = (np.concatenate([wing_corners[:, i] for wing_corners in corners]) for i in range(4))
    cross = np.cross(front_right - back_left, front_left - back_right)
    cross_norm = np.linalg.norm(cross, axis=1)
    normals = cross / cross_norm[:, None]
    left = 0.75 * front_left + 0.25 * back_left
    right = 0.75 * front_right + 0.25 * back_right
    bound_legs = right - left
    centers = (left + right) / 2.0
    induced = np.array(calculate_induced_velocity_horseshoe(
        x_field=centers[:, :1], y_field=centers[:, 1:2], z_field=centers[:, 2:3],
        x_left=left[None, :, 0], y_left=left[None, :, 1], z_left=left[None, :, 2],
        x_right=right[None, :, 0], y_right=right[None, :, 1], z_right=right[None, :, 2],
        trailing_vortex_direction=LIFTING_LINE_TRAILING_DIRECTION, gamma=1.0, vortex_core_radius=LIFTING_LINE_CORE_RADIUS_M,
    ))
    groups: dict[str, tuple[asb.Airfoil, list[int]]] = {}
    for index, airfoil in enumerate(airfoils):
        groups.setdefault(airfoil.name, (airfoil, []))[1].append(index)
    return LiftingLineMesh(
        owners=np.array(panel_owners), normals=normals, areas=cross_norm / 2.0,
        chords=np.linalg.norm((back_left + back_right) / 2.0 - (front_left + front_right) / 2.0, axis=1),
        forward=np.cross(normals, bound_legs / np.linalg.norm(bound_legs, axis=1)[:, None]),
        centers=centers, bound_legs=bound_legs,
        airfoil_groups=tuple((airfoil, np.array(indices)) for airfoil, indices in groups.values()),
        controls=np.array(panel_controls), hinges={number: hinge for number, hinge in hinges.items() if number},
        induced=induced,
    )


def lifting_line_sweep(
    mesh: LiftingLineMesh, xyz_ref, surface_count: int, atmosphere: asb.Atmosphere, airspeed_m_s: float, aoa_values: np.ndarray,
    deflections: dict[int, float] | None = None,
) -> SurfaceLoads:
    """asb.LiftingLine.run for every AoA at once. Its influence matrix does not depend on AoA (trailing legs along +x),
    so it is built once, and each section polar pass is one NeuralFoil call per airfoil and deflected control over all
    panels and AoAs. Like asb.LiftingLine: section lift linearised with a 2 pi slope at the geometric AoA, one solve for
    the circulation, then section drag and moment at the induced AoA. deflections: degrees per control number
    (positive = TE away from the panel normal); controls not listed are neutral."""
    deflections = deflections or {}
    op_points = [asb.OperatingPoint(atmosphere=atmosphere, velocity=airspeed_m_s, alpha=float(aoa)) for aoa in aoa_values]
    freestream = np.array([op_point.compute_freestream_velocity_geometry_axes() for op_point in op_points])  # (aoa, 3), where the wind goes
    freestream_dir = freestream / np.linalg.norm(freestream, axis=1, keepdims=True)
    alpha_geometric = 90.0 - np.degrees(np.arccos(np.clip(mesh.normals @ freestream_dir.T, -1.0, 1.0)))  # (panels, aoa)
    cos_sweep = -(mesh.forward @ freestream_dir.T)
    mach = op_points[0].mach() * cos_sweep
    reynolds = airspeed_m_s * mesh.chords[:, None] / atmosphere.kinematic_viscosity() * cos_sweep

    def section_polars(alpha: np.ndarray) -> dict[str, np.ndarray]:
        polars = {key: np.empty_like(alpha) for key in ("CL", "CD", "CM")}
        for airfoil, group in mesh.airfoil_groups:
            for number in np.unique(mesh.controls[group]):
                indices = group[mesh.controls[group] == number]
                deflection = deflections.get(int(number), 0.0) if number else 0.0
                surfaces = [asb.ControlSurface(deflection=deflection, hinge_point=mesh.hinges[int(number)])] if deflection else None
                aero = airfoil.get_aero_from_neuralfoil(
                    alpha=alpha[indices].ravel(), Re=reynolds[indices].ravel(), mach=mach[indices].ravel(), model_size=AERO_MODEL_SIZE,
                    control_surfaces=surfaces,
                )
                for key, values in polars.items():
                    values[indices] = np.reshape(aero[key], alpha[indices].shape)
        return polars

    alpha_influence = np.einsum("knm,nk->nm", mesh.induced, mesh.normals) / airspeed_m_s  # rad of AoA per unit circulation
    cl_geometric = section_polars(alpha_geometric)["CL"]
    circulation = np.empty_like(alpha_geometric)
    for column, velocity in enumerate(freestream):
        v_cross_l = np.linalg.norm(np.cross(velocity, mesh.bound_legs), axis=1)
        system = alpha_influence * 2.0 * np.pi - np.diag(2.0 * v_cross_l / (airspeed_m_s * cos_sweep[:, column]) ** 2 / mesh.areas)
        circulation[:, column] = np.linalg.solve(system, -cl_geometric[:, column])
    sections = section_polars(alpha_geometric + np.degrees(alpha_influence @ circulation))

    density = atmosphere.density()
    arms = mesh.centers - np.asarray(xyz_ref, dtype=float)
    bound_legs_yz = mesh.bound_legs * np.array([0.0, 1.0, 1.0])
    membership = (mesh.owners[None, :] == np.arange(surface_count)[:, None]).astype(float)  # (surfaces, panels)
    loads = SurfaceLoads(*(np.empty((surface_count, len(op_points))) for _ in range(3)), *(np.empty((3, len(op_points))) for _ in range(2)))
    for column, op_point in enumerate(op_points):
        velocities = np.einsum("knm,m->nk", mesh.induced, circulation[:, column]) + freestream[column]
        speeds = np.linalg.norm(velocities, axis=1)
        forces = (
            density * np.cross(velocities, mesh.bound_legs) * circulation[:, column, None]
            + 0.5 * density * velocities * (speeds * sections["CD"][:, column] * mesh.areas)[:, None]
        )
        moments = np.cross(arms, forces) + (0.5 * density * speeds ** 2 * sections["CM"][:, column] * mesh.chords ** 2)[:, None] * bound_legs_yz
        force_wind = op_point.convert_axes(*(membership @ forces).T, from_axes="geometry", to_axes="wind")
        moment_body = op_point.convert_axes(*(membership @ moments).T, from_axes="geometry", to_axes="body")
        loads.lift_n[:, column] = -force_wind[2]
        loads.drag_n[:, column] = -force_wind[0]
        loads.pitch_moment_nm[:, column] = moment_body[1]
        loads.force_body_n[:, column] = np.sum(op_point.convert_axes(*(membership @ forces).T, from_axes="geometry", to_axes="body"), axis=1)
        loads.moment_body_nm[:, column] = np.sum(moment_body, axis=1)
    return loads


def analyze_aero(resolved: ResolvedAirframe) -> AeroResult:
    operating_point = resolved.airframe.operating_point
    if not AOA_SWEEP_DEG.start <= operating_point.aoa_deg <= AOA_SWEEP_DEG.stop - 1:
        raise OperatingPointError(f"aero covers AoA {AOA_SWEEP_DEG.start} to {AOA_SWEEP_DEG.stop - 1} deg; operating point AoA is {operating_point.aoa_deg} deg")
    cg_x = resolved.cg_m[0]
    reference = resolved.reference
    atmosphere = asb.Atmosphere(altitude=operating_point.altitude_m)
    hybrid = build_hybrid_airplanes(resolved)

    # The sweep plus the operating point appended at eval_index, in one pass of each model.
    sweep_aoa = np.array(AOA_SWEEP_DEG, dtype=float)
    eval_index = len(sweep_aoa)
    aoa_values = np.append(sweep_aoa, operating_point.aoa_deg)
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s, alpha=aoa_values)
    body = asb.AeroBuildup(airplane=hybrid.bodies, op_point=op_point, model_size=AERO_MODEL_SIZE).run()
    wings = lifting_line_sweep(
        build_lifting_line_mesh(hybrid.wings, hybrid.owners, hybrid.controls), hybrid.wings.xyz_ref, len(resolved.surfaces), atmosphere,
        operating_point.airspeed_m_s, aoa_values,
    )
    dynamic_pressure = float(asb.OperatingPoint(atmosphere=atmosphere, velocity=operating_point.airspeed_m_s).dynamic_pressure())
    force_scale = dynamic_pressure * reference.area_m2
    lift_all = wings.lift_n.sum(axis=0) + np.asarray(body["L"])
    drag_all = wings.drag_n.sum(axis=0) + np.asarray(body["D"])
    moment_all = wings.pitch_moment_nm.sum(axis=0) + np.asarray(body["M_b"][1])
    cl_all = lift_all / force_scale
    cd_all = drag_all / force_scale
    cm_all = moment_all / (force_scale * reference.chord_m)

    with np.errstate(divide="ignore", invalid="ignore"):
        sweep_x_np = cg_x - np.gradient(cm_all[:eval_index], sweep_aoa) / np.gradient(cl_all[:eval_index], sweep_aoa) * reference.chord_m
    sweep = []
    for index, aoa in enumerate(sweep_aoa):
        cm = float(cm_all[index])
        pitch = "nose-up" if cm > 0.0 else "nose-down" if cm < 0.0 else "trim"
        sweep.append(SweepRow(float(aoa), float(cl_all[index]), float(cd_all[index]), cm, float(lift_all[index]), float(sweep_x_np[index]), pitch))

    # NeuralFoil's low-Re section curves are bumpy, so a local derivative jitters by centimetres.
    # The neutral point uses the least-squares dCm/dCL over sweep points within the window around the operating point.
    window = np.abs(sweep_aoa - operating_point.aoa_deg) <= NEUTRAL_POINT_FIT_HALF_WINDOW_DEG
    dcm_dcl = float(np.polyfit(cl_all[:eval_index][window], cm_all[:eval_index][window], 1)[0])

    surfaces = []
    for index, surface_set in enumerate(resolved.surfaces):
        lift = float(wings.lift_n[index, eval_index])
        drag = float(wings.drag_n[index, eval_index])
        area = surface_set.planform_area_m2
        surfaces.append(SurfaceForce(
            name=surface_set.name, airfoil=surface_set.definition.airfoil, area_m2=area,
            cl=lift / (dynamic_pressure * area), cd=drag / (dynamic_pressure * area), lift_n=lift, drag_n=drag,
            x_lift_m=cg_x - float(wings.pitch_moment_nm[index, eval_index]) / lift if abs(lift) > 1e-9 else float("nan"),
        ))

    lift = float(lift_all[eval_index])
    barrowman_cp, barrowman_rows = barrowman(resolved)
    return AeroResult(
        aoa_deg=operating_point.aoa_deg, airspeed_m_s=operating_point.airspeed_m_s, altitude_m=operating_point.altitude_m,
        density_kg_m3=float(atmosphere.density()), dynamic_pressure_pa=dynamic_pressure,
        lift_n=lift, drag_n=float(drag_all[eval_index]), cl=float(cl_all[eval_index]), cd=float(cd_all[eval_index]),
        cm=float(cm_all[eval_index]),
        lift_center_x_m=cg_x - float(moment_all[eval_index]) / lift if abs(lift) > 1e-9 else float("nan"),
        surfaces=tuple(surfaces),
        body_lift_n=float(body["L"][eval_index]), body_drag_n=float(body["D"][eval_index]),
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

