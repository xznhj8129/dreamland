"""
Usage:
from libs.geometry import resolve_airframe
resolved = resolve_airframe(airframe, parts_data)

Derived model of an airframe/2 definition (PLAN.md section 3): body stations, lifting panels with explicit
frame vectors, motors, props and pods, volumes, mass points, CG and aero reference values.
Every consumer (aero, drawing, CAD, JSBSim) reads this and does not re-derive geometry.

Frame: x aft from the nose tip, y right, z up; metres and kilograms.
Panel normal convention: (0, -dir_z, dir_y), sign-flipped on left-hand panels so XZ-mirror pairs share it
(points up on H and X wings); every vertical fin uses (0, -1, 0), so fin incidence makes side force, not roll.
Positive incidence rotates the chord leading-edge-up about the leading edge, towards the normal.
"""

import itertools
import math
from dataclasses import dataclass

import aerosandbox as asb

from libs.airframe import Airframe, AirframeError, Surface
from libs.pointed_shapes import sample_pointed_profile

END_PROFILE_SAMPLES = 14
POD_PROFILE_SAMPLES = 14
PERIMETER_SAMPLES = 72
SPAN_STRIPS = 64
ROOT_CHORD_SAMPLES = 16  # body radius samples under a root chord for the equivalent aero root
PROP_SHAFT_LEN_M = 0.015
MAX_RANGED_MASSES = 12  # CG range is found by enumerating min/max of every ranged mass
STANDARD_GRAVITY_M_S2 = 9.80665
GRAMS_FORCE_PER_NEWTON = 1000.0 / STANDARD_GRAVITY_M_S2

Vec = tuple[float, float, float]


@dataclass(frozen=True)
class BodyStation:
    x_m: float
    width_m: float
    height_m: float


@dataclass(frozen=True)
class Body:
    stations: tuple[BodyStation, ...]
    shape: float
    length_m: float
    nose_end_x_m: float
    tail_start_x_m: float


@dataclass(frozen=True)
class Panel:
    surface: str
    index: int
    xz_mirror: str | None  # "primary", "copy" or None (unpaired fin)
    attach_le: Vec  # true root leading edge, on the body surface
    attach_te: Vec  # true root trailing edge, on the body surface (follows tapering nose/tail cones)
    root_le: Vec  # equivalent aero root: at the mean body radius under the root chord (= attach_le on a straight tube)
    tip_le: Vec
    span_dir: Vec
    normal: Vec
    chord_root_m: float
    chord_tip_m: float
    span_m: float
    incidence_deg: float
    airfoil: str


@dataclass(frozen=True)
class SurfaceSet:
    name: str
    definition: Surface
    panels: tuple[Panel, ...]
    planform_area_m2: float
    mac_m: float
    body_half_extent_m: float  # body radius-like half extent at the root LE station (Barrowman)


@dataclass(frozen=True)
class MotorPlacement:
    name: str
    position: Vec  # motor bell centre
    thrust_dir: Vec  # unit vector of thrust
    prop_center: Vec
    prop_radius_m: float
    bell_diameter_m: float
    bell_length_m: float
    pod_stations: tuple[tuple[float, float], ...]  # (x_m, radius_m) on the motor's axis line, nose to tail


@dataclass(frozen=True)
class MassPoint:
    name: str
    kind: str  # structure, surface, volume, motor, prop
    position: Vec
    mass_kg: tuple[float, float]  # (min, max)
    computed: bool  # True when derived from geometry rather than given


@dataclass(frozen=True)
class Reference:
    basis: str  # "surface:<name>" or "body"
    area_m2: float
    chord_m: float
    span_m: float


@dataclass(frozen=True)
class ResolvedAirframe:
    airframe: Airframe
    body: Body
    surfaces: tuple[SurfaceSet, ...]
    motors: tuple[MotorPlacement, ...]
    mass_points: tuple[MassPoint, ...]
    mass_kg: tuple[float, float]
    cg_m: Vec  # with every ranged mass at the middle of its range
    cg_x_range_m: tuple[float, float]  # forward-most and aft-most over all ranged-mass combinations
    reference: Reference


def trapezoid_mac(chord_root: float, chord_tip: float) -> float:
    return (
        (2.0 / 3.0)
        * ((chord_root * chord_root) + (chord_root * chord_tip) + (chord_tip * chord_tip))
        / (chord_root + chord_tip)
    )


def surface_panel_layout(
    layout: str,
    root_radius: float,
    span: float,
    dihedral_deg: float,
    y_offset: float = 0.0,
    fin_pos: str = "top",
    centerline_root: bool = False,
) -> list[dict]:
    """
    Layout defines the wing configuration:
      H  — 2 horizontal wings, roots positioned by y_offset (-1=bottom, 0=equator, 1=top)
      1  — single fin, position set by fin_pos ("top" or "bottom")
      3  — H pair + single fin at fin_pos
      +  — H pair + top and bottom fins
      X  — 4 wings mirrored about the equator; y_offset sets the root angle on the
           tube (-1=equator, 0=45 deg, 1=top/bottom), dihedral tilts each pair
           away from the equator (45 = symmetric X)

    Wing direction is always horizontal (±Y) for H-type panels.
    Fin direction is always vertical (±Z) for fin-type panels.
    Dihedral tilts H-type wings from horizontal; fins are unaffected.
    """
    d = math.radians(dihedral_deg)
    cos_d = math.cos(d)
    sin_d = math.sin(d)
    oz = max(-root_radius, min(root_radius, y_offset * root_radius))
    oy = math.sqrt(max(0.0, root_radius ** 2 - oz ** 2))
    fin_z = root_radius if fin_pos == "top" else -root_radius
    fin_dz = 1.0 if fin_pos == "top" else -1.0

    # (root_y, root_z, dir_y, dir_z, xz_mirror) — fully explicit, no angles.
    # xz_mirror: "primary" panels have an XZ-plane mirror "copy" in the same set; None = unpaired fin.
    if layout == "1":
        specs = [(0.0, fin_z, 0.0, fin_dz, None)]
    elif layout == "3":
        specs = [(+oy, oz, +cos_d, +sin_d, "primary"), (-oy, oz, -cos_d, +sin_d, "copy"), (0.0, fin_z, 0.0, fin_dz, None)]
    elif layout == "+":
        specs = [(+oy, oz, +cos_d, +sin_d, "primary"), (-oy, oz, -cos_d, +sin_d, "copy"),
                 (0.0, +root_radius, 0.0, +1.0, None), (0.0, -root_radius, 0.0, -1.0, None)]
    elif layout == "X":
        root_ang = math.radians((max(-1.0, min(1.0, y_offset)) + 1.0) * 45.0)
        xy = root_radius * math.cos(root_ang)
        xz = root_radius * math.sin(root_ang)
        specs = [(+xy, +xz, +cos_d, +sin_d, "primary"), (-xy, +xz, -cos_d, +sin_d, "copy"),
                 (+xy, -xz, +cos_d, -sin_d, "primary"), (-xy, -xz, -cos_d, -sin_d, "copy")]
    else:  # "H" and fallback
        specs = [(+oy, oz, +cos_d, +sin_d, "primary"), (-oy, oz, -cos_d, +sin_d, "copy")]

    panels = []
    for index, (ry, rz, dy, dz, xz_mirror) in enumerate(specs):
        norm = math.hypot(dy, dz)
        if norm > 1e-9:
            dy /= norm
            dz /= norm
        if centerline_root:
            ry, rz = 0.0, 0.0
        ty = dy * span if centerline_root else ry + span * dy
        tz = dz * span if centerline_root else rz + span * dz
        panels.append({"index": index, "phase_deg": 0.0, "dihedral_sign": 1.0,
                        "root_y": ry, "root_z": rz, "tip_y": ty, "tip_z": tz,
                        "dir_y": dy, "dir_z": dz, "xz_mirror": xz_mirror})
    return panels


def superellipse_radius(half_width_m: float, half_height_m: float, shape: float, angle_rad: float) -> float:
    cos_term = abs(math.cos(angle_rad)) / half_width_m
    sin_term = abs(math.sin(angle_rad)) / half_height_m
    return (cos_term ** shape + sin_term ** shape) ** (-1.0 / shape)


def superellipse_area(width_m: float, height_m: float, shape: float) -> float:
    return width_m * height_m * math.gamma(1.0 + 1.0 / shape) ** 2 / math.gamma(1.0 + 2.0 / shape)


def superellipse_perimeter(width_m: float, height_m: float, shape: float) -> float:
    if width_m <= 0.0 or height_m <= 0.0:
        return 0.0
    points = []
    for index in range(PERIMETER_SAMPLES):
        angle = 2.0 * math.pi * index / PERIMETER_SAMPLES
        radius = superellipse_radius(width_m / 2.0, height_m / 2.0, shape, angle)
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return sum(math.dist(a, b) for a, b in zip(points, points[1:] + points[:1]))


def build_body(airframe: Airframe) -> Body:
    fuselage = airframe.fuselage
    width, height = fuselage.section.width_m, fuselage.section.height_m
    aspect = height / width
    nose = sample_pointed_profile(fuselage.nose.profile, fuselage.nose.length_m, width / 2.0, END_PROFILE_SAMPLES, fuselage.nose.params)
    tail = sample_pointed_profile(fuselage.tail.profile, fuselage.tail.length_m, width / 2.0, END_PROFILE_SAMPLES, fuselage.tail.params)
    tail_start = fuselage.length_m - fuselage.tail.length_m
    raw = [(x, r) for x, r in nose] + [(tail_start, width / 2.0)] + [(fuselage.length_m - x, r) for x, r in reversed(tail)]
    stations = []
    for x, radius in raw:
        if stations and abs(x - stations[-1].x_m) < 1e-9:
            continue
        stations.append(BodyStation(x_m=x, width_m=2.0 * radius, height_m=2.0 * radius * aspect))
    return Body(
        stations=tuple(stations), shape=fuselage.section.shape, length_m=fuselage.length_m,
        nose_end_x_m=fuselage.nose.length_m, tail_start_x_m=tail_start,
    )


def body_half_extents(body: Body, x_m: float) -> tuple[float, float]:
    stations = body.stations
    if not stations[0].x_m <= x_m <= stations[-1].x_m:
        return 0.0, 0.0
    for a, b in zip(stations, stations[1:]):
        if a.x_m <= x_m <= b.x_m:
            t = (x_m - a.x_m) / (b.x_m - a.x_m)
            return (a.width_m + (b.width_m - a.width_m) * t) / 2.0, (a.height_m + (b.height_m - a.height_m) * t) / 2.0
    raise AssertionError(f"x {x_m} not bracketed by body stations")


def panel_normal(dir_y: float, dir_z: float) -> Vec:
    if abs(dir_y) < 1e-9:
        return (0.0, -1.0, 0.0)
    sign = 1.0 if dir_y > 0.0 else -1.0
    return (0.0, -dir_z * sign, dir_y * sign)


def body_surface_point(body: Body, x_m: float, angle_rad: float) -> tuple[Vec, float]:
    """Point on the body surface at station x and section angle; also returns its radius (0 off the body)."""
    half_w, half_h = body_half_extents(body, x_m)
    radius = superellipse_radius(half_w, half_h, body.shape, angle_rad) if half_w > 0.0 and half_h > 0.0 else 0.0
    return (x_m, radius * math.cos(angle_rad), radius * math.sin(angle_rad)), radius


def build_surface(surface: Surface, body: Body) -> SurfaceSet:
    sweep_len = surface.exposed_span_m * math.tan(math.radians(surface.sweep_le_deg))
    incidence = math.radians(surface.incidence_deg)
    chord_root = surface.chord_root_m
    layout = surface_panel_layout(
        layout=surface.layout, root_radius=1.0, span=surface.exposed_span_m, dihedral_deg=surface.dihedral_deg,
        y_offset=surface.root_offset, fin_pos=surface.fin_position,
    )
    panels = []
    aero_root_radii = []
    for spec in layout:
        angle = math.atan2(spec["root_z"], spec["root_y"])
        dir_y, dir_z = spec["dir_y"], spec["dir_z"]
        normal = panel_normal(dir_y, dir_z)
        attach_le, _ = body_surface_point(body, surface.x_root_le_m, angle)
        tip_le = (attach_le[0] + sweep_len, attach_le[1] + surface.exposed_span_m * dir_y, attach_le[2] + surface.exposed_span_m * dir_z)
        te_surface, _ = body_surface_point(body, surface.x_root_le_m + chord_root * math.cos(incidence), angle)
        attach_te = tuple(point - chord_root * math.sin(incidence) * n for point, n in zip(te_surface, normal))
        radii = [body_surface_point(body, surface.x_root_le_m + chord_root * (k + 0.5) / ROOT_CHORD_SAMPLES, angle)[1] for k in range(ROOT_CHORD_SAMPLES)]
        aero_radius = sum(radii) / len(radii)
        aero_root_radii.append(aero_radius)
        root_le = (surface.x_root_le_m, aero_radius * math.cos(angle), aero_radius * math.sin(angle))
        panels.append(Panel(
            surface=surface.name, index=spec["index"], xz_mirror=spec["xz_mirror"],
            attach_le=attach_le, attach_te=attach_te, root_le=root_le, tip_le=tip_le,
            span_dir=(0.0, dir_y, dir_z), normal=normal,
            chord_root_m=chord_root, chord_tip_m=surface.chord_tip_m,
            span_m=math.dist(root_le[1:], tip_le[1:]),
            incidence_deg=surface.incidence_deg, airfoil=surface.airfoil,
        ))
    return SurfaceSet(
        name=surface.name, definition=surface, panels=tuple(panels),
        planform_area_m2=sum(panel.span_m for panel in panels) * (chord_root + surface.chord_tip_m) / 2.0,
        mac_m=trapezoid_mac(chord_root, surface.chord_tip_m),
        body_half_extent_m=aero_root_radii[0],
    )


def panel_strips(panel: Panel):
    """(chord_m, strip_width_m, chord-midpoint position) per spanwise strip, root to tip."""
    width = panel.span_m / SPAN_STRIPS
    for index in range(SPAN_STRIPS):
        t = (index + 0.5) / SPAN_STRIPS
        chord = panel.chord_root_m + (panel.chord_tip_m - panel.chord_root_m) * t
        le = tuple(r + (p - r) * t for r, p in zip(panel.root_le, panel.tip_le))
        yield chord, width, (le[0] + chord / 2.0, le[1], le[2])


def surface_mass_point(surface_set: SurfaceSet, density_kg_m3: float, infill: float | None) -> MassPoint:
    area_sum = 0.0
    moment = [0.0, 0.0, 0.0]
    for panel in surface_set.panels:
        for chord, width, mid in panel_strips(panel):
            area_sum += chord * width
            moment = [m + chord * width * c for m, c in zip(moment, mid)]
    centroid = tuple(m / area_sum for m in moment)
    definition = surface_set.definition
    if definition.mass_kg is not None:
        return MassPoint(f"surface {surface_set.name}", "surface", centroid, (definition.mass_kg, definition.mass_kg), False)
    unit_section_area = float(asb.Airfoil(definition.airfoil).area())
    cr, ct = definition.chord_root_m, definition.chord_tip_m
    volume = len(surface_set.panels) * unit_section_area * definition.exposed_span_m * (cr * cr + cr * ct + ct * ct) / 3.0
    mass = volume * infill * density_kg_m3
    return MassPoint(f"surface {surface_set.name}", "surface", centroid, (mass, mass), True)


def shell_between(body: Body, x0: float, x1: float, wall_m: float, density_kg_m3: float) -> tuple[float, float]:
    """(mass, centroid x) of the body shell between two stations."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    sections = [(x, *body_half_extents(body, x)) for x in xs]
    mass = 0.0
    moment_x = 0.0
    for (xa, wa, ha), (xb, wb, hb) in zip(sections, sections[1:]):
        perimeter = (superellipse_perimeter(2.0 * wa, 2.0 * ha, body.shape) + superellipse_perimeter(2.0 * wb, 2.0 * hb, body.shape)) / 2.0
        segment = perimeter * math.hypot(xb - xa, wb - wa) * wall_m * density_kg_m3
        mass += segment
        moment_x += segment * (xa + xb) / 2.0
    return mass, moment_x / mass if mass > 0.0 else (x0 + x1) / 2.0


def structure_mass_points(airframe: Airframe, body: Body, density_kg_m3: float) -> list[MassPoint]:
    """One mass point per fuselage section (weighed, or shell computed from geometry) and one per coupler."""
    sections = airframe.sections
    bounds = [0.0, *sections.cuts_m, body.length_m]
    points = []
    for index, (x0, x1) in enumerate(zip(bounds, bounds[1:])):
        computed_mass, centroid_x = shell_between(body, x0, x1, airframe.fuselage.section.wall_radial_m, density_kg_m3)
        weighed = sections.mass_kg[index]
        mass = computed_mass if weighed is None else weighed
        points.append(MassPoint(f"section {index + 1} ({x0 * 1000:.0f}-{x1 * 1000:.0f} mm)", "structure", (centroid_x, 0.0, 0.0), (mass, mass), weighed is None))
    coupler_mass = airframe.fuselage.joints.coupler_mass_kg
    if coupler_mass is not None:
        for index, cut in enumerate(sections.cuts_m):
            points.append(MassPoint(f"coupler {index + 1}", "structure", (cut, 0.0, 0.0), (coupler_mass, coupler_mass), False))
    return points


def build_motors(airframe: Airframe, surfaces: tuple[SurfaceSet, ...]) -> tuple[MotorPlacement, ...]:
    propulsion = airframe.propulsion
    bell_len = propulsion.bell_length_m
    motors = []
    for mount_index, mount in enumerate(propulsion.mounts):
        if mount.on == "surface":
            surface_set = next(item for item in surfaces if item.name == mount.surface)
            positions = [
                (f"{mount.surface} {panel.index + 1}", tuple(r + mount.span_position_m * d + (mount.x_offset_m if axis == 0 else 0.0)
                                                            for axis, (r, d) in enumerate(zip(panel.attach_le, panel.span_dir))))
                for panel in surface_set.panels
            ]
        elif mount.on == "nose":
            positions = [("nose", (-bell_len / 2.0, 0.0, 0.0))]
        else:
            positions = [("tail", (airframe.fuselage.length_m + bell_len / 2.0, 0.0, 0.0))]
        prop_side = -1.0 if mount.direction == "tractor" else 1.0  # tractor prop ahead of the motor (towards the nose)
        for name, position in positions:
            pod_stations = ()
            if mount.pod is not None:
                pod_radius = mount.pod.diameter_m / 2.0
                start = position[0] - prop_side * bell_len / 2.0
                flat_end = start - prop_side * mount.pod.flat_length_m
                stations = [(start, pod_radius), (flat_end, pod_radius)]
                curve = sample_pointed_profile(mount.pod.profile, mount.pod.curve_length_m, pod_radius, POD_PROFILE_SAMPLES)
                stations += [(flat_end - prop_side * (mount.pod.curve_length_m - x), r) for x, r in curve[:-1]]
                pod_stations = tuple(sorted(stations))
            motors.append(MotorPlacement(
                name=f"motor {mount_index + 1} {name}", position=position, thrust_dir=(-1.0, 0.0, 0.0),
                prop_center=(position[0] + prop_side * (bell_len / 2.0 + PROP_SHAFT_LEN_M), position[1], position[2]),
                prop_radius_m=propulsion.prop_diameter_m / 2.0, bell_diameter_m=propulsion.bell_diameter_m,
                bell_length_m=bell_len, pod_stations=pod_stations,
            ))
    return tuple(motors)


def center_of_mass(points: tuple[MassPoint, ...], masses: list[float]) -> Vec:
    total = sum(masses)
    return tuple(sum(m * p.position[axis] for m, p in zip(masses, points)) / total for axis in range(3))


def build_reference(surfaces: tuple[SurfaceSet, ...], body: Body) -> Reference:
    def horizontal_area(surface_set: SurfaceSet) -> float:
        return sum(panel.span_dir[1] ** 2 for panel in surface_set.panels) * surface_set.planform_area_m2 / len(surface_set.panels)

    candidates = [item for item in surfaces if horizontal_area(item) > 1e-12]
    if not candidates:
        widest = max(body.stations, key=lambda station: station.width_m)
        return Reference("body", superellipse_area(widest.width_m, widest.height_m, body.shape), widest.width_m, widest.width_m)
    wing = max(candidates, key=horizontal_area)
    tips_y = [panel.tip_le[1] for panel in wing.panels]
    return Reference(f"surface:{wing.name}", wing.planform_area_m2, wing.mac_m, max(tips_y) - min(tips_y))


def volume_checks(airframe: Airframe, body: Body) -> list[str]:
    wall = airframe.fuselage.section.wall_radial_m
    boxes = []
    problems = []
    for volume in airframe.volumes:
        if volume.shape == "point":
            continue
        if volume.shape == "box":
            half = (volume.size_m[0] / 2.0, volume.size_m[1] / 2.0, volume.size_m[2] / 2.0)
        elif volume.shape == "cylinder":
            radius = volume.diameter_m / 2.0
            half = {"x": (volume.length_m / 2.0, radius, radius), "y": (radius, volume.length_m / 2.0, radius), "z": (radius, radius, volume.length_m / 2.0)}[volume.axis]
        else:
            half = (volume.length_m / 2.0, 0.0, 0.0)
        x0, x1 = volume.x_m - half[0], volume.x_m + half[0]
        for x in (x0, volume.x_m, x1):
            half_w, half_h = body_half_extents(body, x)
            inner_w, inner_h = half_w - wall, half_h - wall
            if inner_w <= 0.0 or inner_h <= 0.0:
                problems.append(f"volume {volume.name!r} reaches x {x:.3f} m, outside the fuselage interior")
                break
            corners = [(volume.y_m + sy * half[1], volume.z_m + sz * half[2]) for sy in (-1, 1) for sz in (-1, 1)]
            excess = max(math.hypot(*c) - superellipse_radius(inner_w, inner_h, body.shape, math.atan2(c[1], c[0])) for c in corners)
            if excess > 1e-9:
                problems.append(f"volume {volume.name!r} corners reach {excess * 1000.0:.1f} mm into the wall at x {x:.3f} m")
                break
        if volume.shape == "bore":
            half = (half[0], math.inf, math.inf)  # a bore fills the whole internal section
        boxes.append((volume.name, (x0, volume.y_m - half[1], volume.z_m - half[2]), (x1, volume.y_m + half[1], volume.z_m + half[2])))
    for (name_a, lo_a, hi_a), (name_b, lo_b, hi_b) in itertools.combinations(boxes, 2):
        if all(lo_a[axis] < hi_b[axis] - 1e-9 and lo_b[axis] < hi_a[axis] - 1e-9 for axis in range(3)):
            problems.append(f"volumes {name_a!r} and {name_b!r} overlap")
    return problems


def resolve_airframe(airframe: Airframe, parts_data: dict) -> tuple[ResolvedAirframe, list[str]]:
    """Returns the resolved model and a list of layout problems (volumes outside the body or overlapping)."""
    density = parts_data["material"][airframe.print.material]["density_kg_m3"]
    body = build_body(airframe)
    surfaces = tuple(build_surface(surface, body) for surface in airframe.surfaces)
    motors = build_motors(airframe, surfaces)

    points = structure_mass_points(airframe, body, density)
    points += [surface_mass_point(surface_set, density, airframe.print.surface_infill) for surface_set in surfaces]
    points += [MassPoint(f"volume {volume.name}", "volume", (volume.x_m, volume.y_m, volume.z_m), volume.mass_kg, False) for volume in airframe.volumes]
    for motor in motors:
        points.append(MassPoint(motor.name, "motor", motor.position, (airframe.propulsion.motor_mass_kg,) * 2, False))
        points.append(MassPoint(f"prop {motor.name}", "prop", motor.prop_center, (airframe.propulsion.prop_mass_kg,) * 2, False))
    points = tuple(points)

    ranged = [index for index, point in enumerate(points) if point.mass_kg[0] != point.mass_kg[1]]
    if len(ranged) > MAX_RANGED_MASSES:
        raise AirframeError(f"{len(ranged)} volumes have a mass range; at most {MAX_RANGED_MASSES} are supported")
    nominal = [(point.mass_kg[0] + point.mass_kg[1]) / 2.0 for point in points]
    cg_xs = []
    for choice in itertools.product((0, 1), repeat=len(ranged)):
        masses = [point.mass_kg[0] if index not in ranged else point.mass_kg[choice[ranged.index(index)]] for index, point in enumerate(points)]
        cg_xs.append(center_of_mass(points, masses)[0])

    resolved = ResolvedAirframe(
        airframe=airframe, body=body, surfaces=surfaces, motors=motors, mass_points=points,
        mass_kg=(sum(point.mass_kg[0] for point in points), sum(point.mass_kg[1] for point in points)),
        cg_m=center_of_mass(points, nominal), cg_x_range_m=(min(cg_xs), max(cg_xs)),
        reference=build_reference(surfaces, body),
    )
    return resolved, volume_checks(airframe, body)
