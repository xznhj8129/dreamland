"""
Usage:
from libs.geometry import resolve_airframe
resolved = resolve_airframe(airframe, parts_data)

Derived model of an airframe/2 definition (PLAN.md section 3): body stations, lifting panels with explicit
frame vectors, motors, props and pods, rocket motors, volumes, mass points, CG and aero reference values. Mass and
CG are at liftoff (rocket propellant loaded); dry_mass_properties leaves the propellant out.
Every consumer (aero, drawing, CAD, JSBSim) reads this and does not re-derive geometry.

Frame: x aft from the nose tip, y right, z up; metres and kilograms.
Panel normal convention: incidence_mode symmetric: (0, -dir_z, dir_y), sign-flipped on left-hand panels so XZ-mirror
pairs share it (points up on H and X wings); every vertical fin uses (0, -1, 0), so fin incidence makes side force, not
roll. incidence_mode cant: (0, -dir_z, dir_y) on every panel, the same turn about the body axis, so incidence makes roll;
its panels have no XZ mirror.
Positive incidence rotates the chord leading-edge-up about the leading edge, towards the normal.
"""

import bisect
import itertools
import math
from dataclasses import dataclass

import aerosandbox as asb

from libs.airframe import (
    AXES, BOX_SHAPES, Airframe, AirframeError, AccessEntry, CarryThrough, ControlSurface, MotorMountHoles, Mount, PlacedPart,
    Reinforcement, Surface, Volume, orientation_rows,
)
from libs.pointed_shapes import sample_pointed_profile
from libs.rocketmotors import RocketMotor

END_PROFILE_SAMPLES = 14
BODY_LOFT_SAMPLES = 32  # sampled cubic transitions for drawing, aero and CAD
ROD_ENVELOPE_SAMPLES = 16
POD_PROFILE_SAMPLES = 14
PERIMETER_SAMPLES = 72
SPAN_STRIPS = 64
ROOT_CHORD_SAMPLES = 16  # body radius samples under a root chord for the equivalent aero root
PROP_SHAFT_LEN_M = 0.015
CARRY_THROUGH_FLARE_SAMPLES = 24  # points along a carry-through flare or wall fillet
MAX_RANGED_MASSES = 12  # CG range is found by enumerating min/max of every ranged mass
STANDARD_GRAVITY_M_S2 = 9.80665
GRAMS_FORCE_PER_NEWTON = 1000.0 / STANDARD_GRAVITY_M_S2

Vec = tuple[float, float, float]
# Inertia tensor components (Ixx, Iyy, Izz, Ixy, Ixz, Iyz), products as tensor entries (-sum m a b), in this frame.
Inertia = tuple[float, float, float, float, float, float]
NO_INERTIA: Inertia = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def dot(a: Vec, b: Vec) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec, b: Vec) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(a: Vec) -> Vec:
    length = math.sqrt(dot(a, a))
    return (a[0] / length, a[1] / length, a[2] / length)


def _rotate_about_axis(point: Vec, axis: Vec, angle_rad: float, position: Vec) -> Vec:
    """position turned angle_rad about the line through point along the unit axis (Rodrigues)."""
    relative = tuple(value - base for value, base in zip(position, point))
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    turned = cross(axis, relative)
    along = dot(axis, relative)
    return tuple(base + cosine * value + sine * side + (1.0 - cosine) * along * direction
                 for base, value, side, direction in zip(point, relative, turned, axis))


def resolve_access(access: tuple[AccessEntry, ...]) -> tuple["AccessInstance", ...]:
    """One instance per access entry: the seed frame from its centre and orientation (Rz Ry Rx), then, with a radial
    block, count - 1 copies evenly spaced a full circle about the axis line, position and local axes turned together."""
    axis_vectors = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
    instances = []
    for index, entry in enumerate(access):
        rows = orientation_rows(*entry.orientation_deg)
        axes = tuple(tuple(row[column] for row in rows) for column in range(3))
        if entry.radial is None:
            instances.append(AccessInstance(index=index, copy=0, count=1, instance_id=entry.name,
                                            center=entry.position_m, axes=axes, entry=entry))
            continue
        radial = entry.radial
        axis = axis_vectors[radial.axis]
        for copy in range(radial.count):
            angle = 2.0 * math.pi * copy / radial.count
            center = _rotate_about_axis(radial.position_m, axis, angle, entry.position_m)
            turned = tuple(_rotate_about_axis((0.0, 0.0, 0.0), axis, angle, direction) for direction in axes)  # a direction rotates about the origin
            instance_id = entry.name if radial.count == 1 else f"{entry.name} {copy + 1}/{radial.count}"
            instances.append(AccessInstance(index=index, copy=copy, count=radial.count, instance_id=instance_id,
                                            center=center, axes=turned, entry=entry))
    return tuple(instances)


@dataclass(frozen=True)
class BodyStation:
    x_m: float
    width_m: float
    height_m: float
    shape: float


@dataclass(frozen=True)
class Body:
    stations: tuple[BodyStation, ...]
    length_m: float
    nose_end_x_m: float
    tail_start_x_m: float
    loft_stations: tuple[BodyStation, ...]
    taper_stations: tuple[BodyStation, ...]  # normalized nose/tail widths and heights


@dataclass(frozen=True)
class Panel:
    surface: str
    index: int
    xz_mirror: str | None  # "primary", "copy" or None (unpaired fin)
    attach_le: Vec  # true root leading edge, on the body surface
    attach_te: Vec  # true root trailing edge, on the body surface (follows tapering nose/tail cones)
    root_le: Vec  # equivalent aero root: at the mean body radius under the root chord (= attach_le on a straight tube)
    tip_le: Vec
    tip_te: Vec
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
    body_half_extent_m: float  # mean body radius under the root chord (Barrowman)
    exposed_x_root_le_m: float  # trapezoid outside the body, for Barrowman; where the set meets the body, which places it
                                # (its mass, spars, motors and CAD) in a fuselage section
    exposed_span_m: float
    exposed_chord_root_m: float
    exposed_sweep_len_m: float


@dataclass(frozen=True)
class SparLine:
    """One spar rod (Surface.spars) as a straight line, from its inner end out to the panel tip."""
    surface: str
    panel_index: int
    start: Vec  # the rod's inner end
    end: Vec
    diameter_m: float
    through: bool  # runs on in line into the opposite panel (else it stops where it meets another)
    joined_hole_start: Vec  # where its hole starts inside a carry-through: joining the holes it meets with no sliver between
                            # them (a through pair overlapping half a hole, a spar that stops at a through spar on its
                            # line, two that stop at each other mitred like the arms)


@dataclass(frozen=True)
class CarryThroughArm:
    """One carry-through arm: from start (where it meets the other arms, mitred past that point) along direction to the
    inner wall, with its profiles in its own frame (across, along)."""
    surface: str
    panel_index: int
    start: Vec
    direction: Vec  # along the arm (the panel's span direction)
    across: Vec  # across the arm, in the stem width (the panel normal's part across the span)
    wall_distance_m: float  # from start to the inner wall along direction
    wall_half_extents_m: tuple[float, float]  # the inner wall at the arm's station (half width, half height; the body's shape)
    section_shape: float  # the body's superellipse exponent
    meet_m: float  # from start to where it meets the other arms (the mitre)
    side_profile: tuple[tuple[float, float], ...]  # (x offset from start, distance along direction), closed
    front_profile: tuple[tuple[float, float], ...]  # (offset across, distance along direction), closed


@dataclass(frozen=True)
class ControlPlacement:
    number: int  # 1.. front to back, then clockwise from the top as seen from behind (the front view)
    surface: str
    panel_index: int
    servo: str
    outline: tuple[Vec, Vec, Vec, Vec]  # moving part (after the gap): root hinge (LE when all-moving), root TE, outer TE, outer hinge
    hinge: tuple[Vec, Vec]  # hinge rod line: the opening hinge line shifted gap_m/2 aft (split), or the pivot line (all_moving)
    axis: Vec  # unit hinge direction: a right-hand rotation about it is a positive deflection (TE away from the normal)
    opening: tuple[Vec, Vec, Vec, Vec]  # the region cut out of the fixed panel (on the panel surface)
    moving: tuple[Vec, Vec, Vec, Vec]  # the moving surface region after the gap (on the panel surface)
    gap_m: float  # material removed between the fixed and the moving surface
    rod_diameter_m: float | None  # the pivot rod's outer diameter; None = no rod
    span_length_m: float  # moving span after the end gaps, measured along the panel span
    hinge_length_m: float


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
    spin: str  # "cw" / "ccw" seen from where the thrust points (from above in hover)
    mount_x_m: float  # surface root LE, or the end mount face clamped to the body; assigns attached mass to a section
    panel: Panel | None  # the surface panel it sits on; None on the nose or tail
    mount_holes: MotorMountHoles | None
    mount_clearance_m: float  # Mount.clearance_m: the mount clearance cut is the bell diameter plus this
    mount_face: Vec
    mount_inward: Vec  # physical bell and pod axes are X


@dataclass(frozen=True)
class RocketPlacement:
    """A solid rocket motor loaded into a rocket mount: its casing runs forward from the nozzle end along the thrust
    axis. A surface mount puts one in every panel's position."""
    name: str  # "rocket <mount number> <tail | surface panel>"
    mount: int  # mount number, 1-based
    motor: RocketMotor
    nozzle: Vec  # centre of the aft (nozzle) end, where the thrust acts
    thrust_dir: Vec  # unit vector of thrust
    center: Vec  # casing centre (the propellant's too: a grain along the casing)
    mount_x_m: float  # surface root LE, or the tail face clamped to the body; assigns attached mass to a section
    panel: Panel | None  # the surface panel it sits on; None on the tail


ROCKET_PROPELLANT_KIND = "rocket propellant"  # MassPoint.kind of the propellant: it burns off in flight


@dataclass(frozen=True)
class AccessInstance:
    """One placed Access seed or radial copy, resolved once for every consumer. center is the profile centre and
    axes the local X (length), Y (width) and Z (cut direction) unit vectors in the aircraft frame. index is the
    original airframe.access index; copy 0 is the seed and 1.. its radial copies (count includes the seed);
    instance_id is deterministic (the entry name alone, or "<name> <copy + 1>/<count>" with a pattern)."""
    index: int
    copy: int
    count: int
    instance_id: str
    center: Vec
    axes: tuple[Vec, Vec, Vec]
    entry: AccessEntry

    @property
    def cut_direction(self) -> Vec:
        return self.axes[2]


@dataclass(frozen=True)
class MassPoint:
    name: str
    kind: str  # structure, reinforcement, surface, volume, part, motor, prop, rocket casing, ROCKET_PROPELLANT_KIND
    position: Vec
    mass_kg: tuple[float, float]  # (min, max)
    computed: bool  # True when derived from geometry rather than given
    estimate_kg: float | None  # mass estimated from geometry and print settings, shown next to a weighed value; None = no estimate
    inertia_per_kg: Inertia  # about the point's own centroid, from its shape
    attached_x_m: float | None = None  # where it is attached to the fuselage (surface set: where it meets the body, its exposed root LE; motor and prop: their mount), which places it in a fuselage section; None = its own x
    exact: bool = False  # estimate_kg, centroid and inertia measured on the built CAD solids (ExactStructure), not the analytic estimate


@dataclass(frozen=True)
class ExactPiece:
    """Printed material measured on the built CAD solids: its mass, centroid and inertia about it per kg."""
    mass_kg: float
    position: Vec
    inertia_per_kg: Inertia


@dataclass(frozen=True)
class ExactStructure:
    """The printed structure measured on the built CAD solids (libs.computed_geometry), by the estimate each piece
    replaces: per fuselage section (its own material with its carry-through and hatch pieces), per surface set (its
    roots in the sections, detached segments and controls), per coupler (by cut index). resolve_airframe applies it
    to the design it was built from; weighed masses stay authoritative."""
    sections: tuple[ExactPiece, ...]
    surfaces: dict[str, ExactPiece]
    couplers: dict[int, ExactPiece]


@dataclass(frozen=True)
class SectionMass:
    """One fuselage section's masses, for the editor and the report."""
    x0_m: float
    x1_m: float
    solid: bool
    empty_estimate_kg: float | None  # shell or solid less volumes, from the print settings; None = no bulk density for a weighed solid section
    empty_kg: float  # weighed or the estimate, as used
    complete_itemised_kg: tuple[float, float]  # everything positioned in the section, itemised (min, max)
    complete_kg: float | None  # weighed; replaces the itemised points; None = itemised


@dataclass(frozen=True)
class Reference:
    basis: str  # "surface:<name>" or "body"
    area_m2: float
    chord_m: float
    span_m: float


@dataclass(frozen=True)
class WireRoutes:
    """High-current wiring as drawn and as modelled, from the battery part to the ESC part along the axes (x, then y,
    then z), and from the ESC part to each motor (motor order): to a motor on a surface panel inside the panel (along x
    to the panel root's mid-chord station, out to the root, along the mid-chord to the motor's span station, then to the
    motor); to a nose or tail motor along the axes."""
    battery: tuple[Vec, ...]
    motors: tuple[tuple[Vec, ...], ...]


@dataclass(frozen=True)
class AccessGeometry:
    """Resolved physical body and access frames, available before mass calculation."""
    airframe: Airframe
    body: Body
    access: tuple[AccessInstance, ...]


@dataclass(frozen=True)
class ResolvedAirframe(AccessGeometry):
    surfaces: tuple[SurfaceSet, ...]
    motors: tuple[MotorPlacement, ...]
    rockets: tuple[RocketPlacement, ...]
    controls: tuple[ControlPlacement, ...]
    spars: tuple[SparLine, ...]
    carry_through: tuple[CarryThroughArm, ...]
    mass_points: tuple[MassPoint, ...]
    sections: tuple[SectionMass, ...]
    mass_kg: tuple[float, float]  # at liftoff: rocket propellant loaded
    cg_m: Vec  # with every ranged mass at the middle of its range
    cg_x_range_m: tuple[float, float]  # forward-most and aft-most over all ranged-mass combinations
    inertia_kg_m2: Inertia  # about cg_m, ranged masses at the middle of their range
    reference: Reference
    wires: WireRoutes | None  # None when not powered


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
    vertical_offset: float = 0.0,
    fin_pos: str = "top",
    centerline_root: bool = False,
) -> list[dict]:
    """
    Layout defines the wing configuration:
      H  — 2 horizontal wings, roots positioned by vertical_offset (-1=bottom, 0=equator, 1=top)
      1  — single fin, position set by fin_pos ("top" or "bottom")
      3  — H pair + single fin at fin_pos
      +  — H pair + top and bottom fins
      X  — 4 wings mirrored about the equator; vertical_offset sets the root angle on the
           tube (-1=equator, 0=45 deg, 1=top/bottom), dihedral tilts each pair
           away from the equator (45 = symmetric X)

    Wing direction is always horizontal (±Y) for H-type panels.
    Fin direction is always vertical (±Z) for fin-type panels.
    Dihedral tilts H-type wings from horizontal; fins are unaffected.
    """
    d = math.radians(dihedral_deg)
    cos_d = math.cos(d)
    sin_d = math.sin(d)
    oz = max(-root_radius, min(root_radius, vertical_offset * root_radius))
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
        root_ang = math.radians((max(-1.0, min(1.0, vertical_offset)) + 1.0) * 45.0)
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


def _interpolate_section(stations: tuple[BodyStation, ...], x_m: float, smooth: bool = False) -> BodyStation:
    """Interpolate section parameters; cubic easing has zero slope at each loft control."""
    if x_m <= stations[0].x_m:
        return BodyStation(x_m, stations[0].width_m, stations[0].height_m, stations[0].shape)
    if x_m >= stations[-1].x_m:
        return BodyStation(x_m, stations[-1].width_m, stations[-1].height_m, stations[-1].shape)
    for a, b in zip(stations, stations[1:]):
        if a.x_m <= x_m <= b.x_m:
            t = (x_m - a.x_m) / (b.x_m - a.x_m)
            if smooth:
                t = t * t * (3.0 - 2.0 * t)
            return BodyStation(x_m, a.width_m + (b.width_m - a.width_m) * t,
                               a.height_m + (b.height_m - a.height_m) * t, a.shape + (b.shape - a.shape) * t)
    raise AssertionError(f"x {x_m} not bracketed by body stations")


def _loft_section_at(loft: tuple[BodyStation, ...], taper: tuple[BodyStation, ...], x_m: float) -> BodyStation:
    section = _interpolate_section(loft, x_m, smooth=True)
    if not taper[0].x_m <= x_m <= taper[-1].x_m:
        return BodyStation(x_m, 0.0, 0.0, section.shape)
    factor = _interpolate_section(taper, x_m).width_m
    return BodyStation(x_m, section.width_m * factor, section.height_m * factor, section.shape)


def build_body(airframe: Airframe, end_samples: int = END_PROFILE_SAMPLES) -> Body:
    """Shared loft: independent shape controls, smooth cubic transitions, existing nose/tail taper."""
    fuselage = airframe.fuselage
    loft = tuple(BodyStation(guide.x_m, section.width_m, section.height_m, section.shape)
                 for guide in fuselage.loft_stations
                 for section in (guide.section.resolve(fuselage.section),))
    if not loft:  # An empty guide list explicitly means the global constant section.
        section = fuselage.section
        loft = (BodyStation(0.0, section.width_m, section.height_m, section.shape),)
    nose_end, tail_start = fuselage.nose.length_m, fuselage.length_m - fuselage.tail.length_m
    nose_radius = _interpolate_section(loft, nose_end, smooth=True).width_m / 2.0
    tail_radius = _interpolate_section(loft, tail_start, smooth=True).width_m / 2.0
    nose = sample_pointed_profile(fuselage.nose.profile, nose_end, nose_radius, end_samples, fuselage.nose.params)
    tail = sample_pointed_profile(fuselage.tail.profile, fuselage.tail.length_m, tail_radius, end_samples, fuselage.tail.params)
    factors = {x: r / nose_radius for x, r in nose}
    factors[tail_start] = 1.0
    factors.update({fuselage.length_m - x: r / tail_radius for x, r in reversed(tail)})
    taper = tuple(BodyStation(x, factor, factor, fuselage.section.shape) for x, factor in sorted(factors.items()))
    xs = set(factors)
    for a, b in zip(loft, loft[1:]):
        if (a.width_m, a.height_m, a.shape) != (b.width_m, b.height_m, b.shape):
            xs.update(a.x_m + (b.x_m - a.x_m) * k / BODY_LOFT_SAMPLES for k in range(BODY_LOFT_SAMPLES + 1))
    stations = tuple(_loft_section_at(loft, taper, x) for x in sorted(xs))
    return Body(stations, fuselage.length_m, nose_end, tail_start, loft, taper)


def loft_section_at(body: Body, x_m: float) -> BodyStation:
    """Untapered section at x: the profile edited by independent loft guides."""
    return _interpolate_section(body.loft_stations, x_m, smooth=True)


def body_section_at(body: Body, x_m: float) -> BodyStation:
    """Exact cubic loft envelope at x, multiplied by the sampled nose/tail taper."""
    return _loft_section_at(body.loft_stations, body.taper_stations, x_m)


def body_half_extents(body: Body, x_m: float) -> tuple[float, float]:
    section = body_section_at(body, x_m)
    return section.width_m / 2.0, section.height_m / 2.0


def panel_normal(dir_y: float, dir_z: float) -> Vec:
    if abs(dir_y) < 1e-9:
        return (0.0, -1.0, 0.0)
    sign = 1.0 if dir_y > 0.0 else -1.0
    return (0.0, -dir_z * sign, dir_y * sign)


def body_surface_point(body: Body, x_m: float, angle_rad: float) -> tuple[Vec, float]:
    """Point on the body surface at station x and section angle; also returns its radius (0 off the body)."""
    half_w, half_h = body_half_extents(body, x_m)
    radius = superellipse_radius(half_w, half_h, body_section_at(body, x_m).shape, angle_rad) if half_w > 0.0 and half_h > 0.0 else 0.0
    return (x_m, radius * math.cos(angle_rad), radius * math.sin(angle_rad)), radius


def build_surface(surface: Surface, body: Body) -> SurfaceSet:
    """Panels for one surface set. root "body": the root edge lies on the body surface (following tapering cones) and
    the aero root sits at the mean body radius under the root chord. root "axis": the panels start on the fuselage
    axis and meet in the centre (e.g. delta tail fins); the body hides their inner part."""
    on_axis = surface.root == "axis"
    sweep_len = surface.span_m * math.tan(math.radians(surface.sweep_le_deg))
    incidence = math.radians(surface.incidence_deg)
    chord_root = surface.chord_root_m
    layout = surface_panel_layout(
        layout=surface.layout, root_radius=1.0, span=surface.span_m, dihedral_deg=surface.dihedral_deg,
        vertical_offset=surface.vertical_offset, fin_pos=surface.fin_position, centerline_root=on_axis,
    )
    panels = []
    mean_radii = []
    for spec in layout:
        dir_y, dir_z = spec["dir_y"], spec["dir_z"]
        angle = math.atan2(dir_z, dir_y) if on_axis else math.atan2(spec["root_z"], spec["root_y"])
        cant = surface.incidence_mode == "cant"
        normal = (0.0, -dir_z, dir_y) if cant else panel_normal(dir_y, dir_z)
        radii = [body_surface_point(body, surface.x_root_le_m + chord_root * (k + 0.5) / ROOT_CHORD_SAMPLES, angle)[1] for k in range(ROOT_CHORD_SAMPLES)]
        mean_radius = sum(radii) / len(radii)
        mean_radii.append(mean_radius)
        te_x = surface.x_root_le_m + chord_root * math.cos(incidence)
        if on_axis:
            attach_le = (surface.x_root_le_m, 0.0, 0.0)
            te_point = (te_x, 0.0, 0.0)
            root_le = attach_le
        else:
            attach_le, _ = body_surface_point(body, surface.x_root_le_m, angle)
            te_point, _ = body_surface_point(body, te_x, angle)
            root_le = (surface.x_root_le_m, mean_radius * math.cos(angle), mean_radius * math.sin(angle))
        attach_te = tuple(point - chord_root * math.sin(incidence) * n for point, n in zip(te_point, normal))
        tip_le = (attach_le[0] + sweep_len, attach_le[1] + surface.span_m * dir_y, attach_le[2] + surface.span_m * dir_z)
        tip_te = tuple(le + surface.chord_tip_m * ((math.cos(incidence) if axis == 0 else 0.0) - math.sin(incidence) * n)
                       for axis, (le, n) in enumerate(zip(tip_le, normal)))
        panels.append(Panel(
            surface=surface.name, index=spec["index"], xz_mirror=None if cant else spec["xz_mirror"],
            attach_le=attach_le, attach_te=attach_te, root_le=root_le, tip_le=tip_le, tip_te=tip_te,
            span_dir=(0.0, dir_y, dir_z), normal=normal,
            chord_root_m=chord_root, chord_tip_m=surface.chord_tip_m,
            span_m=math.dist(root_le[1:], tip_le[1:]),
            incidence_deg=surface.incidence_deg, airfoil=surface.airfoil,
        ))
    # Exposed trapezoid: an axis-rooted panel is cut at the mean body radius; a body-rooted panel is all exposed.
    hidden = mean_radii[0] / surface.span_m if on_axis else 0.0
    return SurfaceSet(
        name=surface.name, definition=surface, panels=tuple(panels),
        planform_area_m2=sum(panel.span_m for panel in panels) * (chord_root + surface.chord_tip_m) / 2.0,
        mac_m=trapezoid_mac(chord_root, surface.chord_tip_m),
        body_half_extent_m=mean_radii[0],
        exposed_x_root_le_m=surface.x_root_le_m + sweep_len * hidden,
        exposed_span_m=panels[0].span_m * (1.0 - hidden),
        exposed_chord_root_m=chord_root + (surface.chord_tip_m - chord_root) * hidden,
        exposed_sweep_len_m=sweep_len * (1.0 - hidden),
    )


def panel_point(panel: Panel, span_fraction: float, chord_fraction: float) -> Vec:
    """Point on the panel's true outline: span fraction from the root edge, chord fraction from the leading edge."""
    le = tuple(a + (b - a) * span_fraction for a, b in zip(panel.attach_le, panel.tip_le))
    te = tuple(a + (b - a) * span_fraction for a, b in zip(panel.attach_te, panel.tip_te))
    return tuple(a + (b - a) * chord_fraction for a, b in zip(le, te))


def control_hinge_point(panel: Panel, control: ControlSurface, span_fraction: float) -> Vec:
    """The physical hinge line, anchored inboard; alignment removes its chordwise sweep."""
    fraction = control.chord_start if control.kind == "split" else control.pivot
    if not control.align_hinge_vertically:
        return panel_point(panel, span_fraction, fraction)
    anchor = panel_point(panel, control.span_start, fraction)
    span_m = dot(tuple(b - a for a, b in zip(panel.attach_le, panel.tip_le)), panel.span_dir)
    return tuple(value + (span_fraction - control.span_start) * span_m * d for value, d in zip(anchor, panel.span_dir))


def control_hinge_fraction(panel: Panel, control: ControlSurface, span_fraction: float) -> float:
    """The hinge line's fraction of the local chord at a span station: the stored chord_start (split) / pivot
    (all_moving) while the hinge follows the panel's chord lines; with align_hinge_vertically the chord fraction
    where the hinge anchored at (span_start, chord_start|pivot) runs straight along panel.span_dir projected onto
    the local chord (no chordwise sweep; incidence, dihedral and the panel's own sweep act through the panel's
    frame vectors). A projection off the panel within the control's span is a ValueError, not a silent clamp."""
    anchored = control.chord_start if control.kind == "split" else control.pivot
    if not control.align_hinge_vertically:
        return anchored
    hinge_point = control_hinge_point(panel, control, span_fraction)
    leading_edge = panel_point(panel, span_fraction, 0.0)
    trailing_edge = panel_point(panel, span_fraction, 1.0)
    chord_m = dot(tuple(t - l for t, l in zip(trailing_edge, leading_edge)), chord_direction(panel))  # noqa: E741 (renaming changes the JSBSim export fingerprint)
    fraction = dot(tuple(p - l for p, l in zip(hinge_point, leading_edge)), chord_direction(panel)) / chord_m  # noqa: E741
    span_end = control.span_end if control.kind == "split" else 1.0
    if control.span_start <= span_fraction <= span_end and not 0.0 <= fraction <= 1.0:
        raise AirframeError(
            f"surface {panel.surface!r} panel {panel.index + 1}: the aligned hinge leaves the panel at span fraction "
            f"{span_fraction:.4g} (local chord fraction {fraction:.4g}); reduce the surface sweep or taper, or bring the control's span inboard"
        )
    return fraction


def control_cut_frames(panel: Panel, control: ControlSurface) -> dict:
    """Plane frames for cutting one panel's control: hinge point+normal perpendicular to the hinge for split, span
    start/end points+normal (panel.span_dir), the gap, and the rod endpoints (m), shifted rod_x_offset_m aft
    along the chord_direction and rod_y_offset_m along the panel normal. The hinge edge follows the chord lines,
    or the aligned line from control_hinge_fraction when align_hinge_vertically is set. The opening region is what
    is cut out of the fixed panel; the moving region starts one gap aft of it (and one gap inboard at each
    span end where fixed material borders it)."""
    if control.kind == "split":
        span_start, span_end = control.span_start, control.span_end
    else:
        span_start, span_end = control.span_start, 1.0  # all_moving: from span_start (0 = the root) to the tip
    gap_m = control.gap_m
    chord_dir = chord_direction(panel)
    span_dir = panel.span_dir
    hinge_start = control_hinge_point(panel, control, span_start)
    hinge_end = control_hinge_point(panel, control, span_end)
    hinge_dir = unit(cross(tuple(b - a for a, b in zip(hinge_start, hinge_end)), thickness_direction(panel)))
    if dot(hinge_dir, chord_dir) < 0.0:
        hinge_dir = tuple(-value for value in hinge_dir)

    def chord_at(span_fraction: float) -> float:
        leading_edge, trailing_edge = panel_point(panel, span_fraction, 0.0), panel_point(panel, span_fraction, 1.0)
        return dot(tuple(t - l for t, l in zip(trailing_edge, leading_edge)), hinge_dir)  # noqa: E741

    def along(point: Vec, distance_m: float, direction: Vec) -> Vec:
        return tuple(value + distance_m * d for value, d in zip(point, direction))

    def rod_offset(point: Vec) -> Vec:
        return along(along(point, control.rod_x_offset_m, chord_dir), control.rod_y_offset_m, panel.normal)

    span_start_point = panel_point(panel, span_start, 0.0)
    span_end_point = panel_point(panel, span_end, 0.0)
    true_span_m = dot(tuple(b - a for a, b in zip(panel.attach_le, panel.tip_le)), span_dir)
    moving_start = span_start + gap_m / true_span_m if span_start > 0.0 else span_start
    moving_end = span_end - gap_m / true_span_m if span_end < 1.0 else span_end
    if control.kind == "split":
        control_hinge_fraction(panel, control, span_start)
        hinge_fraction_end = control_hinge_fraction(panel, control, span_end)
        hinge_point = hinge_start
        opening = (hinge_point, panel_point(panel, span_start, 1.0), panel_point(panel, span_end, 1.0),
                   panel_point(panel, span_end, hinge_fraction_end))
        moving = (panel_point(panel, moving_start, control_hinge_fraction(panel, control, moving_start) + gap_m / chord_at(moving_start)),
                  panel_point(panel, moving_start, 1.0),
                  panel_point(panel, moving_end, 1.0),
                  panel_point(panel, moving_end, control_hinge_fraction(panel, control, moving_end) + gap_m / chord_at(moving_end)))
        rod_start = rod_offset(along(hinge_start, gap_m / 2.0, hinge_dir))  # the rod sits centred in the gap before its offsets
        rod_end = rod_offset(along(hinge_end, gap_m / 2.0, hinge_dir))
    else:
        control_hinge_fraction(panel, control, span_start)
        control_hinge_fraction(panel, control, span_end)
        hinge_point = hinge_start
        opening = (span_start_point, panel_point(panel, span_start, 1.0), panel_point(panel, span_end, 1.0), span_end_point)
        moving = (panel_point(panel, moving_start, 0.0), panel_point(panel, moving_start, 1.0), panel_point(panel, span_end, 1.0), span_end_point)
        rod_start = rod_offset(hinge_point)
        rod_end = rod_offset(hinge_end)
    return {
        "kind": control.kind,
        "hinge_point": hinge_point if control.kind == "split" else None,
        "hinge_dir": hinge_dir,
        "gap_m": gap_m,
        "span_start_point": span_start_point,
        "span_end_point": span_end_point,
        "span_dir": span_dir,
        "rod_diameter_m": control.rod_diameter_m,
        "rod_start": rod_start,
        "rod_end": rod_end,
        "opening": opening,
        "moving": moving,
        "moving_span_m": abs(dot(tuple(b - a for a, b in zip(span_start_point, span_end_point)), span_dir))
                         - gap_m * (int(span_start > 0.0) + int(span_end < 1.0)),
    }


def build_controls(surfaces: tuple[SurfaceSet, ...]) -> tuple[ControlPlacement, ...]:
    found = []
    for order, surface_set in enumerate(surfaces):
        control = surface_set.definition.control
        if control is None:
            continue
        for panel in surface_set.panels:
            frames = control_cut_frames(panel, control)
            outline = frames["moving"]  # the placement outline (2D view) is the moving surface, after its gap
            opening, moving = frames["opening"], frames["moving"]
            hinge = (frames["rod_start"], frames["rod_end"])
            clockwise_deg = math.degrees(math.atan2(panel.span_dir[1], panel.span_dir[2])) % 360.0  # 0 top, 90 right
            axis = unit(tuple(b - a for a, b in zip(*hinge)))
            aft = tuple(b - a for a, b in zip(hinge[0], outline[1]))  # hinge to root TE
            if dot(cross(axis, aft), panel.normal) > 0.0:
                axis = tuple(-value for value in axis)
            found.append(((surface_set.definition.x_root_le_m, round(clockwise_deg, 6), order), surface_set.name, panel.index,
                          control.servos[panel.index], outline, hinge, axis, opening, moving, control.gap_m, control.rod_diameter_m,
                          frames["moving_span_m"]))
    found.sort(key=lambda item: item[0])
    return tuple(
        ControlPlacement(number=number, surface=name, panel_index=index, servo=servo, outline=outline, hinge=hinge, axis=axis,
                         opening=opening, moving=moving, gap_m=gap_m, rod_diameter_m=rod_diameter_m,
                         span_length_m=span_length_m, hinge_length_m=math.dist(*hinge))
        for number, (_key, name, index, servo, outline, hinge, axis, opening, moving, gap_m, rod_diameter_m, span_length_m) in enumerate(found, start=1)
    )


def deflected_outline(control: ControlPlacement, deflection_deg: float) -> tuple[Vec, ...]:
    """The control's moving part rotated about its hinge (or pivot) line; positive = TE away from the panel normal."""
    angle = math.radians(deflection_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    origin = control.hinge[0]
    rotated = []
    for point in control.outline:
        r = tuple(a - b for a, b in zip(point, origin))
        k_cross_r = cross(control.axis, r)
        k_dot_r = dot(control.axis, r)
        rotated.append(tuple(o + v * cos_a + c * sin_a + k * k_dot_r * (1.0 - cos_a) for o, v, c, k in zip(origin, r, k_cross_r, control.axis)))
    return tuple(rotated)


def chord_direction(panel: Panel) -> Vec:
    """Unit vector along the chord, leading edge to trailing edge (incidence turns it towards the normal)."""
    incidence = math.radians(panel.incidence_deg)
    return tuple((math.cos(incidence) if axis == 0 else 0.0) - math.sin(incidence) * panel.normal[axis] for axis in range(3))


def closest_approach(origin_a: Vec, dir_a: Vec, origin_b: Vec, dir_b: Vec) -> tuple[float, float, float] | None:
    """(parameter along a, parameter along b, distance) of the closest points of two lines (unit directions); None when
    they are parallel."""
    w = tuple(a - b for a, b in zip(origin_a, origin_b))
    b_dot = dot(dir_a, dir_b)
    denominator = 1.0 - b_dot * b_dot
    if denominator < 1e-12:
        return None
    d, e = dot(dir_a, w), dot(dir_b, w)
    t_a = (b_dot * e - d) / denominator
    t_b = (e - b_dot * d) / denominator
    gap = tuple(oa + t_a * da - ob - t_b * db for oa, da, ob, db in zip(origin_a, dir_a, origin_b, dir_b))
    return t_a, t_b, math.sqrt(dot(gap, gap))


def thickness_direction(panel: Panel) -> Vec:
    """Unit vector through the section thickness, square to the chord (towards the normal; incidence turns it with the chord)."""
    incidence = math.radians(panel.incidence_deg)
    return tuple((math.sin(incidence) if axis == 0 else 0.0) + math.cos(incidence) * panel.normal[axis] for axis in range(3))


def spar_plane_line(panel: Panel, along_m: float, span_m: float, up_m: float) -> tuple[Vec, Vec]:
    """(start, end) of a line along a panel's span, along_m aft of the root LE along the root chord and up_m off the chord
    line through the thickness: from where it passes closest to the fuselage axis out to the tip."""
    point = tuple(le + along_m * c + up_m * t for le, c, t in zip(panel.attach_le, chord_direction(panel), thickness_direction(panel)))
    to_axis = -dot((0.0, point[1], point[2]), panel.span_dir)
    return tuple(p + to_axis * d for p, d in zip(point, panel.span_dir)), tuple(p + span_m * d for p, d in zip(point, panel.span_dir))


def through_partner(surface_set: SurfaceSet, along_m: float) -> int | None:
    """The panel whose spar line is the first panel's own, continued (opposite and in line); None if there is none."""
    first = surface_set.panels[0]
    up = surface_set.definition.spars.y_offset_m
    start, _end = spar_plane_line(first, along_m, surface_set.definition.span_m, up)
    for panel in surface_set.panels[1:]:
        other, _ = spar_plane_line(panel, along_m, surface_set.definition.span_m, up)
        if dot(panel.span_dir, first.span_dir) < -1.0 + 1e-9 and math.dist(other, start) < 1e-9:
            return panel.index
    return None


def _pod_exit(end: Vec, direction: Vec, motor: "MotorPlacement") -> float:
    """How far past end along direction a spar line leaves a motor pod on its far side (0 when it misses the pod or
    leaves it before end). The pod is a body of revolution about the x-parallel line through the motor."""
    stations = motor.pod_stations
    axis = (stations[0][0], motor.position[1], motor.position[2])
    approach = closest_approach(end, direction, axis, (1.0, 0.0, 0.0))
    radius_max = max(radius for _x, radius in stations)
    if approach is None or approach[2] >= radius_max:
        return 0.0

    def outside(t: float) -> float:
        point = tuple(e + t * d for e, d in zip(end, direction))
        x = point[0]
        if not stations[0][0] <= x <= stations[-1][0]:
            return 1.0
        index = min(bisect.bisect_right([station[0] for station in stations], x), len(stations) - 1)
        (x0, r0), (x1, r1) = stations[index - 1], stations[index]
        radius = r0 if x1 == x0 else r0 + (r1 - r0) * (x - x0) / (x1 - x0)
        return math.hypot(point[1] - axis[1], point[2] - axis[2]) - radius

    inside = approach[0]
    if outside(inside) >= 0.0:
        return 0.0
    across = math.hypot(direction[1], direction[2])
    out = inside + 2.0 * radius_max / max(across, 1e-9)
    for _ in range(60):
        middle = (inside + out) / 2.0
        if outside(middle) < 0.0:
            inside = middle
        else:
            out = middle
    return max(out, 0.0)


def build_spars(surfaces: tuple[SurfaceSet, ...], clearance_m: float, motors: tuple["MotorPlacement", ...] = ()) -> tuple[SparLine, ...]:
    """Spar lines. Where the first panel's spar line runs on in line into the opposite panel, that pair's spars run
    through, tip to tip. Every other spar runs from its tip inward and stops where it meets another (by closest approach
    within a diameter): clearance_m clear of a through spar, or of the plane midway between itself and a spar that also
    stops there (dihedral, radially symmetric sets that are not in line). A spar whose line crosses a motor pod on its
    panel runs on to the pod's far skin, so it goes through the pod (its hole comes out the far side)."""
    lines = []
    for surface_set in surfaces:
        spars = surface_set.definition.spars
        if spars is None:
            continue
        radius = spars.diameter_m / 2.0
        hole_radius = radius + clearance_m
        for index in range(spars.count):
            along = spars.x_offset_m + index * spars.spacing_m
            partner = through_partner(surface_set, along)
            through = {0, partner} if partner is not None else set()
            raw = {panel.index: (*spar_plane_line(panel, along, surface_set.definition.span_m, spars.y_offset_m), panel.span_dir) for panel in surface_set.panels}
            for panel_index, (start, end, direction) in list(raw.items()):
                pods = [motor for motor in motors if motor.pod_stations and motor.panel is not None
                        and (motor.panel.surface, motor.panel.index) == (surface_set.name, panel_index)]
                exit_m = max((_pod_exit(end, direction, motor) for motor in pods), default=0.0)
                raw[panel_index] = (start, tuple(e + exit_m * d for e, d in zip(end, direction)), direction)
            for panel_index, (start, end, direction) in raw.items():
                if panel_index in through:
                    lines.append(SparLine(surface_set.name, panel_index, start, end, spars.diameter_m, True, tuple(p - hole_radius * d for p, d in zip(start, direction))))
                    continue
                stops = []  # (stop parameter, hole start parameter)
                for other_index, (other_start, _other_end, other_direction) in raw.items():
                    if other_index == panel_index:
                        continue
                    meeting = closest_approach(start, direction, other_start, other_direction)
                    if meeting is None or meeting[2] >= spars.diameter_m:
                        continue
                    if other_index in through:  # clear of the through spar's surface, whatever the angle
                        crossing = math.acos(min(1.0, abs(dot(direction, other_direction))))
                        stops.append((meeting[0] + (2.0 * radius + clearance_m - radius * (1.0 - math.cos(crossing))) / math.sin(crossing), meeting[0]))
                    else:  # both stop: each clear of the plane midway between them
                        half_angle = math.acos(max(-1.0, min(1.0, dot(direction, other_direction)))) / 2.0
                        stops.append((meeting[0] + (clearance_m / 2.0 + radius * math.cos(half_angle)) / math.sin(half_angle),
                                      meeting[0] - hole_radius / math.tan(half_angle)))
                stop, hole_start = max(stops) if stops else (0.0, 0.0)
                lines.append(SparLine(surface_set.name, panel_index, tuple(p + stop * d for p, d in zip(start, direction)), end, spars.diameter_m, False,
                                      tuple(p + hole_start * d for p, d in zip(start, direction))))
    return tuple(sorted(lines, key=lambda line: (line.surface, line.panel_index)))


def inner_wall_distance(body: Body, wall_m: float, x_m: float, start: Vec, direction: Vec) -> float:
    """Distance from start along direction (y-z) to the body's inner wall at station x, by bisection."""
    half_w, half_h = body_half_extents(body, x_m)
    inner_w, inner_h = half_w - wall_m, half_h - wall_m

    def outside(t: float) -> bool:
        y, z = start[1] + t * direction[1], start[2] + t * direction[2]
        return math.hypot(y, z) > superellipse_radius(inner_w, inner_h, body_section_at(body, x_m).shape, math.atan2(z, y))

    low, high = 0.0, 2.0 * max(inner_w, inner_h)
    for _ in range(60):
        middle = (low + high) / 2.0
        low, high = (low, middle) if outside(middle) else (middle, high)
    return low


def carry_through_side_profile(carry: CarryThrough, meet_m: float, wall_distance_m: float, reach_m: float) -> tuple[tuple[float, float], ...]:
    """Side view of one arm, closed, in (x offset, distance along the arm) from its start: length_m long out to
    flare_radius_m beyond where it meets the other arms (meet_m along it), flaring (a quarter ellipse, concave) to
    root_length_m at the wall, then straight on to reach_m (into the wall, to fuse with it)."""
    flare_start = min(meet_m + carry.flare_radius_m, wall_distance_m)
    flare = [(carry.length_m / 2.0 + (carry.root_length_m - carry.length_m) / 2.0 * (1.0 - math.cos(a)),
              flare_start + (wall_distance_m - flare_start) * math.sin(a))
             for a in (math.pi / 2.0 * k / CARRY_THROUGH_FLARE_SAMPLES for k in range(CARRY_THROUGH_FLARE_SAMPLES + 1))]
    right = [(carry.length_m / 2.0, 0.0), *flare, (carry.root_length_m / 2.0, reach_m)]
    return tuple(right + [(-u, r) for u, r in reversed(right)])


def to_arm_frame(start: Vec, across: Vec, direction: Vec, y: float, z: float) -> tuple[float, float]:
    """(across, along) of a point of the arm's cross-section plane, from the arm's start."""
    dy, dz = y - start[1], z - start[2]
    return dy * across[1] + dz * across[2], dy * direction[1] + dz * direction[2]


def carry_through_front_profile(carry: CarryThrough, start: Vec, across: Vec, direction: Vec, wall_half_extents_m: tuple[float, float], shape: float,
                                cap_offset_m: float) -> tuple[tuple[float, float], ...]:
    """Front view of one arm, closed, in (offset across, distance along the arm) from its start: stem_width_m wide, its
    sides straight into the inner wall (the body's section at wall_half_extents_m, any shape), then across along the wall
    grown by cap_offset_m (into the wall). The wall fillets (wall_fillet_m) are left to post-processing."""
    half = carry.stem_width_m / 2.0
    half_w, half_h = wall_half_extents_m

    def point(v: float, r: float) -> tuple[float, float]:
        return start[1] + v * across[1] + r * direction[1], start[2] + v * across[2] + r * direction[2]

    def side(sign: float) -> tuple[float, float]:
        """Where one side (sign +1 the +across side) meets the wall: its distance along the arm and its polar angle about
        the body axis."""
        low, high = 0.0, 4.0 * max(half_w, half_h)
        for _ in range(60):
            middle = (low + high) / 2.0
            y, z = point(sign * half, middle)
            inside = math.hypot(y, z) < superellipse_radius(half_w, half_h, shape, math.atan2(z, y))
            low, high = (middle, high) if inside else (low, middle)
        y, z = point(sign * half, low)
        return low, math.atan2(z, y)

    right_r, right_angle = side(1.0)
    left_r, left_angle = side(-1.0)
    arm_angle = math.atan2(direction[2], direction[1])
    right_angle = arm_angle + math.remainder(right_angle - arm_angle, 2.0 * math.pi)  # the arc through the arm's direction
    left_angle = arm_angle + math.remainder(left_angle - arm_angle, 2.0 * math.pi)
    cap = []
    for a in (right_angle + (left_angle - right_angle) * k / CARRY_THROUGH_FLARE_SAMPLES for k in range(CARRY_THROUGH_FLARE_SAMPLES + 1)):
        r = superellipse_radius(half_w + cap_offset_m, half_h + cap_offset_m, shape, a)
        cap.append(to_arm_frame(start, across, direction, r * math.cos(a), r * math.sin(a)))
    return tuple([(half, 0.0), (half, right_r), *cap, (-half, left_r), (-half, 0.0)])


def polygon_area(points: tuple[tuple[float, float], ...]) -> float:
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1]))) / 2.0


def build_carry_through(surfaces: tuple[SurfaceSet, ...], body: Body, wall_m: float) -> tuple[CarryThroughArm, ...]:
    """One arm per panel of every surface set whose spars have a carry-through, along the spar plane midway between its
    spars, from where that line meets the other panels' (the axis for a radial set, the middle plane for a dihedral pair;
    else where it passes closest to the axis), mitred past it, to the inner wall."""
    arms = []
    for surface_set in surfaces:
        spars = surface_set.definition.spars
        if spars is None or spars.carry_through is None:
            continue
        along = spars.x_offset_m + (spars.count - 1) * spars.spacing_m / 2.0
        mid_lines = {panel.index: spar_plane_line(panel, along, surface_set.definition.span_m, spars.y_offset_m)[0] for panel in surface_set.panels}
        for panel in surface_set.panels:
            meetings = []  # (parameter along this arm, the other arm's direction)
            for other in surface_set.panels:
                if other.index == panel.index:
                    continue
                meeting = closest_approach(mid_lines[panel.index], panel.span_dir, mid_lines[other.index], other.span_dir)
                if meeting is not None and meeting[2] < spars.carry_through.stem_width_m:
                    meetings.append((meeting[0], other.span_dir))
            meet = max((t for t, _ in meetings), default=0.0)  # the first meeting coming in from the tip
            # mitre: run on past the meeting until the outer edges of the arms meeting there cross
            angles = [math.acos(max(-1.0, min(1.0, dot(panel.span_dir, other)))) for t, other in meetings if abs(t - meet) < 1e-9]
            mitre = max((spars.carry_through.stem_width_m / 2.0 / math.tan(angle / 2.0) for angle in angles if 1e-6 < angle < math.pi - 1e-9), default=0.0)
            start = tuple(p + (meet - mitre) * d for p, d in zip(mid_lines[panel.index], panel.span_dir))
            across = unit(cross(panel.span_dir, (1.0, 0.0, 0.0)))
            wall_distance = inner_wall_distance(body, wall_m, start[0], start, panel.span_dir)
            half_w, half_h = body_half_extents(body, start[0])
            wall_half = (half_w - wall_m, half_h - wall_m)
            arms.append(CarryThroughArm(
                surface=surface_set.name, panel_index=panel.index, start=start, direction=panel.span_dir, across=across, wall_distance_m=wall_distance,
                wall_half_extents_m=wall_half, section_shape=body_section_at(body, start[0]).shape, meet_m=mitre,
                side_profile=carry_through_side_profile(spars.carry_through, mitre, wall_distance, wall_distance),
                front_profile=carry_through_front_profile(spars.carry_through, start, across, panel.span_dir, wall_half, body_section_at(body, start[0]).shape, 0.0),
            ))
    return tuple(arms)


def carry_through_mass_points(surfaces: tuple[SurfaceSet, ...], arms: tuple[CarryThroughArm, ...], airframe: Airframe) -> list[MassPoint]:
    """The printed carry-through of each surface set, attached with the set: side profile x stem width per arm (the fillets
    into the wall and the overlap of the arms at the axis are left out) x print bulk density. None where the section it
    sits in is weighed (its empty mass includes it)."""
    points = []
    for surface_set in surfaces:
        own = [arm for arm in arms if arm.surface == surface_set.name]
        if not own:
            continue
        section = airframe.sections.per_section[section_index(airframe.sections.cuts_m, surface_set.exposed_x_root_le_m)]
        if section.empty_mass_kg is not None:
            continue
        carry = surface_set.definition.spars.carry_through
        pieces = []
        for arm in own:
            volume = polygon_area(arm.side_profile) * carry.stem_width_m
            centre = tuple(s + arm.wall_distance_m / 2.0 * d for s, d in zip(arm.start, arm.direction))
            pieces.append((volume * airframe.print.bulk_density_kg_m3, centre, NO_INERTIA))
        mass, centroid, inertia = combine_inertia(pieces)
        points.append(MassPoint(f"carry-through {surface_set.name}", "structure", centroid, (mass, mass), True, mass, per_unit(inertia, mass),
                                surface_set.exposed_x_root_le_m))
    return points


def spar_mass_points(surfaces: tuple[SurfaceSet, ...], lines: tuple[SparLine, ...], parts_data: dict) -> list[MassPoint]:
    """One mass point per surface set's spars, which go with the set (its root LE places it in a section)."""
    points = []
    for surface_set in surfaces:
        spars = surface_set.definition.spars
        rods = [line for line in lines if line.surface == surface_set.name]
        if not rods:
            continue
        density = parts_data["material"][spars.material]["density_kg_m3"]
        pieces = []
        for line in rods:
            length = math.dist(line.start, line.end)
            direction = unit(tuple(b - a for a, b in zip(line.start, line.end)))
            thin_rod = tuple(length * length / 12.0 * value for value in (
                1.0 - direction[0] ** 2, 1.0 - direction[1] ** 2, 1.0 - direction[2] ** 2,
                -direction[0] * direction[1], -direction[0] * direction[2], -direction[1] * direction[2]))
            mass = math.pi * (line.diameter_m / 2.0) ** 2 * length * density
            pieces.append((mass, tuple((a + b) / 2.0 for a, b in zip(line.start, line.end)), thin_rod))
        mass, centroid, inertia = combine_inertia(pieces)
        points.append(MassPoint(f"spars {surface_set.name}", "reinforcement", centroid, (mass, mass), True, mass, per_unit(inertia, mass),
                                surface_set.exposed_x_root_le_m))
    return points


def spar_checks(surfaces: tuple[SurfaceSet, ...], lines: tuple[SparLine, ...]) -> list[str]:
    """Each spar inside its panel's chord at the root and at the tip."""
    problems = []
    panels = {(surface_set.name, panel.index): panel for surface_set in surfaces for panel in surface_set.panels}
    for line in lines:
        panel = panels[(line.surface, line.panel_index)]
        radius = line.diameter_m / 2.0
        for where, le, te in (("root", panel.attach_le, panel.attach_te), ("tip", panel.tip_le, panel.tip_te)):
            spar_x = line.start[0] if where == "root" else line.end[0]
            if not min(le[0], te[0]) + radius <= spar_x <= max(le[0], te[0]) - radius:
                problems.append(f"surface {line.surface!r} panel {line.panel_index + 1}: spar at x {spar_x * 1000:.1f} mm leaves the {where} chord "
                                f"({le[0] * 1000:.1f}-{te[0] * 1000:.1f} mm)")
    return problems


def panel_strips(panel: Panel):
    """(chord_m, strip_width_m, chord-midpoint position) per spanwise strip, root to tip."""
    width = panel.span_m / SPAN_STRIPS
    for index in range(SPAN_STRIPS):
        t = (index + 0.5) / SPAN_STRIPS
        chord = panel.chord_root_m + (panel.chord_tip_m - panel.chord_root_m) * t
        le = tuple(r + (p - r) * t for r, p in zip(panel.root_le, panel.tip_le))
        incidence = math.radians(panel.incidence_deg)  # mid-chord along the chord line, as the panel outline
        yield chord, width, tuple(e + chord / 2.0 * ((math.cos(incidence) if axis == 0 else 0.0) - math.sin(incidence) * n)
                                  for axis, (e, n) in enumerate(zip(le, panel.normal)))


def combine_inertia(parts: list[tuple[float, Vec, Inertia]]) -> tuple[float, Vec, Inertia]:
    """Total weight, centroid and inertia about the centroid of (weight, position, inertia per unit weight about its
    own centroid) parts."""
    total = sum(weight for weight, _, _ in parts)
    centroid = tuple(sum(weight * position[axis] for weight, position, _ in parts) / total for axis in range(3))
    inertia = [0.0] * 6
    for weight, position, local in parts:
        dx, dy, dz = (position[axis] - centroid[axis] for axis in range(3))
        shift = (dy * dy + dz * dz, dx * dx + dz * dz, dx * dx + dy * dy, -dx * dy, -dx * dz, -dy * dz)
        for index in range(6):
            inertia[index] += weight * (local[index] + shift[index])
    return total, centroid, tuple(inertia)


def per_unit(inertia: Inertia, weight: float) -> Inertia:
    return tuple(value / weight for value in inertia)


def ring_inertia(half_width_m: float, half_height_m: float) -> Inertia:
    """Thin elliptical ring around the x axis, per kg."""
    a2, b2 = half_width_m * half_width_m, half_height_m * half_height_m
    return ((a2 + b2) / 2.0, b2 / 2.0, a2 / 2.0, 0.0, 0.0, 0.0)


def box_inertia(size_m: Vec) -> Inertia:
    sx, sy, sz = (value * value for value in size_m)
    return ((sy + sz) / 12.0, (sx + sz) / 12.0, (sx + sy) / 12.0, 0.0, 0.0, 0.0)


def rotate_inertia(local: Inertia, rotation: tuple[tuple[float, float, float], ...]) -> Inertia:
    """An inertia given in a part's own axes, in the body's (R I R^T; products as tensor entries, as Inertia)."""
    ixx, iyy, izz, ixy, ixz, iyz = local
    tensor = ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))
    turned = [[sum(rotation[i][a] * tensor[a][b] * rotation[j][b] for a in range(3) for b in range(3)) for j in range(3)] for i in range(3)]
    return turned[0][0], turned[1][1], turned[2][2], turned[0][1], turned[0][2], turned[1][2]


def cylinder_inertia(radius_m: float, length_m: float, axis: str) -> Inertia:
    """Solid cylinder along axis x, y or z, per kg."""
    along = radius_m * radius_m / 2.0
    across = radius_m * radius_m / 4.0 + length_m * length_m / 12.0
    return (*(along if name == axis else across for name in "xyz"), 0.0, 0.0, 0.0)


def rotor_inertia(radius_m: float) -> Inertia:
    """Two-blade prop spinning about x, as a rod of length 2R averaged over a turn, per kg."""
    spin = radius_m * radius_m / 3.0
    return (spin, spin / 2.0, spin / 2.0, 0.0, 0.0, 0.0)


def surface_mass_point(surface_set: SurfaceSet, density_kg_m3: float | None, exact: ExactPiece | None = None) -> MassPoint:
    """Mass at the volume centroid of the panels; estimate = solid airfoil volume x printed bulk density. With exact,
    the built solids give the mass, centroid and inertia (a weighed mass_kg still replaces the mass)."""
    definition = surface_set.definition
    if exact is not None:
        mass = definition.mass_kg if definition.mass_kg is not None else exact.mass_kg
        return MassPoint(f"surface {surface_set.name}", "surface", exact.position, (mass, mass), definition.mass_kg is None, exact.mass_kg,
                         exact.inertia_per_kg, surface_set.exposed_x_root_le_m, exact=True)
    strips = [  # weight ~ local airfoil volume (chord^2 x width); each strip is a thin rod along its chord
        (chord * chord * width, mid, (0.0, chord * chord / 12.0, chord * chord / 12.0, 0.0, 0.0, 0.0))
        for panel in surface_set.panels for chord, width, mid in panel_strips(panel)
    ]
    weight, centroid, inertia = combine_inertia(strips)
    estimate = None
    if density_kg_m3 is not None:
        unit_section_area = float(asb.Airfoil(definition.airfoil).area())
        cr, ct = definition.chord_root_m, definition.chord_tip_m
        estimate = len(surface_set.panels) * unit_section_area * definition.span_m * (cr * cr + cr * ct + ct * ct) / 3.0 * density_kg_m3
    mass = definition.mass_kg if definition.mass_kg is not None else estimate
    return MassPoint(f"surface {surface_set.name}", "surface", centroid, (mass, mass), definition.mass_kg is None, estimate, per_unit(inertia, weight),
                     surface_set.exposed_x_root_le_m)


def shell_between(body: Body, x0: float, x1: float, wall_m: float, density_kg_m3: float) -> tuple[float, Vec, Inertia]:
    """(mass, centroid, inertia per kg) of the body shell between two stations, as thin rings along x."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    sections = [(x, *body_half_extents(body, x), body_section_at(body, x).shape) for x in xs]
    rings = []
    for (xa, wa, ha, sa), (xb, wb, hb, sb) in zip(sections, sections[1:]):
        perimeter = (superellipse_perimeter(2.0 * wa, 2.0 * ha, sa) + superellipse_perimeter(2.0 * wb, 2.0 * hb, sb)) / 2.0
        segment = perimeter * math.hypot(xb - xa, ((wb - wa) + (hb - ha)) / 2.0) * wall_m * density_kg_m3  # mean wall slope
        if sa != sb:  # shape morphs also slope between equal-width/height rings
            rings_yz = []
            for x, w, h, shape in ((xa, wa, ha, sa), (xb, wb, hb, sb)):
                ring_points = []
                for k in range(PERIMETER_SAMPLES):
                    angle = 2.0 * math.pi * k / PERIMETER_SAMPLES
                    radius = superellipse_radius(w, h, shape, angle) if min(w, h) > 0.0 else 0.0
                    ring_points.append((x, radius * math.cos(angle), radius * math.sin(angle)))
                rings_yz.append(ring_points)
            area = 0.0
            for k in range(PERIMETER_SAMPLES):
                next_k = (k + 1) % PERIMETER_SAMPLES
                p, q, r, s = rings_yz[0][k], rings_yz[0][next_k], rings_yz[1][k], rings_yz[1][next_k]
                for a, b, c in ((p, q, r), (q, s, r)):
                    normal = cross(tuple(y - x for x, y in zip(a, b)), tuple(y - x for x, y in zip(a, c)))
                    area += math.sqrt(dot(normal, normal)) / 2.0
            segment = area * wall_m * density_kg_m3
        ring = ring_inertia((wa + wb) / 2.0, (ha + hb) / 2.0)
        length_term = (xb - xa) ** 2 / 12.0
        rings.append((segment, ((xa + xb) / 2.0, 0.0, 0.0), (ring[0], ring[1] + length_term, ring[2] + length_term, 0.0, 0.0, 0.0)))
    mass, centroid, inertia = combine_inertia(rings)
    return mass, centroid, per_unit(inertia, mass)


def volume_size(volume: Volume, body: Body, wall_m: float) -> tuple[float, float]:
    """(geometric volume m^3, extent along x m) of a volume; a point has none."""
    if volume.shape in BOX_SHAPES:
        rounded = 0.0
        if volume.shape == "rounded_box":  # the four rounded edges along the axis each lose a square less a quarter circle
            rounded = (4.0 - math.pi) * volume.corner_radius_m ** 2 * volume.size_m[AXES.index(volume.axis)]
        return volume.size_m[0] * volume.size_m[1] * volume.size_m[2] - rounded, volume.size_m[0]
    if volume.shape == "cylinder":
        radius = volume.diameter_m / 2.0
        return math.pi * radius * radius * volume.length_m, volume.length_m if volume.axis == "x" else volume.diameter_m
    if volume.shape == "bore":
        x0, x1 = volume.x_m - volume.length_m / 2.0, volume.x_m + volume.length_m / 2.0
        xs = [x0, *(station.x_m for station in body.stations if x0 < station.x_m < x1), x1]
        areas = [superellipse_area(max(section.width_m - 2.0 * wall_m, 0.0), max(section.height_m - 2.0 * wall_m, 0.0), section.shape)
                 for x in xs for section in (body_section_at(body, x),)]
        return sum((a + b) / 2.0 * (xb - xa) for xa, xb, a, b in zip(xs, xs[1:], areas, areas[1:])), volume.length_m
    return 0.0, 0.0


def solid_between(body: Body, x0: float, x1: float, volumes: tuple[Volume, ...], wall_m: float, density_kg_m3: float) -> tuple[float, Vec, Inertia]:
    """(mass, centroid, inertia per kg) of the body printed solid between two stations, less the volumes inside it.
    Each volume counts by the fraction of its x extent inside the stations; volumes enter as negative masses."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    sections = [(x, *body_half_extents(body, x), body_section_at(body, x).shape) for x in xs]
    pieces = []
    for (xa, wa, ha, sa), (xb, wb, hb, sb) in zip(sections, sections[1:]):
        area = (superellipse_area(2.0 * wa, 2.0 * ha, sa) + superellipse_area(2.0 * wb, 2.0 * hb, sb)) / 2.0
        half_w, half_h, length_term = (wa + wb) / 2.0, (ha + hb) / 2.0, (xb - xa) ** 2 / 12.0
        pieces.append((area * (xb - xa) * density_kg_m3, ((xa + xb) / 2.0, 0.0, 0.0),
                       ((half_w ** 2 + half_h ** 2) / 4.0, half_h ** 2 / 4.0 + length_term, half_w ** 2 / 4.0 + length_term, 0.0, 0.0, 0.0)))
    for volume in volumes:
        size, extent = volume_size(volume, body, wall_m)
        if extent == 0.0:
            continue
        low, high = max(x0, volume.x_m - extent / 2.0), min(x1, volume.x_m + extent / 2.0)
        if high > low:
            pieces.append((-size * (high - low) / extent * density_kg_m3, ((low + high) / 2.0, volume.y_m, volume.z_m), volume_inertia(volume, body, wall_m)))
    mass, centroid, inertia = combine_inertia(pieces)
    if mass <= 0.0:
        raise AirframeError(f"solid section {x0 * 1000:.0f}-{x1 * 1000:.0f} mm: the volumes inside it take up all its material")
    return mass, centroid, per_unit(inertia, mass)


def structure_mass_points(airframe: Airframe, body: Body, density_kg_m3: float, exact: ExactStructure | None = None) -> list[MassPoint]:
    """One mass point per fuselage section, in section order, then one per coupler. A section is a hollow shell or
    solid less its volumes; a weighed empty mass keeps the computed centroid and inertia shape. With exact, the built
    solids give each section's and coupler's mass, centroid and inertia (a weighed mass still replaces the mass)."""
    if exact is not None:
        return _exact_structure_points(airframe, body, exact)
    sections = airframe.sections
    wall = airframe.fuselage.section.wall_radial_m
    bulk_density = airframe.print.bulk_density_kg_m3
    bounds = [0.0, *sections.cuts_m, body.length_m]
    points = []
    for index, (x0, x1) in enumerate(zip(bounds, bounds[1:])):
        row = sections.per_section[index]
        if not row.solid:
            estimate, centroid, inertia = shell_between(body, x0, x1, wall, density_kg_m3)
        elif bulk_density is not None:
            estimate, centroid, inertia = solid_between(body, x0, x1, airframe.volumes, wall, bulk_density)
        else:  # weighed solid section without a bulk density: the shape alone gives centroid and inertia
            estimate, (_, centroid, inertia) = None, solid_between(body, x0, x1, airframe.volumes, wall, 1.0)
        mass = estimate if row.empty_mass_kg is None else row.empty_mass_kg
        points.append(MassPoint(f"section {index + 1} ({x0 * 1000:.0f}-{x1 * 1000:.0f} mm)", "structure", centroid, (mass, mass),
                                row.empty_mass_kg is None, estimate, inertia))
    coupler_mass = airframe.sections.joints.coupler_mass_kg
    if coupler_mass is not None:
        for index, (cut, joint) in enumerate(zip(sections.cuts_m, sections.cut_joints)):
            if joint != "coupler":
                continue
            half_w, half_h = body_half_extents(body, cut)
            points.append(MassPoint(f"coupler {index + 1}", "structure", (cut, 0.0, 0.0), (coupler_mass, coupler_mass), False, None, ring_inertia(half_w - wall, half_h - wall)))
    return points


def _exact_structure_points(airframe: Airframe, body: Body, exact: ExactStructure) -> list[MassPoint]:
    sections = airframe.sections
    bounds = [0.0, *sections.cuts_m, body.length_m]
    points = []
    for index, (x0, x1) in enumerate(zip(bounds, bounds[1:])):
        row, piece = sections.per_section[index], exact.sections[index]
        mass = piece.mass_kg if row.empty_mass_kg is None else row.empty_mass_kg
        points.append(MassPoint(f"section {index + 1} ({x0 * 1000:.0f}-{x1 * 1000:.0f} mm)", "structure", piece.position, (mass, mass),
                                row.empty_mass_kg is None, piece.mass_kg, piece.inertia_per_kg, exact=True))
    coupler_mass = sections.joints.coupler_mass_kg
    for index, piece in sorted(exact.couplers.items()):
        mass = piece.mass_kg if coupler_mass is None else coupler_mass
        points.append(MassPoint(f"coupler {index + 1}", "structure", piece.position, (mass, mass), coupler_mass is None, piece.mass_kg,
                                piece.inertia_per_kg, exact=True))
    return points


def reinforcement_mass_points(reinforcement: Reinforcement, cuts_m: tuple[float, ...], density_kg_m3: float) -> list[MassPoint]:
    """The rods, one mass point per fuselage section they run through, so each section holds its own piece."""
    pattern = reinforcement.pattern
    radius = pattern.diameter_m / 2.0
    bounds = [-math.inf, *cuts_m, math.inf]
    points = []
    for index, (x0, x1) in enumerate(zip(bounds, bounds[1:])):
        low, high = max(x0, reinforcement.from_x_m), min(x1, reinforcement.to_x_m)
        if high <= low:
            continue
        rod_mass = math.pi * radius * radius * (high - low) * density_kg_m3
        rod_inertia = cylinder_inertia(radius, high - low, "x")
        rods = [(rod_mass, ((low + high) / 2.0, y, z), rod_inertia) for y, z in pattern.positions_m()]
        mass, centroid, inertia = combine_inertia(rods)
        points.append(MassPoint(f"rods in section {index + 1} ({low * 1000:.0f}-{high * 1000:.0f} mm)", "reinforcement", centroid, (mass, mass), True, mass,
                                per_unit(inertia, mass)))
    return points


def section_index(cuts_m: tuple[float, ...], x_m: float) -> int:
    """The fuselage section holding x: a point on a cut belongs to the section starting there; anything ahead of the
    nose or behind the tail to the first or last section."""
    return bisect.bisect_right(cuts_m, x_m)


def apply_complete_masses(airframe: Airframe, body: Body, points: list[MassPoint]) -> tuple[list[MassPoint], tuple[SectionMass, ...]]:
    """Per-section masses; a section with a weighed complete mass has every point in it (by position, or by what it is
    attached to) replaced by one point of that mass at their combined centroid and inertia (ranged masses at mid-range)."""
    sections = airframe.sections
    bounds = [0.0, *sections.cuts_m, body.length_m]
    members = [[] for _ in sections.per_section]
    for point in points:
        members[section_index(sections.cuts_m, point.position[0] if point.attached_x_m is None else point.attached_x_m)].append(point)
    replaced = {}
    section_masses = []
    for index, (row, section_point) in enumerate(zip(sections.per_section, points)):
        itemised = (sum(point.mass_kg[0] for point in members[index]), sum(point.mass_kg[1] for point in members[index]))
        section_masses.append(SectionMass(bounds[index], bounds[index + 1], row.solid, section_point.estimate_kg, section_point.mass_kg[0], itemised, row.complete_mass_kg))
        if row.complete_mass_kg is None:
            continue
        weight, centroid, inertia = combine_inertia([((point.mass_kg[0] + point.mass_kg[1]) / 2.0, point.position, point.inertia_per_kg) for point in members[index]])
        replaced[id(members[index][0])] = MassPoint(
            f"section {index + 1} complete ({bounds[index] * 1000:.0f}-{bounds[index + 1] * 1000:.0f} mm)", "structure", centroid,
            (row.complete_mass_kg, row.complete_mass_kg), False, weight, per_unit(inertia, weight))
        replaced.update({id(point): None for point in members[index][1:]})
    kept = [replaced.get(id(point), point) for point in points]
    return [point for point in kept if point is not None], tuple(section_masses)


def socket_checks(airframe: Airframe, body: Body) -> list[str]:
    """A female end (a coupler or a mating spigot enters it) whose body tapers within the insertion depth below its socket
    (the outer surface at the cut less socket_wall_m): the socket breaks through the skin."""
    sections, joints = airframe.sections, airframe.sections.joints
    problems = []
    for index, (cut, joint) in enumerate(zip(sections.cuts_m, sections.cut_joints)):
        if joint == "none":  # no female end: the sections butt, nothing enters the skin
            continue
        ends = {"coupler": ((index, -1.0), (index + 1, 1.0)), "front_male": ((index + 1, 1.0),), "rear_male": ((index, -1.0),)}[joint]
        face = body_section_at(body, cut)
        socket_half = (face.width_m / 2.0 - joints.socket_wall_m, face.height_m / 2.0 - joints.socket_wall_m)
        if min(socket_half) <= joints.fit_clearance_m + joints.coupler_wall_m:
            problems.append(f"section {index + 1}: coupler wall and clearance fill the socket at the {cut * 1000:.1f} mm cut")
            continue
        for section, direction in ends:
            depth = [cut + direction * joints.insertion_depth_m * k / ROOT_CHORD_SAMPLES for k in range(ROOT_CHORD_SAMPLES + 1)]
            clearance = math.inf
            for x in depth:
                local = body_section_at(body, x)
                if min(local.width_m, local.height_m) <= 0.0 or min(socket_half) <= 0.0:
                    clearance = -math.inf
                    break
                for k in range(PERIMETER_SAMPLES):
                    angle = 2.0 * math.pi * k / PERIMETER_SAMPLES
                    clearance = min(clearance, superellipse_radius(local.width_m / 2.0, local.height_m / 2.0, local.shape, angle)
                                    - superellipse_radius(*socket_half, face.shape, angle))
            if clearance <= 0.0:
                problems.append(f"section {section + 1}: its socket at the {cut * 1000:.1f} mm cut ({joint}) breaks through the skin "
                                f"within the {joints.insertion_depth_m * 1000:.0f} mm insertion depth; "
                                f"make that end male or move the cut")
    return problems


def preset_checks(airframe: Airframe) -> list[str]:
    """The fuselage against its preset's limits (data/presets/); none without a preset or limits."""
    preset = airframe.preset
    if preset is None or preset.limits is None:
        return []
    limits, section = preset.limits, airframe.fuselage.section
    problems = []
    checks = [("wall", section.wall_radial_m, limits.wall_radial_m)]
    bounds = [0.0, *airframe.sections.cuts_m, airframe.fuselage.length_m]
    checks += [(f"section {index + 1} length", x1 - x0, limits.section_length_m) for index, (x0, x1) in enumerate(zip(bounds, bounds[1:]))]
    for label, value, (low, high) in checks:
        if not low - 1e-9 <= value <= high + 1e-9:
            problems.append(f"{label} {value * 1000:.2f} mm outside the {preset.name} limits {low * 1000:.2f}-{high * 1000:.2f} mm")
    return problems


def reinforcement_checks(airframe: Airframe, body: Body) -> list[str]:
    """Straight longitudinal rods must remain inside the loft's outer envelope."""
    rods = airframe.fuselage.reinforcement
    if rods is None:
        return []
    xs = [rods.from_x_m, *(station.x_m for station in body.stations if rods.from_x_m < station.x_m < rods.to_x_m), rods.to_x_m]
    radius = rods.pattern.diameter_m / 2.0
    worst = (0.0, rods.from_x_m, 0)
    for x in xs:
        section = body_section_at(body, x)
        for index, (y, z) in enumerate(rods.pattern.positions_m()):
            for k in range(ROD_ENVELOPE_SAMPLES):
                angle = 2.0 * math.pi * k / ROD_ENVELOPE_SAMPLES
                py, pz = y + radius * math.cos(angle), z + radius * math.sin(angle)
                outer = superellipse_radius(section.width_m / 2.0, section.height_m / 2.0, section.shape, math.atan2(pz, py)) if min(section.width_m, section.height_m) > 0.0 else 0.0
                worst = max(worst, (math.hypot(py, pz) - outer, x, index))
    if worst[0] > 1e-9:
        return [f"rod {worst[2] + 1} reaches {worst[0] * 1000:.1f} mm outside the fuselage at x {worst[1]:.3f} m"]
    return []


def volume_inertia(volume: Volume, body: Body, wall_m: float) -> Inertia:
    if volume.shape in BOX_SHAPES:  # a rounded box as its box
        return box_inertia(volume.size_m)
    if volume.shape == "cylinder":
        return cylinder_inertia(volume.diameter_m / 2.0, volume.length_m, volume.axis)
    if volume.shape == "bore":  # fills the internal section: solid elliptic cylinder along x
        half_w, half_h = body_half_extents(body, volume.x_m)
        a2, b2 = (half_w - wall_m) ** 2, (half_h - wall_m) ** 2
        length_term = volume.length_m ** 2 / 12.0
        return ((a2 + b2) / 4.0, b2 / 4.0 + length_term, a2 / 4.0 + length_term, 0.0, 0.0, 0.0)
    return NO_INERTIA


def motor_spins(mount: Mount, positions: list[Vec]) -> tuple[str, ...]:
    """Spin per motor of one mount, seen from where the thrust points. A surface mount's motors alternate cw/ccw going
    round the body axis clockwise from the front in INAV's tailsitter hover frame (multirotor front = aircraft belly (-z),
    right = +y), starting cw for props out: an X quad gets front-right and rear-left cw, as INAV props out."""
    if mount.on != "surface":
        return (mount.spin,)
    if len(positions) % 2:
        raise AirframeError(f"mount on {mount.surface}: {len(positions)} motors cannot spin in pairs; props {mount.spin} needs an even count")
    order = sorted(range(len(positions)), key=lambda index: math.atan2(positions[index][1], -positions[index][2]) % (2.0 * math.pi))
    first, second = ("cw", "ccw") if mount.spin == "out" else ("ccw", "cw")
    spins = [""] * len(positions)
    for rank, index in enumerate(order):
        spins[index] = first if rank % 2 == 0 else second
    return tuple(spins)


def right_angle_route(start: Vec, end: Vec) -> tuple[Vec, ...]:
    return (start, (end[0], start[1], start[2]), (end[0], end[1], start[2]), end)


def route_length_m(route: tuple[Vec, ...]) -> float:
    return sum(math.dist(a, b) for a, b in zip(route, route[1:]))


def build_wires(airframe: Airframe, motors: tuple[MotorPlacement, ...]) -> WireRoutes | None:
    if airframe.propulsion is None or airframe.propulsion.electric is None or airframe.propulsion.electric.esc is None:
        return None  # no ESC placed yet: no routes (electric_checks reports it)
    electric = airframe.propulsion.electric
    parts = {part.name: part for part in airframe.parts}
    battery, esc = electric.battery.placed.position_m, parts[electric.esc].position_m
    return WireRoutes(battery=right_angle_route(battery, esc), motors=tuple(motor_route(esc, motor) for motor in motors))


def motor_route(esc: Vec, motor: MotorPlacement) -> tuple[Vec, ...]:
    """ESC to a motor; inside the panel for a surface motor (WireRoutes)."""
    if motor.panel is None:
        return right_angle_route(esc, motor.position)
    panel = motor.panel
    span_yz = (panel.tip_le[1] - panel.attach_le[1], panel.tip_le[2] - panel.attach_le[2])
    span_length = math.hypot(*span_yz)
    out_yz = (motor.position[1] - panel.attach_le[1], motor.position[2] - panel.attach_le[2])
    fraction = min(1.0, max(0.0, (out_yz[0] * span_yz[0] + out_yz[1] * span_yz[1]) / (span_length * span_length)))
    root = panel_point(panel, 0.0, 0.5)
    route = [esc, (root[0], esc[1], esc[2]), root, panel_point(panel, fraction, 0.5), motor.position]
    return tuple(point for index, point in enumerate(route) if index == 0 or math.dist(point, route[index - 1]) > 1e-9)


def build_motors(airframe: Airframe, surfaces: tuple[SurfaceSet, ...]) -> tuple[MotorPlacement, ...]:
    """The electric motors, in mount order (named by mount number); rocket mounts carry rocket motors instead (build_rockets)."""
    propulsion = airframe.propulsion
    if propulsion is None or propulsion.electric is None:
        return ()
    electric = propulsion.electric
    bell_len = electric.bell_length_m
    placements = []  # (name, position, prop_side, pod_stations, spin, thrust_dir, mount_x, panel, mount)
    for number, mount in propulsion.electric_mounts:
        if mount.on == "surface":
            surface_set = next(item for item in surfaces if item.name == mount.surface)
            positions = [
                (f"{mount.surface} {panel.index + 1}", tuple(r + mount.span_position_m * d + (mount.x_offset_m if axis == 0 else 0.0)
                                                            for axis, (r, d) in enumerate(zip(panel.attach_le, panel.span_dir))), panel)
                for panel in surface_set.panels
            ]
            mount_x = surface_set.exposed_x_root_le_m
        elif mount.on == "nose":
            positions = [("nose", (mount.x_offset_m - bell_len / 2.0, 0.0, 0.0), None)]
            mount_x = min(airframe.fuselage.length_m, max(0.0, mount.x_offset_m))
        else:
            positions = [("tail", (airframe.fuselage.length_m + mount.x_offset_m + bell_len / 2.0, 0.0, 0.0), None)]
            mount_x = min(airframe.fuselage.length_m, max(0.0, airframe.fuselage.length_m + mount.x_offset_m))
        prop_side = -1.0 if mount.direction == "tractor" else 1.0  # tractor prop ahead of the motor (towards the nose)
        tilt = math.radians(mount.thrust_axis_tilt_deg)
        thrust_dir = (-math.cos(tilt), 0.0, math.sin(tilt))
        spins = motor_spins(mount, [position for _name, position, _panel in positions])
        for (name, position, panel), spin in zip(positions, spins):
            pod_stations = ()
            if mount.pod is not None:
                pod_radius = mount.pod.diameter_m / 2.0
                start = position[0] - prop_side * bell_len / 2.0
                flat_end = start - prop_side * mount.pod.flat_length_m
                stations = [(start, pod_radius), (flat_end, pod_radius)]
                curve = sample_pointed_profile(mount.pod.profile, mount.pod.curve_length_m, pod_radius, POD_PROFILE_SAMPLES)
                stations += [(flat_end - prop_side * (mount.pod.curve_length_m - x), r) for x, r in curve[:-1]]
                pod_stations = tuple(sorted(stations))
            placements.append((f"motor {number} {name}", position, prop_side, pod_stations, spin, thrust_dir, mount_x, panel, mount))
    return tuple(
        MotorPlacement(
            name=name, position=position, thrust_dir=thrust_dir,
            prop_center=tuple(p - prop_side * (bell_len / 2.0 + PROP_SHAFT_LEN_M) * t for p, t in zip(position, thrust_dir)),
            prop_radius_m=electric.prop_diameter_m / 2.0, bell_diameter_m=electric.bell_diameter_m,
            bell_length_m=bell_len, pod_stations=pod_stations, spin=spin, mount_x_m=mount_x, panel=panel,
            mount_holes=mount.holes, mount_clearance_m=mount.clearance_m, mount_face=(position[0] - prop_side * bell_len / 2.0, position[1], position[2]),
            mount_inward=(-prop_side, 0.0, 0.0),
        )
        for name, position, prop_side, pod_stations, spin, thrust_dir, mount_x, panel, mount in placements
    )


def build_rockets(airframe: Airframe, surfaces: tuple[SurfaceSet, ...]) -> tuple[RocketPlacement, ...]:
    """The loaded rocket motors, in load order; the nozzle end sits at the mount's x_offset_m from the tail face or the
    surface root LE (on a surface: at span_position_m along every panel)."""
    propulsion = airframe.propulsion
    if propulsion is None:
        return ()
    length = airframe.fuselage.length_m
    placements = []
    for load in propulsion.rocket:
        mount = propulsion.mounts[load.mount - 1]
        if mount.on == "surface":
            surface_set = next(item for item in surfaces if item.name == mount.surface)
            positions = [(f"{mount.surface} {panel.index + 1}", tuple(r + mount.span_position_m * d + (mount.x_offset_m if axis == 0 else 0.0)
                                                                       for axis, (r, d) in enumerate(zip(panel.attach_le, panel.span_dir))), panel)
                         for panel in surface_set.panels]
            mount_x = surface_set.exposed_x_root_le_m
        else:
            positions = [("tail", (length + mount.x_offset_m, 0.0, 0.0), None)]
            mount_x = min(length, max(0.0, length + mount.x_offset_m))
        tilt = math.radians(mount.thrust_axis_tilt_deg)
        thrust_dir = (-math.cos(tilt), 0.0, math.sin(tilt))
        for name, nozzle, panel in positions:
            center = tuple(n + load.motor.length_m / 2.0 * t for n, t in zip(nozzle, thrust_dir))
            placements.append(RocketPlacement(name=f"rocket {load.mount} {name}", mount=load.mount, motor=load.motor, nozzle=nozzle, thrust_dir=thrust_dir,
                                              center=center, mount_x_m=mount_x, panel=panel))
    return tuple(placements)


def rocket_mass_points(rockets: tuple[RocketPlacement, ...]) -> list[MassPoint]:
    """Casing and propellant of each rocket motor at the casing centre, each a solid cylinder along x."""
    points = []
    for rocket in rockets:
        inertia = cylinder_inertia(rocket.motor.diameter_m / 2.0, rocket.motor.length_m, "x")
        points.append(MassPoint(f"{rocket.name} casing", "rocket casing", rocket.center, (rocket.motor.casing_kg,) * 2, False, None, inertia, rocket.mount_x_m))
        points.append(MassPoint(f"{rocket.name} propellant", ROCKET_PROPELLANT_KIND, rocket.center, (rocket.motor.propellant_kg,) * 2, False, None, inertia, rocket.mount_x_m))
    return points


def electric_checks(airframe: Airframe) -> list[str]:
    """What an electric system in progress still lacks (it resolves without it; only what needs it waits)."""
    electric = None if airframe.propulsion is None else airframe.propulsion.electric
    if electric is not None and electric.esc is None:
        return ["propulsion.electric.esc: no ESC part placed yet; the wiring routes, propulsion summary and simulation wait for one"]
    return []


def rocket_checks(airframe: Airframe, body: Body, rockets: tuple[RocketPlacement, ...]) -> list[str]:
    """A tail rocket motor's casing must fit the inner wall wherever it is inside the body (it may stick out aft)."""
    wall = airframe.fuselage.section.wall_radial_m
    problems = []
    for rocket in (rocket for rocket in rockets if rocket.panel is None):
        radius = rocket.motor.diameter_m / 2.0
        ends = [rocket.nozzle[0], rocket.nozzle[0] + rocket.motor.length_m * rocket.thrust_dir[0]]
        x0, x1 = max(0.0, min(ends)), min(body.length_m, max(ends))
        for x in sorted({x0, x1, *(station.x_m for station in body.stations if x0 < station.x_m < x1)}):
            centre = tuple(n + (x - rocket.nozzle[0]) / rocket.thrust_dir[0] * t for n, t in zip(rocket.nozzle, rocket.thrust_dir))
            half_w, half_h = body_half_extents(body, x)
            inner_w, inner_h = half_w - wall, half_h - wall
            if inner_w <= 0.0 or inner_h <= 0.0:
                problems.append(f"{rocket.name}: {rocket.motor.name} reaches x {x:.3f} m, outside the fuselage interior")
                break
            shape = body_section_at(body, x).shape
            edges = [(centre[1] + radius * math.cos(a), centre[2] + radius * math.sin(a)) for a in (2.0 * math.pi * k / PERIMETER_SAMPLES for k in range(PERIMETER_SAMPLES))]
            excess = max(math.hypot(y, z) - superellipse_radius(inner_w, inner_h, shape, math.atan2(z, y)) for y, z in edges)
            if excess > 1e-9:
                problems.append(f"{rocket.name}: {rocket.motor.name} ({rocket.motor.diameter_m * 1000:.1f} mm) reaches {excess * 1000.0:.2f} mm into the wall at x {x:.3f} m")
                break
    return problems


def dry_mass_properties(resolved: "ResolvedAirframe") -> tuple[float, Vec, Inertia]:
    """Mass, CG and inertia (about that CG) without the rocket propellant: at burnout, and what an FDM that burns the
    propellant from its own tanks starts from. Ranged masses at the middle of their range."""
    points = [point for point in resolved.mass_points if point.kind != ROCKET_PROPELLANT_KIND]
    return combine_inertia([((point.mass_kg[0] + point.mass_kg[1]) / 2.0, point.position, point.inertia_per_kg) for point in points])


def center_of_mass(points: tuple[MassPoint, ...], masses: list[float]) -> Vec:
    total = sum(masses)
    return tuple(sum(m * p.position[axis] for m, p in zip(masses, points)) / total for axis in range(3))


def build_reference(surfaces: tuple[SurfaceSet, ...], body: Body) -> Reference:
    def horizontal_area(surface_set: SurfaceSet) -> float:
        return sum(panel.span_dir[1] ** 2 for panel in surface_set.panels) * surface_set.planform_area_m2 / len(surface_set.panels)

    candidates = [item for item in surfaces if horizontal_area(item) > 1e-12]
    if not candidates:
        widest = max(body.stations, key=lambda station: station.width_m)
        return Reference("body", superellipse_area(widest.width_m, widest.height_m, widest.shape), widest.width_m, widest.width_m)
    wing = max(candidates, key=horizontal_area)
    tips_y = [panel.tip_le[1] for panel in wing.panels]
    return Reference(f"surface:{wing.name}", wing.planform_area_m2, wing.mac_m, max(tips_y) - min(tips_y))


def wall_excess(body: Body, wall_m: float, x: float, centre_yz: tuple[float, float], half_yz: tuple[float, float]) -> float | None:
    """How far the corners of a y-z rectangle at station x reach into the wall (m; <= 0 inside); None where the body
    has no interior."""
    half_w, half_h = body_half_extents(body, x)
    inner_w, inner_h = half_w - wall_m, half_h - wall_m
    if inner_w <= 0.0 or inner_h <= 0.0:
        return None
    corners = [(centre_yz[0] + sy * half_yz[0], centre_yz[1] + sz * half_yz[1]) for sy in (-1, 1) for sz in (-1, 1)]
    return max(math.hypot(*c) - superellipse_radius(inner_w, inner_h, body_section_at(body, x).shape, math.atan2(c[1], c[0])) for c in corners)


def volume_checks(airframe: Airframe, body: Body) -> list[str]:
    wall = airframe.fuselage.section.wall_radial_m
    boxes = []
    problems = []
    for volume in airframe.volumes:
        if volume.shape == "point":
            continue
        if volume.shape in BOX_SHAPES:  # a rounded box checked as its box (conservative)
            half = (volume.size_m[0] / 2.0, volume.size_m[1] / 2.0, volume.size_m[2] / 2.0)
        elif volume.shape == "cylinder":
            radius = volume.diameter_m / 2.0
            half = {"x": (volume.length_m / 2.0, radius, radius), "y": (radius, volume.length_m / 2.0, radius), "z": (radius, radius, volume.length_m / 2.0)}[volume.axis]
        else:
            half = (volume.length_m / 2.0, 0.0, 0.0)
        x0, x1 = volume.x_m - half[0], volume.x_m + half[0]
        for x in sorted({x0, volume.x_m, x1, *(station.x_m for station in body.stations if x0 < station.x_m < x1)}):
            excess = wall_excess(body, wall, x, (volume.y_m, volume.z_m), (half[1], half[2]))
            if excess is None:
                problems.append(f"volume {volume.name!r} reaches x {x:.3f} m, outside the fuselage interior")
                break
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


def part_overhang_m(part: PlacedPart, volume: Volume, body: Body, wall_m: float) -> float:
    """How far a part's box (turned by its orientation; a point without library dimensions) reaches outside its volume
    (m; <= 0 fits)."""
    half = part.half_extents_m() or (0.0, 0.0, 0.0)
    offset = part.offset_m
    if volume.shape == "point":
        return max(abs(o) + h for o, h in zip(offset, half))
    if volume.shape in BOX_SHAPES:  # a part in a rounded box is checked against its box: its corners are not
        return max(abs(o) + h - size / 2.0 for o, h, size in zip(offset, half, volume.size_m))
    along = AXES.index(volume.axis) if volume.shape == "cylinder" else 0
    length_overhang = abs(offset[along]) + half[along] - volume.length_m / 2.0
    across = [index for index in range(3) if index != along]
    if volume.shape == "cylinder":
        radial = math.hypot(abs(offset[across[0]]) + half[across[0]], abs(offset[across[1]]) + half[across[1]])
        return max(length_overhang, radial - volume.diameter_m / 2.0)
    position = part.position_m
    x0, x1 = position[0] - half[0], position[0] + half[0]
    xs = sorted({x0, position[0], x1, *(station.x_m for station in body.stations if x0 < station.x_m < x1)})
    excesses = [wall_excess(body, wall_m, x, (position[1], position[2]), (half[1], half[2])) for x in xs]
    return math.inf if None in excesses else max(length_overhang, *excesses)


def part_checks(airframe: Airframe, body: Body) -> list[str]:
    volumes = {volume.name: volume for volume in airframe.volumes}
    problems = []
    for part in (part for part in airframe.parts if part.volume is not None):
        overhang = part_overhang_m(part, volumes[part.volume], body, airframe.fuselage.section.wall_radial_m)
        if overhang > 1e-9:
            reach = "outside the fuselage interior" if math.isinf(overhang) else f"{overhang * 1000.0:.1f} mm outside it"
            problems.append(f"part {part.name!r} does not fit volume {part.volume!r}: reaches {reach}")
    boxes = [(part.name, [c - h for c, h in zip(part.position_m, part.half_extents_m())], [c + h for c, h in zip(part.position_m, part.half_extents_m())])
             for part in airframe.parts if part.size_m is not None]  # turned boxes as their body-axis bounds
    for (name_a, lo_a, hi_a), (name_b, lo_b, hi_b) in itertools.combinations(boxes, 2):
        if all(lo_a[axis] < hi_b[axis] - 1e-9 and lo_b[axis] < hi_a[axis] - 1e-9 for axis in range(3)):
            problems.append(f"parts {name_a!r} and {name_b!r} overlap")
    return problems


def resolve_airframe(airframe: Airframe, parts_data: dict, exact: ExactStructure | None = None) -> tuple[ResolvedAirframe, list[str]]:
    """Returns the resolved model and a list of layout problems (volumes outside the body or overlapping, parts not
    fitting their volume, the fuselage outside its preset's limits). exact (optional) is the printed structure
    measured on this design's built CAD solids (libs.computed_geometry): it replaces the analytic section, coupler,
    carry-through and surface estimates."""
    density = parts_data["material"][airframe.print.material]["density_kg_m3"]
    body = build_body(airframe)
    surfaces = tuple(build_surface(surface, body) for surface in airframe.surfaces)
    motors = build_motors(airframe, surfaces)
    rockets = build_rockets(airframe, surfaces)

    access_geometry = AccessGeometry(airframe, body, resolve_access(airframe.access))
    points = structure_mass_points(airframe, body, density, exact)
    reinforcement = airframe.fuselage.reinforcement
    if reinforcement is not None:
        points += reinforcement_mass_points(reinforcement, airframe.sections.cuts_m, parts_data["material"][reinforcement.pattern.material]["density_kg_m3"])
    points += [surface_mass_point(surface_set, airframe.print.bulk_density_kg_m3, None if exact is None else exact.surfaces[surface_set.name])
               for surface_set in surfaces]
    spars = build_spars(surfaces, airframe.sections.joints.fit_clearance_m, motors)
    points += spar_mass_points(surfaces, spars, parts_data)
    carry_through = build_carry_through(surfaces, body, airframe.fuselage.section.wall_radial_m)
    if exact is None:  # the built sections carry it: ExactStructure counts it in its section
        points += carry_through_mass_points(surfaces, carry_through, airframe)
    wall = airframe.fuselage.section.wall_radial_m
    points += [
        MassPoint(f"volume {volume.name}", "volume", (volume.x_m, volume.y_m, volume.z_m), volume.mass_kg, False, None, volume_inertia(volume, body, wall))
        for volume in airframe.volumes if volume.mass_kg is not None
    ]
    points += [
        MassPoint(f"part {part.name}", "part", part.position_m, part.mass_kg, False, None, NO_INERTIA if part.size_m is None else rotate_inertia(box_inertia(part.size_m), part.rotation()))
        for part in airframe.parts
    ]
    for motor in motors:
        bell = cylinder_inertia(motor.bell_diameter_m / 2.0, motor.bell_length_m, "x")
        points.append(MassPoint(motor.name, "motor", motor.position, (airframe.propulsion.electric.motor_mass_kg,) * 2, False, None, bell, motor.mount_x_m))
        points.append(MassPoint(f"prop {motor.name}", "prop", motor.prop_center, (airframe.propulsion.electric.prop_mass_kg,) * 2, False, None, rotor_inertia(motor.prop_radius_m),
                                motor.mount_x_m))
    points, section_masses = apply_complete_masses(airframe, body, points)
    points = tuple(points) + tuple(rocket_mass_points(rockets))  # loaded per flight: never part of a weighed section

    ranged = [index for index, point in enumerate(points) if point.mass_kg[0] != point.mass_kg[1]]
    if len(ranged) > MAX_RANGED_MASSES:
        raise AirframeError(f"{len(ranged)} volumes or parts have a mass range; at most {MAX_RANGED_MASSES} are supported")
    nominal = [(point.mass_kg[0] + point.mass_kg[1]) / 2.0 for point in points]
    cg_xs = []
    for choice in itertools.product((0, 1), repeat=len(ranged)):
        masses = [point.mass_kg[0] if index not in ranged else point.mass_kg[choice[ranged.index(index)]] for index, point in enumerate(points)]
        cg_xs.append(center_of_mass(points, masses)[0])

    resolved = ResolvedAirframe(
        airframe=airframe, body=body, surfaces=surfaces, motors=motors, rockets=rockets, controls=build_controls(surfaces), spars=spars, carry_through=carry_through,
        access=access_geometry.access, mass_points=points, sections=section_masses,
        mass_kg=(sum(point.mass_kg[0] for point in points), sum(point.mass_kg[1] for point in points)),
        cg_m=center_of_mass(points, nominal), cg_x_range_m=(min(cg_xs), max(cg_xs)),
        inertia_kg_m2=combine_inertia([(mass, point.position, point.inertia_per_kg) for mass, point in zip(nominal, points)])[2],
        reference=build_reference(surfaces, body), wires=build_wires(airframe, motors),
    )
    return resolved, (electric_checks(airframe) + volume_checks(airframe, body) + part_checks(airframe, body) + preset_checks(airframe) + socket_checks(airframe, body)
                      + reinforcement_checks(airframe, body) + spar_checks(surfaces, spars) + rocket_checks(airframe, body, rockets))
