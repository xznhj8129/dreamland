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

import bisect
import itertools
import math
from dataclasses import dataclass

import aerosandbox as asb

from libs.airframe import AXES, Airframe, AirframeError, Mount, PlacedPart, Reinforcement, Surface, Volume
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
    exposed_x_root_le_m: float  # trapezoid outside the body, for Barrowman
    exposed_span_m: float
    exposed_chord_root_m: float
    exposed_sweep_len_m: float


@dataclass(frozen=True)
class ControlPlacement:
    number: int  # 1.. front to back, then clockwise from the top as seen from behind (the front view)
    surface: str
    panel_index: int
    servo: str
    outline: tuple[Vec, Vec, Vec, Vec]  # moving part: root hinge (LE when all-moving), root TE, outer TE, outer hinge
    hinge: tuple[Vec, Vec]  # hinge line, or the pivot line of an all-moving panel
    axis: Vec  # unit hinge direction: a right-hand rotation about it is a positive deflection (TE away from the normal)


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
    mount_x_m: float  # where it is attached to the fuselage: its surface set's root leading edge, the nose (0) or the tail (length)


@dataclass(frozen=True)
class MassPoint:
    name: str
    kind: str  # structure, reinforcement, surface, volume, part, motor, prop
    position: Vec
    mass_kg: tuple[float, float]  # (min, max)
    computed: bool  # True when derived from geometry rather than given
    estimate_kg: float | None  # mass estimated from geometry and print settings, shown next to a weighed value; None = no estimate
    inertia_per_kg: Inertia  # about the point's own centroid, from its shape
    attached_x_m: float | None = None  # where it is attached to the fuselage (surface set: root LE; motor and prop: their mount), which places it in a fuselage section; None = its own x


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
    """High-current wiring as drawn and as modelled: along the axes (x, then y, then z), from the battery part to the
    ESC part and from the ESC part to each motor (motor order)."""
    battery: tuple[Vec, ...]
    motors: tuple[tuple[Vec, ...], ...]


@dataclass(frozen=True)
class ResolvedAirframe:
    airframe: Airframe
    body: Body
    surfaces: tuple[SurfaceSet, ...]
    motors: tuple[MotorPlacement, ...]
    controls: tuple[ControlPlacement, ...]
    mass_points: tuple[MassPoint, ...]
    sections: tuple[SectionMass, ...]
    mass_kg: tuple[float, float]
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


def build_body(airframe: Airframe, end_samples: int = END_PROFILE_SAMPLES) -> Body:
    """Body stations; end_samples sets the nose/tail sampling density (CAD asks for more than aero)."""
    fuselage = airframe.fuselage
    width, height = fuselage.section.width_m, fuselage.section.height_m
    aspect = height / width
    nose = sample_pointed_profile(fuselage.nose.profile, fuselage.nose.length_m, width / 2.0, end_samples, fuselage.nose.params)
    tail = sample_pointed_profile(fuselage.tail.profile, fuselage.tail.length_m, width / 2.0, end_samples, fuselage.tail.params)
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
    """Panels for one surface set. root "body": the root edge lies on the body surface (following tapering cones) and
    the aero root sits at the mean body radius under the root chord. root "axis": the panels start on the fuselage
    axis and meet in the centre (e.g. delta tail fins); the body hides their inner part."""
    on_axis = surface.root == "axis"
    sweep_len = surface.span_m * math.tan(math.radians(surface.sweep_le_deg))
    incidence = math.radians(surface.incidence_deg)
    chord_root = surface.chord_root_m
    layout = surface_panel_layout(
        layout=surface.layout, root_radius=1.0, span=surface.span_m, dihedral_deg=surface.dihedral_deg,
        y_offset=surface.root_offset, fin_pos=surface.fin_position, centerline_root=on_axis,
    )
    panels = []
    mean_radii = []
    for spec in layout:
        dir_y, dir_z = spec["dir_y"], spec["dir_z"]
        angle = math.atan2(dir_z, dir_y) if on_axis else math.atan2(spec["root_z"], spec["root_y"])
        normal = panel_normal(dir_y, dir_z)
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
            surface=surface.name, index=spec["index"], xz_mirror=spec["xz_mirror"],
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


def build_controls(surfaces: tuple[SurfaceSet, ...]) -> tuple[ControlPlacement, ...]:
    found = []
    for order, surface_set in enumerate(surfaces):
        control = surface_set.definition.control
        if control is None:
            continue
        if control.kind == "split":
            span_start, span_end, outline_chord, line_chord = control.span_start, control.span_end, control.chord_start, control.chord_start
        else:
            span_start, span_end, outline_chord, line_chord = 0.0, 1.0, 0.0, control.pivot
        for panel in surface_set.panels:
            clockwise_deg = math.degrees(math.atan2(panel.span_dir[1], panel.span_dir[2])) % 360.0  # 0 top, 90 right
            outline = (panel_point(panel, span_start, outline_chord), panel_point(panel, span_start, 1.0),
                       panel_point(panel, span_end, 1.0), panel_point(panel, span_end, outline_chord))
            hinge = (panel_point(panel, span_start, line_chord), panel_point(panel, span_end, line_chord))
            axis = unit(tuple(b - a for a, b in zip(*hinge)))
            aft = tuple(b - a for a, b in zip(hinge[0], outline[1]))  # hinge to root TE
            if dot(cross(axis, aft), panel.normal) > 0.0:
                axis = tuple(-value for value in axis)
            found.append(((surface_set.definition.x_root_le_m, round(clockwise_deg, 6), order), surface_set.name, panel.index, control.servos[panel.index], outline, hinge, axis))
    found.sort(key=lambda item: item[0])
    return tuple(
        ControlPlacement(number=number, surface=name, panel_index=index, servo=servo, outline=outline, hinge=hinge, axis=axis)
        for number, (_key, name, index, servo, outline, hinge, axis) in enumerate(found, start=1)
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


def cylinder_inertia(radius_m: float, length_m: float, axis: str) -> Inertia:
    """Solid cylinder along axis x, y or z, per kg."""
    along = radius_m * radius_m / 2.0
    across = radius_m * radius_m / 4.0 + length_m * length_m / 12.0
    return (*(along if name == axis else across for name in "xyz"), 0.0, 0.0, 0.0)


def rotor_inertia(radius_m: float) -> Inertia:
    """Two-blade prop spinning about x, as a rod of length 2R averaged over a turn, per kg."""
    spin = radius_m * radius_m / 3.0
    return (spin, spin / 2.0, spin / 2.0, 0.0, 0.0, 0.0)


def surface_mass_point(surface_set: SurfaceSet, density_kg_m3: float | None) -> MassPoint:
    """Mass at the volume centroid of the panels; estimate = solid airfoil volume x printed bulk density."""
    strips = [  # weight ~ local airfoil volume (chord^2 x width); each strip is a thin rod along its chord
        (chord * chord * width, mid, (0.0, chord * chord / 12.0, chord * chord / 12.0, 0.0, 0.0, 0.0))
        for panel in surface_set.panels for chord, width, mid in panel_strips(panel)
    ]
    weight, centroid, inertia = combine_inertia(strips)
    definition = surface_set.definition
    estimate = None
    if density_kg_m3 is not None:
        unit_section_area = float(asb.Airfoil(definition.airfoil).area())
        cr, ct = definition.chord_root_m, definition.chord_tip_m
        estimate = len(surface_set.panels) * unit_section_area * definition.span_m * (cr * cr + cr * ct + ct * ct) / 3.0 * density_kg_m3
    mass = definition.mass_kg if definition.mass_kg is not None else estimate
    return MassPoint(f"surface {surface_set.name}", "surface", centroid, (mass, mass), definition.mass_kg is None, estimate, per_unit(inertia, weight),
                     definition.x_root_le_m)


def shell_between(body: Body, x0: float, x1: float, wall_m: float, density_kg_m3: float) -> tuple[float, Vec, Inertia]:
    """(mass, centroid, inertia per kg) of the body shell between two stations, as thin rings along x."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    sections = [(x, *body_half_extents(body, x)) for x in xs]
    rings = []
    for (xa, wa, ha), (xb, wb, hb) in zip(sections, sections[1:]):
        perimeter = (superellipse_perimeter(2.0 * wa, 2.0 * ha, body.shape) + superellipse_perimeter(2.0 * wb, 2.0 * hb, body.shape)) / 2.0
        segment = perimeter * math.hypot(xb - xa, ((wb - wa) + (hb - ha)) / 2.0) * wall_m * density_kg_m3  # mean wall slope
        ring = ring_inertia((wa + wb) / 2.0, (ha + hb) / 2.0)
        length_term = (xb - xa) ** 2 / 12.0
        rings.append((segment, ((xa + xb) / 2.0, 0.0, 0.0), (ring[0], ring[1] + length_term, ring[2] + length_term, 0.0, 0.0, 0.0)))
    mass, centroid, inertia = combine_inertia(rings)
    return mass, centroid, per_unit(inertia, mass)


def volume_size(volume: Volume, body: Body, wall_m: float) -> tuple[float, float]:
    """(geometric volume m^3, extent along x m) of a volume; a point has none."""
    if volume.shape == "box":
        return volume.size_m[0] * volume.size_m[1] * volume.size_m[2], volume.size_m[0]
    if volume.shape == "cylinder":
        radius = volume.diameter_m / 2.0
        return math.pi * radius * radius * volume.length_m, volume.length_m if volume.axis == "x" else volume.diameter_m
    if volume.shape == "bore":
        half_w, half_h = body_half_extents(body, volume.x_m)
        return superellipse_area(2.0 * (half_w - wall_m), 2.0 * (half_h - wall_m), body.shape) * volume.length_m, volume.length_m
    return 0.0, 0.0


def solid_between(body: Body, x0: float, x1: float, volumes: tuple[Volume, ...], wall_m: float, density_kg_m3: float) -> tuple[float, Vec, Inertia]:
    """(mass, centroid, inertia per kg) of the body printed solid between two stations, less the volumes inside it.
    Each volume counts by the fraction of its x extent inside the stations; volumes enter as negative masses."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    sections = [(x, *body_half_extents(body, x)) for x in xs]
    pieces = []
    for (xa, wa, ha), (xb, wb, hb) in zip(sections, sections[1:]):
        area = (superellipse_area(2.0 * wa, 2.0 * ha, body.shape) + superellipse_area(2.0 * wb, 2.0 * hb, body.shape)) / 2.0
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


def structure_mass_points(airframe: Airframe, body: Body, density_kg_m3: float) -> list[MassPoint]:
    """One mass point per fuselage section, in section order, then one per coupler. A section is a hollow shell or
    solid less its volumes; a weighed empty mass keeps the computed centroid and inertia shape."""
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
    coupler_mass = airframe.fuselage.joints.coupler_mass_kg
    if coupler_mass is not None:
        for index, cut in enumerate(sections.cuts_m):
            half_w, half_h = body_half_extents(body, cut)
            points.append(MassPoint(f"coupler {index + 1}", "structure", (cut, 0.0, 0.0), (coupler_mass, coupler_mass), False, None, ring_inertia(half_w - wall, half_h - wall)))
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


def preset_checks(airframe: Airframe) -> list[str]:
    """The fuselage against its preset's limits (data/specs.json presets); none without a preset or limits."""
    preset = airframe.preset
    if preset is None or preset.limits is None:
        return []
    limits, section = preset.limits, airframe.fuselage.section
    problems = []
    checks = [("inner width", section.width_m - 2.0 * section.wall_radial_m, limits.inner_diameter_m),
              ("inner height", section.height_m - 2.0 * section.wall_radial_m, limits.inner_diameter_m),
              ("wall", section.wall_radial_m, limits.wall_radial_m)]
    bounds = [0.0, *airframe.sections.cuts_m, airframe.fuselage.length_m]
    checks += [(f"section {index + 1} length", x1 - x0, limits.section_length_m) for index, (x0, x1) in enumerate(zip(bounds, bounds[1:]))]
    for label, value, (low, high) in checks:
        if not low - 1e-9 <= value <= high + 1e-9:
            problems.append(f"{label} {value * 1000:.2f} mm outside the {preset.name} limits {low * 1000:.2f}-{high * 1000:.2f} mm")
    return problems


def volume_inertia(volume: Volume, body: Body, wall_m: float) -> Inertia:
    if volume.shape == "box":
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
    if airframe.propulsion is None:
        return None
    parts = {part.name: part for part in airframe.parts}
    battery, esc = parts[airframe.propulsion.battery].position_m, parts[airframe.propulsion.esc].position_m
    return WireRoutes(battery=right_angle_route(battery, esc), motors=tuple(right_angle_route(esc, motor.position) for motor in motors))


def build_motors(airframe: Airframe, surfaces: tuple[SurfaceSet, ...]) -> tuple[MotorPlacement, ...]:
    propulsion = airframe.propulsion
    if propulsion is None:
        return ()
    bell_len = propulsion.bell_length_m
    placements = []  # (name, position, prop_side, pod_stations, spin, thrust_dir, mount_x)
    for mount_index, mount in enumerate(propulsion.mounts):
        if mount.on == "surface":
            surface_set = next(item for item in surfaces if item.name == mount.surface)
            positions = [
                (f"{mount.surface} {panel.index + 1}", tuple(r + mount.span_position_m * d + (mount.x_offset_m if axis == 0 else 0.0)
                                                            for axis, (r, d) in enumerate(zip(panel.attach_le, panel.span_dir))))
                for panel in surface_set.panels
            ]
            mount_x = surface_set.definition.x_root_le_m
        elif mount.on == "nose":
            positions = [("nose", (-bell_len / 2.0, 0.0, 0.0))]
            mount_x = 0.0
        else:
            positions = [("tail", (airframe.fuselage.length_m + bell_len / 2.0, 0.0, 0.0))]
            mount_x = airframe.fuselage.length_m
        prop_side = -1.0 if mount.direction == "tractor" else 1.0  # tractor prop ahead of the motor (towards the nose)
        tilt = math.radians(mount.thrust_axis_tilt_deg)
        thrust_dir = (-math.cos(tilt), 0.0, math.sin(tilt))
        spins = motor_spins(mount, [position for _name, position in positions])
        for (name, position), spin in zip(positions, spins):
            pod_stations = ()
            if mount.pod is not None:
                pod_radius = mount.pod.diameter_m / 2.0
                start = position[0] - prop_side * bell_len / 2.0
                flat_end = start - prop_side * mount.pod.flat_length_m
                stations = [(start, pod_radius), (flat_end, pod_radius)]
                curve = sample_pointed_profile(mount.pod.profile, mount.pod.curve_length_m, pod_radius, POD_PROFILE_SAMPLES)
                stations += [(flat_end - prop_side * (mount.pod.curve_length_m - x), r) for x, r in curve[:-1]]
                pod_stations = tuple(sorted(stations))
            placements.append((f"motor {mount_index + 1} {name}", position, prop_side, pod_stations, spin, thrust_dir, mount_x))
    return tuple(
        MotorPlacement(
            name=name, position=position, thrust_dir=thrust_dir,
            prop_center=tuple(p - prop_side * (bell_len / 2.0 + PROP_SHAFT_LEN_M) * t for p, t in zip(position, thrust_dir)),
            prop_radius_m=propulsion.prop_diameter_m / 2.0, bell_diameter_m=propulsion.bell_diameter_m,
            bell_length_m=bell_len, pod_stations=pod_stations, spin=spin, mount_x_m=mount_x,
        )
        for name, position, prop_side, pod_stations, spin, thrust_dir, mount_x in placements
    )


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


def wall_excess(body: Body, wall_m: float, x: float, centre_yz: tuple[float, float], half_yz: tuple[float, float]) -> float | None:
    """How far the corners of a y-z rectangle at station x reach into the wall (m; <= 0 inside); None where the body
    has no interior."""
    half_w, half_h = body_half_extents(body, x)
    inner_w, inner_h = half_w - wall_m, half_h - wall_m
    if inner_w <= 0.0 or inner_h <= 0.0:
        return None
    corners = [(centre_yz[0] + sy * half_yz[0], centre_yz[1] + sz * half_yz[1]) for sy in (-1, 1) for sz in (-1, 1)]
    return max(math.hypot(*c) - superellipse_radius(inner_w, inner_h, body.shape, math.atan2(c[1], c[0])) for c in corners)


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
    """How far a part's box (a point without library dimensions) reaches outside its volume (m; <= 0 fits)."""
    half = (0.0, 0.0, 0.0) if part.size_m is None else tuple(size / 2.0 for size in part.size_m)
    offset = part.offset_m
    if volume.shape == "point":
        return max(abs(o) + h for o, h in zip(offset, half))
    if volume.shape == "box":
        return max(abs(o) + h - size / 2.0 for o, h, size in zip(offset, half, volume.size_m))
    along = AXES.index(volume.axis) if volume.shape == "cylinder" else 0
    length_overhang = abs(offset[along]) + half[along] - volume.length_m / 2.0
    across = [index for index in range(3) if index != along]
    if volume.shape == "cylinder":
        radial = math.hypot(abs(offset[across[0]]) + half[across[0]], abs(offset[across[1]]) + half[across[1]])
        return max(length_overhang, radial - volume.diameter_m / 2.0)
    position = part.position_m
    excesses = [wall_excess(body, wall_m, x, (position[1], position[2]), (half[1], half[2])) for x in (position[0] - half[0], position[0], position[0] + half[0])]
    return math.inf if None in excesses else max(length_overhang, *excesses)


def part_checks(airframe: Airframe, body: Body) -> list[str]:
    volumes = {volume.name: volume for volume in airframe.volumes}
    problems = []
    for part in airframe.parts:
        overhang = part_overhang_m(part, volumes[part.volume], body, airframe.fuselage.section.wall_radial_m)
        if overhang > 1e-9:
            reach = "outside the fuselage interior" if math.isinf(overhang) else f"{overhang * 1000.0:.1f} mm outside it"
            problems.append(f"part {part.name!r} does not fit volume {part.volume!r}: reaches {reach}")
    boxes = [(part.name, [c - size / 2.0 for c, size in zip(part.position_m, part.size_m)], [c + size / 2.0 for c, size in zip(part.position_m, part.size_m)])
             for part in airframe.parts if part.size_m is not None]
    for (name_a, lo_a, hi_a), (name_b, lo_b, hi_b) in itertools.combinations(boxes, 2):
        if all(lo_a[axis] < hi_b[axis] - 1e-9 and lo_b[axis] < hi_a[axis] - 1e-9 for axis in range(3)):
            problems.append(f"parts {name_a!r} and {name_b!r} overlap")
    return problems


def resolve_airframe(airframe: Airframe, parts_data: dict) -> tuple[ResolvedAirframe, list[str]]:
    """Returns the resolved model and a list of layout problems (volumes outside the body or overlapping, parts not
    fitting their volume, the fuselage outside its preset's limits)."""
    density = parts_data["material"][airframe.print.material]["density_kg_m3"]
    body = build_body(airframe)
    surfaces = tuple(build_surface(surface, body) for surface in airframe.surfaces)
    motors = build_motors(airframe, surfaces)

    points = structure_mass_points(airframe, body, density)
    reinforcement = airframe.fuselage.reinforcement
    if reinforcement is not None:
        points += reinforcement_mass_points(reinforcement, airframe.sections.cuts_m, parts_data["material"][reinforcement.pattern.material]["density_kg_m3"])
    points += [surface_mass_point(surface_set, airframe.print.bulk_density_kg_m3) for surface_set in surfaces]
    wall = airframe.fuselage.section.wall_radial_m
    points += [
        MassPoint(f"volume {volume.name}", "volume", (volume.x_m, volume.y_m, volume.z_m), volume.mass_kg, False, None, volume_inertia(volume, body, wall))
        for volume in airframe.volumes if volume.mass_kg is not None
    ]
    points += [
        MassPoint(f"part {part.name}", "part", part.position_m, part.mass_kg, False, None, NO_INERTIA if part.size_m is None else box_inertia(part.size_m))
        for part in airframe.parts
    ]
    for motor in motors:
        bell = cylinder_inertia(motor.bell_diameter_m / 2.0, motor.bell_length_m, "x")
        points.append(MassPoint(motor.name, "motor", motor.position, (airframe.propulsion.motor_mass_kg,) * 2, False, None, bell, motor.mount_x_m))
        points.append(MassPoint(f"prop {motor.name}", "prop", motor.prop_center, (airframe.propulsion.prop_mass_kg,) * 2, False, None, rotor_inertia(motor.prop_radius_m),
                                motor.mount_x_m))
    points, section_masses = apply_complete_masses(airframe, body, points)
    points = tuple(points)

    ranged = [index for index, point in enumerate(points) if point.mass_kg[0] != point.mass_kg[1]]
    if len(ranged) > MAX_RANGED_MASSES:
        raise AirframeError(f"{len(ranged)} volumes or parts have a mass range; at most {MAX_RANGED_MASSES} are supported")
    nominal = [(point.mass_kg[0] + point.mass_kg[1]) / 2.0 for point in points]
    cg_xs = []
    for choice in itertools.product((0, 1), repeat=len(ranged)):
        masses = [point.mass_kg[0] if index not in ranged else point.mass_kg[choice[ranged.index(index)]] for index, point in enumerate(points)]
        cg_xs.append(center_of_mass(points, masses)[0])

    resolved = ResolvedAirframe(
        airframe=airframe, body=body, surfaces=surfaces, motors=motors, controls=build_controls(surfaces), mass_points=points, sections=section_masses,
        mass_kg=(sum(point.mass_kg[0] for point in points), sum(point.mass_kg[1] for point in points)),
        cg_m=center_of_mass(points, nominal), cg_x_range_m=(min(cg_xs), max(cg_xs)),
        inertia_kg_m2=combine_inertia([(mass, point.position, point.inertia_per_kg) for mass, point in zip(nominal, points)])[2],
        reference=build_reference(surfaces, body), wires=build_wires(airframe, motors),
    )
    return resolved, volume_checks(airframe, body) + part_checks(airframe, body) + preset_checks(airframe)
