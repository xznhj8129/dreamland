"""
Usage:
python -m libs.airframe aircraft/m62v2/aircraft.json

from libs.airframe import load_airframe
airframe = load_airframe(Path("aircraft/m62v2/aircraft.json"), parts_data)

Plane-first aircraft definition (schema "airframe/2"; see PLAN.md section 1).
The whole aircraft is defined first; printable sections are derived from it later.
SI units throughout (m, kg), angles in degrees, positions measured from the nose tip along +x.
Loading fails on any missing required field, unknown field, bad value or unresolved library reference.
"""

import json
import math
import sys
from dataclasses import dataclass
from functools import cache
from pathlib import Path

import aerosandbox as asb

from libs.motorlib import DataRepo
from libs.pointed_shapes import POINTED_SHAPE_PARAM_KEYS, check_profile, normalize_shape_name

SCHEMA = "airframe/2"
DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PARTS_PATH = DATA_DIR / "parts.json"
SPECS_PATH = DATA_DIR / "specs.json"
PARTS_DIMENSION_UNIT_M = 0.001  # parts.json battery dimensions are in mm
INCH_M = 0.0254

LAYOUT_PANEL_COUNTS = {"H": 2, "X": 4, "+": 4, "1": 1, "3": 3}
LAYOUTS = tuple(LAYOUT_PANEL_COUNTS)
FIN_POSITIONS = ("top", "bottom")
SURFACE_ROOTS = ("body", "axis")  # span measured from the body surface at the root LE, or from the fuselage axis (fins joined at the centre)
VOLUME_SHAPES = ("box", "cylinder", "bore", "point")
PART_CATEGORIES = ("battery", "esc")  # data/parts.json categories that can be placed (others are materials, prop masses)
AXES = ("x", "y", "z")
MOUNT_TARGETS = ("nose", "tail", "surface")
THRUST_DIRECTIONS = ("tractor", "pusher")
# Prop spin, seen from where the thrust points (from above in hover). A surface mount carries 2+ motors that spin in
# pairs, props out or in; a nose or tail mount carries one motor, cw or ccw.
SURFACE_PROP_SPINS = ("out", "in")
SINGLE_PROP_SPINS = ("cw", "ccw")
JOINT_TYPES = ("radial_screws",)
REINFORCEMENT_TYPES = ("cf_rod",)
POLAR_SPLITS = (0, 2)  # rods at each polar position: 0 = one, 2 = a pair either side of it
CONTROL_KINDS = ("split", "all_moving")
CONTROL_ROLES = ("pitch", "yaw", "roll", "flaps")  # for reference only
SERVO_OUTPUTS = tuple(f"S{number}" for number in range(1, 13))


class AirframeError(ValueError):
    pass


@dataclass(frozen=True)
class OperatingPoint:
    aoa_deg: float
    airspeed_m_s: float
    altitude_m: float


@dataclass(frozen=True)
class PrintSettings:
    material: str
    bulk_density_kg_m3: float | None  # printed mass per solid volume; needed for surfaces without mass_kg and solid sections without empty_mass_kg
    printer_bed_m: tuple[float, float, float]


@dataclass(frozen=True)
class BodySection:
    width_m: float
    height_m: float
    shape: float  # superellipse exponent, 2 = ellipse/circle
    wall_radial_m: float


@dataclass(frozen=True)
class EndProfile:
    profile: str
    length_m: float
    params: dict


@dataclass(frozen=True)
class Joints:
    type: str
    count: int
    screw: str
    insertion_depth_m: float
    coupler_mass_kg: float | None  # weighed mass of one coupler; None = not counted yet


@dataclass(frozen=True)
class RodPattern:
    """Longitudinal rods round the fuselage axis, mirror-symmetric left/right and top/bottom: one rod position per
    quadrant, polar_offset_angle (deg) up from the horizontal (+y), mirrored into the other three quadrants. polar_split 2
    makes each position a pair at polar_offset_angle +/- polar_split_angle (deg); 0 keeps a single rod. polar_n counts
    every rod. Every rod lies strictly inside its quadrant (0 < angle < 90)."""
    type: str  # REINFORCEMENT_TYPES
    material: str  # data/parts.json material
    diameter_m: float
    radial_position_m: float  # rod centre from the axis
    polar_n: int  # rods in all
    polar_offset_angle: float
    polar_split: int  # POLAR_SPLITS: 0 = a single rod per position, 2 = a pair
    polar_split_angle: float

    def group_size(self) -> int:
        return 2 if self.polar_split == 2 else 1

    def quadrant_angles_deg(self) -> tuple[float, ...]:
        if self.polar_split == 2:
            return self.polar_offset_angle - self.polar_split_angle, self.polar_offset_angle + self.polar_split_angle
        return (self.polar_offset_angle,)

    def angles_deg(self) -> tuple[float, ...]:
        """Every rod, counter-clockwise from the horizontal (+y) seen from behind: quadrant by quadrant."""
        first = self.quadrant_angles_deg()
        return (*first, *(180.0 - angle for angle in reversed(first)), *(180.0 + angle for angle in first), *(360.0 - angle for angle in reversed(first)))

    def positions_m(self) -> tuple[tuple[float, float], ...]:
        """(y, z) of every rod centre."""
        return tuple((self.radial_position_m * math.cos(math.radians(angle)), self.radial_position_m * math.sin(math.radians(angle))) for angle in self.angles_deg())


@dataclass(frozen=True)
class Reinforcement:
    pattern: RodPattern
    from_x_m: float
    to_x_m: float


@dataclass(frozen=True)
class Channel:
    name: str
    diameter_m: float
    from_x_m: float
    to_x_m: float
    angle_deg: float


@dataclass(frozen=True)
class Fuselage:
    length_m: float
    section: BodySection
    nose: EndProfile
    tail: EndProfile
    joints: Joints
    reinforcement: Reinforcement | None
    channels: tuple[Channel, ...]


@dataclass(frozen=True)
class ControlSurface:
    kind: str  # CONTROL_KINDS
    chord_start: float | None  # split: hinge line, fraction of the local chord from the leading edge
    span_start: float | None  # split: fractions of the panel span from the root
    span_end: float | None
    pivot: float | None  # all_moving: pivot line, fraction of the local chord from the leading edge
    roles: tuple[str, ...]
    deflection_min_deg: float  # positive = trailing edge away from the panel normal (down on a wing)
    deflection_max_deg: float
    rate_deg_s: float  # placeholder, not used yet
    servos: tuple[str, ...]  # one SERVO_OUTPUTS entry per panel, in panel order; outputs may be shared


@dataclass(frozen=True)
class Surface:
    name: str
    x_root_le_m: float
    layout: str
    fin_position: str
    dihedral_deg: float
    root_offset: float  # root height on the body, -1 bottom .. 0 equator .. 1 top (X: root angle)
    root: str  # "body" or "axis", see SURFACE_ROOTS
    span_m: float  # per panel, from the root (body surface at the root LE, or the axis) along the panel
    chord_root_m: float
    chord_tip_m: float
    sweep_le_deg: float
    incidence_deg: float
    airfoil: str
    mass_kg: float | None  # whole set; None = computed from geometry
    control: ControlSurface | None  # every panel of the set carries one; None = fixed surface


@dataclass(frozen=True)
class MotorRef:
    manufacturer: str  # motorlib motor (libs/motorlib/data/motordatav2.csv)
    name: str




@dataclass(frozen=True)
class Pod:
    diameter_m: float
    flat_length_m: float
    curve_length_m: float
    profile: str


@dataclass(frozen=True)
class Mount:
    on: str
    surface: str | None
    span_position_m: float | None  # along each panel from its root
    x_offset_m: float | None  # from the surface root leading edge
    direction: str
    spin: str  # SURFACE_PROP_SPINS on a surface, SINGLE_PROP_SPINS on the nose or tail
    thrust_axis_tilt_deg: float  # thrust line tilted up (+z) from the body axis, e.g. to pass through the CG
    pod: Pod | None


@dataclass(frozen=True)
class Propulsion:
    motor: MotorRef
    prop: str  # motorlib propeller (libs/motorlib/data/propellers.csv)
    battery: str  # name of the placed battery part
    esc: str  # name of the placed ESC part (battery leads end and motor leads start there)
    mounts: tuple[Mount, ...]
    motor_mass_kg: float  # per motor, motorlib
    bell_diameter_m: float  # stator diameter, motorlib
    bell_length_m: float  # stator height, motorlib
    prop_diameter_m: float
    prop_mass_kg: float  # per prop, from parts.json prop sizes


@dataclass(frozen=True)
class Volume:
    name: str
    shape: str
    x_m: float  # centre
    y_m: float
    z_m: float
    size_m: tuple[float, float, float] | None  # box
    diameter_m: float | None  # cylinder
    length_m: float | None  # cylinder, bore
    axis: str | None  # cylinder
    mass_kg: tuple[float, float] | None  # its own mass (min, max), e.g. ballast; None = space only (parts carry theirs)


@dataclass(frozen=True)
class PlacedPart:
    """A library component sitting in a volume."""
    name: str
    part: str  # data/parts.json <category>:<name>
    volume: str
    offset_m: tuple[float, float, float]  # from the volume centre
    position_m: tuple[float, float, float]  # volume centre + offset
    size_m: tuple[float, float, float] | None  # library dimensions (x = l, y = w, z = h); None = none in the library (a point)
    mass_kg: tuple[float, float]  # library mass or mass_kg; 0 = counted elsewhere (e.g. in a weighed section)


@dataclass(frozen=True)
class FuselageSection:
    solid: bool  # printed solid less the volumes inside it; False = hollow tube (shell wall)
    empty_mass_kg: float | None  # weighed printed mass; None = computed
    complete_mass_kg: float | None  # weighed printed mass + everything positioned in the section; None = itemised


@dataclass(frozen=True)
class Sections:
    cuts_m: tuple[float, ...]  # cut stations along the fuselage, ascending; section i spans cut i-1 .. cut i
    per_section: tuple[FuselageSection, ...]  # len(cuts_m) + 1


@dataclass(frozen=True)
class PresetLimits:
    inner_diameter_m: tuple[float, float]
    wall_radial_m: tuple[float, float]
    section_length_m: tuple[float, float]


@dataclass(frozen=True)
class PresetCad:
    """Preset data for the CAD export, not used by the model."""
    quadrants: int
    quadrant_offset_deg: float
    coupler_types: tuple[str, ...]
    coupler_inner_diameter_m: float
    coupler_outer_diameter_m: float
    hole_edge_dist_m: float


@dataclass(frozen=True)
class Preset:
    """A tube size standard (data/specs.json presets): the fuselage blocks it fills in, the limits it checks, CAD data."""
    name: str
    section: BodySection
    joints: Joints
    reinforcement: RodPattern | None  # None = no rods
    limits: PresetLimits | None  # None = nothing checked
    cad: PresetCad | None


@dataclass(frozen=True)
class Airframe:
    name: str
    notes: str
    preset: Preset | None  # a label: the aircraft keeps its own values, checked against the preset's limits
    powered: bool
    vtol: bool  # takes off and lands vertically on its motors
    tailsitter: bool  # VTOL standing on its tail: hovers nose up as a multirotor, flies forward on the same motors
    operating_point: OperatingPoint
    print: PrintSettings
    fuselage: Fuselage
    surfaces: tuple[Surface, ...]
    propulsion: Propulsion | None  # None when not powered
    volumes: tuple[Volume, ...]
    parts: tuple[PlacedPart, ...]
    sections: Sections


class _Fields:
    """Reads one JSON object: every field must be consumed exactly once, leftovers are errors."""

    def __init__(self, data: dict, path: str):
        if type(data) is not dict:
            raise AirframeError(f"{path}: expected an object, got {type(data).__name__}")
        self.data = dict(data)
        self.path = path

    def at(self, key: str) -> str:
        return f"{self.path}.{key}"

    def required(self, key: str):
        if key not in self.data:
            raise AirframeError(f"{self.at(key)}: required field missing")
        return self.data.pop(key)

    def optional(self, key: str, default):
        return self.data.pop(key, default)

    def number(self, key: str, minimum: float | None = None, exclusive: bool = False, value=None):
        value = self.required(key) if value is None else value
        if type(value) not in (int, float) or not math.isfinite(value):
            raise AirframeError(f"{self.at(key)}: expected a finite number, got {value!r}")
        if minimum is not None and (value <= minimum if exclusive else value < minimum):
            raise AirframeError(f"{self.at(key)}: {value} must be {'>' if exclusive else '>='} {minimum}")
        return float(value)

    def positive(self, key: str) -> float:
        return self.number(key, minimum=0.0, exclusive=True)

    def choice(self, key: str, options: tuple[str, ...]) -> str:
        value = self.required(key)
        if value not in options:
            raise AirframeError(f"{self.at(key)}: {value!r} is not one of {options}")
        return value

    def flag(self, key: str) -> bool:
        value = self.required(key)
        if type(value) is not bool:
            raise AirframeError(f"{self.at(key)}: expected true or false, got {value!r}")
        return value

    def text(self, key: str) -> str:
        value = self.required(key)
        if type(value) is not str or not value:
            raise AirframeError(f"{self.at(key)}: expected a non-empty string, got {value!r}")
        return value

    def done(self) -> None:
        if self.data:
            raise AirframeError(f"{self.path}: unknown field(s) {sorted(self.data)}")


def _end_profile(data: dict, path: str) -> EndProfile:
    fields = _Fields(data, path)
    profile = normalize_shape_name(fields.text("profile"))
    if profile not in POINTED_SHAPE_PARAM_KEYS:
        raise AirframeError(f"{fields.at('profile')}: {profile!r} is not one of {sorted(POINTED_SHAPE_PARAM_KEYS)}")
    length_m = fields.positive("length_m")
    params_fields = _Fields(fields.required("params"), fields.at("params"))
    params = {key: params_fields.number(key) for key in POINTED_SHAPE_PARAM_KEYS[profile]}
    params_fields.done()
    fields.done()
    return EndProfile(profile=profile, length_m=length_m, params=params)


def _body_section(data: dict, path: str) -> BodySection:
    fields = _Fields(data, path)
    section = BodySection(
        width_m=fields.positive("width_m"),
        height_m=fields.positive("height_m"),
        shape=fields.number("shape", minimum=1.0),
        wall_radial_m=fields.positive("wall_radial_m"),
    )
    fields.done()
    if 2.0 * section.wall_radial_m >= min(section.width_m, section.height_m):
        raise AirframeError(f"{fields.at('wall_radial_m')}: {section.wall_radial_m} leaves no internal space")
    return section


def _joints(data: dict, path: str) -> Joints:
    fields = _Fields(data, path)
    joints = Joints(
        type=fields.choice("type", JOINT_TYPES),
        count=int(fields.number("count", minimum=1)),
        screw=fields.text("screw"),
        insertion_depth_m=fields.positive("insertion_depth_m"),
        coupler_mass_kg=None if "coupler_mass_kg" not in fields.data else fields.number("coupler_mass_kg", minimum=0.0, exclusive=True),
    )
    fields.done()
    return joints


def _rod_pattern(fields: _Fields, section: BodySection, parts_data: dict) -> RodPattern:
    """Reads the pattern keys of a reinforcement block; the rods must be inside the outer surface."""
    pattern = RodPattern(
        type=fields.choice("type", REINFORCEMENT_TYPES),
        material=fields.text("material"),
        diameter_m=fields.positive("diameter_m"),
        radial_position_m=fields.positive("radial_position_m"),
        polar_n=int(fields.number("polar_n", minimum=1)),
        polar_offset_angle=fields.number("polar_offset_angle"),
        polar_split=int(fields.choice("polar_split", POLAR_SPLITS)),
        polar_split_angle=fields.number("polar_split_angle", minimum=0.0),
    )
    if pattern.material not in parts_data["material"]:
        raise AirframeError(f"{fields.at('material')}: {pattern.material!r} not in data/parts.json material")
    if pattern.polar_n != 4 * pattern.group_size():
        raise AirframeError(f"{fields.at('polar_n')}: {pattern.polar_n} rods is not one {'pair' if pattern.polar_split == 2 else 'rod'} per quadrant "
                            f"({4 * pattern.group_size()})")
    angles = pattern.quadrant_angles_deg()
    if not (0.0 < angles[0] and angles[-1] < 90.0) or (len(angles) > 1 and pattern.polar_split_angle == 0.0):
        raise AirframeError(f"{fields.path}: rods at {list(angles)} deg from the horizontal; each must be distinct and strictly inside its quadrant (0-90)")
    if pattern.radial_position_m + pattern.diameter_m / 2.0 >= min(section.width_m, section.height_m) / 2.0:
        raise AirframeError(f"{fields.at('radial_position_m')}: rods of {pattern.diameter_m} m at {pattern.radial_position_m} m reach the outer surface")
    return pattern


def _reinforcement(data: dict, path: str, section: BodySection, length_m: float, parts_data: dict) -> Reinforcement:
    fields = _Fields(data, path)
    pattern = _rod_pattern(fields, section, parts_data)
    reinforcement = Reinforcement(pattern=pattern, from_x_m=fields.number("from_x_m", minimum=0.0), to_x_m=fields.positive("to_x_m"))
    fields.done()
    if not reinforcement.from_x_m < reinforcement.to_x_m <= length_m:
        raise AirframeError(f"{path}: needs from_x_m < to_x_m <= fuselage length {length_m}")
    return reinforcement


def _fuselage(data: dict, path: str, parts_data: dict) -> Fuselage:
    fields = _Fields(data, path)
    length_m = fields.positive("length_m")
    section = _body_section(fields.required("section"), fields.at("section"))

    nose = _end_profile(fields.required("nose"), fields.at("nose"))
    tail = _end_profile(fields.required("tail"), fields.at("tail"))
    if nose.length_m + tail.length_m > length_m:
        raise AirframeError(f"{path}: nose + tail length {nose.length_m + tail.length_m} exceeds fuselage length {length_m}")
    for end_name, end in (("nose", nose), ("tail", tail)):
        try:
            check_profile(end.profile, end.length_m, section.width_m / 2.0, end.params)
        except ValueError as error:
            raise AirframeError(f"{path}.{end_name}: {error}") from error

    joints = _joints(fields.required("joints"), fields.at("joints"))
    reinforcement_data = fields.optional("reinforcement", None)
    reinforcement = None if reinforcement_data is None else _reinforcement(reinforcement_data, fields.at("reinforcement"), section, length_m, parts_data)

    channels = []
    for index, channel_data in enumerate(fields.optional("channels", [])):
        channel_fields = _Fields(channel_data, f"{fields.at('channels')}[{index}]")
        channel = Channel(
            name=channel_fields.text("name"),
            diameter_m=channel_fields.positive("diameter_m"),
            from_x_m=channel_fields.number("from_x_m", minimum=0.0),
            to_x_m=channel_fields.number("to_x_m", minimum=0.0),
            angle_deg=channel_fields.number("angle_deg"),
        )
        channel_fields.done()
        if not channel.from_x_m < channel.to_x_m <= length_m:
            raise AirframeError(f"{channel_fields.path}: needs from_x_m < to_x_m <= fuselage length {length_m}")
        channels.append(channel)

    fields.done()
    return Fuselage(
        length_m=length_m,
        section=section,
        nose=nose,
        tail=tail,
        joints=joints,
        reinforcement=reinforcement,
        channels=tuple(channels),
    )


def _fraction(fields: _Fields, key: str) -> float:
    value = fields.number(key, minimum=0.0)
    if value > 1.0:
        raise AirframeError(f"{fields.at(key)}: {value} must be <= 1")
    return value


def _control(data: dict, path: str, panel_count: int) -> ControlSurface:
    fields = _Fields(data, path)
    kind = fields.choice("kind", CONTROL_KINDS)
    chord_start = span_start = span_end = pivot = None
    if kind == "split":
        chord_start = _fraction(fields, "chord_start")
        span_start = _fraction(fields, "span_start")
        span_end = _fraction(fields, "span_end")
        if not 0.0 < chord_start < 1.0:
            raise AirframeError(f"{fields.at('chord_start')}: {chord_start} must be between 0 and 1 (exclusive)")
        if span_end <= span_start:
            raise AirframeError(f"{fields.at('span_end')}: {span_end} must be greater than span_start {span_start}")
    else:
        pivot = _fraction(fields, "pivot")
    roles = fields.required("roles")
    if type(roles) is not list or any(role not in CONTROL_ROLES for role in roles) or len(set(roles)) != len(roles):
        raise AirframeError(f"{fields.at('roles')}: expected a list of distinct roles from {CONTROL_ROLES}, got {roles!r}")
    deflection_min_deg = fields.number("deflection_min_deg")
    deflection_max_deg = fields.number("deflection_max_deg")
    if not deflection_min_deg <= 0.0 <= deflection_max_deg or deflection_max_deg == deflection_min_deg:
        raise AirframeError(f"{fields.at('deflection_max_deg')}: limits {deflection_min_deg} .. {deflection_max_deg} must span neutral (min <= 0 <= max, min < max)")
    rate_deg_s = fields.positive("rate_deg_s")
    servos = fields.required("servos")
    if type(servos) is not list or len(servos) != panel_count or any(servo not in SERVO_OUTPUTS for servo in servos):
        raise AirframeError(f"{fields.at('servos')}: expected {panel_count} servo outputs (one per panel) from {SERVO_OUTPUTS[0]}..{SERVO_OUTPUTS[-1]}, got {servos!r}")
    fields.done()
    return ControlSurface(
        kind=kind, chord_start=chord_start, span_start=span_start, span_end=span_end, pivot=pivot, roles=tuple(roles),
        deflection_min_deg=deflection_min_deg, deflection_max_deg=deflection_max_deg, rate_deg_s=rate_deg_s, servos=tuple(servos),
    )


def _surface(data: dict, path: str, fuselage_length_m: float) -> Surface:
    fields = _Fields(data, path)
    mass = fields.optional("mass_kg", None)
    layout = fields.choice("layout", LAYOUTS)
    control = fields.optional("control", None)
    surface = Surface(
        name=fields.text("name"),
        x_root_le_m=fields.number("x_root_le_m", minimum=0.0),
        layout=layout,
        fin_position=fields.choice("fin_position", FIN_POSITIONS),
        dihedral_deg=fields.number("dihedral_deg"),
        root_offset=fields.number("root_offset"),
        root=fields.choice("root", SURFACE_ROOTS),
        span_m=fields.positive("span_m"),
        chord_root_m=fields.positive("chord_root_m"),
        chord_tip_m=fields.positive("chord_tip_m"),
        sweep_le_deg=fields.number("sweep_le_deg"),
        incidence_deg=fields.number("incidence_deg"),
        airfoil=fields.text("airfoil"),
        mass_kg=None if mass is None else fields.number("mass_kg", minimum=0.0, exclusive=True, value=mass),
        control=None if control is None else _control(control, f"{path}.control", LAYOUT_PANEL_COUNTS[layout]),
    )
    fields.done()
    if not -1.0 <= surface.root_offset <= 1.0:
        raise AirframeError(f"{path}.root_offset: {surface.root_offset} must be within [-1, 1]")
    if surface.x_root_le_m >= fuselage_length_m:
        raise AirframeError(f"{path}.x_root_le_m: {surface.x_root_le_m} is not on the fuselage (length {fuselage_length_m}); chords may overhang the tail, roots may not")
    if asb.Airfoil(surface.airfoil).coordinates is None:
        raise AirframeError(f"{path}.airfoil: {surface.airfoil!r} is not a known AeroSandbox airfoil")
    return surface


def _mass_range(value, path: str) -> tuple[float, float]:
    if type(value) is list:
        if len(value) != 2:
            raise AirframeError(f"{path}: a mass range is [min, max], got {value!r}")
        low = _Fields({}, path).number("min", minimum=0.0, value=value[0])
        high = _Fields({}, path).number("max", minimum=0.0, value=value[1])
        if not 0.0 < low <= high:
            raise AirframeError(f"{path}: needs 0 < min <= max, got {value!r}")
        return low, high
    mass = _Fields({}, path).number("mass_kg", minimum=0.0, exclusive=True, value=value)
    return mass, mass


def _volume(data: dict, path: str) -> Volume:
    fields = _Fields(data, path)
    name = fields.text("name")
    shape = fields.choice("shape", VOLUME_SHAPES)
    x_m = fields.number("x_m", minimum=0.0)
    y_m = fields.number("y_m") if "y_m" in fields.data else 0.0
    z_m = fields.number("z_m") if "z_m" in fields.data else 0.0

    size_m = diameter_m = length_m = axis = None
    if shape == "box":
        size = fields.required("size_m")
        if type(size) is not list or len(size) != 3:
            raise AirframeError(f"{fields.at('size_m')}: expected [x, y, z], got {size!r}")
        size_m = tuple(fields.number("size_m", minimum=0.0, exclusive=True, value=item) for item in size)
    elif shape == "cylinder":
        diameter_m = fields.positive("diameter_m")
        length_m = fields.positive("length_m")
        axis = fields.choice("axis", AXES)
    elif shape == "bore":
        length_m = fields.positive("length_m")

    mass_kg = _mass_range(fields.required("mass_kg"), fields.at("mass_kg")) if "mass_kg" in fields.data else None
    fields.done()
    return Volume(
        name=name, shape=shape, x_m=x_m, y_m=y_m, z_m=z_m, size_m=size_m, diameter_m=diameter_m,
        length_m=length_m, axis=axis, mass_kg=mass_kg,
    )


def _part(data: dict, path: str, volumes: dict[str, Volume], parts_data: dict) -> PlacedPart:
    fields = _Fields(data, path)
    name = fields.text("name")
    part = fields.text("part")
    category, _, part_name = part.partition(":")
    if category not in PART_CATEGORIES or part_name not in parts_data[category]:
        raise AirframeError(f"{fields.at('part')}: {part!r} not found; use '<category>:<name>' from data/parts.json, category one of {PART_CATEGORIES}")
    part_data = parts_data[category][part_name]
    volume_name = fields.text("volume")
    if volume_name not in volumes:
        raise AirframeError(f"{fields.at('volume')}: {volume_name!r} is not a defined volume")
    volume = volumes[volume_name]
    offset_m = tuple(fields.number(key) if key in fields.data else 0.0 for key in ("x_offset_m", "y_offset_m", "z_offset_m"))
    if "mass_kg" in fields.data:
        value = fields.required("mass_kg")
        mass_kg = _mass_range(value, fields.at("mass_kg")) if type(value) is list else (fields.number("mass_kg", minimum=0.0, value=value),) * 2
    elif part_data.get("mass"):  # null or 0 in the library = not known
        mass_kg = _mass_range(part_data["mass"], f"{fields.at('part')} ({part}) mass")
    else:
        raise AirframeError(f"{fields.at('mass_kg')}: {part!r} has no mass in data/parts.json; give mass_kg (0 = counted elsewhere)")
    fields.done()
    dims = part_data.get("dimensions")
    return PlacedPart(
        name=name, part=part, volume=volume_name, offset_m=offset_m,
        position_m=(volume.x_m + offset_m[0], volume.y_m + offset_m[1], volume.z_m + offset_m[2]),
        size_m=None if dims is None else (dims["l"] * PARTS_DIMENSION_UNIT_M, dims["w"] * PARTS_DIMENSION_UNIT_M, dims["h"] * PARTS_DIMENSION_UNIT_M),
        mass_kg=mass_kg,
    )


@cache
def motor_repo() -> DataRepo:
    """motorlib's motors, props, cells, ESCs and wires, loaded once."""
    return DataRepo()


def motorlib_motor(motor: MotorRef):
    """The motorlib motor for a reference (AirframeError if it is not there)."""
    repo = motor_repo()
    code = repo.manufacturer_by_name.get(motor.manufacturer.lower())
    found = repo.motor_by_key.get((code, motor.name))
    if found is None:
        names = sorted(m.name for m in repo.motors_for(code)) if code is not None else []
        raise AirframeError(f"motor {motor.manufacturer} {motor.name!r} not in motorlib; "
                            + (f"{motor.manufacturer} motors: {names}" if code is not None else f"unknown manufacturer {motor.manufacturer!r}"))
    return found


def prop_mass_kg(diameter_in: float, parts_data: dict) -> float:
    """Prop mass from parts.json prop entries, linear in diameter between listed sizes."""
    table = sorted((entry["diameter_in"], entry["mass"]) for entry in parts_data["prop"].values())
    for (d0, m0), (d1, m1) in zip(table, table[1:]):
        if d0 <= diameter_in <= d1:
            return m0 + (m1 - m0) * (diameter_in - d0) / (d1 - d0)
    if diameter_in == table[-1][0]:
        return table[-1][1]
    raise AirframeError(f"prop diameter {diameter_in} in is outside parts.json prop sizes {[d for d, _ in table]}")


def _propulsion(data: dict, path: str, surfaces: tuple[Surface, ...], parts: tuple[PlacedPart, ...], parts_data: dict) -> Propulsion:
    fields = _Fields(data, path)
    motor_fields = _Fields(fields.required("motor"), fields.at("motor"))
    motor = MotorRef(manufacturer=motor_fields.text("manufacturer"), name=motor_fields.text("name"))
    motor_fields.done()
    try:
        motor_data = motorlib_motor(motor)
    except AirframeError as error:
        raise AirframeError(f"{fields.at('motor')}: {error}") from error
    if motor_data.stator_diameter_mm is None or motor_data.stator_height_mm is None:
        raise AirframeError(f"{fields.at('motor')}: motorlib has no stator size for {motor.manufacturer} {motor.name}")
    prop = fields.text("prop")
    if prop not in motor_repo().prop_by_name:
        raise AirframeError(f"{fields.at('prop')}: {prop!r} not in motorlib propellers")
    prop_data = motor_repo().prop(prop)

    categories = {part.name: part.part.partition(":")[0] for part in parts}
    battery, esc = fields.text("battery"), fields.text("esc")
    for key, name, category in (("battery", battery, "battery"), ("esc", esc, "esc")):
        if categories.get(name) != category:
            raise AirframeError(f"{fields.at(key)}: {name!r} must name a part whose part is '{category}:<name>'; {category} parts: "
                                f"{sorted(n for n, c in categories.items() if c == category)}")

    surface_names = {surface.name: surface for surface in surfaces}
    mounts = []
    for index, mount_data in enumerate(fields.required("mounts")):
        mount_fields = _Fields(mount_data, f"{fields.at('mounts')}[{index}]")
        on = mount_fields.choice("on", MOUNT_TARGETS)
        surface_name = span_position_m = x_offset_m = None
        if on == "surface":
            surface_name = mount_fields.text("surface")
            if surface_name not in surface_names:
                raise AirframeError(f"{mount_fields.at('surface')}: {surface_name!r} is not a defined surface")
            span_position_m = mount_fields.number("span_position_m", minimum=0.0)
            if span_position_m > surface_names[surface_name].span_m:
                raise AirframeError(f"{mount_fields.at('span_position_m')}: {span_position_m} is past the panel span {surface_names[surface_name].span_m}")
            x_offset_m = mount_fields.number("x_offset_m")
        direction = mount_fields.choice("direction", THRUST_DIRECTIONS)
        thrust_axis_tilt_deg = mount_fields.number("thrust_axis_tilt_deg")
        if (on, direction) in (("nose", "pusher"), ("tail", "tractor")):
            raise AirframeError(f"{mount_fields.at('direction')}: a {on} motor must be a {'tractor' if on == 'nose' else 'pusher'}")
        spin = mount_fields.choice("spin", SURFACE_PROP_SPINS if on == "surface" else SINGLE_PROP_SPINS)
        pod_data = mount_fields.optional("pod", None)
        pod = None
        if pod_data is not None:
            pod_fields = _Fields(pod_data, mount_fields.at("pod"))
            pod = Pod(
                diameter_m=pod_fields.positive("diameter_m"),
                flat_length_m=pod_fields.number("flat_length_m", minimum=0.0),
                curve_length_m=pod_fields.number("curve_length_m", minimum=0.0),
                profile=normalize_shape_name(pod_fields.text("profile")),
            )
            pod_fields.done()
            if POINTED_SHAPE_PARAM_KEYS.get(pod.profile) != []:
                raise AirframeError(f"{mount_fields.at('pod')}.profile: {pod.profile!r} must be a parameter-free profile {[key for key, value in POINTED_SHAPE_PARAM_KEYS.items() if not value]}")
            try:
                check_profile(pod.profile, pod.curve_length_m, pod.diameter_m / 2.0)
            except ValueError as error:
                raise AirframeError(f"{mount_fields.at('pod')}: {error}") from error
        mount_fields.done()
        mounts.append(Mount(on=on, surface=surface_name, span_position_m=span_position_m, x_offset_m=x_offset_m, direction=direction, spin=spin,
                            thrust_axis_tilt_deg=thrust_axis_tilt_deg, pod=pod))
    if not mounts:
        raise AirframeError(f"{fields.at('mounts')}: at least one mount is required")
    fields.done()
    return Propulsion(
        motor=motor, prop=prop, battery=battery, esc=esc, mounts=tuple(mounts), motor_mass_kg=motor_data.weight_g / 1000.0,
        bell_diameter_m=motor_data.stator_diameter_mm / 1000.0, bell_length_m=motor_data.stator_height_mm / 1000.0,
        prop_diameter_m=prop_data.diam_in * INCH_M, prop_mass_kg=prop_mass_kg(prop_data.diam_in, parts_data),
    )


def _limit_range(fields: _Fields, key: str) -> tuple[float, float]:
    value = fields.required(key)
    if type(value) is not list or len(value) != 2:
        raise AirframeError(f"{fields.at(key)}: a limit is [min, max], got {value!r}")
    low, high = (fields.number(key, minimum=0.0, exclusive=True, value=item) for item in value)
    if low > high:
        raise AirframeError(f"{fields.at(key)}: needs min <= max, got {value!r}")
    return low, high


def _preset(data: dict, name: str, parts_data: dict) -> Preset:
    fields = _Fields(data, f"data/specs.json presets.{name}")
    section = _body_section(fields.required("section"), fields.at("section"))
    joints = _joints(fields.required("joints"), fields.at("joints"))
    reinforcement = limits = cad = None
    if "reinforcement" in fields.data:
        rod_fields = _Fields(fields.required("reinforcement"), fields.at("reinforcement"))
        reinforcement = _rod_pattern(rod_fields, section, parts_data)
        rod_fields.done()
    if "limits" in fields.data:
        limit_fields = _Fields(fields.required("limits"), fields.at("limits"))
        limits = PresetLimits(
            inner_diameter_m=_limit_range(limit_fields, "inner_diameter_m"),
            wall_radial_m=_limit_range(limit_fields, "wall_radial_m"),
            section_length_m=_limit_range(limit_fields, "section_length_m"),
        )
        limit_fields.done()
    if "cad" in fields.data:
        cad_fields = _Fields(fields.required("cad"), fields.at("cad"))
        coupler_types = cad_fields.required("coupler_types")
        if type(coupler_types) is not list or not all(type(item) is str and item for item in coupler_types):
            raise AirframeError(f"{cad_fields.at('coupler_types')}: expected a list of names, got {coupler_types!r}")
        cad = PresetCad(
            quadrants=int(cad_fields.number("quadrants", minimum=1)),
            quadrant_offset_deg=cad_fields.number("quadrant_offset_deg"),
            coupler_types=tuple(coupler_types),
            coupler_inner_diameter_m=cad_fields.positive("coupler_inner_diameter_m"),
            coupler_outer_diameter_m=cad_fields.positive("coupler_outer_diameter_m"),
            hole_edge_dist_m=cad_fields.positive("hole_edge_dist_m"),
        )
        cad_fields.done()
    fields.done()
    return Preset(name=name, section=section, joints=joints, reinforcement=reinforcement, limits=limits, cad=cad)


def presets_from_specs(parts_data: dict) -> dict[str, Preset]:
    """The size presets in data/specs.json, validated."""
    with open(SPECS_PATH, "r") as file:
        presets = json.load(file)["presets"]
    return {name: _preset(data, name, parts_data) for name, data in presets.items()}


def _fuselage_section(data: dict, path: str) -> FuselageSection:
    fields = _Fields(data, path)
    section = FuselageSection(
        solid=fields.flag("solid"),
        empty_mass_kg=None if "empty_mass_kg" not in fields.data else fields.positive("empty_mass_kg"),
        complete_mass_kg=None if "complete_mass_kg" not in fields.data else fields.positive("complete_mass_kg"),
    )
    fields.done()
    return section


def parse_airframe(data: dict, parts_data: dict, path: str = "airframe") -> Airframe:
    fields = _Fields(data, path)
    schema = fields.required("schema")
    if schema != SCHEMA:
        raise AirframeError(f"{fields.at('schema')}: expected {SCHEMA!r}, got {schema!r}")
    name = fields.text("name")
    notes = fields.optional("notes", "")
    preset_name = fields.optional("preset", None)
    preset = None
    if preset_name is not None:
        presets = presets_from_specs(parts_data)
        if preset_name not in presets:
            raise AirframeError(f"{fields.at('preset')}: {preset_name!r} not in data/specs.json presets {sorted(presets)}")
        preset = presets[preset_name]
    powered, vtol, tailsitter = fields.flag("powered"), fields.flag("vtol"), fields.flag("tailsitter")
    if tailsitter and not vtol:
        raise AirframeError(f"{fields.at('tailsitter')}: a tailsitter is a VTOL; set vtol true")
    if vtol and not powered:
        raise AirframeError(f"{fields.at('vtol')}: a VTOL needs motors; set powered true")

    op_fields = _Fields(fields.required("operating_point"), fields.at("operating_point"))
    operating_point = OperatingPoint(
        aoa_deg=op_fields.number("aoa_deg"),
        airspeed_m_s=op_fields.positive("airspeed_m_s"),
        altitude_m=op_fields.number("altitude_m"),
    )
    op_fields.done()

    print_fields = _Fields(fields.required("print"), fields.at("print"))
    material = print_fields.text("material")
    if material not in parts_data["material"]:
        raise AirframeError(f"{print_fields.at('material')}: {material!r} not in data/parts.json material")
    bed = print_fields.required("printer_bed_m")
    if type(bed) is not list or len(bed) != 3:
        raise AirframeError(f"{print_fields.at('printer_bed_m')}: expected [x, y, z], got {bed!r}")
    bulk_density = print_fields.optional("bulk_density_kg_m3", None)
    print_settings = PrintSettings(
        material=material,
        bulk_density_kg_m3=None if bulk_density is None else print_fields.number("bulk_density_kg_m3", minimum=0.0, exclusive=True, value=bulk_density),
        printer_bed_m=tuple(print_fields.number("printer_bed_m", minimum=0.0, exclusive=True, value=item) for item in bed),
    )
    print_fields.done()
    if print_settings.bulk_density_kg_m3 is not None and print_settings.bulk_density_kg_m3 > parts_data["material"][material]["density_kg_m3"]:
        raise AirframeError(f"{print_fields.at('bulk_density_kg_m3')}: {print_settings.bulk_density_kg_m3} is denser than solid {material}")

    fuselage = _fuselage(fields.required("fuselage"), fields.at("fuselage"), parts_data)

    surfaces = tuple(
        _surface(surface_data, f"{fields.at('surfaces')}[{index}]", fuselage.length_m)
        for index, surface_data in enumerate(fields.required("surfaces"))
    )
    unweighed = [surface.name for surface in surfaces if surface.mass_kg is None]
    if unweighed and print_settings.bulk_density_kg_m3 is None:
        raise AirframeError(f"{print_fields.at('bulk_density_kg_m3')}: required to compute the mass of surfaces {unweighed}")
    volumes = tuple(
        _volume(volume_data, f"{fields.at('volumes')}[{index}]")
        for index, volume_data in enumerate(fields.required("volumes"))
    )
    volumes_by_name = {volume.name: volume for volume in volumes}
    parts = tuple(
        _part(part_data, f"{fields.at('parts')}[{index}]", volumes_by_name, parts_data)
        for index, part_data in enumerate(fields.required("parts"))
    )
    for collection, label in ((surfaces, "surfaces"), (volumes, "volumes"), (parts, "parts")):
        names = [item.name for item in collection]
        duplicates = sorted({item for item in names if names.count(item) > 1})
        if duplicates:
            raise AirframeError(f"{fields.at(label)}: duplicate names {duplicates}")
    for volume in volumes:
        if volume.x_m > fuselage.length_m:
            raise AirframeError(f"{fields.at('volumes')} {volume.name!r}: x_m {volume.x_m} is past the fuselage length {fuselage.length_m}")

    propulsion_data = fields.optional("propulsion", None)
    if powered != (propulsion_data is not None):
        raise AirframeError(f"{fields.at('propulsion')}: required when powered is true, absent when false (powered is {str(powered).lower()})")
    propulsion = None if propulsion_data is None else _propulsion(propulsion_data, fields.at("propulsion"), surfaces, parts, parts_data)

    sections_fields = _Fields(fields.required("sections"), fields.at("sections"))
    cuts = tuple(
        sections_fields.number("cuts_m", minimum=0.0, exclusive=True, value=item)
        for item in sections_fields.required("cuts_m")
    )
    if list(cuts) != sorted(set(cuts)) or (cuts and cuts[-1] >= fuselage.length_m):
        raise AirframeError(f"{sections_fields.at('cuts_m')}: {list(cuts)} must be strictly ascending and inside the fuselage (length {fuselage.length_m})")
    rows = sections_fields.required("per_section")
    if type(rows) is not list or len(rows) != len(cuts) + 1:
        raise AirframeError(f"{sections_fields.at('per_section')}: needs one entry per section ({len(cuts) + 1})")
    per_section = tuple(_fuselage_section(row, f"{sections_fields.at('per_section')}[{index}]") for index, row in enumerate(rows))
    sections_fields.done()
    unweighed_solid = [f"section {index + 1}" for index, section in enumerate(per_section) if section.solid and section.empty_mass_kg is None]
    if unweighed_solid and print_settings.bulk_density_kg_m3 is None:
        raise AirframeError(f"{print_fields.at('bulk_density_kg_m3')}: required to compute the mass of solid {unweighed_solid}")

    fields.done()
    return Airframe(
        name=name, notes=notes, preset=preset, powered=powered, vtol=vtol, tailsitter=tailsitter, operating_point=operating_point, print=print_settings,
        fuselage=fuselage, surfaces=surfaces, propulsion=propulsion, volumes=volumes, parts=parts,
        sections=Sections(cuts_m=cuts, per_section=per_section),
    )


def load_airframe(path: Path, parts_data: dict) -> Airframe:
    with open(path, "r") as file:
        data = json.load(file)
    return parse_airframe(data, parts_data, path=path.name)


def main() -> None:
    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    for argument in sys.argv[1:]:
        airframe = load_airframe(Path(argument), parts_data)
        motor_count = 0
        motor_mass = 0.0
        if airframe.propulsion is not None:
            for mount in airframe.propulsion.mounts:
                surface = next((item for item in airframe.surfaces if item.name == mount.surface), None)
                motor_count += LAYOUT_PANEL_COUNTS[surface.layout] if surface is not None else 1
            motor_mass = motor_count * (airframe.propulsion.motor_mass_kg + airframe.propulsion.prop_mass_kg)
        volume_mass = [sum(volume.mass_kg[i] for volume in airframe.volumes if volume.mass_kg is not None) for i in (0, 1)]
        surface_mass = sum(surface.mass_kg for surface in airframe.surfaces if surface.mass_kg is not None)
        section_mass = sum(section.empty_mass_kg for section in airframe.sections.per_section if section.empty_mass_kg is not None)
        coupler_mass = len(airframe.sections.cuts_m) * (airframe.fuselage.joints.coupler_mass_kg or 0.0)
        declared = [volume_mass[i] + surface_mass + motor_mass + section_mass + coupler_mass for i in (0, 1)]
        print(f"{argument}: {airframe.name!r} valid")
        print(f"  fuselage {airframe.fuselage.length_m:.3f} m, {len(airframe.surfaces)} surface sets, {len(airframe.volumes)} volumes, {motor_count} motors")
        print(f"  declared mass {declared[0]:.4f}-{declared[1]:.4f} kg (volumes {volume_mass[0]:.4f}-{volume_mass[1]:.4f}, surfaces {surface_mass:.4f}, motors+props {motor_mass:.4f}, sections {section_mass:.4f}, couplers {coupler_mass:.4f})")
        computed = [f"section {index + 1}" for index, section in enumerate(airframe.sections.per_section) if section.empty_mass_kg is None]
        computed += [f"surface {surface.name}" for surface in airframe.surfaces if surface.mass_kg is None]
        if computed:
            print(f"  to be computed from geometry (not in declared mass): {', '.join(computed)}")


if __name__ == "__main__":
    main()
