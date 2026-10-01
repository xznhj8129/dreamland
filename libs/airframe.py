"""
Usage:
python -m libs.airframe aircraft/m62v2.json

from libs.airframe import load_airframe
airframe = load_airframe(Path("aircraft/m62v2.json"), parts_data)

Plane-first aircraft definition (schema "airframe/2"; see PLAN.md section 1).
The whole aircraft is defined first; printable sections are derived from it later.
SI units throughout (m, kg), angles in degrees, positions measured from the nose tip along +x.
Loading fails on any missing required field, unknown field, bad value or unresolved library reference.
"""

import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import aerosandbox as asb

from libs.pointed_shapes import POINTED_SHAPE_PARAM_KEYS, normalize_shape_name
from libs.simlib2 import TestedMotorData

SCHEMA = "airframe/2"
DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PARTS_PATH = DATA_DIR / "parts.json"
MOTORDATA_PATH = Path(__file__).resolve().parent / "motordata.csv"
PARTS_DIMENSION_UNIT_M = 0.001  # parts.json battery dimensions are in mm

LAYOUTS = ("H", "X", "+", "1", "3")
FIN_POSITIONS = ("top", "bottom")
VOLUME_SHAPES = ("box", "cylinder", "bore", "point")
AXES = ("x", "y", "z")
MOUNT_TARGETS = ("nose", "tail", "surface")
THRUST_DIRECTIONS = ("tractor", "pusher")
JOINT_TYPES = ("radial_screws",)


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
    surface_infill: float | None  # needed only for surfaces without mass_kg
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


@dataclass(frozen=True)
class SparRods:
    count: int
    diameter_m: float
    radial_position_m: float
    angle_offset_deg: float
    material: str


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
    spar_rods: SparRods | None
    channels: tuple[Channel, ...]
    structure_mass_kg: float | None  # weighed/known value; None = computed from geometry


@dataclass(frozen=True)
class Surface:
    name: str
    x_root_le_m: float
    layout: str
    fin_position: str
    dihedral_deg: float
    root_offset: float  # root height on the body, -1 bottom .. 0 equator .. 1 top (X: root angle)
    exposed_span_m: float  # from the body surface at the root leading-edge station, along the panel
    chord_root_m: float
    chord_tip_m: float
    sweep_le_deg: float
    incidence_deg: float
    airfoil: str
    mass_kg: float | None  # whole set; None = computed from geometry


@dataclass(frozen=True)
class MotorRef:
    brand: str
    model: str
    kv: int
    prop: str


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
    pod: Pod | None


@dataclass(frozen=True)
class Propulsion:
    motor: MotorRef
    battery_volume: str
    mounts: tuple[Mount, ...]
    motor_mass_kg: float  # per motor, from bench data


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
    mass_kg: tuple[float, float]  # (min, max); equal for a fixed mass
    part: str | None


@dataclass(frozen=True)
class Sections:
    locked_cuts_m: tuple[float, ...]


@dataclass(frozen=True)
class Airframe:
    name: str
    notes: str
    preset: str | None
    operating_point: OperatingPoint
    print: PrintSettings
    fuselage: Fuselage
    surfaces: tuple[Surface, ...]
    propulsion: Propulsion
    volumes: tuple[Volume, ...]
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


def _fuselage(data: dict, path: str) -> Fuselage:
    fields = _Fields(data, path)
    length_m = fields.positive("length_m")

    section_fields = _Fields(fields.required("section"), fields.at("section"))
    section = BodySection(
        width_m=section_fields.positive("width_m"),
        height_m=section_fields.positive("height_m"),
        shape=section_fields.number("shape", minimum=1.0),
        wall_radial_m=section_fields.positive("wall_radial_m"),
    )
    section_fields.done()
    if 2.0 * section.wall_radial_m >= min(section.width_m, section.height_m):
        raise AirframeError(f"{path}.section.wall_radial_m: {section.wall_radial_m} leaves no internal space")

    nose = _end_profile(fields.required("nose"), fields.at("nose"))
    tail = _end_profile(fields.required("tail"), fields.at("tail"))
    if nose.length_m + tail.length_m > length_m:
        raise AirframeError(f"{path}: nose + tail length {nose.length_m + tail.length_m} exceeds fuselage length {length_m}")

    joint_fields = _Fields(fields.required("joints"), fields.at("joints"))
    joints = Joints(
        type=joint_fields.choice("type", JOINT_TYPES),
        count=int(joint_fields.number("count", minimum=1)),
        screw=joint_fields.text("screw"),
        insertion_depth_m=joint_fields.positive("insertion_depth_m"),
    )
    joint_fields.done()

    spar_data = fields.optional("spar_rods", None)
    spar_rods = None
    if spar_data is not None:
        spar_fields = _Fields(spar_data, fields.at("spar_rods"))
        spar_rods = SparRods(
            count=int(spar_fields.number("count", minimum=1)),
            diameter_m=spar_fields.positive("diameter_m"),
            radial_position_m=spar_fields.positive("radial_position_m"),
            angle_offset_deg=spar_fields.number("angle_offset_deg"),
            material=spar_fields.text("material"),
        )
        spar_fields.done()

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

    structure_mass = fields.optional("structure_mass_kg", None)
    structure_mass_kg = None if structure_mass is None else fields.number("structure_mass_kg", minimum=0.0, value=structure_mass)
    fields.done()
    return Fuselage(
        length_m=length_m,
        section=section,
        nose=nose,
        tail=tail,
        joints=joints,
        spar_rods=spar_rods,
        channels=tuple(channels),
        structure_mass_kg=structure_mass_kg,
    )


def _surface(data: dict, path: str, fuselage_length_m: float) -> Surface:
    fields = _Fields(data, path)
    mass = fields.optional("mass_kg", None)
    surface = Surface(
        name=fields.text("name"),
        x_root_le_m=fields.number("x_root_le_m", minimum=0.0),
        layout=fields.choice("layout", LAYOUTS),
        fin_position=fields.choice("fin_position", FIN_POSITIONS),
        dihedral_deg=fields.number("dihedral_deg"),
        root_offset=fields.number("root_offset"),
        exposed_span_m=fields.positive("exposed_span_m"),
        chord_root_m=fields.positive("chord_root_m"),
        chord_tip_m=fields.positive("chord_tip_m"),
        sweep_le_deg=fields.number("sweep_le_deg"),
        incidence_deg=fields.number("incidence_deg"),
        airfoil=fields.text("airfoil"),
        mass_kg=None if mass is None else fields.number("mass_kg", minimum=0.0, exclusive=True, value=mass),
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


def _volume(data: dict, path: str, parts_data: dict) -> Volume:
    fields = _Fields(data, path)
    name = fields.text("name")
    shape = fields.choice("shape", VOLUME_SHAPES)
    x_m = fields.number("x_m", minimum=0.0)
    y_m = fields.number("y_m") if "y_m" in fields.data else 0.0
    z_m = fields.number("z_m") if "z_m" in fields.data else 0.0
    part = fields.optional("part", None)

    part_data = None
    if part is not None:
        category, _, part_name = str(part).partition(":")
        if category not in parts_data or part_name not in parts_data[category]:
            raise AirframeError(f"{fields.at('part')}: {part!r} not found; use '<category>:<name>' from data/parts.json")
        part_data = parts_data[category][part_name]

    size_m = diameter_m = length_m = axis = None
    if shape == "box":
        if "size_m" in fields.data:
            size = fields.required("size_m")
            if type(size) is not list or len(size) != 3:
                raise AirframeError(f"{fields.at('size_m')}: expected [x, y, z], got {size!r}")
            size_m = tuple(fields.number("size_m", minimum=0.0, exclusive=True, value=item) for item in size)
        elif part_data is not None and "dimensions" in part_data:
            dims = part_data["dimensions"]
            size_m = (dims["l"] * PARTS_DIMENSION_UNIT_M, dims["w"] * PARTS_DIMENSION_UNIT_M, dims["h"] * PARTS_DIMENSION_UNIT_M)
        else:
            raise AirframeError(f"{fields.at('size_m')}: box needs size_m or a part with dimensions")
    elif shape == "cylinder":
        diameter_m = fields.positive("diameter_m")
        length_m = fields.positive("length_m")
        axis = fields.choice("axis", AXES)
    elif shape == "bore":
        length_m = fields.positive("length_m")

    if "mass_kg" in fields.data:
        mass_kg = _mass_range(fields.required("mass_kg"), fields.at("mass_kg"))
    elif part_data is not None:
        mass_kg = _mass_range(part_data["mass"], f"{fields.at('part')} ({part}) mass")
    else:
        raise AirframeError(f"{fields.at('mass_kg')}: required field missing (no part to take it from)")
    fields.done()
    return Volume(
        name=name, shape=shape, x_m=x_m, y_m=y_m, z_m=z_m, size_m=size_m, diameter_m=diameter_m,
        length_m=length_m, axis=axis, mass_kg=mass_kg, part=part,
    )


def bench_motor_mass_kg(motor: MotorRef) -> float:
    """Motor mass from the bench data; every row for this motor and KV must agree, whatever the cell count."""
    with open(MOTORDATA_PATH, newline="") as file:
        weights_g = {
            float(row["Weight g"])
            for row in csv.DictReader(file)
            if row["Brand"] == motor.brand and row["Motor"] == motor.model and int(row["KV"]) == motor.kv
        }
    if len(weights_g) != 1:
        raise AirframeError(f"{motor.brand} {motor.model} {motor.kv}KV: motordata.csv has motor weights {sorted(weights_g)} g, need exactly one")
    return weights_g.pop() / 1000.0


def _propulsion(data: dict, path: str, surfaces: tuple[Surface, ...], volumes: tuple[Volume, ...], parts_data: dict) -> Propulsion:
    fields = _Fields(data, path)
    motor_fields = _Fields(fields.required("motor"), fields.at("motor"))
    motor = MotorRef(
        brand=motor_fields.text("brand"),
        model=motor_fields.text("model"),
        kv=int(motor_fields.positive("kv")),
        prop=motor_fields.text("prop"),
    )
    motor_fields.done()

    battery_volume = fields.text("battery_volume")
    battery = next((volume for volume in volumes if volume.name == battery_volume), None)
    if battery is None or battery.part is None or not battery.part.startswith("battery:"):
        raise AirframeError(f"{fields.at('battery_volume')}: {battery_volume!r} must name a volume whose part is 'battery:<name>'")
    cells = int(parts_data["battery"][battery.part.partition(":")[2]]["s"])
    try:
        tested = TestedMotorData(brand=motor.brand, motor=motor.model, kv=motor.kv, batt_s=cells, csv_path=MOTORDATA_PATH)
    except ValueError as error:
        raise AirframeError(f"{fields.at('motor')} with {cells}S battery {battery.part!r}: {error}") from error
    if motor.prop not in tested.available_props():
        raise AirframeError(f"{fields.at('motor')}: no bench data for prop {motor.prop!r} on {motor.brand} {motor.model} {motor.kv}KV {cells}S; tested props: {tested.available_props()}")

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
            if span_position_m > surface_names[surface_name].exposed_span_m:
                raise AirframeError(f"{mount_fields.at('span_position_m')}: {span_position_m} is past the panel span {surface_names[surface_name].exposed_span_m}")
            x_offset_m = mount_fields.number("x_offset_m")
        direction = mount_fields.choice("direction", THRUST_DIRECTIONS)
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
        mount_fields.done()
        mounts.append(Mount(on=on, surface=surface_name, span_position_m=span_position_m, x_offset_m=x_offset_m, direction=direction, pod=pod))
    if not mounts:
        raise AirframeError(f"{fields.at('mounts')}: at least one mount is required")
    fields.done()
    return Propulsion(motor=motor, battery_volume=battery_volume, mounts=tuple(mounts), motor_mass_kg=bench_motor_mass_kg(motor))


def parse_airframe(data: dict, parts_data: dict, path: str = "airframe") -> Airframe:
    fields = _Fields(data, path)
    schema = fields.required("schema")
    if schema != SCHEMA:
        raise AirframeError(f"{fields.at('schema')}: expected {SCHEMA!r}, got {schema!r}")
    name = fields.text("name")
    notes = fields.optional("notes", "")
    preset = fields.optional("preset", None)

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
    infill = print_fields.optional("surface_infill", None)
    print_settings = PrintSettings(
        material=material,
        surface_infill=None if infill is None else print_fields.number("surface_infill", minimum=0.0, exclusive=True, value=infill),
        printer_bed_m=tuple(print_fields.number("printer_bed_m", minimum=0.0, exclusive=True, value=item) for item in bed),
    )
    print_fields.done()
    if print_settings.surface_infill is not None and print_settings.surface_infill > 1.0:
        raise AirframeError(f"{print_fields.at('surface_infill')}: {print_settings.surface_infill} must be <= 1")

    fuselage = _fuselage(fields.required("fuselage"), fields.at("fuselage"))
    if fuselage.spar_rods is not None and fuselage.spar_rods.material not in parts_data["material"]:
        raise AirframeError(f"{path}.fuselage.spar_rods.material: {fuselage.spar_rods.material!r} not in data/parts.json material")

    surfaces = tuple(
        _surface(surface_data, f"{fields.at('surfaces')}[{index}]", fuselage.length_m)
        for index, surface_data in enumerate(fields.required("surfaces"))
    )
    unweighed = [surface.name for surface in surfaces if surface.mass_kg is None]
    if unweighed and print_settings.surface_infill is None:
        raise AirframeError(f"{print_fields.at('surface_infill')}: required to compute the mass of surfaces {unweighed}")
    volumes = tuple(
        _volume(volume_data, f"{fields.at('volumes')}[{index}]", parts_data)
        for index, volume_data in enumerate(fields.required("volumes"))
    )
    for collection, label in ((surfaces, "surfaces"), (volumes, "volumes")):
        names = [item.name for item in collection]
        duplicates = sorted({item for item in names if names.count(item) > 1})
        if duplicates:
            raise AirframeError(f"{fields.at(label)}: duplicate names {duplicates}")
    for volume in volumes:
        if volume.x_m > fuselage.length_m:
            raise AirframeError(f"{fields.at('volumes')} {volume.name!r}: x_m {volume.x_m} is past the fuselage length {fuselage.length_m}")

    propulsion = _propulsion(fields.required("propulsion"), fields.at("propulsion"), surfaces, volumes, parts_data)

    sections_fields = _Fields(fields.required("sections"), fields.at("sections"))
    locked = tuple(
        sections_fields.number("locked_cuts_m", minimum=0.0, exclusive=True, value=item)
        for item in sections_fields.required("locked_cuts_m")
    )
    sections_fields.done()
    for cut in locked:
        if cut >= fuselage.length_m:
            raise AirframeError(f"{fields.at('sections')}.locked_cuts_m: {cut} is not inside the fuselage")

    fields.done()
    return Airframe(
        name=name, notes=notes, preset=preset, operating_point=operating_point, print=print_settings,
        fuselage=fuselage, surfaces=surfaces, propulsion=propulsion, volumes=volumes,
        sections=Sections(locked_cuts_m=tuple(sorted(locked))),
    )


def load_airframe(path: Path, parts_data: dict) -> Airframe:
    with open(path, "r") as file:
        data = json.load(file)
    return parse_airframe(data, parts_data, path=path.name)


def panel_count(layout: str) -> int:
    return {"1": 1, "H": 2, "3": 3, "+": 4, "X": 4}[layout]


def main() -> None:
    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    for argument in sys.argv[1:]:
        airframe = load_airframe(Path(argument), parts_data)
        motor_count = 0
        for mount in airframe.propulsion.mounts:
            surface = next((item for item in airframe.surfaces if item.name == mount.surface), None)
            motor_count += panel_count(surface.layout) if surface is not None else 1
        volume_mass = [sum(volume.mass_kg[i] for volume in airframe.volumes) for i in (0, 1)]
        surface_mass = sum(surface.mass_kg for surface in airframe.surfaces if surface.mass_kg is not None)
        motor_mass = motor_count * airframe.propulsion.motor_mass_kg
        declared = [volume_mass[i] + surface_mass + motor_mass + (airframe.fuselage.structure_mass_kg or 0.0) for i in (0, 1)]
        print(f"{argument}: {airframe.name!r} valid")
        print(f"  fuselage {airframe.fuselage.length_m:.3f} m, {len(airframe.surfaces)} surface sets, {len(airframe.volumes)} volumes, {motor_count} motors")
        print(f"  declared mass {declared[0]:.4f}-{declared[1]:.4f} kg (volumes {volume_mass[0]:.4f}-{volume_mass[1]:.4f}, surfaces {surface_mass:.4f}, motors {motor_mass:.4f})")
        computed = [label for label, value in (("fuselage structure", airframe.fuselage.structure_mass_kg),) if value is None]
        computed += [f"surface {surface.name}" for surface in airframe.surfaces if surface.mass_kg is None]
        if computed:
            print(f"  to be computed from geometry (not in declared mass): {', '.join(computed)}")


if __name__ == "__main__":
    main()
