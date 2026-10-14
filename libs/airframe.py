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
from libs.rocketmotors import RocketMotor, RocketMotorError, rocket_motor

SCHEMA = "airframe/2"
SECTION_PROFILE_KEYS = ("width_m", "height_m", "shape")
DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PARTS_DIR = DATA_DIR / "parts"  # the part library: one <category>.json per category
PRESETS_DIR = DATA_DIR / "presets"  # size presets: one <name>.json each, in the aircraft definition's own form
PARTS_DIMENSION_UNIT_M = 0.001  # part library battery dimensions are in mm
INCH_M = 0.0254
PROP_MASS_SCALING_EXPONENT = 3.0  # a prop outside data/parts/prop.json's listed sizes: geometrically similar blades, mass ~ diameter^3

LAYOUT_PANEL_COUNTS = {"H": 2, "X": 4, "+": 4, "1": 1, "3": 3}
LAYOUTS = tuple(LAYOUT_PANEL_COUNTS)
FIN_POSITIONS = ("top", "bottom")
INCIDENCE_MODES = ("symmetric", "cant")  # incidence pitches XZ-mirror panels alike (lift, like AoA), or turns every panel the same way about the body axis (roll)
SURFACE_ROOTS = ("body", "axis")  # span measured from the body surface at the root LE, or from the fuselage axis (fins joined at the centre)
VOLUME_SHAPES = ("box", "rounded_box", "cylinder", "bore", "point")
BOX_SHAPES = ("box", "rounded_box")  # sized by size_m [x, y, z]; a rounded box has its edges along axis rounded by corner_radius_m
BATTERY_MODELS = ("commercial", "custom_liion")  # a data/parts/battery.json pack, or one built from motorlib cells
BATTERY_PART_NAME = "battery"  # the propulsion battery among the placed parts (the parts list must not use the name)
CELL_FORMATS = {  # cylindrical cell formats in motorlib cell names: (diameter, length) in m
    "18650": (0.018, 0.065), "21700": (0.021, 0.070), "26650": (0.026, 0.065),
    "38120": (0.038, 0.120), "40146": (0.040, 0.146), "40152": (0.040, 0.152), "40160": (0.040, 0.160),
}
CELL_SPACING_M = 0.002  # custom pack: added to each cell's size along every axis (wrap, strip, gaps)
NOMINAL_CELL_V = {"lipo": 3.7, "liion": 3.6}  # commercial packs, by type
# Parts identified as a specific product with its own properties: one file each, data/parts/<role>/<name>.json (the role
# is the folder). Common fields: mass (kg), dimensions {l, w, h} (mm, x/y/z in the part's own frame), mount (a
# data/parts/mounting.json pattern, or null), wiring_origin_mm [x, y, z] (where its wires start, from its centre in its
# own frame), generic (true = typical values for layout, not a real product), notes; role-specific fields beside them.
PART_ROLES = ("fc", "esc", "camera", "companion", "vtx", "receiver", "gps", "servo")
PART_TABLES = ("battery", "connector", "prop", "material", "mounting", "screw")  # data/parts/<table>.json: {name: entry}
PART_CATEGORIES = ("battery", *PART_ROLES)  # what can be placed in an airframe (parts[], the battery)
AXES = ("x", "y", "z")
ORIENTATION_KEYS = ("yaw_deg", "pitch_deg", "roll_deg")  # a placed part's turn (PlacedPart), each optional (absent = 0)
MOUNT_TYPES = ("electric", "rocket")  # what a motor mount carries: an electric motor and prop, or a solid rocket motor
MOUNT_TARGETS = ("nose", "tail", "surface")
ROCKET_MOUNT_TARGETS = ("tail", "surface")  # a rocket motor fires aft: not from the nose
ELECTRIC_MOUNT_KEYS = ("direction", "spin", "clearance_m", "pod", "holes")  # mount fields only an electric mount has
THRUST_DIRECTIONS = ("tractor", "pusher")
# Prop spin, seen from where the thrust points (from above in hover). A surface mount carries 2+ motors that spin in
# pairs, props out or in; a nose or tail mount carries one motor, cw or ccw.
SURFACE_PROP_SPINS = ("out", "in")
SINGLE_PROP_SPINS = ("cw", "ccw")
JOINT_TYPES = ("radial_screws",)
CUT_JOINTS = ("coupler", "front_male", "rear_male", "none")  # at a cut: a separate coupler into both sections, the section ahead / behind carries the male end, or none (the sections butt with no coupler or spigot; reference the seam in CAD)
REINFORCEMENT_TYPES = ("cf_rod",)
POLAR_SPLITS = (0, 2)  # rods at each polar position: 0 = one, 2 = a pair either side of it
CONTROL_KINDS = ("split", "all_moving")
CONTROL_ROLES = ("pitch", "yaw", "roll", "flaps")  # for reference only
SERVO_OUTPUTS = tuple(f"S{number}" for number in range(1, 13))
ACCESS_KINDS = ("hole", "hatch")  # a hole subtracts material; a hatch keeps the original material inside an inset profile
ACCESS_PROFILES = ("slot", "rounded_rectangle")  # a slot is a capsule (equal extents = a circle); a rounded rectangle gives its radius
ACCESS_EXTENTS = ("depth", "wall", "all")  # depth_m along the local +Z; wall = into the first wall; all = through the whole body


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
class SectionProfile:
    """Optional cross-section overrides; absent values inherit fuselage.section."""
    width_m: float | None = None
    height_m: float | None = None
    shape: float | None = None

    def resolve(self, default: BodySection) -> BodySection:
        return BodySection(
            default.width_m if self.width_m is None else self.width_m,
            default.height_m if self.height_m is None else self.height_m,
            default.shape if self.shape is None else self.shape,
            default.wall_radial_m,
        )


@dataclass(frozen=True)
class EndProfile:
    profile: str
    length_m: float
    params: dict


@dataclass(frozen=True)
class Joints:
    """The seam at every cut, where the sections butt: a coupler (or a male spigot) of coupler_wall_m slides
    insertion_depth_m into a socket in each female end, with fit_clearance_m radial play (also round rods and spars
    through it). A socket leaves socket_wall_m of the section's skin over the coupler (bored to the outer surface less
    socket_wall_m), whatever the section's wall elsewhere. count radial screw holes per section end, hole_edge_dist_m
    from the cut, evenly round the axis from hole_angle_offset_deg (from +y), along the cut profile's wall normals."""
    type: str
    count: int  # screws per section end
    screw: str
    insertion_depth_m: float
    coupler_wall_m: float
    fit_clearance_m: float
    socket_wall_m: float
    hole_edge_dist_m: float
    hole_diameter_m: float
    hole_angle_offset_deg: float
    coupler_mass_kg: float | None  # weighed mass of one coupler; None = not counted yet

    def hole_angles_deg(self) -> tuple[float, ...]:
        return tuple(self.hole_angle_offset_deg + 360.0 * index / self.count for index in range(self.count))


@dataclass(frozen=True)
class RodPattern:
    """Longitudinal rods round the fuselage axis, mirror-symmetric left/right and top/bottom: one rod position per
    quadrant, polar_offset_angle (deg) up from the horizontal (+y), mirrored into the other three quadrants. polar_split 2
    makes each position a pair at polar_offset_angle +/- polar_split_angle (deg); 0 keeps a single rod. polar_n counts
    every rod. Every rod lies strictly inside its quadrant (0 < angle < 90)."""
    type: str  # REINFORCEMENT_TYPES
    material: str  # data/parts/material.json
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
class LoftStation:
    x_m: float
    section: SectionProfile


@dataclass(frozen=True)
class Fuselage:
    length_m: float
    section: BodySection
    nose: EndProfile
    tail: EndProfile
    loft_stations: tuple[LoftStation, ...]
    reinforcement: Reinforcement | None
    channels: tuple[Channel, ...]


@dataclass(frozen=True)
class ControlSurface:
    kind: str  # CONTROL_KINDS
    chord_start: float | None  # split: hinge line, fraction of the local chord from the leading edge
    span_start: float | None  # split: moving-surface inboard span fraction; all_moving: where the moving surface starts (0 = the root)
    span_end: float | None  # split: moving-surface outboard span fraction; all_moving: always None
    pivot: float | None  # all_moving: pivot line, fraction of the local chord from the leading edge
    align_hinge_vertically: bool  # True: the hinge runs straight across the panel span from the inboard anchor, not along the chord lines
    roles: tuple[str, ...]
    deflection_min_deg: float  # positive = trailing edge away from the panel normal (down on a wing)
    deflection_max_deg: float
    rate_deg_s: float  # placeholder, not used yet
    servos: tuple[str, ...]  # one SERVO_OUTPUTS entry per panel, in panel order; outputs may be shared
    gap_m: float  # material removed between the fixed and the moving surface (the moving surface's edges are inset by it); 0 = none
    rod_diameter_m: float | None  # the pivot rod the surface turns on, outer diameter; None = no rod
    rod_x_offset_m: float  # pivot rod offset along the chord direction (LE to TE); + aft
    rod_y_offset_m: float  # pivot rod offset towards the panel normal (up on a wing)


@dataclass(frozen=True)
class CarryThrough:
    """The wing carry-through: a printed web inside the tube, one piece with it and the panel roots, that the spars run
    through. One arm per panel along its spar line, stem_width_m thick across the panel plane, from the axis to the
    inner wall (wall_fillet_m, the fillet into it, is recorded for post-processing; nothing builds it); length_m long along the fuselage, centred between the spars, out to
    flare_radius_m from the axis, then flaring to root_length_m where it meets the wall."""
    stem_width_m: float
    length_m: float
    flare_radius_m: float
    root_length_m: float
    wall_fillet_m: float


@dataclass(frozen=True)
class Spars:
    """Straight rods through every panel of a set along its span, in the chord plane: from where the panel's span line
    passes closest to the fuselage axis (through the body, into the mirror panel) out to the tip. The first is x_offset_m
    aft of the root leading edge along the root chord, each further one spacing_m aft of it; all y_offset_m off the chord
    line through the thickness (+ towards the panel normal: up on a wing). Where the spar lines of
    several panel pairs cross (X, +), the first panel's pair runs through; the others stop at it."""
    material: str  # data/parts/material.json
    diameter_m: float
    count: int  # per panel
    spacing_m: float
    x_offset_m: float
    y_offset_m: float  # off the chord line through the thickness, + towards the panel normal (up on a wing)
    carry_through: CarryThrough | None  # None = the rods only


@dataclass(frozen=True)
class RootBase:
    """The base a panel's root sits on, to be blended into the body. Recorded only; nothing builds it yet."""
    length_m: float
    width_m: float
    height_m: float
    forward_offset_m: float
    leading_cut_m: float
    trailing_cut_m: float


@dataclass(frozen=True)
class SurfaceSections:
    cuts_m: tuple[float, ...]  # span distances from the true panel root; first segment stays on the fuselage


@dataclass(frozen=True)
class Surface:
    name: str
    x_root_le_m: float
    layout: str
    fin_position: str
    dihedral_deg: float
    vertical_offset: float  # root height on the body, -1 bottom .. 0 equator .. 1 top (X: root angle)
    root: str  # "body" or "axis", see SURFACE_ROOTS
    span_m: float  # per panel, from the root (body surface at the root LE, or the axis) along the panel
    chord_root_m: float
    chord_tip_m: float
    sweep_le_deg: float
    incidence_deg: float
    incidence_mode: str  # see INCIDENCE_MODES
    airfoil: str
    mass_kg: float | None  # whole set; None = computed from geometry
    control: ControlSurface | None  # every panel of the set carries one; None = fixed surface
    spars: Spars | None  # None = no spars
    root_base: RootBase | None  # None = none (not built yet either way)
    sections: SurfaceSections


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
class MetricScrew:
    """One metric screw size from data/parts/screw.json: the threaded-hole primitive a printed part is cut for. The
    clearance bore is what the screw passes through; the head bore is what it is inserted through; tap_drill_m is the
    bore a screw cuts its own thread in."""
    size: str
    clearance_m: float
    tap_drill_m: float
    head_diameter_m: float
    head_height_m: float


MOUNT_HOLE_TYPES = ("round", "slot")


@dataclass(frozen=True)
class MotorMountHoles:
    """The motor's fastening pattern on the mount face: count holes, the first at angle_deg (0 = towards +y, positive
    towards +z) and the rest evenly round: round holes on pattern_diameter_m, or radial slots from inner_diameter_m to
    outer_diameter_m. Each is the screw's clearance bore through the mount face; with a head bore (head_diameter_m and
    head_depth_m), the head bore runs from that seat depth on through the material in front, the way the screw is
    inserted. The centre relief clears the shaft."""
    type: str  # MOUNT_HOLE_TYPES
    count: int
    angle_deg: float
    screw: MetricScrew
    pattern_diameter_m: float | None  # round only
    inner_diameter_m: float | None  # slot only
    outer_diameter_m: float | None  # slot only
    center_diameter_m: float
    center_depth_m: float
    head_diameter_m: float | None  # head bore diameter (heads vary; the screw table's head diameter is only the default); None = no head bore
    head_depth_m: float | None  # seat distance inward from the mounting face; None = no head bore

    def segments_m(self) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
        """(start, end) of each hole in the mount-face plane (y, z), metres; a round hole has start == end."""
        segments = []
        for index in range(self.count):
            angle = math.radians(self.angle_deg + 360.0 * index / self.count)
            direction = (math.cos(angle), math.sin(angle))
            inner, outer = ((self.pattern_diameter_m,) * 2 if self.type == "round" else (self.inner_diameter_m, self.outer_diameter_m))
            segments.append((tuple(inner / 2.0 * d for d in direction), tuple(outer / 2.0 * d for d in direction)))
        return tuple(segments)


@dataclass(frozen=True)
class Mount:
    type: str  # MOUNT_TYPES
    on: str  # MOUNT_TARGETS; a rocket mount ROCKET_MOUNT_TARGETS
    surface: str | None
    span_position_m: float | None  # along each panel from its root
    x_offset_m: float  # from the surface root leading edge or the nose/tail mounting face; positive aft. A rocket
                       # mount: where the motor's aft (nozzle) end sits; the motor runs forward from there
    direction: str | None  # electric: THRUST_DIRECTIONS; None on a rocket mount (the nozzle points aft)
    spin: str | None  # electric: SURFACE_PROP_SPINS on a surface, SINGLE_PROP_SPINS on the nose or tail; None on a rocket mount
    thrust_axis_tilt_deg: float  # thrust line tilted up (+z) from the body axis, e.g. to pass through the CG
    pod: Pod | None  # electric only
    holes: MotorMountHoles | None = None  # electric only
    clearance_m: float | None = None  # electric only: added to the bell diameter for the mount clearance cut (everything of
                                      # the carrying wing/pod or section on the motor's side of the mount face, within it)


@dataclass(frozen=True)
class WireRun:
    """One cable run: wires of one gauge and the connectors along it, in order from its start."""
    gauge_awg: int  # motorlib wire gauge
    conductors: int  # wires in the cable (2 battery leads, 3 motor phases)
    plugs: tuple[str, ...]  # data/parts/connector.json names


@dataclass(frozen=True)
class Battery:
    """The propulsion pack, commercial (data/parts/battery.json) or a custom stack of motorlib cells."""
    model: str  # BATTERY_MODELS
    label: str  # the commercial pack's name, or e.g. "6S2P Molicel INR21700-P45B 7/10C"
    chemistry: str  # "lipo" / "liion" (commercial: its type), or the cell's name for a custom pack
    series: int
    parallel: int
    capacity_mah: float  # the whole pack
    cell_voltage_v: float  # nominal
    ir_ohm_per_cell: float | None  # None = not known
    connector: str  # data/parts/connector.json: the first plug along the battery run
    lead_awg: int | None  # the pack's own leads; None = not known (not modelled)
    lead_length_m: float | None
    placed: "PlacedPart"  # where it sits, its size and mass (also in Airframe.parts as BATTERY_PART_NAME)


@dataclass(frozen=True)
class Wiring:
    battery: WireRun  # battery -> ESC part; its plugs come after the battery's own connector
    motors: tuple[WireRun, ...]  # ESC part -> each motor, in motor order


@dataclass(frozen=True)
class ElectricPropulsion:
    """The electric system: one motor and prop model on every electric mount, the pack, the ESC and the wiring."""
    motor: MotorRef
    prop: str  # motorlib propeller (libs/motorlib/data/propellers.csv)
    battery: Battery
    esc: str | None  # name of the placed ESC part (battery leads end and motor leads start there); None = none placed yet
                     # (saved as null or ""): the wiring, propulsion summary and simulation wait for one
    wiring: Wiring  # motors: one run per electric motor, in motor order
    motor_mass_kg: float  # per motor, motorlib
    bell_diameter_m: float  # stator diameter, motorlib
    bell_length_m: float  # stator height, motorlib
    prop_diameter_m: float
    prop_mass_kg: float  # per prop, from data/parts/prop.json sizes


@dataclass(frozen=True)
class RocketLoad:
    """A solid rocket motor loaded into a rocket mount (on a surface mount: into every panel's position)."""
    mount: int  # mount number, 1-based as the editor shows "Mount N"
    motor: RocketMotor  # data/rocket_motors/<motor.name>.eng


@dataclass(frozen=True)
class Propulsion:
    mounts: tuple[Mount, ...]
    electric: ElectricPropulsion | None  # required when a mount is electric
    rocket: tuple[RocketLoad, ...]  # at most one per rocket mount; an empty rocket mount fires nothing

    @property
    def electric_mounts(self) -> tuple[tuple[int, Mount], ...]:
        """(mount number, mount) of every electric mount, in mount order."""
        return tuple((number, mount) for number, mount in enumerate(self.mounts, start=1) if mount.type == "electric")


@dataclass(frozen=True)
class Volume:
    name: str
    shape: str
    x_m: float  # centre
    y_m: float
    z_m: float
    size_m: tuple[float, float, float] | None  # box, rounded_box
    diameter_m: float | None  # cylinder
    length_m: float | None  # cylinder, bore
    axis: str | None  # cylinder; rounded_box: the rounded edges run along it
    corner_radius_m: float | None  # rounded_box
    mass_kg: tuple[float, float] | None  # its own mass (min, max), e.g. ballast; None = space only (parts carry theirs)


def orientation_rows(yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple[tuple[float, float, float], ...]:
    """Rows of the matrix turning an object's own axes into the body's: Rz(yaw) Ry(pitch) Rx(roll), roll first."""
    yaw, pitch, roll = (math.radians(angle) for angle in (yaw_deg, pitch_deg, roll_deg))
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


@dataclass(frozen=True)
class PlacedPart:
    """A library component, sitting in a volume or placed on its own, turned by orientation_deg (yaw about z, pitch about
    y, roll about x, applied roll first, about the body axes; all 0 = its own l, w, h along the body x, y, z)."""
    name: str
    part: str  # <category>:<name> in data/parts/<category>.json
    volume: str | None  # None = placed at its own x_m, y_m, z_m
    offset_m: tuple[float, float, float]  # from the volume centre; (0, 0, 0) without a volume
    position_m: tuple[float, float, float]  # volume centre + offset, or its own position
    size_m: tuple[float, float, float] | None  # library dimensions (x = l, y = w, z = h); None = none in the library (a point)
    mass_kg: tuple[float, float]  # library mass or mass_kg; 0 = counted elsewhere (e.g. in a weighed section)
    orientation_deg: tuple[float, float, float]  # yaw, pitch, roll

    def rotation(self) -> tuple[tuple[float, float, float], ...]:
        """Rows of the matrix turning the part's own axes into the body's: Rz(yaw) Ry(pitch) Rx(roll)."""
        return orientation_rows(*self.orientation_deg)

    def half_extents_m(self) -> tuple[float, float, float] | None:
        """Half extents along the body axes of its turned box; None without library dimensions."""
        if self.size_m is None:
            return None
        return tuple(sum(abs(value) * size / 2.0 for value, size in zip(row, self.size_m)) for row in self.rotation())


@dataclass(frozen=True)
class AccessRadial:
    """Radial repetition of one Access entry about an explicit axis line: count includes the seed; copy k (0 = the
    seed) and its local axes are turned 360 * k / count deg about the line, position and orientation together."""
    count: int  # >= 1, includes the seed
    axis: str  # AXES
    position_m: tuple[float, float, float]  # a point on the rotation axis, explicit


@dataclass(frozen=True)
class AccessEntry:
    """One named Hole or Hatch, saved on the airframe; shared physical cuts are built by libs.access. The
    profile lies in the entry's local XY plane: local X is its length, local Y its width, local +Z the extrusion/cut
    direction. width_m and length_m are the total opening extents (equal slot extents make a circle; a slot's length
    is its longest dimension); corner_radius_m is a rounded_rectangle's corner radius; gap_m (a hatch) is the
    clearance on each edge in the profile plane, so the retained material is the original profile inset by it.
    extent: depth_m along the local +Z, into the first wall (wall), or through the whole body (all)."""
    name: str
    kind: str  # ACCESS_KINDS
    profile: str  # ACCESS_PROFILES
    width_m: float
    length_m: float
    corner_radius_m: float | None  # rounded_rectangle only
    gap_m: float | None  # hatch only
    position_m: tuple[float, float, float]  # profile centre in the aircraft frame
    orientation_deg: tuple[float, float, float]  # yaw, pitch, roll; optional in the file, each 0
    extent: str  # ACCESS_EXTENTS
    depth_m: float | None  # depth only
    radial: AccessRadial | None  # None = the seed alone


@dataclass(frozen=True)
class FuselageSection:
    solid: bool  # printed solid less the volumes inside it; False = hollow tube (shell wall)
    empty_mass_kg: float | None  # weighed printed mass; None = computed
    complete_mass_kg: float | None  # weighed printed mass + everything positioned in the section; None = itemised
    fixed_length: bool = False  # preserve this section's length during automatic editor resizing


@dataclass(frozen=True)
class Sections:
    cuts_m: tuple[float, ...]  # cut stations along the fuselage, ascending; section i spans cut i-1 .. cut i
    per_section: tuple[FuselageSection, ...]  # len(cuts_m) + 1
    cut_joints: tuple[str, ...]  # CUT_JOINTS, one per cut
    joints: Joints


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


@dataclass(frozen=True)
class Preset:
    """A size preset (data/presets/<name>.json): the fuselage blocks it fills in, the spars a surface set starts with,
    the limits it checks, CAD data."""
    name: str
    section: BodySection
    joints: Joints
    reinforcement: RodPattern | None  # None = no rods
    spars: Spars | None  # None = none to start from
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
    access: tuple[AccessEntry, ...]  # named holes and hatches; resolved frames in libs.geometry.AccessInstance
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


def _end_profile(data: dict, path: str, flat_allowed: bool) -> EndProfile:
    """flat_allowed: length 0 is a flat end (a rocket's base) instead of a taper."""
    fields = _Fields(data, path)
    profile = normalize_shape_name(fields.text("profile"))
    if profile not in POINTED_SHAPE_PARAM_KEYS:
        raise AirframeError(f"{fields.at('profile')}: {profile!r} is not one of {sorted(POINTED_SHAPE_PARAM_KEYS)}")
    length_m = fields.number("length_m", minimum=0.0, exclusive=not flat_allowed)
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
        coupler_wall_m=fields.positive("coupler_wall_m"),
        fit_clearance_m=fields.number("fit_clearance_m", minimum=0.0),
        socket_wall_m=fields.positive("socket_wall_m"),
        hole_edge_dist_m=fields.positive("hole_edge_dist_m"),
        hole_diameter_m=fields.positive("hole_diameter_m"),
        hole_angle_offset_deg=fields.number("hole_angle_offset_deg"),
        coupler_mass_kg=None if "coupler_mass_kg" not in fields.data else fields.number("coupler_mass_kg", minimum=0.0, exclusive=True),
    )
    fields.done()
    if joints.hole_edge_dist_m + joints.hole_diameter_m / 2.0 > joints.insertion_depth_m:
        raise AirframeError(f"{fields.at('hole_edge_dist_m')}: screw holes {joints.hole_edge_dist_m} m from the section end (diameter {joints.hole_diameter_m}) "
                            f"miss the coupler, which reaches {joints.insertion_depth_m} m in")
    return joints


def _check_joints_fit(joints: Joints, section: BodySection, path: str) -> None:
    """A socket is bored out of the section's wall, never thicker than it; the coupler must leave a bore inside it."""
    if joints.socket_wall_m > section.wall_radial_m:
        raise AirframeError(f"{path}.socket_wall_m: {joints.socket_wall_m} is thicker than the section wall {section.wall_radial_m}")
    inner = min(section.width_m, section.height_m) / 2.0 - joints.socket_wall_m - joints.fit_clearance_m - joints.coupler_wall_m
    if inner <= 0.0:
        raise AirframeError(f"{path}.coupler_wall_m: {joints.coupler_wall_m} with {joints.fit_clearance_m} clearance fills the whole socket")


def _rod_pattern(fields: _Fields, parts_data: dict) -> RodPattern:
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
        raise AirframeError(f"{fields.at('material')}: {pattern.material!r} not in data/parts/material.json")
    if pattern.polar_n != 4 * pattern.group_size():
        raise AirframeError(f"{fields.at('polar_n')}: {pattern.polar_n} rods is not one {'pair' if pattern.polar_split == 2 else 'rod'} per quadrant "
                            f"({4 * pattern.group_size()})")
    angles = pattern.quadrant_angles_deg()
    if not (0.0 < angles[0] and angles[-1] < 90.0) or (len(angles) > 1 and pattern.polar_split_angle == 0.0):
        raise AirframeError(f"{fields.path}: rods at {list(angles)} deg from the horizontal; each must be distinct and strictly inside its quadrant (0-90)")
    return pattern


def _reinforcement(data: dict, path: str, section: BodySection, length_m: float, parts_data: dict) -> Reinforcement:
    fields = _Fields(data, path)
    pattern = _rod_pattern(fields, parts_data)
    reinforcement = Reinforcement(pattern=pattern, from_x_m=fields.number("from_x_m", minimum=0.0), to_x_m=fields.positive("to_x_m"))
    fields.done()
    if not reinforcement.from_x_m < reinforcement.to_x_m <= length_m:
        raise AirframeError(f"{path}: needs from_x_m < to_x_m <= fuselage length {length_m}")
    return reinforcement


def _fuselage(data: dict, path: str, parts_data: dict) -> Fuselage:
    fields = _Fields(data, path)
    length_m = fields.positive("length_m")
    section = _body_section(fields.required("section"), fields.at("section"))

    nose = _end_profile(fields.required("nose"), fields.at("nose"), flat_allowed=False)
    tail = _end_profile(fields.required("tail"), fields.at("tail"), flat_allowed=True)
    if nose.length_m + tail.length_m > length_m:
        raise AirframeError(f"{path}: nose + tail length {nose.length_m + tail.length_m} exceeds fuselage length {length_m}")
    for end_name, end in (("nose", nose), ("tail", tail)):
        try:
            check_profile(end.profile, end.length_m, None, end.params)  # build_body validates against the loft shoulder radius
        except ValueError as error:
            raise AirframeError(f"{path}.{end_name}: {error}") from error

    guide_data = fields.required("loft_stations")
    if type(guide_data) is not list:
        raise AirframeError(f"{fields.at('loft_stations')}: expected an array")
    guides = tuple(_loft_station(row, f"{fields.at('loft_stations')}[{index}]", section)
                   for index, row in enumerate(guide_data))
    positions = [guide.x_m for guide in guides]
    if positions != sorted(set(positions)) or any(x < 0 or x > length_m for x in positions):
        raise AirframeError(f"{fields.at('loft_stations')}: positions must be strictly ascending within 0..{length_m} m")
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
        loft_stations=guides,
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
        span_start = _fraction(fields, "span_start") if "span_start" in fields.data else 0.0
        if not 0.0 <= span_start < 1.0:
            raise AirframeError(f"{fields.at('span_start')}: {span_start} must be within [0, 1)")
        pivot = _fraction(fields, "pivot")
    gap_m = fields.optional("gap_m", 0.0)
    gap_m = fields.number("gap_m", minimum=0.0, value=gap_m)
    rod_diameter_m = fields.optional("rod_diameter_m", None)
    if rod_diameter_m is not None:
        rod_diameter_m = fields.number("rod_diameter_m", minimum=0.0, exclusive=True, value=rod_diameter_m)
    rod_x_offset_m = fields.number("rod_x_offset_m", value=fields.optional("rod_x_offset_m", 0.0))
    rod_y_offset_m = fields.number("rod_y_offset_m", value=fields.optional("rod_y_offset_m", 0.0))
    align_hinge_vertically = fields.flag("align_hinge_vertically") if "align_hinge_vertically" in fields.data else False
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
        kind=kind, chord_start=chord_start, span_start=span_start, span_end=span_end, pivot=pivot,
        align_hinge_vertically=align_hinge_vertically, roles=tuple(roles),
        deflection_min_deg=deflection_min_deg, deflection_max_deg=deflection_max_deg, rate_deg_s=rate_deg_s, servos=tuple(servos),
        gap_m=gap_m, rod_diameter_m=rod_diameter_m, rod_x_offset_m=rod_x_offset_m, rod_y_offset_m=rod_y_offset_m,
    )


def _carry_through(data: dict, path: str) -> CarryThrough:
    fields = _Fields(data, path)
    carry = CarryThrough(
        stem_width_m=fields.positive("stem_width_m"),
        length_m=fields.positive("length_m"),
        flare_radius_m=fields.number("flare_radius_m", minimum=0.0),
        root_length_m=fields.positive("root_length_m"),
        wall_fillet_m=fields.number("wall_fillet_m", minimum=0.0),
    )
    fields.done()
    if carry.root_length_m < carry.length_m:
        raise AirframeError(f"{fields.at('root_length_m')}: {carry.root_length_m} is shorter than length_m {carry.length_m}")
    return carry


def _spars(data: dict, path: str, parts_data: dict) -> Spars:
    fields = _Fields(data, path)
    carry = fields.optional("carry_through", None)
    spars = Spars(
        material=fields.text("material"),
        diameter_m=fields.positive("diameter_m"),
        count=int(fields.number("count", minimum=1)),
        spacing_m=fields.number("spacing_m", minimum=0.0),
        x_offset_m=fields.number("x_offset_m", minimum=0.0),
        y_offset_m=fields.number("y_offset_m"),
        carry_through=None if carry is None else _carry_through(carry, f"{path}.carry_through"),
    )
    fields.done()
    if spars.carry_through is not None and spars.carry_through.stem_width_m <= spars.diameter_m:
        raise AirframeError(f"{fields.at('carry_through')}.stem_width_m: {spars.carry_through.stem_width_m} does not hold spars of {spars.diameter_m}")
    if spars.material not in parts_data["material"]:
        raise AirframeError(f"{fields.at('material')}: {spars.material!r} not in data/parts/material.json")
    if spars.count > 1 and spars.spacing_m <= spars.diameter_m:
        raise AirframeError(f"{fields.at('spacing_m')}: {spars.spacing_m} does not clear spars of {spars.diameter_m}")
    return spars


def _root_base(data: dict, path: str) -> RootBase:
    fields = _Fields(data, path)
    base = RootBase(
        length_m=fields.number("length_m", minimum=0.0),
        width_m=fields.number("width_m", minimum=0.0),
        height_m=fields.number("height_m", minimum=0.0),
        forward_offset_m=fields.number("forward_offset_m"),
        leading_cut_m=fields.number("leading_cut_m", minimum=0.0),
        trailing_cut_m=fields.number("trailing_cut_m", minimum=0.0),
    )
    fields.done()
    return base


def _surface_sections(data: dict, path: str, span_m: float) -> SurfaceSections:
    fields = _Fields(data, path)
    values = fields.required("cuts_m")
    if type(values) is not list:
        raise AirframeError(f"{fields.at('cuts_m')}: expected an array")
    cuts = tuple(fields.number("cuts_m", minimum=0.0, exclusive=True, value=value) for value in values)
    fields.done()
    if list(cuts) != sorted(set(cuts)) or any(cut >= span_m for cut in cuts):
        raise AirframeError(f"{path}.cuts_m: cuts must be strictly ascending between 0 and span {span_m} m")
    return SurfaceSections(cuts_m=cuts)


def _surface(data: dict, path: str, fuselage_length_m: float, parts_data: dict) -> Surface:
    fields = _Fields(data, path)
    mass = fields.optional("mass_kg", None)
    layout = fields.choice("layout", LAYOUTS)
    control = fields.optional("control", None)
    spars = fields.optional("spars", None)
    root_base = fields.optional("root_base", None)
    span_m = fields.positive("span_m")
    surface = Surface(
        name=fields.text("name"),
        x_root_le_m=fields.number("x_root_le_m", minimum=0.0),
        layout=layout,
        fin_position=fields.choice("fin_position", FIN_POSITIONS),
        dihedral_deg=fields.number("dihedral_deg"),
        vertical_offset=fields.number("vertical_offset"),
        root=fields.choice("root", SURFACE_ROOTS),
        span_m=span_m,
        chord_root_m=fields.positive("chord_root_m"),
        chord_tip_m=fields.positive("chord_tip_m"),
        sweep_le_deg=fields.number("sweep_le_deg"),
        incidence_deg=fields.number("incidence_deg"),
        incidence_mode=fields.choice("incidence_mode", INCIDENCE_MODES),
        airfoil=fields.text("airfoil"),
        mass_kg=None if mass is None else fields.number("mass_kg", minimum=0.0, exclusive=True, value=mass),
        control=None if control is None else _control(control, f"{path}.control", LAYOUT_PANEL_COUNTS[layout]),
        spars=None if spars is None else _spars(spars, f"{path}.spars", parts_data),
        root_base=None if root_base is None else _root_base(root_base, f"{path}.root_base"),
        sections=_surface_sections(fields.required("sections"), f"{path}.sections", span_m),
    )
    fields.done()
    if not -1.0 <= surface.vertical_offset <= 1.0:
        raise AirframeError(f"{path}.vertical_offset: {surface.vertical_offset} must be within [-1, 1]")
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

    size_m = diameter_m = length_m = axis = corner_radius_m = None
    if shape in BOX_SHAPES:
        size = fields.required("size_m")
        if type(size) is not list or len(size) != 3:
            raise AirframeError(f"{fields.at('size_m')}: expected [x, y, z], got {size!r}")
        size_m = tuple(fields.number("size_m", minimum=0.0, exclusive=True, value=item) for item in size)
    if shape == "rounded_box":
        axis = fields.choice("axis", AXES)
        corner_radius_m = fields.positive("corner_radius_m")
        across = [value for name, value in zip(AXES, size_m) if name != axis]
        if 2.0 * corner_radius_m > min(across):
            raise AirframeError(f"{fields.at('corner_radius_m')}: {corner_radius_m} is more than half the {min(across)} m side across {axis}")
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
        length_m=length_m, axis=axis, corner_radius_m=corner_radius_m, mass_kg=mass_kg,
    )


def _placement(fields: "_Fields", volumes: dict[str, Volume]) -> tuple[str | None, tuple[float, float, float], tuple[float, float, float]]:
    """(volume, offset from its centre, position): in a volume with x/y/z_offset_m, or at its own x_m, y_m, z_m."""
    if "volume" in fields.data:
        volume_name = fields.text("volume")
        if volume_name not in volumes:
            raise AirframeError(f"{fields.at('volume')}: {volume_name!r} is not a defined volume")
        volume = volumes[volume_name]
        offset_m = tuple(fields.number(key) if key in fields.data else 0.0 for key in ("x_offset_m", "y_offset_m", "z_offset_m"))
        return volume_name, offset_m, (volume.x_m + offset_m[0], volume.y_m + offset_m[1], volume.z_m + offset_m[2])
    return None, (0.0, 0.0, 0.0), (fields.number("x_m"), *(fields.number(key) if key in fields.data else 0.0 for key in ("y_m", "z_m")))


def _orientation(fields: "_Fields") -> tuple[float, float, float]:
    """yaw_deg, pitch_deg, roll_deg of a placed part (each absent = 0)."""
    return tuple(fields.number(key) if key in fields.data else 0.0 for key in ORIENTATION_KEYS)


def _part(data: dict, path: str, volumes: dict[str, Volume], parts_data: dict) -> PlacedPart:
    fields = _Fields(data, path)
    name = fields.text("name")
    part = fields.text("part")
    category, _, part_name = part.partition(":")
    if category not in PART_CATEGORIES or part_name not in parts_data[category]:
        raise AirframeError(f"{fields.at('part')}: {part!r} not found; use '<category>:<name>' from data/parts/<category>.json, category one of {PART_CATEGORIES}")
    part_data = parts_data[category][part_name]
    if category in PART_ROLES:
        _check_role_part(f"data/parts/{category}/{part_name}.json", part_data, parts_data["mounting"])
    volume_name, offset_m, position_m = _placement(fields, volumes)
    orientation_deg = _orientation(fields)
    if "mass_kg" in fields.data:
        value = fields.required("mass_kg")
        mass_kg = _mass_range(value, fields.at("mass_kg")) if type(value) is list else (fields.number("mass_kg", minimum=0.0, value=value),) * 2
    elif part_data.get("mass"):  # null or 0 in the library = not known
        mass_kg = _mass_range(part_data["mass"], f"{fields.at('part')} ({part}) mass")
    else:
        raise AirframeError(f"{fields.at('mass_kg')}: {part!r} has no mass in its data/parts/ file; give mass_kg (0 = counted elsewhere)")
    fields.done()
    dims = part_data.get("dimensions")
    return PlacedPart(
        name=name, part=part, volume=volume_name, offset_m=offset_m, position_m=position_m,
        size_m=None if dims is None else (dims["l"] * PARTS_DIMENSION_UNIT_M, dims["w"] * PARTS_DIMENSION_UNIT_M, dims["h"] * PARTS_DIMENSION_UNIT_M),
        mass_kg=mass_kg, orientation_deg=orientation_deg,
    )


def _access(data: dict, path: str) -> AccessEntry:
    """One saved access entry: fields only applicable to its kind, profile or extent must be present exactly then."""
    fields = _Fields(data, path)
    name = fields.text("name")
    kind = fields.choice("kind", ACCESS_KINDS)
    profile = fields.choice("profile", ACCESS_PROFILES)
    width_m = fields.positive("width_m")
    length_m = fields.positive("length_m")
    if profile == "slot" and length_m < width_m:
        raise AirframeError(f"{fields.at('length_m')}: a slot's length is its longest dimension; {length_m} < width_m {width_m}")
    corner_radius_m = None
    if profile == "rounded_rectangle":
        corner_radius_m = fields.positive("corner_radius_m")
        if corner_radius_m > min(width_m, length_m) / 2.0:
            raise AirframeError(f"{fields.at('corner_radius_m')}: {corner_radius_m} is more than half the smaller profile extent {min(width_m, length_m)}")
    gap_m = fields.number("gap_m", minimum=0.0) if kind == "hatch" else None
    if gap_m is not None and min(width_m, length_m) <= 2.0 * gap_m:
        raise AirframeError(f"{fields.at('gap_m')}: {gap_m} on each edge leaves no material inside the profile (its smaller extent is {min(width_m, length_m)})")
    extent = fields.choice("extent", ACCESS_EXTENTS)
    depth_m = fields.positive("depth_m") if extent == "depth" else None
    radial_data = fields.optional("radial", None)
    radial = None
    if radial_data is not None:
        radial_fields = _Fields(radial_data, fields.at("radial"))
        count = radial_fields.number("count", minimum=1)
        if count != int(count):
            raise AirframeError(f"{radial_fields.at('count')}: expected a whole number, got {count!r}")
        radial = AccessRadial(
            count=int(count),
            axis=radial_fields.choice("axis", AXES),
            position_m=tuple(radial_fields.number(key) for key in ("x_m", "y_m", "z_m")),
        )
        radial_fields.done()
    x_m = fields.number("x_m")
    y_m = fields.number("y_m") if "y_m" in fields.data else 0.0
    z_m = fields.number("z_m") if "z_m" in fields.data else 0.0
    orientation_deg = _orientation(fields)
    fields.done()
    return AccessEntry(
        name=name, kind=kind, profile=profile, width_m=width_m, length_m=length_m, corner_radius_m=corner_radius_m,
        gap_m=gap_m, position_m=(x_m, y_m, z_m), orientation_deg=orientation_deg, extent=extent,
        depth_m=depth_m, radial=radial,
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
    """Prop mass from data/parts/prop.json entries: linear in diameter between listed sizes; outside them, the nearest listed
    size scaled by diameter^PROP_MASS_SCALING_EXPONENT."""
    table = sorted((entry["diameter_in"], entry["mass"]) for entry in parts_data["prop"].values())
    if not table[0][0] <= diameter_in <= table[-1][0]:
        nearest_in, nearest_kg = table[0] if diameter_in < table[0][0] else table[-1]
        return nearest_kg * (diameter_in / nearest_in) ** PROP_MASS_SCALING_EXPONENT
    for (d0, m0), (d1, m1) in zip(table, table[1:]):
        if d0 <= diameter_in <= d1:
            return m0 + (m1 - m0) * (diameter_in - d0) / (d1 - d0)
    return table[-1][1]


def motor_count(mounts, layouts: dict[str, str]) -> int:
    """Motors from (on, surface name) per mount: one per panel of a surface mount, else one; layouts by surface name."""
    return sum(LAYOUT_PANEL_COUNTS[layouts[surface]] if on == "surface" else 1 for on, surface in mounts)


def cell_format(name: str) -> tuple[float, float] | None:
    """(diameter, length) of a motorlib cell from the CELL_FORMATS format in its name; None when it has none."""
    return next((size for code, size in CELL_FORMATS.items() if code in name), None)


def _wire_gauge(fields: "_Fields", key: str) -> int:
    gauge = fields.required(key)
    if type(gauge) is not int or gauge not in {wire.gauge for wire in motor_repo().wires}:
        raise AirframeError(f"{fields.at(key)}: {gauge!r} is not a motorlib wire gauge {sorted(wire.gauge for wire in motor_repo().wires)}")
    return gauge


def _connector(fields: "_Fields", value, key: str, parts_data: dict) -> str:
    connectors = parts_data["connector"]
    if value not in connectors or "contact_resistance_ohm" not in connectors[value]:
        usable = sorted(name for name, connector in connectors.items() if "contact_resistance_ohm" in connector)
        raise AirframeError(f"{fields.at(key)}: {value!r} needs to be a data/parts/connector.json entry with a contact_resistance_ohm {usable}")
    return value


def _battery(data: dict, path: str, volumes: dict[str, Volume], parts_data: dict) -> Battery:
    fields = _Fields(data, path)
    model = fields.choice("model", BATTERY_MODELS)
    volume_name, offset_m, position_m = _placement(fields, volumes)
    orientation_deg = _orientation(fields)
    if model == "commercial":
        name = fields.text("part")
        if name not in parts_data["battery"]:
            raise AirframeError(f"{fields.at('part')}: {name!r} is not in data/parts/battery.json {sorted(parts_data['battery'])}")
        entry = parts_data["battery"][name]
        if "mass_kg" in fields.data:
            value = fields.required("mass_kg")
            mass_kg = _mass_range(value, fields.at("mass_kg")) if type(value) is list else (fields.number("mass_kg", minimum=0.0, value=value),) * 2
        elif entry.get("mass"):
            mass_kg = _mass_range(entry["mass"], f"data/parts/battery.json {name} mass")
        else:
            raise AirframeError(f"{fields.at('part')}: {name!r} has no mass in data/parts/battery.json; give mass_kg")
        for key in ("type", "s", "p", "mah"):
            if key not in entry:
                raise AirframeError(f"data/parts/battery.json {name}: {key} is required for a propulsion battery")
        dims = entry.get("dimensions")
        size_m = None if dims is None else (dims["l"] * PARTS_DIMENSION_UNIT_M, dims["w"] * PARTS_DIMENSION_UNIT_M, dims["h"] * PARTS_DIMENSION_UNIT_M)
        leads = "lead_awg" in entry and "lead_length_mm" in entry
        if leads and entry["lead_awg"] not in {wire.gauge for wire in motor_repo().wires}:
            raise AirframeError(f"data/parts/battery.json {name}: lead_awg {entry['lead_awg']!r} is not a motorlib wire gauge {sorted(wire.gauge for wire in motor_repo().wires)}")
        fields.done()
        connector = _connector(fields, entry.get("connector"), "part", parts_data)
        return Battery(
            model=model, label=name, chemistry=entry["type"], series=entry["s"], parallel=entry["p"], capacity_mah=entry["mah"] * entry["p"],
            cell_voltage_v=NOMINAL_CELL_V[entry["type"]], ir_ohm_per_cell=entry["ir_mohm_per_cell"] / 1000.0 if "ir_mohm_per_cell" in entry else None,
            connector=connector, lead_awg=entry["lead_awg"] if leads else None, lead_length_m=entry["lead_length_mm"] * PARTS_DIMENSION_UNIT_M if leads else None,
            placed=PlacedPart(name=BATTERY_PART_NAME, part=f"battery:{name}", volume=volume_name, offset_m=offset_m, position_m=position_m, size_m=size_m, mass_kg=mass_kg,
                              orientation_deg=orientation_deg),
        )
    series, parallel = (fields.required(key) for key in ("s", "p"))
    for key, value in (("s", series), ("p", parallel)):
        if type(value) is not int or value < 1:
            raise AirframeError(f"{fields.at(key)}: expected a whole number >= 1, got {value!r}")
    cell_name = fields.text("cell")
    cells = motor_repo().cell_by_name
    if cell_name not in cells or cell_format(cell_name) is None:
        stackable = sorted(name for name in cells if cell_format(name) is not None)
        raise AirframeError(f"{fields.at('cell')}: {cell_name!r} needs to be a motorlib cell (libs/motorlib/data/cells.csv) with a format {sorted(CELL_FORMATS)} "
                            f"in its name: {stackable}")
    cell = cells[cell_name]
    connector = _connector(fields, fields.text("connector"), "connector", parts_data)
    axis = fields.choice("cell_axis", AXES)
    stacking = fields.required("stacking")
    if type(stacking) is not list or len(stacking) != 3 or any(type(count) is not int or count < 1 for count in stacking):
        raise AirframeError(f"{fields.at('stacking')}: expected cells along x, y, z as three whole numbers >= 1, got {stacking!r}")
    if stacking[0] * stacking[1] * stacking[2] != series * parallel:
        raise AirframeError(f"{fields.at('stacking')}: {stacking[0]} x {stacking[1]} x {stacking[2]} = {stacking[0] * stacking[1] * stacking[2]} cells, "
                            f"but {series}S{parallel}P has {series * parallel}")
    lead_awg = _wire_gauge(fields, "lead_awg")
    lead_length_m = fields.number("lead_length_m", minimum=0.0)
    fields.done()
    diameter_m, length_m = cell_format(cell_name)
    size_m = tuple(count * ((length_m if AXES[index] == axis else diameter_m) + CELL_SPACING_M) for index, count in enumerate(stacking))
    mass_kg = series * parallel * cell.weight_g / 1000.0
    label = f"{series}S{parallel}P {cell_name}"
    return Battery(
        model=model, label=label, chemistry=cell_name, series=series, parallel=parallel, capacity_mah=cell.cap_mah * parallel,
        cell_voltage_v=cell.volt, ir_ohm_per_cell=cell.res_ohm, connector=connector, lead_awg=lead_awg, lead_length_m=lead_length_m,
        placed=PlacedPart(name=BATTERY_PART_NAME, part=f"battery:{label}", volume=volume_name, offset_m=offset_m, position_m=position_m, size_m=size_m,
                          mass_kg=(mass_kg, mass_kg), orientation_deg=orientation_deg),
    )


def _wire_run(data: dict, path: str, parts_data: dict) -> WireRun:
    fields = _Fields(data, path)
    gauge = _wire_gauge(fields, "gauge_awg")
    conductors = fields.required("conductors")
    if type(conductors) is not int or conductors < 2:
        raise AirframeError(f"{fields.at('conductors')}: expected a whole number >= 2, got {conductors!r}")
    plugs = fields.required("plugs")
    connectors = parts_data["connector"]
    if type(plugs) is not list or any(plug not in connectors or "contact_resistance_ohm" not in connectors[plug] for plug in plugs):
        usable = sorted(name for name, connector in connectors.items() if "contact_resistance_ohm" in connector)
        raise AirframeError(f"{fields.at('plugs')}: expected a list of data/parts/connector.json entries with a contact_resistance_ohm {usable}, got {plugs!r}")
    fields.done()
    return WireRun(gauge_awg=gauge, conductors=conductors, plugs=tuple(plugs))


def metric_screw(size: str, screws: dict, path: str) -> MetricScrew:
    """The data/parts/screw.json entry of a screw size, in metres."""
    if size not in screws:
        raise AirframeError(f"{path}: {size!r} is not a size in data/parts/screw.json {list(screws)}")
    fields = _Fields(dict(screws[size]), f"data/parts/screw.json {size}")
    return MetricScrew(size=size, clearance_m=fields.positive("clearance_mm") / 1000.0, tap_drill_m=fields.positive("tap_drill_mm") / 1000.0,
                       head_diameter_m=fields.positive("head_diameter_mm") / 1000.0, head_height_m=fields.positive("head_height_mm") / 1000.0)


def _motor_mount_holes(data: dict, path: str, screws: dict) -> MotorMountHoles:
    """A mount's holes: the fields of its type exactly (pattern_diameter_m round; inner/outer_diameter_m slot); the head
    bore's head_diameter_m and head_depth_m together or neither."""
    fields = _Fields(data, path)
    head_bore = "head_diameter_m" in data or "head_depth_m" in data
    hole_type = fields.choice("type", MOUNT_HOLE_TYPES)
    count = fields.required("count")
    if type(count) is not int or count < 1:
        raise AirframeError(f"{fields.at('count')}: must be a whole number >= 1, got {count!r}")
    holes = MotorMountHoles(
        type=hole_type, count=count, angle_deg=fields.number("angle_deg"),
        screw=metric_screw(fields.text("screw"), screws, fields.at("screw")),
        pattern_diameter_m=fields.number("pattern_diameter_m", minimum=0.0) if hole_type == "round" else None,
        inner_diameter_m=fields.number("inner_diameter_m", minimum=0.0) if hole_type == "slot" else None,
        outer_diameter_m=fields.positive("outer_diameter_m") if hole_type == "slot" else None,
        center_diameter_m=fields.number("center_diameter_m", minimum=0.0),
        center_depth_m=fields.number("center_depth_m", minimum=0.0),
        head_diameter_m=fields.positive("head_diameter_m") if head_bore else None,
        head_depth_m=fields.number("head_depth_m", minimum=0.0) if head_bore else None,
    )
    fields.done()
    if holes.head_diameter_m is not None and holes.head_diameter_m < holes.screw.clearance_m:
        raise AirframeError(f"{fields.at('head_diameter_m')}: {holes.head_diameter_m} is less than the {holes.screw.size} clearance bore {holes.screw.clearance_m}")
    if holes.type == "slot" and holes.outer_diameter_m <= holes.inner_diameter_m:
        raise AirframeError(f"{fields.at('outer_diameter_m')}: {holes.outer_diameter_m} must be more than inner_diameter_m {holes.inner_diameter_m}")
    return holes


def _mount(data: dict, path: str, surfaces: dict[str, Surface], screws: dict) -> Mount:
    fields = _Fields(data, path)
    mount_type = fields.choice("type", MOUNT_TYPES)
    on = fields.choice("on", MOUNT_TARGETS if mount_type == "electric" else ROCKET_MOUNT_TARGETS)
    surface_name = span_position_m = None
    x_offset_m = fields.number("x_offset_m")
    if on == "surface":
        surface_name = fields.text("surface")
        if surface_name not in surfaces:
            raise AirframeError(f"{fields.at('surface')}: {surface_name!r} is not a defined surface")
        span_position_m = fields.number("span_position_m", minimum=0.0)
        if span_position_m > surfaces[surface_name].span_m:
            raise AirframeError(f"{fields.at('span_position_m')}: {span_position_m} is past the panel span {surfaces[surface_name].span_m}")
    thrust_axis_tilt_deg = fields.number("thrust_axis_tilt_deg")
    if mount_type == "rocket":
        fields.done()
        return Mount(type=mount_type, on=on, surface=surface_name, span_position_m=span_position_m, x_offset_m=x_offset_m, direction=None,
                     spin=None, thrust_axis_tilt_deg=thrust_axis_tilt_deg, pod=None)
    direction = fields.choice("direction", THRUST_DIRECTIONS)
    if (on, direction) in (("nose", "pusher"), ("tail", "tractor")):
        raise AirframeError(f"{fields.at('direction')}: a {on} motor must be a {'tractor' if on == 'nose' else 'pusher'}")
    spin = fields.choice("spin", SURFACE_PROP_SPINS if on == "surface" else SINGLE_PROP_SPINS)
    clearance_m = fields.number("clearance_m", minimum=0.0)
    pod_data = fields.optional("pod", None)
    pod = None
    if pod_data is not None:
        pod_fields = _Fields(pod_data, fields.at("pod"))
        pod = Pod(
            diameter_m=pod_fields.positive("diameter_m"),
            flat_length_m=pod_fields.number("flat_length_m", minimum=0.0),
            curve_length_m=pod_fields.number("curve_length_m", minimum=0.0),
            profile=normalize_shape_name(pod_fields.text("profile")),
        )
        pod_fields.done()
        if POINTED_SHAPE_PARAM_KEYS.get(pod.profile) != []:
            raise AirframeError(f"{fields.at('pod')}.profile: {pod.profile!r} must be a parameter-free profile {[key for key, value in POINTED_SHAPE_PARAM_KEYS.items() if not value]}")
        try:
            check_profile(pod.profile, pod.curve_length_m, pod.diameter_m / 2.0)
        except ValueError as error:
            raise AirframeError(f"{fields.at('pod')}: {error}") from error
    holes_data = fields.optional("holes", None)
    holes = None if holes_data is None else _motor_mount_holes(holes_data, fields.at("holes"), screws)
    fields.done()
    return Mount(type=mount_type, on=on, surface=surface_name, span_position_m=span_position_m, x_offset_m=x_offset_m, direction=direction, spin=spin,
                 thrust_axis_tilt_deg=thrust_axis_tilt_deg, pod=pod, holes=holes, clearance_m=clearance_m)


def _electric(data: dict, path: str, mounts: tuple[Mount, ...], surfaces: tuple[Surface, ...], parts: tuple[PlacedPart, ...], volumes: dict[str, Volume],
              parts_data: dict) -> ElectricPropulsion:
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
    battery = _battery(fields.required("battery"), fields.at("battery"), volumes, parts_data)
    esc = fields.required("esc")
    if esc in (None, ""):  # not placed yet: a design in progress (geometry.electric_checks reports it)
        esc = None
    elif categories.get(esc) != "esc":
        raise AirframeError(f"{fields.at('esc')}: {esc!r} must name a part whose part is 'esc:<name>'; esc parts: {sorted(n for n, c in categories.items() if c == 'esc')}")

    wiring_fields = _Fields(fields.required("wiring"), fields.at("wiring"))
    count = motor_count(((mount.on, mount.surface) for mount in mounts if mount.type == "electric"), {surface.name: surface.layout for surface in surfaces})
    motor_runs = wiring_fields.required("motors")
    if type(motor_runs) is not list or len(motor_runs) != count:
        raise AirframeError(f"{wiring_fields.at('motors')}: needs one run per electric motor ({count}), in motor order")
    wiring = Wiring(battery=_wire_run(wiring_fields.required("battery"), wiring_fields.at("battery"), parts_data),
                    motors=tuple(_wire_run(run, f"{wiring_fields.at('motors')}[{index}]", parts_data) for index, run in enumerate(motor_runs)))
    wiring_fields.done()
    fields.done()
    return ElectricPropulsion(
        motor=motor, prop=prop, battery=battery, esc=esc, wiring=wiring, motor_mass_kg=motor_data.weight_g / 1000.0,
        bell_diameter_m=motor_data.stator_diameter_mm / 1000.0, bell_length_m=motor_data.stator_height_mm / 1000.0,
        prop_diameter_m=prop_data.diam_in * INCH_M, prop_mass_kg=prop_mass_kg(prop_data.diam_in, parts_data),
    )


def _rocket(data: dict, path: str, mounts: tuple[Mount, ...]) -> tuple[RocketLoad, ...]:
    fields = _Fields(data, path)
    loads = []
    for index, load_data in enumerate(fields.required("motors")):
        load_fields = _Fields(load_data, f"{fields.at('motors')}[{index}]")
        number = load_fields.required("mount")
        if type(number) is not int or not 1 <= number <= len(mounts) or mounts[number - 1].type != "rocket":
            rocket_numbers = [n for n, mount in enumerate(mounts, start=1) if mount.type == "rocket"]
            raise AirframeError(f"{load_fields.at('mount')}: {number!r} must be the number of a rocket mount {rocket_numbers}")
        if number in (load.mount for load in loads):
            raise AirframeError(f"{load_fields.at('mount')}: mount {number} already holds a motor")
        name = load_fields.text("motor")
        try:
            motor = rocket_motor(name)
        except RocketMotorError as error:
            raise AirframeError(f"{load_fields.at('motor')}: {error}") from error
        load_fields.done()
        loads.append(RocketLoad(mount=number, motor=motor))
    fields.done()
    return tuple(loads)


def _propulsion(data: dict, path: str, surfaces: tuple[Surface, ...], parts: tuple[PlacedPart, ...], volumes: dict[str, Volume], parts_data: dict) -> Propulsion:
    fields = _Fields(data, path)
    surface_names = {surface.name: surface for surface in surfaces}
    mounts = tuple(_mount(mount_data, f"{fields.at('mounts')}[{index}]", surface_names, parts_data["screw"]) for index, mount_data in enumerate(fields.required("mounts")))
    if not mounts:
        raise AirframeError(f"{fields.at('mounts')}: at least one mount is required")
    electric_data = fields.optional("electric", None)
    if electric_data is None and any(mount.type == "electric" for mount in mounts):
        raise AirframeError(f"{fields.at('electric')}: required: electric mounts {[n for n, mount in enumerate(mounts, start=1) if mount.type == 'electric']} need the electric system")
    electric = None if electric_data is None else _electric(electric_data, fields.at("electric"), mounts, surfaces, parts, volumes, parts_data)
    rocket_data = fields.optional("rocket", None)
    rocket = () if rocket_data is None else _rocket(rocket_data, fields.at("rocket"), mounts)
    fields.done()
    return Propulsion(mounts=mounts, electric=electric, rocket=rocket)


def _limit_range(fields: _Fields, key: str) -> tuple[float, float]:
    value = fields.required(key)
    if type(value) is not list or len(value) != 2:
        raise AirframeError(f"{fields.at(key)}: a limit is [min, max], got {value!r}")
    low, high = (fields.number(key, minimum=0.0, exclusive=True, value=item) for item in value)
    if low > high:
        raise AirframeError(f"{fields.at(key)}: needs min <= max, got {value!r}")
    return low, high


def _preset(data: dict, name: str, parts_data: dict) -> Preset:
    fields = _Fields(data, f"data/presets/{name}.json")
    section = _body_section(fields.required("section"), fields.at("section"))
    joints = _joints(fields.required("joints"), fields.at("joints"))
    _check_joints_fit(joints, section, fields.at("joints"))
    reinforcement = limits = cad = spars = None
    if "spars" in fields.data:
        spars = _spars(fields.required("spars"), fields.at("spars"), parts_data)
    if "reinforcement" in fields.data:
        rod_fields = _Fields(fields.required("reinforcement"), fields.at("reinforcement"))
        reinforcement = _rod_pattern(rod_fields, parts_data)
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
        )
        cad_fields.done()
    fields.done()
    return Preset(name=name, section=section, joints=joints, reinforcement=reinforcement, spars=spars, limits=limits, cad=cad)


def load_presets(parts_data: dict) -> dict[str, Preset]:
    """The size presets in data/presets/, validated."""
    return {path.stem: _preset(json.loads(path.read_text()), path.stem, parts_data) for path in sorted(PRESETS_DIR.glob("*.json"))}


def _section_profile(data: dict, path: str, default: BodySection) -> SectionProfile:
    fields = _Fields(data, path)
    profile = SectionProfile(
        width_m=fields.positive("width_m") if "width_m" in data else None,
        height_m=fields.positive("height_m") if "height_m" in data else None,
        shape=fields.number("shape", minimum=1.0) if "shape" in data else None,
    )
    fields.done()
    resolved = profile.resolve(default)
    if min(resolved.width_m, resolved.height_m) <= 2.0 * resolved.wall_radial_m:
        raise AirframeError(f"{path}: width/height leaves no internal space with wall {resolved.wall_radial_m} m")
    return profile


def _loft_station(data: dict, path: str, default: BodySection) -> LoftStation:
    fields = _Fields(data, path)
    x_m = fields.number("x_m", minimum=0.0)
    return LoftStation(x_m, _section_profile(fields.data, path, default))


def _fuselage_section(data: dict, path: str) -> FuselageSection:
    fields = _Fields(data, path)
    section = FuselageSection(
        solid=fields.flag("solid"),
        empty_mass_kg=None if "empty_mass_kg" not in fields.data else fields.positive("empty_mass_kg"),
        complete_mass_kg=None if "complete_mass_kg" not in fields.data else fields.positive("complete_mass_kg"),
        fixed_length=fields.flag("fixed_length") if "fixed_length" in fields.data else False,
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
        presets = load_presets(parts_data)
        if preset_name not in presets:
            raise AirframeError(f"{fields.at('preset')}: {preset_name!r} not in data/presets/ {sorted(presets)}")
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
        raise AirframeError(f"{print_fields.at('material')}: {material!r} not in data/parts/material.json")
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
        _surface(surface_data, f"{fields.at('surfaces')}[{index}]", fuselage.length_m, parts_data)
        for index, surface_data in enumerate(fields.required("surfaces"))
    )
    unweighed = [surface.name for surface in surfaces if surface.mass_kg is None]
    unweighed += [f"{surface.name} carry-through" for surface in surfaces if surface.spars is not None and surface.spars.carry_through is not None]
    if unweighed and print_settings.bulk_density_kg_m3 is None:
        raise AirframeError(f"{print_fields.at('bulk_density_kg_m3')}: required to compute the mass of {unweighed}")
    volumes = tuple(
        _volume(volume_data, f"{fields.at('volumes')}[{index}]")
        for index, volume_data in enumerate(fields.required("volumes"))
    )
    volumes_by_name = {volume.name: volume for volume in volumes}
    parts = tuple(
        _part(part_data, f"{fields.at('parts')}[{index}]", volumes_by_name, parts_data)
        for index, part_data in enumerate(fields.required("parts"))
    )
    access_list = fields.required("access")
    if type(access_list) is not list:
        raise AirframeError(f"{fields.at('access')}: expected a list, got {type(access_list).__name__}")
    access = tuple(
        _access(entry_data, f"{fields.at('access')}[{index}]")
        for index, entry_data in enumerate(access_list)
    )
    for collection, label in ((surfaces, "surfaces"), (volumes, "volumes"), (parts, "parts"), (access, "access")):
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
    if BATTERY_PART_NAME in {part.name for part in parts}:
        raise AirframeError(f"{fields.at('parts')}: {BATTERY_PART_NAME!r} is the propulsion battery's name; rename that part")
    propulsion = None if propulsion_data is None else _propulsion(propulsion_data, fields.at("propulsion"), surfaces, parts, volumes_by_name, parts_data)
    if vtol and not propulsion.electric_mounts:
        raise AirframeError(f"{fields.at('vtol')}: a VTOL hovers on electric motors; it needs an electric mount")
    if propulsion is not None and propulsion.electric is not None:
        parts = (*parts, propulsion.electric.battery.placed)

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
    cut_joints = sections_fields.required("cut_joints")
    if type(cut_joints) is not list or len(cut_joints) != len(cuts) or any(joint not in CUT_JOINTS for joint in cut_joints):
        raise AirframeError(f"{sections_fields.at('cut_joints')}: needs one of {list(CUT_JOINTS)} per cut ({len(cuts)}), got {cut_joints!r}")
    joints = _joints(sections_fields.required("joints"), sections_fields.at("joints"))
    if joints.socket_wall_m > fuselage.section.wall_radial_m:
        raise AirframeError(f"{sections_fields.at('joints')}.socket_wall_m exceeds the fuselage wall")
    sections_fields.done()
    unweighed_solid = [f"section {index + 1}" for index, section in enumerate(per_section) if section.solid and section.empty_mass_kg is None]
    if unweighed_solid and print_settings.bulk_density_kg_m3 is None:
        raise AirframeError(f"{print_fields.at('bulk_density_kg_m3')}: required to compute the mass of solid {unweighed_solid}")

    fields.done()
    return Airframe(
        name=name, notes=notes, preset=preset, powered=powered, vtol=vtol, tailsitter=tailsitter, operating_point=operating_point, print=print_settings,
        fuselage=fuselage, surfaces=surfaces, propulsion=propulsion, volumes=volumes, parts=parts, access=access,
        sections=Sections(cuts_m=cuts, per_section=per_section, cut_joints=tuple(cut_joints), joints=joints),
    )


def load_parts_data() -> dict[str, dict]:
    """The part library as {category: {name: entry}}: the tables from data/parts/<table>.json, each role's parts from
    data/parts/<role>/<name>.json (named by the file). A role part is checked where an airframe places it."""
    data = {path.stem: json.loads(path.read_text()) for path in sorted(PARTS_DIR.glob("*.json"))}
    for role in PART_ROLES:
        data[role] = {path.stem: json.loads(path.read_text()) for path in sorted((PARTS_DIR / role).glob("*.json"))}
    return data


def _check_role_part(path: str, entry: dict, mounting: dict) -> None:
    """The common fields of a role part (PART_ROLES); a mass of 0 or null = not known."""
    fields = _Fields(dict(entry), path)
    if fields.optional("mass", None) is not None:
        fields.number("mass", minimum=0.0, value=entry["mass"])
    dims = _Fields(fields.required("dimensions"), fields.at("dimensions"))
    for key in ("l", "w", "h"):
        dims.positive(key)
    dims.done()
    mount = fields.required("mount")
    if mount is not None and mount not in mounting:
        raise AirframeError(f"{fields.at('mount')}: {mount!r} is not a pattern in data/parts/mounting.json {sorted(mounting)}")
    origin = fields.required("wiring_origin_mm")
    if type(origin) is not list or len(origin) != 3:
        raise AirframeError(f"{fields.at('wiring_origin_mm')}: expected [x, y, z], got {origin!r}")
    for value in origin:
        fields.number("wiring_origin_mm", value=value)


def save_parts_category(category: str, entries: dict) -> None:
    """Write a category back: a role as one file per part (files of parts no longer listed removed), a table as its file."""
    if category not in PART_ROLES:
        (PARTS_DIR / f"{category}.json").write_text(json.dumps(entries, indent=4) + "\n")
        return
    folder = PARTS_DIR / category
    folder.mkdir(exist_ok=True)
    for path in folder.glob("*.json"):
        if path.stem not in entries:
            path.unlink()
    for name, entry in entries.items():
        (folder / f"{name}.json").write_text(json.dumps(entry, indent=4) + "\n")


def load_airframe(path: Path, parts_data: dict) -> Airframe:
    with open(path, "r") as file:
        data = json.load(file)
    return parse_airframe(data, parts_data, path=path.name)


def blank_definition(name: str) -> dict:
    """The starting point for File > New: a valid airframe holding one default tube, with nothing on it yet.
    Surfaces, volumes and parts start empty, so the editors open on an empty aircraft that still resolves."""
    return {
        "schema": SCHEMA,
        "powered": False,
        "vtol": False,
        "tailsitter": False,
        "name": name,
        "operating_point": {"aoa_deg": 3.0, "airspeed_m_s": 20.0, "altitude_m": 0.0},
        "print": {"material": "elegoo_pla_plus", "bulk_density_kg_m3": 318.3, "printer_bed_m": [0.24, 0.24, 0.25]},
        "fuselage": {
            "length_m": 0.4,
            "section": {"width_m": 0.05, "height_m": 0.05, "shape": 2.0, "wall_radial_m": 0.00125},
            "nose": {"profile": "elliptical", "length_m": 0.05, "params": {}},
            "tail": {"profile": "parabolic", "length_m": 0.05, "params": {"k_prime": 1.0}},
            "loft_stations": [],
            "channels": [],
        },
        "surfaces": [],
        "volumes": [],
        "parts": [],
        "access": [],
        "sections": {"cuts_m": [], "per_section": [{"solid": False}], "cut_joints": [],
                     "joints": {"type": "radial_screws", "count": 8, "screw": "M3", "insertion_depth_m": 0.01,
                       "coupler_wall_m": 0.0045, "fit_clearance_m": 0.0003, "socket_wall_m": 0.00125,
                       "hole_edge_dist_m": 0.006, "hole_diameter_m": 0.003, "hole_angle_offset_deg": 22.5}},
    }


def main() -> None:
    parts_data = load_parts_data()
    for argument in sys.argv[1:]:
        airframe = load_airframe(Path(argument), parts_data)
        motors = 0
        motor_mass = 0.0
        if airframe.propulsion is not None and airframe.propulsion.electric is not None:
            electric = airframe.propulsion.electric
            motors = len(electric.wiring.motors)
            motor_mass = motors * (electric.motor_mass_kg + electric.prop_mass_kg)
        volume_mass = [sum(volume.mass_kg[i] for volume in airframe.volumes if volume.mass_kg is not None) for i in (0, 1)]
        surface_mass = sum(surface.mass_kg for surface in airframe.surfaces if surface.mass_kg is not None)
        section_mass = sum(section.empty_mass_kg for section in airframe.sections.per_section if section.empty_mass_kg is not None)
        coupler_mass = airframe.sections.cut_joints.count("coupler") * (airframe.sections.joints.coupler_mass_kg or 0.0)
        declared = [volume_mass[i] + surface_mass + motor_mass + section_mass + coupler_mass for i in (0, 1)]
        print(f"{argument}: {airframe.name!r} valid")
        print(f"  fuselage {airframe.fuselage.length_m:.3f} m, {len(airframe.surfaces)} surface sets, {len(airframe.volumes)} volumes, {motors} motors")
        print(f"  declared mass {declared[0]:.4f}-{declared[1]:.4f} kg (volumes {volume_mass[0]:.4f}-{volume_mass[1]:.4f}, surfaces {surface_mass:.4f}, motors+props {motor_mass:.4f}, sections {section_mass:.4f}, couplers {coupler_mass:.4f})")
        computed = [f"section {index + 1}" for index, section in enumerate(airframe.sections.per_section) if section.empty_mass_kg is None]
        computed += [f"surface {surface.name}" for surface in airframe.surfaces if surface.mass_kg is None]
        if computed:
            print(f"  to be computed from geometry (not in declared mass): {', '.join(computed)}")


if __name__ == "__main__":
    main()
