"""Fastener cut geometry shared by printable CAD and the live drawing previews.

Coordinates are metres in the aircraft frame. Joint axes use the cut profile's
normal so both mating pieces are drilled on the same line, even across a loft.
"""

import bisect
import math
from dataclasses import dataclass

from libs.geometry import MotorPlacement, ResolvedAirframe, Vec, body_section_at, cross, superellipse_radius, unit

CUT_MARGIN_M = 0.0005
OPENING_PROFILE_SAMPLES = 24
SKIN_REACH_SAMPLES = 5


@dataclass(frozen=True)
class JointHole:
    name: str
    start: Vec
    end: Vec
    diameter_m: float

    def rings(self) -> tuple[tuple[Vec, ...], tuple[Vec, ...]]:
        axis = unit(tuple(b - a for a, b in zip(self.start, self.end)))
        u = (1.0, 0.0, 0.0)
        v = cross(axis, u)
        return tuple(tuple(tuple(c + self.diameter_m / 2 * (math.cos(2 * math.pi * step / OPENING_PROFILE_SAMPLES) * ui
                                                         + math.sin(2 * math.pi * step / OPENING_PROFILE_SAMPLES) * vi)
                                 for c, ui, vi in zip(center, u, v))
                           for step in range(OPENING_PROFILE_SAMPLES)) for center in (self.start, self.end))


@dataclass(frozen=True)
class MountOpening:
    """One mount-face cut: a capsule from start to end in the face plane (y, z), diameter_m wide, from depth0_m to
    depth1_m along inward. carrier is the (surface, panel index) whose wing and pod it is cut into, None for the
    fuselage (a nose or tail motor)."""
    name: str
    face: Vec
    inward: Vec
    start: tuple[float, float]
    end: tuple[float, float]
    diameter_m: float
    depth0_m: float
    depth1_m: float
    carrier: tuple[str, int] | None

    def rings(self) -> tuple[tuple[Vec, ...], tuple[Vec, ...]]:
        """Two capsule outlines; coincident slot centres produce a circular hole."""
        angle = math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])
        profile = []
        for center, turn in ((self.end, 0), (self.start, 1)):
            for step in range(OPENING_PROFILE_SAMPLES // 2 + 1):
                theta = angle - math.pi / 2 + math.pi * (turn + step / (OPENING_PROFILE_SAMPLES // 2))
                profile.append((center[0] + self.diameter_m / 2 * math.cos(theta),
                                center[1] + self.diameter_m / 2 * math.sin(theta)))
        return tuple(tuple((self.face[0] + depth * self.inward[0], self.face[1] + y, self.face[2] + z)
                           for y, z in profile) for depth in (self.depth0_m, self.depth1_m))


def joint_hole_stations(resolved: ResolvedAirframe, cut_index: int) -> tuple[float, ...]:
    joints = resolved.airframe.sections.joints
    cut = resolved.airframe.sections.cuts_m[cut_index]
    joint = resolved.airframe.sections.cut_joints[cut_index]
    if joint == 'coupler':
        return cut - joints.hole_edge_dist_m, cut + joints.hole_edge_dist_m
    if joint == 'front_male':
        return (cut + joints.hole_edge_dist_m,)
    if joint == 'rear_male':
        return (cut - joints.hole_edge_dist_m,)
    return ()


def _skin_exit(y: float, z: float, normal: Vec, half_w: float, half_h: float, shape: float) -> float:
    """Exit of an outward ray through a convex superellipse (zero if already outside)."""
    if min(half_w, half_h) <= 0 or (abs(y / half_w) ** shape + abs(z / half_h) ** shape) >= 1:
        return 0.0
    low, high = 0.0, 2 * math.hypot(half_w, half_h)
    for _ in range(48):
        middle = (low + high) / 2
        value = abs((y + middle * normal[1]) / half_w) ** shape + abs((z + middle * normal[2]) / half_h) ** shape
        if value < 1:
            low = middle
        else:
            high = middle
    return high


def joint_holes_at(resolved: ResolvedAirframe, cut_index: int, x_m: float) -> tuple[JointHole, ...]:
    joints = resolved.airframe.sections.joints
    cut = body_section_at(resolved.body, resolved.airframe.sections.cuts_m[cut_index])
    half_w, half_h = cut.width_m / 2, cut.height_m / 2
    inset = joints.socket_wall_m + joints.fit_clearance_m + joints.coupler_wall_m
    holes = []
    for index, degrees in enumerate(joints.hole_angles_deg()):
        angle = math.radians(degrees)
        radius = superellipse_radius(half_w, half_h, cut.shape, angle)
        y, z = radius * math.cos(angle), radius * math.sin(angle)
        normal = unit((0.0,
                       math.copysign(abs(y / half_w) ** (cut.shape - 1) / half_w, y) if abs(y) > 1e-15 else 0.0,
                       math.copysign(abs(z / half_h) ** (cut.shape - 1) / half_h, z) if abs(z) > 1e-15 else 0.0))
        radius = superellipse_radius(half_w - inset, half_h - inset, cut.shape, angle)
        y, z = radius * math.cos(angle), radius * math.sin(angle)
        reach = _skin_exit(y, z, normal, half_w, half_h, cut.shape)
        # A hole has finite width along x; include the surrounding loft, not just its centre.
        for sample in range(SKIN_REACH_SAMPLES):
            sample_x = x_m + joints.hole_diameter_m * (sample / (SKIN_REACH_SAMPLES - 1) - .5)
            local = body_section_at(resolved.body, min(resolved.body.length_m, max(0.0, sample_x)))
            reach = max(reach, _skin_exit(y, z, normal, local.width_m / 2, local.height_m / 2, local.shape))
        start = (x_m, y - CUT_MARGIN_M * normal[1], z - CUT_MARGIN_M * normal[2])
        end = (x_m, y + (reach + CUT_MARGIN_M) * normal[1], z + (reach + CUT_MARGIN_M) * normal[2])
        holes.append(JointHole(f'joint {cut_index + 1} screw {index + 1}', start, end, joints.hole_diameter_m))
    return tuple(holes)


def joint_holes(resolved: ResolvedAirframe) -> tuple[JointHole, ...]:
    return tuple(hole for index in range(len(resolved.airframe.sections.cuts_m))
                 for station in joint_hole_stations(resolved, index) for hole in joint_holes_at(resolved, index, station))


def _extent_x(resolved: ResolvedAirframe, motor: MotorPlacement, direction: float) -> float:
    """The last x of the material that carries a motor, going along x in direction (+1 aft, -1 forward): a surface
    motor's whole panel and pod; a nose or tail motor's fuselage section on that side of its mount face."""
    if motor.panel is not None:
        panel = motor.panel
        xs = [panel.attach_le[0], panel.attach_te[0], panel.tip_le[0], panel.tip_te[0], *(x for x, _radius in motor.pod_stations)]
        return max(xs) if direction > 0 else min(xs)
    bounds = (0.0, *resolved.airframe.sections.cuts_m, resolved.body.length_m)
    cuts = resolved.airframe.sections.cuts_m
    index = bisect.bisect_right(cuts, motor.mount_face[0]) if direction > 0 else bisect.bisect_left(cuts, motor.mount_face[0])
    return bounds[index + 1] if direction > 0 else bounds[index]


def mount_perimeter_m(motor: MotorPlacement) -> float:
    """The mount clearance cut's diameter: the motor bell's plus the mount's clearance."""
    return motor.bell_diameter_m + motor.mount_clearance_m


def motor_mount_openings(resolved: ResolvedAirframe) -> tuple[MountOpening, ...]:
    """Each electric motor's mount cuts. The mount clearance: everything of the carrying wing and pod (or fuselage
    section) on the motor's side of the mount face, within the mount's perimeter, so a pod can sit recessed into the
    wing with a channel behind it. With holes: the screw's clearance bores through the mount face and on through the
    material in front (the screws go in from there), the head bores from the seat depth on, and the shaft relief."""
    openings = []
    for motor in resolved.motors:
        inward = motor.mount_inward[0]
        carrier = None if motor.panel is None else (motor.panel.surface, motor.panel.index)
        behind = (_extent_x(resolved, motor, -inward) - motor.mount_face[0]) * -inward + CUT_MARGIN_M
        if behind > 0:
            openings.append(MountOpening(f'{motor.name} mount clearance', motor.mount_face, motor.mount_inward, (0., 0.), (0., 0.),
                                         mount_perimeter_m(motor), -behind, 0.0, carrier))
        holes = motor.mount_holes
        if holes is None:
            continue
        reach = (_extent_x(resolved, motor, inward) - motor.mount_face[0]) * inward + CUT_MARGIN_M
        if reach <= 0:
            continue
        for number, (start, end) in enumerate(holes.segments_m(), start=1):
            name = f'{motor.name} bolt {number}'
            openings.append(MountOpening(name, motor.mount_face, motor.mount_inward, start, end, holes.screw.clearance_m, -CUT_MARGIN_M, reach, carrier))
            if holes.head_depth_m is not None and holes.head_depth_m < reach:
                openings.append(MountOpening(f'{name} head bore', motor.mount_face, motor.mount_inward, start, end,
                                             holes.head_diameter_m, holes.head_depth_m, reach, carrier))
        if holes.center_diameter_m and holes.center_depth_m:
            openings.append(MountOpening(f'{motor.name} shaft relief', motor.mount_face, motor.mount_inward, (0., 0.), (0., 0.),
                                         holes.center_diameter_m, -CUT_MARGIN_M, min(holes.center_depth_m, reach), carrier))
    return tuple(openings)
