"""
Usage:
from libs.cad import build_airframe_assembly, export_sections, export_step, export_stl
export_step(resolved, "out.step", mode="assembly")
written, problems = export_sections(resolved, project.sections_dir)

CadQuery solids of a resolved airframe (libs/geometry.py): the whole airframe (assembly grouped by fuselage section,
or one fused outer shape), and the printable parts: each fuselage section with what is attached in it, and the
couplers. Solid geometry is built in millimetres so STEP/STL files open at correct scale in FreeCAD and meshing tools.
cadquery is imported lazily so the GUI starts light. FreeCAD varsets: libs/varsets.py.
"""

import math

from libs.airframe import BOX_SHAPES
from libs.geometry import (
    Body, CarryThroughArm, Panel, ResolvedAirframe, body_half_extents, build_body, carry_through_front_profile, carry_through_side_profile,
    chord_direction, section_index, superellipse_radius, thickness_direction,
)

M_TO_MM = 1000.0
PROFILE_SAMPLES = 48
SECTION_POLYGON_SAMPLES = 72
AIRFOIL_POINTS_PER_SIDE = 40
PROP_DISC_THICKNESS_MM = 1.0
GEOM_EPS_MM = 1e-4
LOFT_TIP_MM = 0.05  # non-circular bodies loft to a tiny section instead of a point
FLAT_PLATE_SECTION = ((0.0, 0.01), (1.0, 0.01), (1.0, -0.01), (0.0, -0.01))  # chord fractions, for an airfoil with no thickness
FUSE_ROOT_EXTEND_FRACTION = 0.75  # of the body half width: body-rooted panels reach this far into the body when fused
CUT_OVERSHOOT_MM = 0.5  # holes and bores reach this far past the material they cut, so no skin is left
CARRY_THROUGH_WALL_EMBED = 0.5  # of the wall: carry-through arms reach this far into the tube wall, fusing to it
_PRINTABLE_PARTS: dict[str, list] = {}  # the last design's printable parts, by its definition (build_printable_parts)
SPIGOT_WALL_EMBED = 0.5  # of the wall: a male spigot's root ring reaches this far into the carrying section's wall, fusing to it


def _revolve_profile_solid(stations: list[tuple[float, float]]):
    """Revolve an (x_mm, radius_mm) station polyline around the X axis into a solid.

    Curved sections are represented by dense station sampling rather than
    splines: an interpolating spline through a pointed-tip profile can
    overshoot to negative radius and self-intersect the revolve, and the
    polyline volume is closer to analytic anyway at PROFILE_SAMPLES density.
    The profile is closed against the axis at both ends.
    """
    import cadquery as cq

    points: list[tuple[float, float]] = []
    for pt in stations:
        if points and math.hypot(pt[0] - points[-1][0], pt[1] - points[-1][1]) < GEOM_EPS_MM:
            continue
        points.append(pt)
    if len(points) < 2:
        raise RuntimeError("Revolve profile needs at least two distinct stations")

    first = points[0]
    last = points[-1]
    workplane = cq.Workplane("XY").moveTo(*first)
    for pt in points[1:]:
        workplane = workplane.lineTo(*pt)
    if last[1] > GEOM_EPS_MM:
        workplane = workplane.lineTo(last[0], 0.0)
    if first[1] > GEOM_EPS_MM:
        workplane = workplane.lineTo(first[0], 0.0)
    return workplane.close().revolve(360.0, (0, 0, 0), (1, 0, 0))


def _airfoil_section_coordinates(airfoil_name: str):
    import aerosandbox as asb

    coordinates = asb.Airfoil(airfoil_name).repanel(n_points_per_side=AIRFOIL_POINTS_PER_SIDE).coordinates
    thickness = float(coordinates[:, 1].max() - coordinates[:, 1].min())
    if thickness < 1e-4:
        return list(FLAT_PLATE_SECTION)
    return [(float(x), float(y)) for x, y in coordinates]


def _clipped_stations(body: Body, x0: float, x1: float) -> list[tuple[float, float, float]]:
    """(x, half_width, half_height) from x0 to x1, with interpolated end stations."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    return [(x, *body_half_extents(body, x)) for x in xs]


def _superellipse_wire(x_mm: float, half_width_mm: float, half_height_mm: float, shape: float):
    """A section of the body's shape at station x as one closed periodic spline through SECTION_POLYGON_SAMPLES points, so
    a loft between sections is one face per segment rather than one per sample (which made booleans crawl)."""
    import cadquery as cq

    half_width_mm = max(half_width_mm, LOFT_TIP_MM)
    half_height_mm = max(half_height_mm, LOFT_TIP_MM)
    points = []
    for index in range(SECTION_POLYGON_SAMPLES):
        angle = 2.0 * math.pi * index / SECTION_POLYGON_SAMPLES
        radius = superellipse_radius(half_width_mm, half_height_mm, shape, angle)
        points.append(cq.Vector(x_mm, radius * math.cos(angle), radius * math.sin(angle)))
    return cq.Wire.assembleEdges([cq.Edge.makeSpline(points, periodic=True)])


def build_body_solid(resolved: ResolvedAirframe, x0: float, x1: float, inset_m: float = 0.0):
    """Fuselage between two stations, capped flat at the ends: revolved when circular, lofted otherwise. inset_m shrinks
    it radially (the bore inside a wall of inset_m), down to one point station where the body is thinner than that;
    None when nothing is left."""
    import cadquery as cq

    body = build_body(resolved.airframe, PROFILE_SAMPLES)
    stations = [(x, max(half_w - inset_m, 0.0), max(half_h - inset_m, 0.0)) for x, half_w, half_h in _clipped_stations(body, x0, x1)]
    solid = [index for index, (_x, half_w, half_h) in enumerate(stations) if min(half_w, half_h) > 0.0]
    if not solid:
        return None
    stations = stations[max(solid[0] - 1, 0): solid[-1] + 2]  # one point station either side of the part with a section
    circular = body.shape == 2.0 and all(abs(half_w - half_h) < 1e-12 for _x, half_w, half_h in stations)
    if circular:
        return _revolve_profile_solid([(x * M_TO_MM, half_w * M_TO_MM) for x, half_w, _half_h in stations])
    wires = [_superellipse_wire(x * M_TO_MM, half_w * M_TO_MM, half_h * M_TO_MM, body.shape) for x, half_w, half_h in stations]
    return cq.Workplane("XY").newObject([cq.Solid.makeLoft(wires, ruled=True)])


def build_wing_solid(panel: Panel, root_extend_m: float = 0.0):
    """Loft one panel between its root and tip airfoil sections, in the panel's own chord/normal frame.

    Body-rooted panels start at their true root leading edge and reach into the body far enough to meet a
    tapering cone under the root chord (plus root_extend_m for fused exports); axis-rooted panels start on the axis.
    """
    import cadquery as cq

    coordinates = _airfoil_section_coordinates(panel.airfoil)
    chord_dir = chord_direction(panel)
    thick_dir = thickness_direction(panel)

    def section_wire(leading_edge, chord_m: float):
        """The airfoil as one spline from the upper trailing edge round the leading edge to the lower one, closed by a
        straight trailing edge (kept sharp): a loft is then two faces per segment, smooth round the leading edge, rather
        than one per sample (slow booleans, and no outline at the leading edge in a hidden-line drawing)."""
        chord_mm = chord_m * M_TO_MM
        points = [
            cq.Vector(*(leading_edge[axis] * M_TO_MM + chord_mm * (x * chord_dir[axis] + y * thick_dir[axis]) for axis in range(3)))
            for x, y in coordinates
        ]
        if len(points) == len(FLAT_PLATE_SECTION):
            return cq.Wire.makePolygon(points, close=True)
        if (points[0] - points[-1]).Length < GEOM_EPS_MM:
            points = points[:-1]
        return cq.Wire.assembleEdges([cq.Edge.makeSpline(points), cq.Edge.makeLine(points[-1], points[0])])

    root_drop = max(0.0, math.hypot(panel.attach_le[1], panel.attach_le[2]) - math.hypot(panel.attach_te[1], panel.attach_te[2]))
    extend = root_drop + root_extend_m
    wires = [section_wire(panel.attach_le, panel.chord_root_m), section_wire(panel.tip_le, panel.chord_tip_m)]
    if extend > 0.0:
        inner = tuple(point - extend * direction for point, direction in zip(panel.attach_le, panel.span_dir))
        wires.insert(0, section_wire(inner, panel.chord_root_m))
    return cq.Workplane("XY").newObject([cq.Solid.makeLoft(wires, ruled=True)])


def build_motor_solid(motor):
    """Motor bell as a cylinder along X centred on the motor position."""
    import cadquery as cq

    x, y, z = motor.position
    start = cq.Vector((x - motor.bell_length_m / 2.0) * M_TO_MM, y * M_TO_MM, z * M_TO_MM)
    solid = cq.Solid.makeCylinder(motor.bell_diameter_m / 2.0 * M_TO_MM, motor.bell_length_m * M_TO_MM, start, cq.Vector(1, 0, 0))
    return cq.Workplane("XY").newObject([solid])


def build_pod_solid(motor):
    """Motor pod revolved from its resolved stations, on the motor's axis line."""
    stations = [(x * M_TO_MM, radius * M_TO_MM) for x, radius in motor.pod_stations]
    return _revolve_profile_solid(stations).translate((0.0, motor.position[1] * M_TO_MM, motor.position[2] * M_TO_MM))


def build_prop_disc_solid(motor):
    """Thin disc at the propeller plane, for actuator-disc CFD domain marking."""
    import cadquery as cq

    x, y, z = motor.prop_center
    start = cq.Vector(x * M_TO_MM - PROP_DISC_THICKNESS_MM / 2.0, y * M_TO_MM, z * M_TO_MM)
    solid = cq.Solid.makeCylinder(motor.prop_radius_m * M_TO_MM, PROP_DISC_THICKNESS_MM, start, cq.Vector(1, 0, 0))
    return cq.Workplane("XY").newObject([solid])


def _profile_prism(origin, u_dir, v_dir, profile, depth_dir, depth_mm: float):
    """A closed (u, v) profile (m) in the plane through origin spanned by u_dir and v_dir, extruded depth_mm both ways
    along depth_dir, as a cq.Solid."""
    import cadquery as cq

    base = cq.Vector(*origin) * M_TO_MM - cq.Vector(*depth_dir) * depth_mm
    points = [base + (cq.Vector(*u_dir) * u + cq.Vector(*v_dir) * v) * M_TO_MM for u, v in profile]
    return cq.Solid.extrudeLinear(cq.Wire.makePolygon(points, close=True), [], cq.Vector(*depth_dir) * (2.0 * depth_mm))


def build_carry_through_arm(resolved: ResolvedAirframe, arm: CarryThroughArm):
    """One carry-through arm (cq.Solid): its side profile and its front profile, each extruded across the other, intersected;
    it reaches into the tube wall so it fuses with it. The side profile runs on well past the wall (a wall not square to
    the arm, e.g. on a dihedral wing, is further away at one side); the front profile's cap inside the wall bounds it."""
    carry = next(surface_set.definition.spars.carry_through for surface_set in resolved.surfaces if surface_set.name == arm.surface)
    wall = resolved.airframe.fuselage.section.wall_radial_m
    embed = CARRY_THROUGH_WALL_EMBED * wall
    reach = arm.wall_distance_m + carry.stem_width_m + wall
    half_width_mm = carry.stem_width_m / 2.0 * M_TO_MM + CUT_OVERSHOOT_MM
    half_length_mm = carry.root_length_m / 2.0 * M_TO_MM + CUT_OVERSHOOT_MM
    side = _profile_prism(arm.start, (1.0, 0.0, 0.0), arm.direction, carry_through_side_profile(carry, arm.meet_m, arm.wall_distance_m, reach), arm.across, half_width_mm)
    front = _profile_prism(arm.start, arm.across, arm.direction, carry_through_front_profile(carry, arm.start, arm.across, arm.direction, arm.wall_half_extents_m, arm.section_shape, embed),
                           (1.0, 0.0, 0.0), half_length_mm)
    return side.intersect(front)


def section_bounds(resolved: ResolvedAirframe) -> list[tuple[float, float]]:
    bounds = [0.0, *resolved.airframe.sections.cuts_m, resolved.body.length_m]
    return list(zip(bounds, bounds[1:]))


def build_airframe_assembly(resolved: ResolvedAirframe, include_prop_discs: bool = False):
    """Nested CadQuery assembly: one sub-assembly per fuselage section with its body piece and the surfaces,
    motors and pods whose root or position lies in it."""
    import cadquery as cq

    assembly = cq.Assembly(name=resolved.airframe.name or "airframe")
    children = {index: cq.Assembly(name=f"S{index + 1:02d}") for index in range(len(section_bounds(resolved)))}
    for index, (x0, x1) in enumerate(section_bounds(resolved)):
        children[index].add(build_body_solid(resolved, x0, x1), name=f"S{index + 1:02d}_body")
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            index = section_index(resolved.airframe.sections.cuts_m, panel.attach_le[0])
            children[index].add(build_wing_solid(panel), name=f"S{index + 1:02d}_{surface_set.name}_{panel.index + 1}")
    for motor_index, motor in enumerate(resolved.motors, start=1):
        index = section_index(resolved.airframe.sections.cuts_m, motor.position[0])
        children[index].add(build_motor_solid(motor), name=f"S{index + 1:02d}_motor_{motor_index}")
        if motor.pod_stations:
            children[index].add(build_pod_solid(motor), name=f"S{index + 1:02d}_pod_{motor_index}")
        if include_prop_discs:
            children[index].add(build_prop_disc_solid(motor), name=f"S{index + 1:02d}_prop_{motor_index}")
    for index, child in children.items():
        assembly.add(child, name=f"S{index + 1:02d}")
    return assembly


def _airframe_solids(resolved: ResolvedAirframe, include_prop_discs: bool, root_extend_m: float) -> list:
    solids = [build_body_solid(resolved, 0.0, resolved.body.length_m)]
    for surface_set in resolved.surfaces:
        extend = 0.0 if surface_set.definition.root == "axis" else root_extend_m
        solids += [build_wing_solid(panel, extend) for panel in surface_set.panels]
    for motor in resolved.motors:
        solids.append(build_motor_solid(motor))
        if motor.pod_stations:
            solids.append(build_pod_solid(motor))
        if include_prop_discs:
            solids.append(build_prop_disc_solid(motor))
    return solids


def _fuse_airframe(resolved: ResolvedAirframe, include_prop_discs: bool = False) -> tuple[object, str]:
    """Boolean-union all solids into one shape; fall back to a compound if the fuse fails.

    Body-rooted panels are extended into the fuselage so they merge with it instead of only touching.
    """
    import cadquery as cq

    root_extend_m = FUSE_ROOT_EXTEND_FRACTION * resolved.airframe.fuselage.section.width_m / 2.0
    shapes = [workplane.val() for workplane in _airframe_solids(resolved, include_prop_discs, root_extend_m)]
    try:
        fused = shapes[0]
        for shape in shapes[1:]:
            fused = fused.fuse(shape)
        return fused.clean(), "fused"
    except Exception as error:  # OCC boolean failures: report and fall back to an unfused compound
        print(f"cad: fuse failed ({error}); exporting unfused compound instead")
        return cq.Compound.makeCompound(shapes), "compound"


def export_step(resolved: ResolvedAirframe, path, mode: str = "assembly", include_prop_discs: bool = False) -> str:
    """Export the airframe as STEP. mode = "assembly" (named parts per section) | "fused" (single OML).

    Returns the mode actually written ("assembly", "fused", or "compound" if the boolean fuse failed).
    """
    import cadquery as cq

    if mode == "assembly":
        build_airframe_assembly(resolved, include_prop_discs).export(str(path), "STEP")
        return "assembly"
    if mode == "fused":
        shape, result_mode = _fuse_airframe(resolved, include_prop_discs)
        cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STEP")
        return result_mode
    raise ValueError(f"Unknown STEP export mode: {mode}")


def export_stl(resolved: ResolvedAirframe, path, include_prop_discs: bool = False) -> str:
    """Export the airframe as a single STL mesh (fused where possible)."""
    import cadquery as cq

    shape, result_mode = _fuse_airframe(resolved, include_prop_discs)
    cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STL")
    return result_mode


def _prism(x0_m: float, x1_m: float, half_width_m: float, half_height_m: float, shape: float):
    """A straight piece of the body's cross-section shape along x (a cylinder when circular), as a cq.Solid."""
    import cadquery as cq

    x0_mm, x1_mm = x0_m * M_TO_MM, x1_m * M_TO_MM
    if shape == 2.0 and abs(half_width_m - half_height_m) < 1e-12:
        return cq.Solid.makeCylinder(half_width_m * M_TO_MM, x1_mm - x0_mm, cq.Vector(x0_mm, 0, 0), cq.Vector(1, 0, 0))
    wires = [_superellipse_wire(x, half_width_m * M_TO_MM, half_height_m * M_TO_MM, shape) for x in (x0_mm, x1_mm)]
    return cq.Solid.makeLoft(wires, ruled=True)


def _taper(x0_m: float, x1_m: float, half0: tuple[float, float], half1: tuple[float, float], shape: float):
    """A straight taper of the body's cross-section shape along x, half extents half0 at x0_m to half1 at x1_m (a cone
    when circular), as a cq.Solid."""
    import cadquery as cq

    x0_mm, x1_mm = x0_m * M_TO_MM, x1_m * M_TO_MM
    if shape == 2.0 and abs(half0[0] - half0[1]) < 1e-12 and abs(half1[0] - half1[1]) < 1e-12:
        return cq.Solid.makeCone(half0[0] * M_TO_MM, half1[0] * M_TO_MM, x1_mm - x0_mm, cq.Vector(x0_mm, 0, 0), cq.Vector(1, 0, 0))
    wires = [_superellipse_wire(x, half[0] * M_TO_MM, half[1] * M_TO_MM, shape) for x, half in ((x0_mm, half0), (x1_mm, half1))]
    return cq.Solid.makeLoft(wires, ruled=True)


def _ring(x0_m: float, x1_m: float, outer_half: tuple[float, float], inner_half: tuple[float, float], shape: float):
    return _prism(x0_m, x1_m, *outer_half, shape).cut(_prism(x0_m, x1_m, *inner_half, shape))


def _rod(start, end, diameter_m: float):
    """A cylinder between two points (m), as a cq.Solid."""
    import cadquery as cq

    axis = cq.Vector(*(b - a for a, b in zip(start, end))) * M_TO_MM
    return cq.Solid.makeCylinder(diameter_m / 2.0 * M_TO_MM, axis.Length, cq.Vector(*start) * M_TO_MM, axis.normalized())


def bore_half_extents(resolved: ResolvedAirframe, x_m: float) -> tuple[float, float]:
    """The body's inner half width and height at a station (inside the wall)."""
    wall = resolved.airframe.fuselage.section.wall_radial_m
    half_w, half_h = body_half_extents(resolved.body, x_m)
    return half_w - wall, half_h - wall


def joint_half_extents(resolved: ResolvedAirframe, x_m: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """Outer and inner half extents of a coupler or male spigot at a station: in the socket (the outer surface less
    socket_wall_m) with the fit clearance, coupler_wall_m thick."""
    joints = resolved.airframe.fuselage.joints
    outside = tuple(value - joints.socket_wall_m - joints.fit_clearance_m for value in body_half_extents(resolved.body, x_m))
    return outside, tuple(value - joints.coupler_wall_m for value in outside)


def _socket(resolved: ResolvedAirframe, face_m: float, direction: float):
    """The socket bored into a female end at a cut, insertion_depth_m into the section (direction +1 aft, -1 forward)."""
    joints = resolved.airframe.fuselage.joints
    socket = tuple(value - joints.socket_wall_m for value in body_half_extents(resolved.body, face_m))
    x = sorted((face_m - direction * CUT_OVERSHOOT_MM / M_TO_MM, face_m + direction * joints.insertion_depth_m))
    return _prism(*x, *socket, resolved.body.shape)


def joint_hole_stations(resolved: ResolvedAirframe, cut_index: int) -> tuple[float, ...]:
    """Screw hole stations of one cut: one per female end, hole_edge_dist_m from the cut (a coupler carries both, a male
    spigot the one of the section it enters)."""
    joints = resolved.airframe.fuselage.joints
    cut = resolved.airframe.sections.cuts_m[cut_index]
    joint = resolved.airframe.sections.cut_joints[cut_index]
    if joint == "coupler":
        return cut - joints.hole_edge_dist_m, cut + joints.hole_edge_dist_m
    if joint == "front_male":
        return (cut + joints.hole_edge_dist_m,)
    if joint == "rear_male":
        return (cut - joints.hole_edge_dist_m,)
    return ()  # none: no coupler, no screws


def _screw_holes(resolved: ResolvedAirframe, x_m: float) -> list:
    """The radial screw holes at one station, from inside the coupler bore out through the skin."""
    import cadquery as cq

    joints = resolved.airframe.fuselage.joints
    half_w, half_h = body_half_extents(resolved.body, x_m)
    inner = min(joint_half_extents(resolved, x_m)[1]) - CUT_OVERSHOOT_MM / M_TO_MM
    length_mm = (max(half_w, half_h) - inner) * M_TO_MM + CUT_OVERSHOOT_MM
    holes = []
    for angle in map(math.radians, joints.hole_angles_deg()):
        direction = cq.Vector(0.0, math.cos(angle), math.sin(angle))
        holes.append(cq.Solid.makeCylinder(joints.hole_diameter_m / 2.0 * M_TO_MM, length_mm, cq.Vector(x_m * M_TO_MM, 0, 0) + direction * inner * M_TO_MM, direction))
    return holes


def _through_holes(resolved: ResolvedAirframe, x0_m: float, x1_m: float) -> list:
    """Every screw hole station, longitudinal rod and fin spar reaching between two stations, each opened by the fit
    clearance."""
    airframe = resolved.airframe
    clearance = airframe.fuselage.joints.fit_clearance_m
    holes = []
    for cut_index in range(len(airframe.sections.cuts_m)):
        for station in joint_hole_stations(resolved, cut_index):
            if x0_m <= station <= x1_m:
                holes += _screw_holes(resolved, station)
    reinforcement = airframe.fuselage.reinforcement
    if reinforcement is not None:
        low, high = max(x0_m, reinforcement.from_x_m), min(x1_m, reinforcement.to_x_m)
        if high > low:
            diameter = reinforcement.pattern.diameter_m + 2.0 * clearance
            holes += [_rod((low, y, z), (high, y, z), diameter) for y, z in reinforcement.pattern.positions_m()]
    carried = {surface_set.name for surface_set in resolved.surfaces if surface_set.definition.spars is not None and surface_set.definition.spars.carry_through is not None}
    for spar in resolved.spars:
        if x0_m - spar.diameter_m <= spar.start[0] <= x1_m + spar.diameter_m:
            direction = tuple((b - a) / math.dist(spar.start, spar.end) for a, b in zip(spar.start, spar.end))
            # in a carry-through, joined to the holes it meets (no sliver between them, none breaking out of the arms); else
            # ending a clearance short of the spar's own end, so holes never meet
            if spar.surface in carried:
                start = spar.joined_hole_start
            else:
                start = tuple(s + clearance * d for s, d in zip(spar.start, direction))
            end = tuple(e + CUT_OVERSHOOT_MM / M_TO_MM * d for e, d in zip(spar.end, direction))
            holes.append(_rod(start, end, spar.diameter_m + 2.0 * clearance))
    return holes


def _spigot(resolved: ResolvedAirframe, cut_m: float, direction: float):
    """A male spigot at a cut, reaching insertion_depth_m into the mating section's socket (direction +1 aft, -1 forward)
    and as far back into the carrying section, where it is fused into the wall (and clipped to its outer surface, which
    may taper there, e.g. a tail cone). Inside the carrying section it is chamfered from the fuselage ID at its far end to
    the coupler ID hole_edge_dist_m short of the fastener hole centre (at the cut), so there is no step to print."""
    joints = resolved.airframe.fuselage.joints
    shape = resolved.body.shape
    outside, inside = joint_half_extents(resolved, cut_m)
    bore = bore_half_extents(resolved, cut_m)
    embed = tuple(value + SPIGOT_WALL_EMBED * resolved.airframe.fuselage.section.wall_radial_m for value in bore)
    out_x = sorted((cut_m, cut_m + direction * joints.insertion_depth_m))
    far_end = cut_m - direction * joints.insertion_depth_m
    hole_centre = cut_m + direction * joints.hole_edge_dist_m
    chamfer_end = hole_centre - direction * joints.hole_edge_dist_m
    in_x = sorted((cut_m, far_end))
    overshoot = CUT_OVERSHOOT_MM / M_TO_MM
    chamfer = _taper(*sorted((far_end, chamfer_end)), *((bore, inside) if far_end < chamfer_end else (inside, bore)), shape)
    chamfer = chamfer.fuse(_prism(*sorted((far_end, far_end - direction * overshoot)), *bore, shape))  # past the far end, for a clean cut
    root = _prism(*in_x, *embed, shape).cut(chamfer).cut(_prism(*sorted((chamfer_end, cut_m + direction * overshoot)), *inside, shape))
    root = root.intersect(build_body_solid(resolved, *in_x).val())
    return _ring(*out_x, outside, inside, shape).fuse(root)


def build_section_solid(resolved: ResolvedAirframe, index: int) -> tuple[object, list[str]]:
    """One printable fuselage section (cq.Shape) and notes on it: the body between its cuts with the surface sets and pods
    attached in it fused on, which stop at the outer surface wherever they run over another section (an axis-rooted fin
    keeps its planform; only what lies inside the body is removed); hollowed to the wall (or, printed solid, less its
    volumes), so a surface goes through its own section's wall and no further; the carry-through of the surface sets in it;
    a socket at each female end
    (where a coupler or a mating spigot enters), a spigot at each male end; screw, rod and spar holes cut."""
    airframe = resolved.airframe
    sections, joints = airframe.sections, airframe.fuselage.joints
    cuts = sections.cuts_m
    x0, x1 = section_bounds(resolved)[index]
    notes = []
    shape = build_body_solid(resolved, x0, x1).val()
    root_extend_m = FUSE_ROOT_EXTEND_FRACTION * airframe.fuselage.section.width_m / 2.0
    for surface_set in resolved.surfaces:
        definition = surface_set.definition
        if section_index(cuts, surface_set.exposed_x_root_le_m) != index:
            continue
        extend = 0.0 if definition.root == "axis" else root_extend_m
        for panel in surface_set.panels:
            shape = shape.fuse(build_wing_solid(panel, extend).val())
        root_start = surface_set.exposed_x_root_le_m
        root_end = max(max(panel.attach_le[0], panel.attach_te[0]) for panel in surface_set.panels)
        if (index > 0 and root_start < x0) or (index < len(cuts) and root_end > x1):  # over a neighbouring section, not past a body end
            notes.append(f"surface {surface_set.name!r} root chord {root_start * 1000:.1f}-{root_end * 1000:.1f} mm runs past the section ends "
                         f"{x0 * 1000:.1f}-{x1 * 1000:.1f} mm")
    for motor in resolved.motors:
        if motor.pod_stations and section_index(cuts, motor.mount_x_m) == index:
            shape = shape.fuse(build_pod_solid(motor).val())
    for a, b in ((0.0, x0), (x1, resolved.body.length_m)):  # surfaces and pods stop at the other sections' outer surface
        if b > a:
            shape = shape.cut(build_body_solid(resolved, a, b).val())

    wall = airframe.fuselage.section.wall_radial_m
    if not sections.per_section[index].solid:
        bore = build_body_solid(resolved, x0 - CUT_OVERSHOOT_MM / M_TO_MM, x1 + CUT_OVERSHOOT_MM / M_TO_MM, inset_m=wall)
        if bore is not None:
            shape = shape.cut(bore.val())
    else:
        for volume in airframe.volumes:
            if volume.shape != "point":
                shape = shape.cut(build_volume_solid(resolved, volume))
    if index > 0 and sections.cut_joints[index - 1] in ("coupler", "front_male"):
        shape = shape.cut(_socket(resolved, x0, 1.0))
    if index < len(cuts) and sections.cut_joints[index] in ("coupler", "rear_male"):
        shape = shape.cut(_socket(resolved, x1, -1.0))
    arms = [build_carry_through_arm(resolved, arm) for arm in resolved.carry_through  # after the bore, which would remove them
            if section_index(cuts, next(item for item in resolved.surfaces if item.name == arm.surface).exposed_x_root_le_m) == index]
    if arms:  # one boolean for all: each one pays a full setup
        shape = shape.fuse(*arms)
    reach0, reach1 = x0, x1  # extent of this part's material along x, spigots included
    if index > 0 and sections.cut_joints[index - 1] == "rear_male":
        shape = shape.fuse(_spigot(resolved, cuts[index - 1], -1.0))
        reach0 = cuts[index - 1] - joints.insertion_depth_m
    if index < len(cuts) and sections.cut_joints[index] == "front_male":
        shape = shape.fuse(_spigot(resolved, cuts[index], 1.0))
        reach1 = cuts[index] + joints.insertion_depth_m
    holes = _through_holes(resolved, reach0, reach1)
    if holes:
        shape = shape.cut(*holes)
    return shape.clean(), notes


def build_coupler_solid(resolved: ResolvedAirframe, cut_index: int):
    """The separate coupler at a coupler cut: a ring reaching insertion_depth_m into the sockets of both sections, screw,
    rod and spar holes cut."""
    joints = resolved.airframe.fuselage.joints
    cut = resolved.airframe.sections.cuts_m[cut_index]
    half_length = joints.insertion_depth_m
    solid = _ring(cut - half_length, cut + half_length, *joint_half_extents(resolved, cut), resolved.body.shape)
    holes = _through_holes(resolved, cut - half_length, cut + half_length)
    if holes:
        solid = solid.cut(*holes)
    return solid.clean()


def build_volume_solid(resolved: ResolvedAirframe, volume):
    """A volume's space as a cq.Solid (box, rounded box, cylinder, or the bore over its length)."""
    import cadquery as cq

    centre = cq.Vector(volume.x_m, volume.y_m, volume.z_m) * M_TO_MM
    if volume.shape in BOX_SHAPES:
        size = cq.Vector(*volume.size_m) * M_TO_MM
        box = cq.Solid.makeBox(size.x, size.y, size.z, centre - size * 0.5)
        if volume.shape == "box":
            return box
        return cq.Workplane("XY").add(box).edges(f"|{volume.axis.upper()}").fillet(volume.corner_radius_m * M_TO_MM).val()
    if volume.shape == "cylinder":
        axis = cq.Vector(*(1.0 if name == volume.axis else 0.0 for name in "xyz"))
        return cq.Solid.makeCylinder(volume.diameter_m / 2.0 * M_TO_MM, volume.length_m * M_TO_MM, centre - axis * (volume.length_m / 2.0 * M_TO_MM), axis)
    if volume.shape == "bore":
        return _prism(volume.x_m - volume.length_m / 2.0, volume.x_m + volume.length_m / 2.0, *bore_half_extents(resolved, volume.x_m), resolved.body.shape)
    raise ValueError(f"volume {volume.name!r}: a {volume.shape} has no solid")


def build_rod_solids(resolved: ResolvedAirframe) -> list[tuple[str, object]]:
    """The carbon rods at their own diameter: (name, cq.Solid) for each longitudinal rod and each wing spar."""
    rods = []
    reinforcement = resolved.airframe.fuselage.reinforcement
    if reinforcement is not None:
        rods += [("rods", _rod((reinforcement.from_x_m, y, z), (reinforcement.to_x_m, y, z), reinforcement.pattern.diameter_m))
                 for y, z in reinforcement.pattern.positions_m()]
    rods += [(f"spars {spar.surface}", _rod(spar.start, spar.end, spar.diameter_m)) for spar in resolved.spars]
    return rods


def _bed_note(name: str, shape, bed_m: tuple[float, float, float]) -> str | None:
    """A part whose bounding box does not fit the printer bed in any axis-aligned orientation."""
    box = shape.BoundingBox()
    size = sorted((box.xlen, box.ylen, box.zlen))
    bed = sorted(value * M_TO_MM for value in bed_m)
    if all(a <= b for a, b in zip(size, bed)):
        return None
    return f"{name}: {size[2]:.1f} x {size[1]:.1f} x {size[0]:.1f} mm does not fit the printer bed {bed[2]:g} x {bed[1]:g} x {bed[0]:g} mm"


def build_printable_parts(resolved: ResolvedAirframe) -> list[tuple[str, object, list[str]]]:
    """(name, cq.Shape, notes) of every printable part, front to back: sections S01.., then couplers C01.. (by cut).
    Built once per airframe definition: the section export and the drawings of the same design share them."""
    key = repr(resolved.airframe)
    if key not in _PRINTABLE_PARTS:
        _PRINTABLE_PARTS.clear()  # one design at a time
        _PRINTABLE_PARTS[key] = _build_printable_parts(resolved)
    return _PRINTABLE_PARTS[key]


def _build_printable_parts(resolved: ResolvedAirframe) -> list[tuple[str, object, list[str]]]:
    parts = []
    for index in range(len(section_bounds(resolved))):
        shape, notes = build_section_solid(resolved, index)
        parts.append((f"S{index + 1:02d}", shape, notes))
    for cut_index, joint in enumerate(resolved.airframe.sections.cut_joints):
        if joint == "coupler":
            parts.append((f"C{cut_index + 1:02d}", build_coupler_solid(resolved, cut_index), []))
    return parts


def export_sections(resolved: ResolvedAirframe, directory) -> tuple[list[str], list[str]]:
    """Write every printable part as STEP and STL: sections S01.. and couplers C01.. (numbered by their cut). Returns one
    line per part written, and the problems found (root chords past a section end, parts larger than the printer bed, parts
    that come out in pieces or as an invalid solid)."""
    import cadquery as cq
    from pathlib import Path

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written, all_notes = [], []
    for name, shape, notes in build_printable_parts(resolved):
        for suffix in ("step", "stl"):
            path = directory / f"{name}.{suffix}"
            cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType=suffix.upper())
        written.append(f"{name}: {directory / name}.step, .stl ({shape.Volume() / 1000.0:.1f} cm^3)")
        bed = _bed_note(name, shape, resolved.airframe.print.printer_bed_m)
        pieces = len(shape.Solids())
        split = [f"{name}: comes out as {pieces} separate solids (something cuts it through, e.g. a socket deeper than a tapering end)"] if pieces > 1 else []
        invalid = [] if shape.isValid() else [f"{name}: not a valid solid (a CAD boolean failed; usually two features touching exactly, e.g. a "
                                              f"hole edge through a pointed pod nose); move one of them slightly"]
        all_notes += [f"{name}: {note}" for note in notes] + ([bed] if bed else []) + split + invalid
    return written, all_notes
