"""
Usage:
from libs.cad import build_airframe_assembly, export_sections, export_step, export_stl
export_step(resolved, "out.step", mode="assembly")
written, problems = export_sections(resolved, project.sections_dir, "stl")  # "step" (default) or "stl"

CadQuery solids of a resolved airframe (libs/geometry.py): the whole airframe (assembly grouped by fuselage section,
or one fused outer shape), and the printable parts: each fuselage section with what is attached in it, and the
couplers. Solid geometry is built in millimetres so STEP/STL files open at correct scale in FreeCAD and meshing tools.
cadquery is imported lazily so the GUI starts light. FreeCAD varsets: libs/varsets.py.
"""

import math
from collections.abc import Callable

from libs.airframe import BOX_SHAPES, ControlSurface
from libs.geometry import (
    Body, CarryThroughArm, Panel, ResolvedAirframe, body_half_extents, body_section_at, build_body, carry_through_front_profile, carry_through_side_profile,
    chord_direction, control_cut_frames, section_index, superellipse_radius, thickness_direction,
)
from libs.print_features import CUT_MARGIN_M, joint_hole_stations, joint_holes_at, motor_mount_openings

M_TO_MM = 1000.0
PROFILE_SAMPLES = 48
SECTION_POLYGON_SAMPLES = 72
AIRFOIL_POINTS_PER_SIDE = 40
PROP_DISC_THICKNESS_MM = 1.0
GEOM_EPS_MM = 1e-4
LOFT_TIP_MM = 0.05  # non-circular bodies loft to a tiny section instead of a point
FLAT_PLATE_SECTION = ((0.0, 0.01), (1.0, 0.01), (1.0, -0.01), (0.0, -0.01))  # chord fractions, for an airfoil with no thickness
CUT_OVERSHOOT_MM = CUT_MARGIN_M * M_TO_MM  # holes and bores reach past the material, so no skin is left
CARRY_THROUGH_WALL_EMBED = 0.5  # of the wall: carry-through arms reach this far into the tube wall, fusing to it
ROOT_EMBED_MARGIN_M = 0.001  # the root airfoil ends up this far inside the outer body surface
_PRINTABLE_PARTS: dict[str, list] = {}  # the last design's printable parts, by its definition (build_printable_parts)
SOCKET_BORE_COUNTERBORE_M = 0.00005  # a socket cutter clears the bore surface: a coincident boolean leaves a zero-thickness face
SPIGOT_WALL_EMBED = 0.5  # of the wall: a male spigot's root ring reaches this far into the carrying section's wall, fusing to it

# Notification hook of the long exports, called on the exporting thread: (label, done, total). Optional everywhere.
Progress = Callable[[str, int, int], None]


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


def _clipped_stations(body: Body, x0: float, x1: float) -> list[tuple[float, float, float, float]]:
    """(x, half_width, half_height, shape) from x0 to x1, with interpolated end stations."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    return [(x, *body_half_extents(body, x), body_section_at(body, x).shape) for x in xs]


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
    stations = [(x, max(half_w - inset_m, 0.0), max(half_h - inset_m, 0.0), shape) for x, half_w, half_h, shape in _clipped_stations(body, x0, x1)]
    solid = [index for index, (_x, half_w, half_h, _shape) in enumerate(stations) if min(half_w, half_h) > 0.0]
    if not solid:
        return None
    stations = stations[max(solid[0] - 1, 0): solid[-1] + 2]  # one point station either side of the part with a section
    circular = all(shape == 2.0 and abs(half_w - half_h) < 1e-12 for _x, half_w, half_h, shape in stations)
    if circular:
        return _revolve_profile_solid([(x * M_TO_MM, half_w * M_TO_MM) for x, half_w, _half_h, _shape in stations])
    wires = [_superellipse_wire(x * M_TO_MM, half_w * M_TO_MM, half_h * M_TO_MM, shape) for x, half_w, half_h, shape in stations]
    return cq.Workplane("XY").newObject([cq.Solid.makeLoft(wires, ruled=True)])


def _root_le(panel: Panel, coordinates, body: Body) -> tuple[float, float, float]:
    """The panel's root leading edge with the root airfoil slid inward along the panel axis (root to tip) until no
    part of the section pokes through the outer body (plus ROOT_EMBED_MARGIN_M). The panel stays a straight loft
    between this root and the tip, so the body boolean crosses its surfaces instead of meeting the root at a
    tangential seam. Axis-rooted panels sit on the fuselage axis already; a root the body cannot contain (over a
    tapered tip) keeps its placed position; a chord overhanging the body keeps the points outside it as they are.
    """
    root = panel.attach_le
    if math.hypot(root[1], root[2]) <= 1e-9:
        return root
    axis = tuple(tip - point for tip, point in zip(panel.tip_le, root))
    axis_length = math.sqrt(sum(value * value for value in axis))
    if axis_length <= 1e-12:
        return root
    axis = tuple(value / axis_length for value in axis)
    chord_dir = chord_direction(panel)
    thick_dir = thickness_direction(panel)
    section = [tuple(point + panel.chord_root_m * (x * cd + y * td) for point, cd, td in zip(root, chord_dir, thick_dir))
               for x, y in coordinates]
    section = [point for point in section if body_half_extents(body, point[0])[0] > 0.0]
    if not section:
        return root

    def inside(point, depth: float) -> bool:
        x, y, z = (value - depth * direction for value, direction in zip(point, axis))
        half_w, half_h = body_half_extents(body, x)
        if half_w <= 0.0 or half_h <= 0.0:
            return False
        return math.hypot(y, z) <= superellipse_radius(half_w, half_h, body_section_at(body, x).shape, math.atan2(z, y)) - ROOT_EMBED_MARGIN_M

    def depth_for(point) -> float:
        low = 0.0
        if inside(point, low):
            return 0.0
        # Stop where this span line passes closest to the fuselage axis. A radial-distance bound can
        # overshoot the entire interior of an offset/dihedral root; doubling it then runs off indefinitely.
        high = (point[1] * axis[1] + point[2] * axis[2]) / (axis[1] ** 2 + axis[2] ** 2)
        if high <= 0.0 or not inside(point, high):
            return 0.0  # this root point cannot be contained: keep its placed geometry
        for _ in range(40):
            middle = (low + high) / 2.0
            if inside(point, middle):
                high = middle
            else:
                low = middle
        return high

    depth = max(depth_for(point) for point in section)
    return tuple(value - depth * direction for value, direction in zip(root, axis))


def build_wing_solid(panel: Panel, body: Body):
    """Loft one panel between its root and tip airfoil sections, in the panel's own chord/normal frame.

    A body-rooted panel's root airfoil is slid inward along the panel axis until it lies inside the outer body
    (_root_le), so the loft runs straight through the skin and a boolean against the body crosses the panel's
    surfaces instead of meeting the root at a tangential seam. Axis-rooted panels already start on the axis.
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

    wires = [section_wire(_root_le(panel, coordinates, body), panel.chord_root_m), section_wire(panel.tip_le, panel.chord_tip_m)]
    return cq.Workplane("XY").newObject([cq.Solid.makeLoft(wires, ruled=True)])


def _side_box(origin_m, normal, size_mm: float = 3000.0):
    """The half-space box on the +normal side of the plane through origin_m (m), big enough to cover a panel: the
    solid a boolean is intersected with to stop a cut at that plane."""
    import cadquery as cq

    return cq.Workplane(cq.Plane(origin=cq.Vector(*origin_m) * M_TO_MM, normal=cq.Vector(*normal))).rect(size_mm, size_mm).extrude(size_mm)


def build_panel_solids(resolved: ResolvedAirframe, panel: Panel, control: ControlSurface | None) -> tuple[object | None, tuple[object, ...]]:
    """One panel split into its fixed solid and its moving control surface(s), from control_cut_frames: the control's
    opening is cut out of the fixed side; the moving side is bordered by the gap (and the hinge rod hole, when the
    control has one), and any root buried in the body is cut off the moving side. No control: the whole panel is fixed.
    Fixed is None when the whole panel moves (all_moving from the root)."""
    if control is None:
        return build_wing_solid(panel, resolved.body).val(), ()
    frames = control_cut_frames(panel, control)
    panel_solid = build_wing_solid(panel, resolved.body).val()
    gap_m = frames["gap_m"]
    span_dir = frames["span_dir"]
    back = tuple(-value for value in span_dir)

    def offset(point, distance_m, direction):
        return tuple(value + distance_m * d for value, d in zip(point, direction))

    if frames["kind"] == "split":
        opening = panel_solid.intersect(_side_box(frames["hinge_point"], frames["hinge_dir"]).val())
        if control.span_start > 0.0:
            opening = opening.intersect(_side_box(frames["span_start_point"], span_dir).val())
        if control.span_end < 1.0:
            opening = opening.intersect(_side_box(frames["span_end_point"], back).val())
        fixed = panel_solid.cut(opening)
        moving = panel_solid.intersect(_side_box(offset(frames["hinge_point"], gap_m, frames["hinge_dir"]), frames["hinge_dir"]).val())
        if control.span_start > 0.0:
            moving = moving.intersect(_side_box(offset(frames["span_start_point"], gap_m, span_dir), span_dir).val())
        if control.span_end < 1.0:
            moving = moving.intersect(_side_box(offset(frames["span_end_point"], -gap_m, span_dir), back).val())
    elif control.span_start > 0.0:  # all_moving: an inboard fixed base, the rest moves
        fixed = panel_solid.intersect(_side_box(frames["span_start_point"], back).val())
        moving = panel_solid.intersect(_side_box(offset(frames["span_start_point"], gap_m, span_dir), span_dir).val())
    else:  # all_moving from the root: the whole panel moves
        fixed, moving = None, panel_solid
    body = build_body_solid(resolved, 0.0, resolved.body.length_m)
    if body is not None:  # a control rooted in the body keeps no buried root stub
        moving = moving.cut(body.val())
    if frames["rod_diameter_m"] is not None:
        clearance = resolved.airframe.sections.joints.fit_clearance_m  # as spar holes: a fit clearance all round
        hole = _rod(frames["rod_start"], frames["rod_end"], frames["rod_diameter_m"] + 2.0 * clearance)
        moving = moving.cut(hole)
        if fixed is not None:
            fixed = fixed.cut(hole)
    return (None if fixed is None else fixed.clean()), (moving.clean(),)


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


def _mount_opening(opening):
    """A circular hole or radial slot along the physical motor/pod axis."""
    import cadquery as cq

    inward = opening.inward
    start, end = opening.start, opening.end
    origin = tuple((p + opening.depth0_m * d) * M_TO_MM for p, d in zip(opening.face, inward))
    plane = cq.Plane(origin=origin, xDir=(0, 1, 0), normal=inward)
    work = cq.Workplane(plane).center((start[0] + end[0]) * M_TO_MM / 2.0,
                                     (start[1] + end[1]) * inward[0] * M_TO_MM / 2.0)
    length = math.dist(start, end)
    if length <= 1e-12:
        work = work.circle(opening.diameter_m * M_TO_MM / 2.0)
    else:
        angle = math.degrees(math.atan2((end[1] - start[1]) * inward[0], end[0] - start[0]))
        work = work.slot2D((length + opening.diameter_m) * M_TO_MM, opening.diameter_m * M_TO_MM, angle)
    return work.extrude((opening.depth1_m - opening.depth0_m) * M_TO_MM).val()


def cut_motor_mount_holes(shape, resolved: ResolvedAirframe):
    """Cut fastening holes, shaft relief and bolt-entry clearance in printed material, leaving motors intact."""
    cutters = [_mount_opening(opening) for opening in motor_mount_openings(resolved)]
    return shape.cut(*cutters).clean() if cutters else shape


def build_airframe_assembly(resolved: ResolvedAirframe, include_prop_discs: bool = False, *, progress: Progress | None = None):
    """Nested CadQuery assembly: one sub-assembly per fuselage section with its body piece and the surfaces,
    motors and pods whose root or position lies in it. Body-rooted panels are carried through the body to the
    axis (build_wing_solid), so they overlap the body piece instead of only touching it. A panel with a control
    contributes its fixed side, and the moving surface(s) as separate children named S<n>_CS<number>.

    progress (optional): "assembling S<nn>" per section child, done/total = section count, then "writing" when
    the assembly is complete and the STEP file is being written."""
    import cadquery as cq

    assembly = cq.Assembly(name=resolved.airframe.name or "airframe")
    children = {index: cq.Assembly(name=f"S{index + 1:02d}") for index in range(len(section_bounds(resolved)))}
    for index, (x0, x1) in enumerate(section_bounds(resolved)):
        body = cut_motor_mount_holes(build_body_solid(resolved, x0, x1).val(), resolved)
        children[index].add(cq.Workplane("XY").newObject([body]), name=f"S{index + 1:02d}_body")
    placements = {(control.surface, control.panel_index): control.number for control in resolved.controls}
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            index = section_index(resolved.airframe.sections.cuts_m, panel.attach_le[0])
            fixed, moving = build_panel_solids(resolved, panel, surface_set.definition.control)
            if fixed is not None:
                children[index].add(cq.Workplane("XY").newObject([cut_motor_mount_holes(fixed, resolved)]), name=f"S{index + 1:02d}_{surface_set.name}_{panel.index + 1}")
            for solid in moving:
                children[index].add(cq.Workplane("XY").newObject([solid]), name=f"S{index + 1:02d}_CS{placements[(surface_set.name, panel.index)]}")
    for motor_index, motor in enumerate(resolved.motors, start=1):
        index = section_index(resolved.airframe.sections.cuts_m, motor.position[0])
        children[index].add(build_motor_solid(motor), name=f"S{index + 1:02d}_motor_{motor_index}")
        if motor.pod_stations:
            pod = cut_motor_mount_holes(build_pod_solid(motor).val(), resolved)
            children[index].add(cq.Workplane("XY").newObject([pod]), name=f"S{index + 1:02d}_pod_{motor_index}")
        if include_prop_discs:
            children[index].add(build_prop_disc_solid(motor), name=f"S{index + 1:02d}_prop_{motor_index}")
    for index, child in children.items():
        assembly.add(child, name=f"S{index + 1:02d}")
        if progress is not None:
            progress(f"assembling S{index + 1:02d}", index + 1, len(children))
    return assembly


def _airframe_solids(resolved: ResolvedAirframe, include_prop_discs: bool) -> list:
    import cadquery as cq

    body = cut_motor_mount_holes(build_body_solid(resolved, 0.0, resolved.body.length_m).val(), resolved)
    solids = [cq.Workplane("XY").newObject([body])]
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            fixed, moving = build_panel_solids(resolved, panel, surface_set.definition.control)  # the gap keeps a control a separate solid
            if fixed is not None:
                solids.append(cq.Workplane("XY").newObject([cut_motor_mount_holes(fixed, resolved)]))
            solids += [cq.Workplane("XY").newObject([solid]) for solid in moving]
    for motor in resolved.motors:
        solids.append(build_motor_solid(motor))
        if motor.pod_stations:
            pod = cut_motor_mount_holes(build_pod_solid(motor).val(), resolved)
            solids.append(cq.Workplane("XY").newObject([pod]))
        if include_prop_discs:
            solids.append(build_prop_disc_solid(motor))
    return solids


def _fuse_airframe(resolved: ResolvedAirframe, include_prop_discs: bool = False, *, progress: Progress | None = None) -> tuple[object, str]:
    """Boolean-union all solids into one shape; fall back to a compound if the fuse fails.

    Body-rooted panels are carried through the fuselage to its axis, so they merge with it instead of only touching.
    progress (optional): coarse phases "building sections" (1/3) and "fusing" (2/3); the caller reports "writing".
    """
    import cadquery as cq

    if progress is not None:
        progress("building sections", 1, 3)
    shapes = [workplane.val() for workplane in _airframe_solids(resolved, include_prop_discs)]
    if progress is not None:
        progress("fusing", 2, 3)
    try:
        fused = shapes[0]
        for shape in shapes[1:]:
            fused = fused.fuse(shape)
        return fused.clean(), "fused"
    except Exception as error:  # OCC boolean failures: report and fall back to an unfused compound
        print(f"cad: fuse failed ({error}); exporting unfused compound instead")
        return cq.Compound.makeCompound(shapes), "compound"


def export_step(resolved: ResolvedAirframe, path, mode: str = "assembly", include_prop_discs: bool = False, *, progress: Progress | None = None) -> str:
    """Export the airframe as STEP. mode = "assembly" (named parts per section) | "fused" (single OML).

    Returns the mode actually written ("assembly", "fused", or "compound" if the boolean fuse failed).
    progress (optional): assembly mode reports each section child assembled, fused mode the phases
    "building sections", "fusing" and "writing" (done/total = 3), as does export_stl.
    """
    import cadquery as cq

    if mode == "assembly":
        assembly = build_airframe_assembly(resolved, include_prop_discs, progress=progress)
        if progress is not None:
            sections = len(section_bounds(resolved))
            progress("writing", sections, sections)
        assembly.export(str(path), "STEP")
        return "assembly"
    if mode == "fused":
        shape, result_mode = _fuse_airframe(resolved, include_prop_discs, progress=progress)
        if progress is not None:
            progress("writing", 3, 3)
        cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STEP")
        return result_mode
    raise ValueError(f"Unknown STEP export mode: {mode}")


def export_stl(resolved: ResolvedAirframe, path, include_prop_discs: bool = False, *, progress: Progress | None = None) -> str:
    """Export the airframe as a single STL mesh (fused where possible).

    progress (optional): the phases "building sections", "fusing" and "writing" (done/total = 3)."""
    import cadquery as cq

    shape, result_mode = _fuse_airframe(resolved, include_prop_discs, progress=progress)
    if progress is not None:
        progress("writing", 3, 3)
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
    joints = resolved.airframe.sections.joints
    outside = tuple(value - joints.socket_wall_m - joints.fit_clearance_m for value in body_half_extents(resolved.body, x_m))
    return outside, tuple(value - joints.coupler_wall_m for value in outside)


def _socket(resolved: ResolvedAirframe, face_m: float, direction: float):
    """The socket bored into a female end at a cut, insertion_depth_m into the section (direction +1 aft, -1 forward)."""
    joints = resolved.airframe.sections.joints
    # The default socket wall equals the skin wall, which would put this cutter's surface exactly on the bore's;
    # OCCT then leaves a zero-thickness face the later clean cannot unify (S02/S03 crash on the lofted body).
    # Bite 0.05 mm into the bore instead: a real socket mouth clearance, and no coincident surfaces.
    socket_wall = min(joints.socket_wall_m, resolved.airframe.fuselage.section.wall_radial_m - SOCKET_BORE_COUNTERBORE_M)
    socket = tuple(value - socket_wall for value in body_half_extents(resolved.body, face_m))
    x = sorted((face_m - direction * CUT_OVERSHOOT_MM / M_TO_MM, face_m + direction * joints.insertion_depth_m))
    return _prism(*x, *socket, body_section_at(resolved.body, face_m).shape)


def _screw_holes(resolved: ResolvedAirframe, x_m: float, cut_index: int) -> list:
    """Normal-to-profile screw holes, coaxial across both mating pieces."""
    return [_rod(hole.start, hole.end, hole.diameter_m) for hole in joint_holes_at(resolved, cut_index, x_m)]


def _through_holes(resolved: ResolvedAirframe, x0_m: float, x1_m: float) -> list:
    """Every screw hole station, longitudinal rod and fin spar reaching between two stations, each opened by the fit
    clearance."""
    airframe = resolved.airframe
    clearance = airframe.sections.joints.fit_clearance_m
    holes = []
    for cut_index in range(len(airframe.sections.cuts_m)):
        for station in joint_hole_stations(resolved, cut_index):
            if x0_m <= station <= x1_m:
                holes += _screw_holes(resolved, station, cut_index)
    reinforcement = airframe.fuselage.reinforcement
    if reinforcement is not None:
        low, high = max(x0_m, reinforcement.from_x_m), min(x1_m, reinforcement.to_x_m)
        if high > low:
            diameter = reinforcement.pattern.diameter_m + 2.0 * clearance
            holes += [_rod((low, y, z), (high, y, z), diameter) for y, z in reinforcement.pattern.positions_m()]
    for spar in resolved.spars:
        if x0_m - spar.diameter_m <= spar.start[0] <= x1_m + spar.diameter_m:
            holes.append(_spar_hole(resolved, spar))
    return holes


def _spar_hole(resolved: ResolvedAirframe, spar):
    clearance = resolved.airframe.sections.joints.fit_clearance_m
    surface = next(row.definition for row in resolved.surfaces if row.name == spar.surface)
    direction = tuple((b - a) / math.dist(spar.start, spar.end) for a, b in zip(spar.start, spar.end))
    start = spar.joined_hole_start if surface.spars.carry_through is not None else tuple(s + clearance * d for s, d in zip(spar.start, direction))
    end = tuple(e + CUT_OVERSHOOT_MM / M_TO_MM * d for e, d in zip(spar.end, direction))
    return _rod(start, end, spar.diameter_m + 2.0 * clearance)


def _spigot(resolved: ResolvedAirframe, cut_m: float, direction: float):
    """A male spigot at a cut, reaching insertion_depth_m into the mating section's socket (direction +1 aft, -1 forward)
    and as far back into the carrying section, where it is fused into the wall (and clipped to its outer surface, which
    may taper there, e.g. a tail cone). Inside the carrying section it is chamfered from the fuselage ID at its far end to
    the coupler ID hole_edge_dist_m short of the fastener hole centre (at the cut), so there is no step to print."""
    joints = resolved.airframe.sections.joints
    shape = body_section_at(resolved.body, cut_m).shape
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


def _clip_x(solid, x0_m: float, x1_m: float):
    """solid cut to the slab between two x stations (the section's cuts); None when nothing or a sliver is left."""
    box = solid.BoundingBox()
    if box.xmax <= x0_m * M_TO_MM or box.xmin >= x1_m * M_TO_MM:
        return None
    try:
        piece = solid.cut(_side_box((x0_m, 0.0, 0.0), (-1.0, 0.0, 0.0)).val())
        piece = piece.cut(_side_box((x1_m, 0.0, 0.0), (1.0, 0.0, 0.0)).val())
    except ValueError:  # the boolean emptied: the solid lies wholly on a removed side
        return None
    return None if piece.Volume() <= 1e-3 else piece


def _clip_span(solid, panel: Panel, start_m: float | None, end_m: float | None):
    """Clip fixed material to span planes, measured from the true attachment root."""
    piece = solid
    for distance, sign in ((start_m, -1.0), (end_m, 1.0)):
        if distance is not None:
            origin = tuple(a + distance * d for a, d in zip(panel.attach_le, panel.span_dir))
            normal = tuple(sign * d for d in panel.span_dir)
            piece = piece.cut(_side_box(origin, normal).val())
    return piece.clean()


def _panel_attachments(resolved: ResolvedAirframe) -> tuple[list[tuple[str, object]], list[tuple[str, object]], list[tuple[str, object]]]:
    """Root stubs for fuselage sections, detached fixed segments, and unsplit moving controls."""
    fixed: list[tuple[str, object]] = []
    segments: list[tuple[str, object]] = []
    controls: list[tuple[str, object]] = []
    numbers = {(control.surface, control.panel_index): control.number for control in resolved.controls}
    outer_body = None
    for surface_index, surface_set in enumerate(resolved.surfaces, start=1):
        for panel in surface_set.panels:
            fixed_solid, moving = build_panel_solids(resolved, panel, surface_set.definition.control)
            for motor in resolved.motors:
                if motor.pod_stations and motor.panel is not None and (motor.panel.surface, motor.panel.index) == (panel.surface, panel.index):
                    pod = build_pod_solid(motor).val()
                    fixed_solid = pod if fixed_solid is None else fixed_solid.fuse(pod)
            if fixed_solid is not None:
                cuts = surface_set.definition.sections.cuts_m
                root = _clip_span(fixed_solid, panel, None, cuts[0]) if cuts else fixed_solid
                fixed.append((f"surface {surface_set.name!r} panel {panel.index + 1}", root))
                if cuts:
                    if outer_body is None:
                        outer_body = build_body_solid(resolved, 0.0, resolved.airframe.fuselage.length_m).val()
                    holes = [_spar_hole(resolved, spar) for spar in resolved.spars
                             if (spar.surface, spar.panel_index) == (panel.surface, panel.index)]
                    for segment_index, start in enumerate(cuts, start=2):
                        end = cuts[segment_index - 1] if segment_index - 1 < len(cuts) else None
                        piece = _clip_span(fixed_solid, panel, start, end).cut(outer_body)
                        if holes:
                            piece = piece.cut(*holes)
                        piece = cut_motor_mount_holes(piece, resolved).clean()
                        name = f"W{surface_index}_P{panel.index + 1}_S{segment_index}"
                        if piece.Volume() <= 1e-3:
                            raise ValueError(f"{name}: span segment has no fixed material outside the fuselage; move the cut or remove it")
                        segments.append((name, piece))
            if moving:
                controls.append((f"CS{numbers[(surface_set.name, panel.index)]}", moving[0]))
    for motor_index, motor in enumerate(resolved.motors, start=1):
        if motor.pod_stations and motor.panel is None:
            fixed.append((f"pod {motor_index}", build_pod_solid(motor).val()))
    return fixed, segments, controls


def build_section_solid(resolved: ResolvedAirframe, index: int, attachments: list[tuple[str, object]]) -> tuple[object, list[str]]:
    """One printable fuselage section (cq.Shape) and notes on it: the body between its cuts with every attachment
    (each panel's fixed side, each pod) clipped to the section's x-range and fused on, so a surface or pod crossing
    a cut is divided between the sections and gets flat faces at the cuts; the outer end sections retain attachments
    beyond the body ends (a control's opening is carried in its
    panel's fixed side; the moving surfaces are separate parts, CS1..); hollowed to the wall (or, printed solid,
    less its volumes), so a surface goes through its own section's wall and no further; the carry-through of the
    surface sets in it; a socket at each female end (where a coupler or a mating spigot enters), a spigot at each
    male end; screw, rod and spar holes cut."""
    airframe = resolved.airframe
    sections, joints = airframe.sections, airframe.sections.joints
    cuts = sections.cuts_m
    x0, x1 = section_bounds(resolved)[index]
    notes: list[str] = []
    shape = build_body_solid(resolved, x0, x1).val()
    for _label, solid in attachments:
        bounds = solid.BoundingBox()
        piece = _clip_x(solid, min(x0, bounds.xmin / M_TO_MM) if index == 0 else x0,
                        max(x1, bounds.xmax / M_TO_MM) if index == len(cuts) else x1)
        if piece is not None:
            shape = shape.fuse(piece)

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
    return cut_motor_mount_holes(shape, resolved).clean(), notes


def build_coupler_solid(resolved: ResolvedAirframe, cut_index: int):
    """The separate coupler at a coupler cut: a ring reaching insertion_depth_m into the sockets of both sections, screw,
    rod and spar holes cut."""
    joints = resolved.airframe.sections.joints
    cut = resolved.airframe.sections.cuts_m[cut_index]
    half_length = joints.insertion_depth_m
    solid = _ring(cut - half_length, cut + half_length, *joint_half_extents(resolved, cut), body_section_at(resolved.body, cut).shape)
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
        bore = build_body_solid(resolved, volume.x_m - volume.length_m / 2.0, volume.x_m + volume.length_m / 2.0,
                               inset_m=resolved.airframe.fuselage.section.wall_radial_m)
        if bore is None:
            raise ValueError(f"volume {volume.name!r}: no fuselage interior over its length")
        return bore.val()
    raise ValueError(f"volume {volume.name!r}: a {volume.shape} has no solid")


def build_rod_solids(resolved: ResolvedAirframe) -> list[tuple[str, object]]:
    """The carbon rods at their own diameter: (name, cq.Solid) for each longitudinal rod, each wing spar, and each
    control's hinge rod (named after its control, CS1..)."""
    rods = []
    reinforcement = resolved.airframe.fuselage.reinforcement
    if reinforcement is not None:
        rods += [("rods", _rod((reinforcement.from_x_m, y, z), (reinforcement.to_x_m, y, z), reinforcement.pattern.diameter_m))
                 for y, z in reinforcement.pattern.positions_m()]
    rods += [(f"spars {spar.surface}", _rod(spar.start, spar.end, spar.diameter_m)) for spar in resolved.spars]
    surface_sets = {surface_set.name: surface_set for surface_set in resolved.surfaces}
    for control in resolved.controls:
        surface_set = surface_sets[control.surface]
        panel = next(panel for panel in surface_set.panels if panel.index == control.panel_index)
        frames = control_cut_frames(panel, surface_set.definition.control)
        if frames["rod_diameter_m"] is not None:
            rods.append((f"spars CS{control.number}", _rod(frames["rod_start"], frames["rod_end"], frames["rod_diameter_m"])))
    return rods


def _bed_note(name: str, shape, bed_m: tuple[float, float, float]) -> str | None:
    """A part whose bounding box does not fit the printer bed in any axis-aligned orientation."""
    box = shape.BoundingBox()
    size = sorted((box.xlen, box.ylen, box.zlen))
    bed = sorted(value * M_TO_MM for value in bed_m)
    if all(a <= b for a, b in zip(size, bed)):
        return None
    return f"{name}: {size[2]:.1f} x {size[1]:.1f} x {size[0]:.1f} mm does not fit the printer bed {bed[2]:g} x {bed[1]:g} x {bed[0]:g} mm"


def build_printable_parts(resolved: ResolvedAirframe, *, progress: Progress | None = None) -> list[tuple[str, object, list[str]]]:
    """(name, cq.Shape, notes) of every printable part: sections S01.., then couplers C01.. (by cut), then control
    surfaces CS1.. (the moving panel, by control number). Built once per airframe definition: the section export and
    the drawings of the same design share them.

    progress (optional): "building <name>" per part actually built (done/total = part count); a cached
    design emits nothing."""
    key = repr(resolved.airframe)
    if key not in _PRINTABLE_PARTS:
        _PRINTABLE_PARTS.clear()  # one design at a time
        _PRINTABLE_PARTS[key] = _build_printable_parts(resolved, progress=progress)
    return _PRINTABLE_PARTS[key]


def _build_printable_parts(resolved: ResolvedAirframe, *, progress: Progress | None = None) -> list[tuple[str, object, list[str]]]:
    attachments, segments, controls = _panel_attachments(resolved)
    parts = []
    couplers = sum(1 for joint in resolved.airframe.sections.cut_joints if joint == "coupler")
    total = len(section_bounds(resolved)) + couplers + len(segments) + len(controls)

    def built(name: str) -> None:
        if progress is not None:
            progress(f"building {name}", len(parts), total)

    for index in range(len(section_bounds(resolved))):
        shape, notes = build_section_solid(resolved, index, attachments)
        parts.append((f"S{index + 1:02d}", shape, notes))
        built(f"S{index + 1:02d}")
    for cut_index, joint in enumerate(resolved.airframe.sections.cut_joints):
        if joint == "coupler":
            parts.append((f"C{cut_index + 1:02d}", build_coupler_solid(resolved, cut_index), []))
            built(f"C{cut_index + 1:02d}")
    for name, solid in segments:
        parts.append((name, solid, []))
        built(name)
    for name, solid in controls:
        parts.append((name, solid, []))  # the moving surfaces, by control number
        built(name)
    return parts


def export_sections(resolved: ResolvedAirframe, directory, export_format: str = "step", *, progress: Progress | None = None) -> tuple[list[str], list[str]]:
    """Write every printable part as STEP or STL (export_format, "step" by default): sections S01.., couplers C01..
    (numbered by their cut) and control surfaces CS1.. (numbered front to back like their placements). Returns one line
    per part written, and the problems found (root chords past a section end, parts larger than the printer bed, parts
    that come out in pieces or as an invalid solid).

    progress (optional): "building <name>" per part as it is built (only on a cache miss), then
    "writing <name>" per part as it is written, done/total = part count."""
    if export_format not in ("step", "stl"):
        raise ValueError(f"export_format must be 'step' or 'stl', not {export_format!r}")
    import cadquery as cq
    from pathlib import Path

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written, all_notes = [], []
    parts = build_printable_parts(resolved, progress=progress)
    for index, (name, shape, notes) in enumerate(parts, start=1):
        if progress is not None:
            progress(f"writing {name}", index, len(parts))
        path = directory / f"{name}.{export_format}"
        cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType=export_format.upper())
        written.append(f"{name}: {directory / name}.{export_format} ({shape.Volume() / 1000.0:.1f} cm^3)")
        bed = _bed_note(name, shape, resolved.airframe.print.printer_bed_m)
        pieces = len(shape.Solids())
        split = [f"{name}: comes out as {pieces} separate solids (something cuts it through, e.g. a socket deeper than a tapering end)"] if pieces > 1 else []
        invalid = [] if shape.isValid() else [f"{name}: not a valid solid (a CAD boolean failed; usually two features touching exactly, e.g. a "
                                              f"hole edge through a pointed pod nose); move one of them slightly"]
        all_notes += [f"{name}: {note}" for note in notes] + ([bed] if bed else []) + split + invalid
    return written, all_notes
