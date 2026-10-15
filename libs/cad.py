"""
Usage:
from libs.cad import build_airframe_assembly, export_sections, export_step, export_stl
export_step(resolved, "out.step", mode="assembly")
written, problems = export_sections(resolved, project.sections_dir, "stl")  # "step" (default) or "stl"

CadQuery solids of a resolved airframe (libs/geometry.py): the whole airframe (assembly grouped by fuselage section,
or one fused outer shape), and the printable parts: each fuselage section with what is attached in it, and the
couplers. Solid geometry is built in millimetres so STEP/STL files open at correct scale in FreeCAD and meshing tools.
Every expensive solid is built once per design through libs.solid_cache and shared by all the exports (the assembly,
fused STEP and STL share one set of airframe pieces; sections and drawings one set of printable parts).
cadquery is imported lazily so the GUI starts light. FreeCAD varsets: libs/varsets.py.
"""

import math
from collections.abc import Callable

from libs.access import ACCESS_EPS_MM3, access_composition
from libs.airframe import ControlSurface
from libs.body_solids import (
    CUT_OVERSHOOT_MM, GEOM_EPS_MM, M_TO_MM, _prism, _revolve_profile_solid, _ring, _taper,
    build_body_solid, hollowed_section, printed_section_material, section_bounds,
)
from libs.geometry import (
    Body, CarryThroughArm, Panel, ResolvedAirframe, body_half_extents, body_section_at, carry_through_front_profile, carry_through_side_profile,
    chord_direction, control_cut_frames, embedded_root_fraction, section_index, thickness_direction,
)
from libs.print_features import joint_hole_stations, joint_holes_at, motor_mount_openings
from libs.solid_cache import cached

AIRFOIL_POINTS_PER_SIDE = 40
PROP_DISC_THICKNESS_MM = 1.0
FLAT_PLATE_SECTION = ((0.0, 0.01), (1.0, 0.01), (1.0, -0.01), (0.0, -0.01))  # chord fractions, for an airfoil with no thickness
CARRY_THROUGH_WALL_EMBED = 0.5  # of the wall: carry-through arms reach this far into the tube wall, fusing to it
SOCKET_BORE_COUNTERBORE_M = 0.00005  # a socket cutter clears the bore surface: a coincident boolean leaves a zero-thickness face
SPIGOT_WALL_EMBED = 0.5  # of the wall: a male spigot's root ring reaches this far into the carrying section's wall, fusing to it

# Notification hook of the long exports, called on the exporting thread: (label, done, total). Optional everywhere.
Progress = Callable[[str, int, int], None]


def _airfoil_section_coordinates(airfoil_name: str):
    import aerosandbox as asb

    coordinates = asb.Airfoil(airfoil_name).repanel(n_points_per_side=AIRFOIL_POINTS_PER_SIDE).coordinates
    thickness = float(coordinates[:, 1].max() - coordinates[:, 1].min())
    if thickness < 1e-4:
        return list(FLAT_PLATE_SECTION)
    return [(float(x), float(y)) for x, y in coordinates]


def build_wing_solid(panel: Panel, body: Body):
    """Loft one panel between its root and tip airfoil sections, in the panel's own chord/normal frame.

    A body-rooted panel is lofted from its own outline carried inward until the root airfoil lies inside the outer body
    (embedded_root_fraction): no gap where the body tapers under the root chord, a boolean against the body crosses
    the panel's surfaces instead of meeting the root at a tangential seam, and the part outside the body is the
    designed panel. Axis-rooted panels already start on the axis.
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

    root = embedded_root_fraction(panel, coordinates, body)
    root_le = tuple(a + root * (b - a) for a, b in zip(panel.attach_le, panel.tip_le))
    root_chord = panel.chord_root_m + root * (panel.chord_tip_m - panel.chord_root_m)
    wires = [section_wire(root_le, root_chord), section_wire(panel.tip_le, panel.chord_tip_m)]
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
    Fixed is None when the whole panel moves (all_moving from the root). Built once per design (libs.solid_cache)."""
    return cached(resolved, ("panel", panel.surface, panel.index), lambda: _build_panel_solids(resolved, panel, control))


def _build_panel_solids(resolved: ResolvedAirframe, panel: Panel, control: ControlSurface | None) -> tuple[object | None, tuple[object, ...]]:
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


def cut_motor_mount_holes(shape, resolved: ResolvedAirframe, panel: Panel | None = None):
    """Cut the fastening bores, head bores and shaft relief of the motors one carrier holds: a surface panel (its wing
    with its pod fused on, cut before it is clipped into sections) or, with panel None, the fuselage (nose and tail
    motors, cut into the finished section). Motors stay intact."""
    carrier = None if panel is None else (panel.surface, panel.index)
    cutters = cached(resolved, ("mount cutters", carrier),
                     lambda: [_mount_opening(opening) for opening in motor_mount_openings(resolved) if opening.carrier == carrier])
    return shape.cut(*cutters).clean() if cutters else shape


def airframe_pieces(resolved: ResolvedAirframe) -> list[tuple[int, str, object, str]]:
    """The whole airframe as named solids, each in the fuselage section it belongs to: (section index, name, cq.Shape,
    kind), kind body/hatch/panel/control/motor/pod, front to back by section as the assembly lists them. Body pieces
    are the solid-volume proxy per section, or with access the printed section material less its openings (so an
    opening reaches the real cavity) and the retained hatch pieces beside it. Body-rooted panels are carried through
    the body to the axis (build_wing_solid), so they overlap the body piece instead of only touching it. Built once per
    design and shared by the STEP assembly, the fused STEP and the STL."""
    return cached(resolved, ("airframe pieces",), lambda: _build_airframe_pieces(resolved))


def _build_airframe_pieces(resolved: ResolvedAirframe) -> list[tuple[int, str, object, str]]:
    pieces = []
    access = access_composition(resolved)
    for index, (x0, x1) in enumerate(section_bounds(resolved)):
        section_access = access.get(index)
        if section_access is None:  # untouched sections keep the existing solid-volume proxy, byte for byte
            body = build_body_solid(resolved, x0, x1).val()
        else:  # a section with access exports its printed material, so an opening reaches the real cavity
            body = printed_section_material(resolved, index)
            if section_access.opening is not None:
                body = body.cut(section_access.opening)
        pieces.append((index, f"S{index + 1:02d}_body", cut_motor_mount_holes(body, resolved), "body"))
        for piece in () if section_access is None else section_access.retained:
            pieces.append((index, piece.part_id, piece.solid, "hatch"))
    placements = {(control.surface, control.panel_index): control.number for control in resolved.controls}
    for surface_set in resolved.surfaces:
        for panel in surface_set.panels:
            index = section_index(resolved.airframe.sections.cuts_m, panel.attach_le[0])
            fixed, moving = build_panel_solids(resolved, panel, surface_set.definition.control)
            if fixed is not None:
                pieces.append((index, f"S{index + 1:02d}_{surface_set.name}_{panel.index + 1}", cut_motor_mount_holes(fixed, resolved, panel), "panel"))
            for solid in moving:
                pieces.append((index, f"S{index + 1:02d}_CS{placements[(surface_set.name, panel.index)]}", solid, "control"))
    for motor_index, motor in enumerate(resolved.motors, start=1):
        index = section_index(resolved.airframe.sections.cuts_m, motor.position[0])
        pieces.append((index, f"S{index + 1:02d}_motor_{motor_index}", build_motor_solid(motor).val(), "motor"))
        if motor.pod_stations:
            pieces.append((index, f"S{index + 1:02d}_pod_{motor_index}", cut_motor_mount_holes(build_pod_solid(motor).val(), resolved, motor.panel), "pod"))
    return pieces


def _prop_disc_pieces(resolved: ResolvedAirframe) -> list[tuple[int, str, object, str]]:
    """Thin prop discs (actuator-disc CFD domain marking), as airframe pieces; cheap, so built when asked."""
    pieces = []
    for motor_index, motor in enumerate(resolved.motors, start=1):
        index = section_index(resolved.airframe.sections.cuts_m, motor.position[0])
        pieces.append((index, f"S{index + 1:02d}_prop_{motor_index}", build_prop_disc_solid(motor).val(), "prop"))
    return pieces


def build_airframe_assembly(resolved: ResolvedAirframe, include_prop_discs: bool = False, *, progress: Progress | None = None):
    """Nested CadQuery assembly of airframe_pieces: one sub-assembly per fuselage section with its body piece, hatch
    pieces and the surfaces, motors and pods whose root or position lies in it. A panel with a control contributes its
    fixed side, and the moving surface(s) as separate children named S<n>_CS<number>; prop discs as S<n>_prop_<i>.

    progress (optional): "assembling S<nn>" per section child, done/total = section count, then "writing" when
    the assembly is complete and the STEP file is being written."""
    import cadquery as cq

    pieces = airframe_pieces(resolved) + (_prop_disc_pieces(resolved) if include_prop_discs else [])
    assembly = cq.Assembly(name=resolved.airframe.name or "airframe")
    children = {index: cq.Assembly(name=f"S{index + 1:02d}") for index in range(len(section_bounds(resolved)))}
    for index, name, shape, _kind in pieces:
        children[index].add(cq.Workplane("XY").newObject([shape]), name=name)
    for index, child in children.items():
        assembly.add(child, name=f"S{index + 1:02d}")
        if progress is not None:
            progress(f"assembling S{index + 1:02d}", index + 1, len(children))
    return assembly


def _airframe_solids(resolved: ResolvedAirframe, include_prop_discs: bool) -> list:
    """The airframe pieces as one list of shapes for the fuse: without access the whole body is one piece (the
    sections' union, with fewer booleans), with access the per-section printed material and its hatch pieces."""
    pieces = airframe_pieces(resolved)
    if resolved.airframe.access:
        shapes = [shape for _index, _name, shape, _kind in pieces]
    else:
        whole = cut_motor_mount_holes(build_body_solid(resolved, 0.0, resolved.body.length_m).val(), resolved)
        shapes = [whole] + [shape for _index, _name, shape, kind in pieces if kind != "body"]
    if include_prop_discs:
        shapes += [disc for _index, _name, disc, _kind in _prop_disc_pieces(resolved)]
    return shapes


def fused_airframe(resolved: ResolvedAirframe, include_prop_discs: bool = False, *, progress: Progress | None = None) -> tuple[object, str]:
    """Boolean-union all solids into one shape; fall back to a compound if the fuse fails. Built once per design (and
    prop-disc choice).

    Body-rooted panels are carried through the fuselage to its axis, so they merge with it instead of only touching.
    progress (optional): coarse phases "building sections" (1/3) and "fusing" (2/3); the caller reports "writing".
    """
    return cached(resolved, ("fused", include_prop_discs), lambda: _build_fused_airframe(resolved, include_prop_discs, progress))


def _build_fused_airframe(resolved: ResolvedAirframe, include_prop_discs: bool, progress: Progress | None) -> tuple[object, str]:
    import cadquery as cq

    if progress is not None:
        progress("building sections", 1, 3)
    shapes = _airframe_solids(resolved, include_prop_discs)
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
        shape, result_mode = fused_airframe(resolved, include_prop_discs, progress=progress)
        if progress is not None:
            progress("writing", 3, 3)
        cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STEP")
        return result_mode
    raise ValueError(f"Unknown STEP export mode: {mode}")


def export_stl(resolved: ResolvedAirframe, path, include_prop_discs: bool = False, *, progress: Progress | None = None) -> str:
    """Export the airframe as a single STL mesh (fused where possible).

    progress (optional): the phases "building sections", "fusing" and "writing" (done/total = 3)."""
    import cadquery as cq

    shape, result_mode = fused_airframe(resolved, include_prop_discs, progress=progress)
    if progress is not None:
        progress("writing", 3, 3)
    cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STL")
    return result_mode


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


Attachment = tuple[str, object, str | None]  # (label or part name, cq.Shape, owning surface set; None = the fuselage section)


def panel_attachments(resolved: ResolvedAirframe) -> tuple[list[Attachment], list[Attachment], list[Attachment]]:
    """Root stubs for fuselage sections (a wing pod fused to its panel's; a body pod owned by its section), detached
    fixed segments W<surface>_P<panel>_S<segment>, and unsplit moving controls CS<n>; each with the surface set it
    belongs to. Built once per design (libs.solid_cache)."""
    return cached(resolved, ("attachments",), lambda: _build_panel_attachments(resolved))


def _build_panel_attachments(resolved: ResolvedAirframe) -> tuple[list[Attachment], list[Attachment], list[Attachment]]:
    fixed: list[Attachment] = []
    segments: list[Attachment] = []
    controls: list[Attachment] = []
    numbers = {(control.surface, control.panel_index): control.number for control in resolved.controls}
    outer_body = None
    for surface_index, surface_set in enumerate(resolved.surfaces, start=1):
        for panel in surface_set.panels:
            fixed_solid, moving = build_panel_solids(resolved, panel, surface_set.definition.control)
            for motor in resolved.motors:
                if motor.pod_stations and motor.panel is not None and (motor.panel.surface, motor.panel.index) == (panel.surface, panel.index):
                    pod = build_pod_solid(motor).val()
                    fixed_solid = pod if fixed_solid is None else fixed_solid.fuse(pod)
            if fixed_solid is not None:  # wing and pods fused first, then the motor bores cut through them whole
                fixed_solid = cut_motor_mount_holes(fixed_solid, resolved, panel)
            if fixed_solid is not None:
                cuts = surface_set.definition.sections.cuts_m
                root = _clip_span(fixed_solid, panel, None, cuts[0]) if cuts else fixed_solid
                fixed.append((f"surface {surface_set.name!r} panel {panel.index + 1}", root, surface_set.name))
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
                        piece = piece.clean()
                        name = f"W{surface_index}_P{panel.index + 1}_S{segment_index}"
                        if piece.Volume() <= 1e-3:
                            raise ValueError(f"{name}: span segment has no fixed material outside the fuselage; move the cut or remove it")
                        segments.append((name, piece, surface_set.name))
            if moving:
                controls.append((f"CS{numbers[(surface_set.name, panel.index)]}", moving[0], surface_set.name))
    for motor_index, motor in enumerate(resolved.motors, start=1):
        if motor.pod_stations and motor.panel is None:
            fixed.append((f"pod {motor_index}", build_pod_solid(motor).val(), None))
    return fixed, segments, controls


def _intersects(first, second) -> bool:
    """True when the boolean common has real volume; a kernel error is left to expose its own traceback."""
    return first.intersect(second).Volume() > ACCESS_EPS_MM3


def _access_support_notes(resolved: ResolvedAirframe, index: int) -> list[str]:
    """Export diagnostics for an access opening reaching hardware the cut must not take with it: the printed
    reinforcement rods and spars, and the joint screw holes at a nearby seam. The cut stays in the fuselage body;
    these notes make the unsupported interaction visible."""
    section_access = access_composition(resolved).get(index)
    if section_access is None or section_access.opening is None:
        return []
    opening = section_access.opening
    box = opening.BoundingBox()
    notes = []
    for name, rod in build_rod_solids(resolved):
        rod_box = rod.BoundingBox()
        if rod_box.xmax < box.xmin or rod_box.xmin > box.xmax or rod_box.ymax < box.ymin or rod_box.ymin > box.ymax or rod_box.zmax < box.zmin or rod_box.zmin > box.zmax:
            continue
        if _intersects(opening, rod):
            notes.append(f"access opening reaches the {name}; the printed reinforcement and spars are not cut")
    for cut_index, station in enumerate(resolved.airframe.sections.cuts_m):
        if not box.xmin - 10.0 <= station * M_TO_MM <= box.xmax + 10.0:
            continue
        for hole in _screw_holes(resolved, station, cut_index):
            if _intersects(opening, hole):
                notes.append(f"access opening reaches the joint screw holes at the seam {station * M_TO_MM:.1f} mm; the joint hardware is not cut")
                break
    return notes


def build_section_solid(resolved: ResolvedAirframe, index: int, attachments: list[Attachment], opening=None) -> tuple[object, list[str]]:
    """One printable fuselage section (cq.Shape) and notes on it: the body between its cuts with every attachment
    (each panel's fixed side, each pod) clipped to the section's x-range and fused on, so a surface or pod crossing
    a cut is divided between the sections and gets flat faces at the cuts; the outer end sections retain attachments
    beyond the body ends (a control's opening is carried in its
    panel's fixed side; the moving surfaces are separate parts, CS1..); hollowed to the wall (or, printed solid,
    less its volumes), so a surface goes through its own section's wall and no further; the carry-through of the
    surface sets in it; a socket at each female end (where a coupler or a mating spigot enters), a spigot at each
    male end; screw, rod and spar holes cut. opening (optional) is the union access cutter: it is cut from the
    section body before the attachments are fused, so access never cuts a wing root or pod, and an attachment that
    covers it is reported."""
    airframe = resolved.airframe
    sections, joints = airframe.sections, airframe.sections.joints
    cuts = sections.cuts_m
    x0, x1 = section_bounds(resolved)[index]
    notes: list[str] = []
    shape = build_body_solid(resolved, x0, x1).val()
    if opening is not None:
        shape = shape.cut(opening)
    for label, solid, _owner in attachments:
        bounds = solid.BoundingBox()
        piece = _clip_x(solid, min(x0, bounds.xmin / M_TO_MM) if index == 0 else x0,
                        max(x1, bounds.xmax / M_TO_MM) if index == len(cuts) else x1)
        if piece is not None:
            if opening is not None and _intersects(opening, piece):
                notes.append(f"access opening reaches the attached {label}; it is cut in the fuselage body only, not in the attachment")
            shape = shape.fuse(piece)

    shape = hollowed_section(shape, resolved, index, x0, x1)
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


def build_rod_solids(resolved: ResolvedAirframe) -> list[tuple[str, object]]:
    """The carbon rods at their own diameter: (name, cq.Solid) for each longitudinal rod, each wing spar, and each
    control's hinge rod (named after its control, CS1..). Built once per design."""
    return cached(resolved, ("rods",), lambda: _build_rod_solids(resolved))


def _build_rod_solids(resolved: ResolvedAirframe) -> list[tuple[str, object]]:
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

    progress (optional): "building <name>" per part actually built (done/total = part count); a design already
    built (libs.solid_cache) emits nothing."""
    return cached(resolved, ("printable parts",), lambda: _build_printable_parts(resolved, progress=progress))


def _build_printable_parts(resolved: ResolvedAirframe, *, progress: Progress | None = None) -> list[tuple[str, object, list[str]]]:
    attachments, segments, controls = panel_attachments(resolved)
    access = access_composition(resolved)
    hatch_parts = [(piece.part_id, piece.solid, list(piece.notes)) for index in sorted(access) for piece in access[index].retained]
    parts = []
    couplers = sum(1 for joint in resolved.airframe.sections.cut_joints if joint == "coupler")
    total = len(section_bounds(resolved)) + couplers + len(segments) + len(controls) + len(hatch_parts)

    def built(name: str) -> None:
        if progress is not None:
            progress(f"building {name}", len(parts), total)

    for index in range(len(section_bounds(resolved))):
        section_access = access.get(index)
        shape, notes = build_section_solid(resolved, index, attachments, opening=None if section_access is None else section_access.opening)
        if section_access is not None:
            notes = [*notes, *section_access.notes, *_access_support_notes(resolved, index)]
        parts.append((f"S{index + 1:02d}", shape, notes))
        built(f"S{index + 1:02d}")
    for cut_index, joint in enumerate(resolved.airframe.sections.cut_joints):
        if joint == "coupler":
            parts.append((f"C{cut_index + 1:02d}", build_coupler_solid(resolved, cut_index), []))
            built(f"C{cut_index + 1:02d}")
    for name, solid, _owner in segments:
        parts.append((name, solid, []))
        built(name)
    for name, solid, _owner in controls:
        parts.append((name, solid, []))  # the moving surfaces, by control number
        built(name)
    for name, solid, notes in hatch_parts:  # hatch pieces kept from the section material, H01.. beside the sections
        parts.append((name, solid, notes))
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
