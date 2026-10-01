"""
Usage:
from libs.cad import build_freecad_exports, build_airframe_assembly, export_step, export_stl
exports = build_freecad_exports(resolved, standards_data)
export_step(resolved, "out.step", mode="assembly")

CAD export of a resolved airframe (libs/geometry.py): FreeCAD varset values and CadQuery solids. Solid geometry
is built in millimetres so STEP/STL files open at correct scale in FreeCAD and meshing tools. The assembly groups
parts by fuselage section (the section cuts). cadquery is imported lazily so the GUI starts light.
"""

import math

from libs.geometry import Body, Panel, ResolvedAirframe, body_half_extents, build_body, superellipse_radius

M_TO_MM = 1000.0
PROFILE_SAMPLES = 48
SECTION_POLYGON_SAMPLES = 72
AIRFOIL_POINTS_PER_SIDE = 40
PROP_DISC_THICKNESS_MM = 1.0
GEOM_EPS_MM = 1e-4
LOFT_TIP_MM = 0.05  # non-circular bodies loft to a tiny section instead of a point
FUSE_ROOT_EXTEND_FRACTION = 0.75  # of the body half width: body-rooted panels reach this far into the body when fused


def freecad_property(prop_type: str, value, unit: str | None = None, expr: str | None = None) -> dict:
    prop = {
        "Type": prop_type,
        "Value": value,
    }
    if unit is not None:
        prop["Unit"] = unit
    if expr is not None:
        prop["Expr"] = expr
    return prop


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
        half = 0.01
        return [(0.0, half), (1.0, half), (1.0, -half), (0.0, -half)]
    return [(float(x), float(y)) for x, y in coordinates]


def build_airframe_params_export(resolved: ResolvedAirframe, standards_data: dict) -> dict:
    airframe = resolved.airframe
    section = airframe.fuselage.section
    width_mm = section.width_m * M_TO_MM
    wall_mm = section.wall_radial_m * M_TO_MM
    surfaces_export = {"SurfaceSetNum": freecad_property("App::PropertyInteger", str(len(resolved.surfaces)))}
    for surface_index, surface_set in enumerate(resolved.surfaces, start=1):
        surface_name = f"Surface{surface_index}"
        surfaces_export[f"{surface_name}Phase"] = freecad_property("App::PropertyAngle", 0.0, "deg")
        surfaces_export[f"{surface_name}Count"] = freecad_property("App::PropertyInteger", str(len(surface_set.panels)))
        surfaces_export[f"{surface_name}Span"] = freecad_property("App::PropertyDistance", surface_set.definition.span_m * M_TO_MM, "mm")
    bed = [value * M_TO_MM for value in airframe.print.printer_bed_m]
    export = {
        "Fuselage": {
            "ID": freecad_property("App::PropertyLength", width_mm - 2.0 * wall_mm, "mm"),
            "OD": freecad_property("App::PropertyLength", width_mm, "mm"),
            "Thickness": freecad_property("App::PropertyLength", wall_mm, "mm"),
            "TotalLen": freecad_property("App::PropertyDistance", airframe.fuselage.length_m * M_TO_MM, "mm"),
        },
        "Base": {
            "Label": freecad_property("App::PropertyString", "'AirframeParams'"),
            "Label2": freecad_property("App::PropertyString", "''"),
        },
        "Production": {
            "PrinterMax": freecad_property("App::PropertyPosition", f"Vector ({bed[0]:g}, {bed[1]:g}, {bed[2]:g})"),
        },
        "Surfaces": surfaces_export,
    }
    if resolved.motors:
        motor = resolved.motors[0]
        export["Propulsion"] = {
            "MotorDiameter": freecad_property("App::PropertyLength", motor.bell_diameter_m * M_TO_MM, "mm"),
            "MotorHeight": freecad_property("App::PropertyLength", motor.bell_length_m * M_TO_MM, "mm"),
            "PropDiameter": freecad_property("App::PropertyLength", motor.prop_radius_m * 2.0 * M_TO_MM, "mm"),
        }
    if airframe.preset in standards_data["specs"]:
        export["Modules"] = {
            "CouplerHoleDist": freecad_property("App::PropertyLength", standards_data["specs"][airframe.preset]["coupling"]["hole_edge_dist"], "mm"),
            "CouplerLen": freecad_property("App::PropertyLength", airframe.fuselage.joints.insertion_depth_m * 2.0 * M_TO_MM, "mm"),
            "WingSecFuselageThickness": freecad_property("App::PropertyLength", wall_mm, "mm"),
        }
    return export


def build_freecad_exports(resolved: ResolvedAirframe, standards_data: dict) -> dict:
    return {
        "AirframeParams": build_airframe_params_export(resolved, standards_data),
    }


def _clipped_stations(body: Body, x0: float, x1: float) -> list[tuple[float, float, float]]:
    """(x, half_width, half_height) from x0 to x1, with interpolated end stations."""
    xs = [x0] + [station.x_m for station in body.stations if x0 < station.x_m < x1] + [x1]
    return [(x, *body_half_extents(body, x)) for x in xs]


def _superellipse_wire(x_mm: float, half_width_mm: float, half_height_mm: float, shape: float):
    import cadquery as cq

    half_width_mm = max(half_width_mm, LOFT_TIP_MM)
    half_height_mm = max(half_height_mm, LOFT_TIP_MM)
    points = []
    for index in range(SECTION_POLYGON_SAMPLES):
        angle = 2.0 * math.pi * index / SECTION_POLYGON_SAMPLES
        radius = superellipse_radius(half_width_mm, half_height_mm, shape, angle)
        points.append(cq.Vector(x_mm, radius * math.cos(angle), radius * math.sin(angle)))
    return cq.Wire.makePolygon(points, close=True)


def build_body_solid(resolved: ResolvedAirframe, x0: float, x1: float):
    """Fuselage between two stations, capped flat at the ends: revolved when circular, lofted otherwise."""
    import cadquery as cq

    body = build_body(resolved.airframe, PROFILE_SAMPLES)
    stations = _clipped_stations(body, x0, x1)
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
    incidence = math.radians(panel.incidence_deg)
    chord_dir = tuple((math.cos(incidence) if axis == 0 else 0.0) - math.sin(incidence) * panel.normal[axis] for axis in range(3))
    thick_dir = tuple((math.sin(incidence) if axis == 0 else 0.0) + math.cos(incidence) * panel.normal[axis] for axis in range(3))

    def section_wire(leading_edge, chord_m: float):
        chord_mm = chord_m * M_TO_MM
        points = [
            cq.Vector(*(leading_edge[axis] * M_TO_MM + chord_mm * (x * chord_dir[axis] + y * thick_dir[axis]) for axis in range(3)))
            for x, y in coordinates
        ]
        return cq.Wire.makePolygon(points, close=True)

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


def section_bounds(resolved: ResolvedAirframe) -> list[tuple[float, float]]:
    bounds = [0.0, *resolved.airframe.sections.cuts_m, resolved.body.length_m]
    return list(zip(bounds, bounds[1:]))


def section_index_at(resolved: ResolvedAirframe, x_m: float) -> int:
    bounds = section_bounds(resolved)
    for index, (_x0, x1) in enumerate(bounds):
        if x_m < x1:
            return index
    return len(bounds) - 1


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
            index = section_index_at(resolved, panel.attach_le[0])
            children[index].add(build_wing_solid(panel), name=f"S{index + 1:02d}_{surface_set.name}_{panel.index + 1}")
    for motor_index, motor in enumerate(resolved.motors, start=1):
        index = section_index_at(resolved, motor.position[0])
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
