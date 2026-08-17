"""
Usage:
from libs.cad import build_airframe_params_export, build_freecad_exports
from libs.cad import build_airframe_assembly, export_step, export_stl

Solid geometry is built in millimeters so exported STEP/STL files open at
correct scale in FreeCAD and meshing tools. cadquery is imported lazily so
the varset-export path and GUI startup stay light.
"""

import math

from libs.analysis import PROP_SHAFT_LEN_M, resolve_config_with_standard
from libs.pointed_shapes import pointed_shape_param_values, sample_pointed_profile

M_TO_MM = 1000.0
PROFILE_SAMPLES = 48
AIRFOIL_POINTS_PER_SIDE = 40
PROP_DISC_THICKNESS_MM = 1.0
GEOM_EPS_MM = 1e-4


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


def build_airframe_params_export(config: dict, analysis: dict, standards_data: dict) -> dict:
    resolved_config = resolve_config_with_standard(config, standards_data)
    surfaces_export = {
        "SurfaceSetNum": freecad_property("App::PropertyInteger", str(len(analysis["surface_sets"]))),
    }
    for surface_index, surface in enumerate(analysis["surface_sets"], start=1):
        surface_name = f"Surface{surface_index}"
        surfaces_export[f"{surface_name}Phase"] = freecad_property("App::PropertyAngle", surface["wing_phase"], "deg")
        surfaces_export[f"{surface_name}Count"] = freecad_property("App::PropertyInteger", str(surface["count"]))
        surfaces_export[f"{surface_name}Span"] = freecad_property("App::PropertyDistance", surface["span"] * 1000.0, "mm")
    export = {
        "Fuselage": {
            "ID": freecad_property("App::PropertyLength", resolved_config["fuselage_id_mm"], "mm"),
            "OD": freecad_property("App::PropertyLength", resolved_config["fuselage_od_mm"], "mm"),
            "Thickness": freecad_property("App::PropertyLength", (resolved_config["fuselage_od_mm"] - resolved_config["fuselage_id_mm"]) / 2.0, "mm"),
            "TotalLen": freecad_property("App::PropertyDistance", analysis["fuselage_length"] * 1000.0, "mm"),
        },
        "Base": {
            "Label": freecad_property("App::PropertyString", "'AirframeParams'"),
            "Label2": freecad_property("App::PropertyString", "''"),
        },
        "Production": {
            "PrinterMax": freecad_property(
                "App::PropertyPosition",
                f"Vector ({standards_data['3dp']['constraints']['x']}.0, {standards_data['3dp']['constraints']['y']}.0, {standards_data['3dp']['constraints']['z']}.0)",
            ),
        },
        "Surfaces": surfaces_export,
    }
    if analysis["motor_draw_rows"]:
        export["Propulsion"] = {
            "MotorDiameter": freecad_property("App::PropertyLength", analysis["motor_draw_rows"][0]["motor_bell_diam_m"] * 1000.0, "mm"),
            "MotorHeight": freecad_property("App::PropertyLength", analysis["motor_draw_rows"][0]["motor_bell_len_m"] * 1000.0, "mm"),
            "PropDiameter": freecad_property("App::PropertyLength", analysis["motor_draw_rows"][0]["prop_diam_m"] * 1000.0, "mm"),
        }
    if config["apply_standard"] and config["standard_profile"]:
        standard_spec = standards_data["specs"][config["standard_profile"]]
        export["Modules"] = {
            "CouplerHoleDist": freecad_property("App::PropertyLength", standard_spec["coupling"]["hole_edge_dist"], "mm"),
            "CouplerLen": freecad_property("App::PropertyLength", standard_spec["coupling"]["coupler_depth"] * 2.0, "mm"),
            "WingSecFuselageThickness": freecad_property("App::PropertyLength", (standard_spec["module_size"]["nominal_od"] - standard_spec["module_size"]["nominal_id"]) / 2.0, "mm"),
        }
    return export


def build_freecad_exports(config: dict, analysis: dict, standards_data: dict) -> dict:
    return {
        "AirframeParams": build_airframe_params_export(config, analysis, standards_data),
    }


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


def _structural_module_specs(analysis: dict) -> list[tuple[dict, str]]:
    """Ordered structural module rows with their fuselage role: nose, tail, or straight."""
    structural_rows = [row for row in analysis["module_rows"] if row["module_type"] in {"body", "end", "coupler", "wingmount"}]
    if not structural_rows:
        raise RuntimeError("No structural modules to build a fuselage from")
    specs = []
    end_count = 0
    for row in structural_rows:
        role = "straight"
        if row["module_type"] == "end" and end_count == 0:
            role = "nose"
            end_count += 1
        elif row["module_type"] == "end" and end_count == 1:
            role = "tail"
            end_count += 1
        specs.append((row, role))
    return specs


def _module_stations(row: dict, tube_radius_m: float, role: str) -> list[tuple[float, float]]:
    """(x_mm, r_mm) fuselage stations for one structural module."""
    module_len = row["x_end"] - row["x_start"]
    if role == "nose":
        profile = sample_pointed_profile(
            row["pointed_shape"]["shape"],
            module_len,
            tube_radius_m,
            PROFILE_SAMPLES,
            pointed_shape_param_values(row["pointed_shape"]),
        )
        return [((row["x_start"] + x_local) * M_TO_MM, radius * M_TO_MM) for x_local, radius in profile]
    if role == "tail":
        profile = sample_pointed_profile(
            row["pointed_shape"]["shape"],
            module_len,
            tube_radius_m,
            PROFILE_SAMPLES,
            pointed_shape_param_values(row["pointed_shape"]),
        )
        return [((row["x_end"] - x_local) * M_TO_MM, radius * M_TO_MM) for x_local, radius in reversed(profile)]
    tube_radius_mm = tube_radius_m * M_TO_MM
    return [(row["x_start"] * M_TO_MM, tube_radius_mm), (row["x_end"] * M_TO_MM, tube_radius_mm)]


def build_module_solid(row: dict, analysis: dict, role: str):
    """Fuselage segment for one structural module, capped flat at its joints."""
    return _revolve_profile_solid(_module_stations(row, analysis["tube_diam"] / 2.0, role))


def build_fuselage_solid(analysis: dict):
    """Body of revolution for the full fuselage OML, same traversal as build_asb_airplane."""
    stations: list[tuple[float, float]] = []
    for row, role in _structural_module_specs(analysis):
        stations.extend(_module_stations(row, analysis["tube_diam"] / 2.0, role))
    return _revolve_profile_solid(stations)


def _airfoil_section_coordinates(airfoil_name: str):
    import aerosandbox as asb

    coordinates = asb.Airfoil(airfoil_name).repanel(n_points_per_side=AIRFOIL_POINTS_PER_SIDE).coordinates
    thickness = float(coordinates[:, 1].max() - coordinates[:, 1].min())
    if thickness < 1e-4:
        half = 0.01
        return [(0.0, half), (1.0, half), (1.0, -half), (0.0, -half)]
    return [(float(x), float(y)) for x, y in coordinates]


def build_wing_solid(surface: dict, panel: dict, root_extend_m: float = 0.0):
    """Loft one wing/fin panel between its root and tip airfoil sections.

    root_extend_m > 0 adds a constant-section prism extending the root inward
    along the panel direction, so the panel penetrates the fuselage for
    boolean fusing. The geometry outboard of the true root is unchanged.
    """
    import cadquery as cq

    coordinates = _airfoil_section_coordinates(surface["airfoil"])
    aoa_rad = math.radians(surface["aoa"])
    cos_a = math.cos(aoa_rad)
    sin_a = math.sin(aoa_rad)
    dir_y = panel["dir_y"]
    dir_z = panel["dir_z"]
    # Thickness direction perpendicular to the span, mirrored on -Y panels so
    # left/right pairs get mirrored sections (+Z for both horizontal wings)
    # and positive AoA pitches every panel leading-edge-up.
    side = -1.0 if dir_y < 0.0 else 1.0
    thick_y = -dir_z * side
    thick_z = dir_y * side

    x_root_le = surface["x_root_le"] * M_TO_MM
    x_tip_le = x_root_le + surface["sweep_len"] * M_TO_MM
    root_le = (x_root_le, panel["root_y"] * M_TO_MM, panel["root_z"] * M_TO_MM)
    tip_le = (x_tip_le, panel["tip_y"] * M_TO_MM, panel["tip_z"] * M_TO_MM)

    def section_wire(leading_edge: tuple[float, float, float], chord_m: float):
        chord_mm = chord_m * M_TO_MM
        points = []
        for coord_x, coord_y in coordinates:
            chordwise = coord_x * cos_a + coord_y * sin_a
            thickwise = -coord_x * sin_a + coord_y * cos_a
            points.append(
                cq.Vector(
                    leading_edge[0] + chord_mm * chordwise,
                    leading_edge[1] + chord_mm * thickwise * thick_y,
                    leading_edge[2] + chord_mm * thickwise * thick_z,
                )
            )
        return cq.Wire.makePolygon(points, close=True)

    wires = [section_wire(root_le, surface["chord_root"]), section_wire(tip_le, surface["chord_tip"])]
    if root_extend_m > 0.0:
        extend_mm = root_extend_m * M_TO_MM
        inner_le = (root_le[0], root_le[1] - dir_y * extend_mm, root_le[2] - dir_z * extend_mm)
        wires.insert(0, section_wire(inner_le, surface["chord_root"]))
    solid = cq.Solid.makeLoft(wires, ruled=True)
    return cq.Workplane("XY").newObject([solid])


def build_motor_solid(motor_row: dict):
    """Motor bell as a cylinder along X centered on the motor position."""
    import cadquery as cq

    radius_mm = motor_row["motor_bell_diam_m"] / 2.0 * M_TO_MM
    length_mm = motor_row["motor_bell_len_m"] * M_TO_MM
    start = cq.Vector(
        (motor_row["x_m"] - motor_row["motor_bell_len_m"] / 2.0) * M_TO_MM,
        motor_row["y_m"] * M_TO_MM,
        motor_row["z_m"] * M_TO_MM,
    )
    solid = cq.Solid.makeCylinder(radius_mm, length_mm, start, cq.Vector(1, 0, 0))
    return cq.Workplane("XY").newObject([solid])


def build_pod_solid(motor_row: dict):
    """Motor pod: flat cylinder section plus revolved pointed taper, on the motor axis."""
    front_sign = -1.0 if motor_row["direction"] == "front" else 1.0
    pod_sign = -front_sign
    pod_radius = motor_row["pod_diam_m"] / 2.0
    pod_x0 = motor_row["x_m"] + pod_sign * (motor_row["motor_bell_len_m"] / 2.0)

    stations: list[tuple[float, float]] = [(pod_x0 * M_TO_MM, pod_radius * M_TO_MM)]
    pod_x1 = pod_x0
    if motor_row["pod_len_flat_m"] > 0.0:
        pod_x1 = pod_x0 + pod_sign * motor_row["pod_len_flat_m"]
        stations.append((pod_x1 * M_TO_MM, pod_radius * M_TO_MM))
    if motor_row["pod_len_curve_m"] > 0.0:
        curve_len = motor_row["pod_len_curve_m"]
        profile = sample_pointed_profile(motor_row["pod_shape"], curve_len, pod_radius, PROFILE_SAMPLES)
        stations.extend(
            ((pod_x1 + pod_sign * (curve_len - curve_x)) * M_TO_MM, curve_radius * M_TO_MM)
            for curve_x, curve_radius in reversed(profile[:-1])
        )
    return _revolve_profile_solid(stations).translate((0.0, motor_row["y_m"] * M_TO_MM, motor_row["z_m"] * M_TO_MM))


def build_prop_disc_solid(motor_row: dict):
    """Thin disc at the propeller plane, for actuator-disc CFD domain marking."""
    import cadquery as cq

    front_sign = -1.0 if motor_row["direction"] == "front" else 1.0
    x_center_mm = (motor_row["x_m"] + front_sign * (motor_row["motor_bell_len_m"] / 2.0 + PROP_SHAFT_LEN_M)) * M_TO_MM
    start = cq.Vector(
        x_center_mm - PROP_DISC_THICKNESS_MM / 2.0,
        motor_row["y_m"] * M_TO_MM,
        motor_row["z_m"] * M_TO_MM,
    )
    solid = cq.Solid.makeCylinder(motor_row["prop_diam_m"] / 2.0 * M_TO_MM, PROP_DISC_THICKNESS_MM, start, cq.Vector(1, 0, 0))
    return cq.Workplane("XY").newObject([solid])


def _airframe_solids(analysis: dict, include_prop_discs: bool = False, wing_root_extend_m: float = 0.0) -> list[tuple[str, object]]:
    solids = [("fuselage", build_fuselage_solid(analysis))]
    for set_index, surface in enumerate(analysis["surface_sets"], start=1):
        for panel in surface["panels"]:
            name = f"surface{set_index}_{surface['name']}_{panel['index'] + 1}"
            solids.append((name, build_wing_solid(surface, panel, root_extend_m=wing_root_extend_m)))
    for motor_index, motor_row in enumerate(analysis["motor_draw_rows"], start=1):
        solids.append((f"motor_{motor_index}", build_motor_solid(motor_row)))
        if motor_row["pod_len_flat_m"] > 0.0 or motor_row["pod_len_curve_m"] > 0.0:
            solids.append((f"pod_{motor_index}", build_pod_solid(motor_row)))
        if include_prop_discs:
            solids.append((f"prop_{motor_index}", build_prop_disc_solid(motor_row)))
    return solids


def build_airframe_assembly(analysis: dict, include_prop_discs: bool = False):
    """Nested CadQuery assembly: one sub-assembly per structural module, holding
    that module's fuselage segment plus its mounted surfaces, motors, and pods."""
    import cadquery as cq

    surfaces_by_part: dict[int, list[dict]] = {}
    for surface in analysis["surface_sets"]:
        surfaces_by_part.setdefault(surface["part_index"], []).append(surface)
    motors_by_part: dict[int, list[dict]] = {}
    for motor_row in analysis["motor_draw_rows"]:
        motors_by_part.setdefault(motor_row["part_index"], []).append(motor_row)

    assembly = cq.Assembly(name=analysis["config_name"] or "airframe")
    for position, (row, role) in enumerate(_structural_module_specs(analysis), start=1):
        prefix = f"m{position:02d}_{row['module_name']}"
        module_assembly = cq.Assembly(name=prefix)
        if row["x_end"] - row["x_start"] > 1e-9:
            module_assembly.add(build_module_solid(row, analysis, role), name=f"{prefix}_body")
        for surface in surfaces_by_part.get(row["part_index"], []):
            for panel in surface["panels"]:
                module_assembly.add(build_wing_solid(surface, panel), name=f"{prefix}_{surface['name']}_{panel['index'] + 1}")
        for motor_index, motor_row in enumerate(motors_by_part.get(row["part_index"], []), start=1):
            module_assembly.add(build_motor_solid(motor_row), name=f"{prefix}_motor_{motor_index}")
            if motor_row["pod_len_flat_m"] > 0.0 or motor_row["pod_len_curve_m"] > 0.0:
                module_assembly.add(build_pod_solid(motor_row), name=f"{prefix}_pod_{motor_index}")
            if include_prop_discs:
                module_assembly.add(build_prop_disc_solid(motor_row), name=f"{prefix}_prop_{motor_index}")
        if module_assembly.children:
            assembly.add(module_assembly, name=prefix)

    structural_part_indices = {row["part_index"] for row, _ in _structural_module_specs(analysis)}
    orphans = (set(surfaces_by_part) | set(motors_by_part)) - structural_part_indices
    if orphans:
        raise RuntimeError(f"Surfaces/motors reference non-structural part indices: {sorted(orphans)}")
    return assembly


def _fuse_airframe(analysis: dict, include_prop_discs: bool = False) -> tuple[object, str]:
    """Boolean-union all solids into one shape; fall back to a compound if fuse fails.

    Wing roots are extended into the fuselage so surface-mounted panels merge
    with the body instead of only touching it tangentially.
    """
    import cadquery as cq

    root_extend_m = 0.75 * analysis["tube_diam"] / 2.0
    shapes = [workplane.val() for _, workplane in _airframe_solids(analysis, include_prop_discs, wing_root_extend_m=root_extend_m)]
    try:
        fused = shapes[0]
        for shape in shapes[1:]:
            fused = fused.fuse(shape)
        return fused.clean(), "fused"
    except Exception as error:
        print(f"cad: fuse failed ({error}); exporting unfused compound instead")
        return cq.Compound.makeCompound(shapes), "compound"


def export_step(analysis: dict, path, mode: str = "assembly", include_prop_discs: bool = False) -> str:
    """Export the airframe as STEP. mode = "assembly" (named components) | "fused" (single OML).

    Returns the mode actually written ("assembly", "fused", or "compound" if
    the boolean fuse failed and an unfused compound was written instead).
    """
    import cadquery as cq

    if mode == "assembly":
        build_airframe_assembly(analysis, include_prop_discs).export(str(path), "STEP")
        return "assembly"
    if mode == "fused":
        shape, result_mode = _fuse_airframe(analysis, include_prop_discs)
        cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STEP")
        return result_mode
    raise ValueError(f"Unknown STEP export mode: {mode}")


def export_stl(analysis: dict, path, include_prop_discs: bool = False) -> str:
    """Export the airframe as a single STL mesh (fused where possible)."""
    import cadquery as cq

    shape, result_mode = _fuse_airframe(analysis, include_prop_discs)
    cq.exporters.export(cq.Workplane("XY").newObject([shape]), str(path), exportType="STL")
    return result_mode
