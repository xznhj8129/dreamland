"""
Usage:
from libs.varsets import build_varsets, export_varsets
varsets = build_varsets(airframe)  # {varset label: {group: {name: property}}}
export_varsets(airframe, project.varsets_dir)

FreeCAD varsets of an airframe/2 definition, one JSON file per varset in the layout Varset_FROM_JSON.FCMacro reads
({group: {name: {Type, Value, Unit}}}; FreeCAD names the property <group>_<name>). The builder drives every value:
each one lives in exactly one varset, nothing is copied between them, and FreeCAD geometry that needs a value from
another varset references it there (<<Airframe>>.Fuselage_Width). Lengths in mm, angles in degrees, as FreeCAD
shows them. Values are the definition's; nothing computed is written.
"""

import json
from pathlib import Path

from libs.airframe import Airframe, Mount, Surface

M_TO_MM = 1000.0
AIRFRAME_VARSET = "Airframe"
PROPULSION_VARSET = "Propulsion"


def length(value_m: float) -> dict:
    return {"Type": "App::PropertyLength", "Value": value_m * M_TO_MM, "Unit": "mm"}


def distance(value_m: float) -> dict:
    """A signed length (offsets, positions that may be negative)."""
    return {"Type": "App::PropertyDistance", "Value": value_m * M_TO_MM, "Unit": "mm"}


def angle(value_deg: float) -> dict:
    return {"Type": "App::PropertyAngle", "Value": value_deg, "Unit": "deg"}


def number(value: float) -> dict:
    return {"Type": "App::PropertyFloat", "Value": value}


def integer(value: int) -> dict:
    return {"Type": "App::PropertyInteger", "Value": value}


def flag(value: bool) -> dict:
    return {"Type": "App::PropertyBool", "Value": value}


def text(value: str) -> dict:
    """Quoted, so the macro's literal_eval keeps "1" or "3" a string."""
    return {"Type": "App::PropertyString", "Value": repr(value)}


def surface_varset_label(surface: Surface) -> str:
    return f"Surface_{surface.name}"


def mount_varset_label(index: int) -> str:
    return f"Mount{index + 1}"


def airframe_varset(airframe: Airframe) -> dict:
    fuselage = airframe.fuselage
    section, joints = fuselage.section, fuselage.joints
    groups = {
        "Fuselage": {"Length": length(fuselage.length_m), "Width": length(section.width_m), "Height": length(section.height_m),
                     "Shape": number(section.shape), "Wall": length(section.wall_radial_m)},
        "Joints": {"Type": text(joints.type), "Count": integer(joints.count), "Screw": text(joints.screw), "InsertionDepth": length(joints.insertion_depth_m),
                   "CouplerWall": length(joints.coupler_wall_m), "FitClearance": length(joints.fit_clearance_m), "SocketWall": length(joints.socket_wall_m),
                   "HoleEdgeDist": length(joints.hole_edge_dist_m), "HoleDiameter": length(joints.hole_diameter_m),
                   "HoleAngleOffset": angle(joints.hole_angle_offset_deg)},
        "Print": {"Material": text(airframe.print.material)} | {f"PrinterBed{axis}": length(value) for axis, value in zip("XYZ", airframe.print.printer_bed_m)},
    }
    for end_name, end in (("Nose", fuselage.nose), ("Tail", fuselage.tail)):
        groups[end_name] = {"Profile": text(end.profile), "Length": length(end.length_m)} | {key: number(value) for key, value in end.params.items()}
    if fuselage.reinforcement is not None:
        rods = fuselage.reinforcement
        pattern = rods.pattern
        groups["Rods"] = {"Material": text(pattern.material), "Diameter": length(pattern.diameter_m), "RadialPosition": length(pattern.radial_position_m),
                          "Count": integer(pattern.polar_n), "OffsetAngle": angle(pattern.polar_offset_angle), "Split": integer(pattern.polar_split),
                          "SplitAngle": angle(pattern.polar_split_angle), "FromX": length(rods.from_x_m), "ToX": length(rods.to_x_m)}
    sections = airframe.sections
    groups["Sections"] = {"CutCount": integer(len(sections.cuts_m))}
    for index, (cut, joint) in enumerate(zip(sections.cuts_m, sections.cut_joints), start=1):
        groups["Sections"] |= {f"Cut{index}": length(cut), f"Cut{index}Joint": text(joint)}
    for index, row in enumerate(sections.per_section, start=1):
        groups["Sections"][f"Section{index}Solid"] = flag(row.solid)
    return groups


def surface_varset(surface: Surface) -> dict:
    groups = {
        "Planform": {"XRootLE": length(surface.x_root_le_m), "Layout": text(surface.layout), "FinPosition": text(surface.fin_position),
                     "Root": text(surface.root), "VerticalOffset": number(surface.vertical_offset), "Span": length(surface.span_m),
                     "ChordRoot": length(surface.chord_root_m), "ChordTip": length(surface.chord_tip_m), "SweepLE": angle(surface.sweep_le_deg),
                     "Dihedral": angle(surface.dihedral_deg), "Incidence": angle(surface.incidence_deg), "IncidenceMode": text(surface.incidence_mode)},
        "Airfoil": {"Name": text(surface.airfoil)},
    }
    if surface.spars is not None:
        spars = surface.spars
        groups["Spars"] = {"Material": text(spars.material), "Diameter": length(spars.diameter_m), "Count": integer(spars.count),
                           "Spacing": length(spars.spacing_m), "XOffset": length(spars.x_offset_m),
                           "YOffset": distance(spars.y_offset_m)}
        if spars.carry_through is not None:
            carry = spars.carry_through
            groups["CarryThrough"] = {"StemWidth": length(carry.stem_width_m), "Length": length(carry.length_m), "FlareRadius": length(carry.flare_radius_m),
                                      "RootLength": length(carry.root_length_m), "WallFillet": length(carry.wall_fillet_m)}
    if surface.root_base is not None:
        base = surface.root_base
        groups["RootBase"] = {"Length": length(base.length_m), "Width": length(base.width_m), "Height": length(base.height_m),
                              "ForwardOffset": distance(base.forward_offset_m), "LeadingCut": length(base.leading_cut_m), "TrailingCut": length(base.trailing_cut_m)}
    if surface.control is not None:
        control = surface.control
        groups["Control"] = {"Kind": text(control.kind), "DeflectionMin": angle(control.deflection_min_deg), "DeflectionMax": angle(control.deflection_max_deg)}
        if control.kind == "split":
            groups["Control"] |= {"ChordStart": number(control.chord_start), "SpanStart": number(control.span_start), "SpanEnd": number(control.span_end)}
        else:
            groups["Control"]["Pivot"] = number(control.pivot)
    return groups


def propulsion_varset(airframe: Airframe) -> dict:
    propulsion = airframe.propulsion
    return {
        "Motor": {"Manufacturer": text(propulsion.motor.manufacturer), "Name": text(propulsion.motor.name),
                  "BellDiameter": length(propulsion.bell_diameter_m), "BellLength": length(propulsion.bell_length_m)},
        "Prop": {"Name": text(propulsion.prop), "Diameter": length(propulsion.prop_diameter_m)},
    }


def mount_varset(mount: Mount) -> dict:
    groups = {"Mount": {"On": text(mount.on), "Direction": text(mount.direction), "Spin": text(mount.spin), "ThrustTilt": angle(mount.thrust_axis_tilt_deg)}}
    if mount.on == "surface":
        groups["Mount"] |= {"Surface": text(mount.surface), "SpanPosition": length(mount.span_position_m), "XOffset": distance(mount.x_offset_m)}
    if mount.pod is not None:
        pod = mount.pod
        groups["Pod"] = {"Diameter": length(pod.diameter_m), "FlatLength": length(pod.flat_length_m), "CurveLength": length(pod.curve_length_m),
                         "Profile": text(pod.profile)}
    return groups


def build_varsets(airframe: Airframe) -> dict[str, dict]:
    varsets = {AIRFRAME_VARSET: airframe_varset(airframe)}
    varsets |= {surface_varset_label(surface): surface_varset(surface) for surface in airframe.surfaces}
    if airframe.propulsion is not None:
        varsets[PROPULSION_VARSET] = propulsion_varset(airframe)
        varsets |= {mount_varset_label(index): mount_varset(mount) for index, mount in enumerate(airframe.propulsion.mounts)}
    return varsets


def export_varsets(airframe: Airframe, directory) -> list[Path]:
    """One <label>.json per varset (label in a Base group, which the macro gives the varset); returns the files written."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    for label, groups in build_varsets(airframe).items():
        path = directory / f"{label}.json"
        path.write_text(json.dumps({"Base": {"Label": text(label)}} | groups, indent=4) + "\n")
        written.append(path)
    return written
