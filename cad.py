"""
Usage:
from cad import build_airframe_params_export, build_freecad_exports
"""

import math

from analysis import resolve_config_with_standard


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
        "Surfaces": {
            "Canards": freecad_property("App::PropertyBool", str(config["canards"])),
            "WingAngle": freecad_property("App::PropertyAngle", analysis["surface_sets"][0]["wing_phase"], "deg"),
            "WingNum": freecad_property("App::PropertyInteger", str(analysis["wing_n"])),
            "WingSpan": freecad_property("App::PropertyDistance", analysis["wing_span"] * 1000.0, "mm"),
        },
    }
    if analysis["tail_n"] > 0:
        export["Surfaces"]["FinAngle"] = freecad_property("App::PropertyAngle", analysis["surface_sets"][-1]["wing_phase"], "deg")
        export["Surfaces"]["FinNum"] = freecad_property("App::PropertyInteger", str(analysis["tail_n"]))
        export["Surfaces"]["FinSpan"] = freecad_property("App::PropertyDistance", analysis["tail_span"] * 1000.0, "mm")
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
