"""
Usage:
from analysis import load_builder_data, recompute_analysis, run_vlm_sweep
"""

import concurrent.futures
import copy
import json
import math
import re
from pathlib import Path

import aerosandbox as asb
import numpy as np
import pandas as pd
from simlib import prop_performance_summary

SCRIPT_DIR = Path(__file__).resolve().parent
LIBS_DIR = SCRIPT_DIR / "libs"
MOTORDATA_PATH = SCRIPT_DIR / "motordata.csv"
MODULES_PATH = LIBS_DIR / "modules.json"
PARTS_PATH = LIBS_DIR / "parts.json"
DEFAULT_CONFIG_PATH = LIBS_DIR / "default.json"
STANDARDS_PATH = LIBS_DIR / "specs.json"

POD_DIAM_TO_ROOT_CHORD = 0.28
POD_LEN_FLAT_TO_ROOT_CHORD = 0.08
POD_LEN_CURVE_TO_ROOT_CHORD = 0.16


def json_safe_data(value):
    value_type = type(value)
    if value_type is dict:
        return {key: json_safe_data(item) for key, item in value.items()}
    if value_type is list:
        return [json_safe_data(item) for item in value]
    if value_type is tuple:
        return [json_safe_data(item) for item in value]
    if value_type is float and not math.isfinite(value):
        return None
    return value


def build_range_values(start_value: float, end_value: float, increment_value: float) -> list[float]:
    return [float(value) for value in np.arange(start_value, end_value + increment_value * 0.5, increment_value)]


def barrowman_finset(
    fin_count: int,
    span: float,
    chord_root: float,
    chord_tip: float,
    sweep_len: float,
    body_radius: float,
    ref_diameter: float,
    x_root_le: float,
) -> tuple[float, float, float]:
    mid_chord_line = math.sqrt(
        span * span + (sweep_len + 0.5 * chord_tip - 0.5 * chord_root) ** 2
    )
    cna = (
        (1.0 + body_radius / (span + body_radius))
        * (4.0 * fin_count * (span / ref_diameter) ** 2)
        / (1.0 + math.sqrt(1.0 + (2.0 * mid_chord_line / (chord_root + chord_tip)) ** 2))
    )
    x_cp_local = (
        sweep_len / 3.0 * ((chord_root + 2.0 * chord_tip) / (chord_root + chord_tip))
        + (1.0 / 6.0)
        * (chord_root + chord_tip - (chord_root * chord_tip) / (chord_root + chord_tip))
    )
    return cna, x_root_le + x_cp_local, mid_chord_line


def trapezoid_mac(chord_root: float, chord_tip: float) -> float:
    return (
        (2.0 / 3.0)
        * ((chord_root * chord_root) + (chord_root * chord_tip) + (chord_tip * chord_tip))
        / (chord_root + chord_tip)
    )


def trapezoid_ac_x(x_root_le: float, span: float, chord_root: float, chord_tip: float, sweep_len: float) -> float:
    taper = chord_tip / chord_root if chord_root > 0.0 else 0.0
    y_mac = span * (1.0 + 2.0 * taper) / (3.0 * (1.0 + taper))
    x_mac_le = x_root_le + sweep_len * y_mac / span if span > 0.0 else x_root_le
    return x_mac_le + 0.25 * trapezoid_mac(chord_root, chord_tip)


def finite_wing_lift_slope(span: float, chord_root: float, chord_tip: float) -> float:
    area = span * (chord_root + chord_tip) / 2.0
    aspect_ratio = (2.0 * span) ** 2 / (2.0 * area)
    a0 = 2.0 * math.pi
    e = 0.9
    return a0 / (1.0 + a0 / (math.pi * e * aspect_ratio))


def surface_panel_layout(count: int, root_radius: float, span: float, wing_phase_deg: float, dihedral_deg: float) -> list[dict]:
    panels = []
    dihedral_rad = math.radians(dihedral_deg)
    phase_eps = 1e-9
    for panel_index in range(count):
        panel_phase_deg = wing_phase_deg + panel_index * 360.0 / count
        panel_phase_rad = math.radians(panel_phase_deg)
        root_y = root_radius * math.cos(panel_phase_rad)
        root_z = root_radius * math.sin(panel_phase_rad)
        dihedral_sign = 1.0
        if count > 2:
            if root_z > phase_eps:
                dihedral_sign = 1.0
            elif root_z < -phase_eps:
                dihedral_sign = -1.0
            else:
                dihedral_sign = 0.0
        panel_dihedral_rad = dihedral_rad * dihedral_sign
        dir_y = math.cos(panel_dihedral_rad) * math.cos(panel_phase_rad)
        dir_z = math.cos(panel_dihedral_rad) * math.sin(panel_phase_rad) + math.sin(panel_dihedral_rad)
        dir_norm = math.hypot(dir_y, dir_z)
        dir_y /= dir_norm
        dir_z /= dir_norm
        panels.append(
            {
                "index": panel_index,
                "phase_deg": panel_phase_deg,
                "dihedral_sign": dihedral_sign,
                "root_y": root_y,
                "root_z": root_z,
                "tip_y": root_y + span * dir_y,
                "tip_z": root_z + span * dir_z,
                "dir_y": dir_y,
                "dir_z": dir_z,
            }
        )
    return panels


def average_mount_factor(panel_layout: list[dict]) -> float:
    return sum(panel["dir_y"] * panel["dir_y"] for panel in panel_layout) / len(panel_layout)


def create_asb_surface(
    idx: int,
    chord_root: float,
    chord_tip: float,
    x_le: float,
    y_le: float,
    z_le: float,
    y_tip: float,
    z_tip: float,
    aoa_deg: float,
    sweep_deg: float,
    name_prefix: str,
    airfoil_name: str,
) -> asb.Wing:
    base_airfoil = asb.Airfoil(airfoil_name)
    span = math.hypot(y_tip - y_le, z_tip - z_le)
    dx_sweep = span * math.tan(math.radians(sweep_deg))
    root = asb.WingXSec(
        xyz_le=[x_le, y_le, z_le],
        chord=chord_root,
        twist=aoa_deg,
        airfoil=base_airfoil,
    )
    tip = asb.WingXSec(
        xyz_le=[x_le + dx_sweep, y_tip, z_tip],
        chord=chord_tip,
        twist=aoa_deg,
        airfoil=base_airfoil,
    )
    return asb.Wing(
        name=f"{name_prefix}_{idx + 1}",
        symmetric=False,
        xsecs=[root, tip],
    )


def build_asb_airplane(analysis: dict) -> asb.Airplane:
    n_cap = 12
    eps = 1e-6
    structural_rows = [row for row in analysis["module_rows"] if row["module_type"] in {"body", "end", "coupler", "wingmount"}]
    tube_radius = analysis["tube_diam"] / 2.0

    def ogive_radius(r_value: float, length_value: float) -> float:
        return (r_value * r_value + length_value * length_value) / (2.0 * r_value)

    def nose_radius(x_value: float, r_value: float, length_value: float) -> float:
        rho = ogive_radius(r_value, length_value)
        value = rho * rho - (length_value - x_value) * (length_value - x_value)
        return math.sqrt(value) + (r_value - rho) if value > 0.0 else 0.0

    def tail_radius(s_value: float, r_value: float, length_value: float) -> float:
        rho = ogive_radius(r_value, length_value)
        value = rho * rho - s_value * s_value
        return math.sqrt(value) + (r_value - rho) if value > 0.0 else 0.0

    xsecs = []

    def add_xsec(x_value: float, radius_value: float) -> None:
        if xsecs and abs(x_value - xsecs[-1].xyz_c[0]) < eps:
            return
        xsecs.append(asb.FuselageXSec(xyz_c=[x_value, 0.0, 0.0], radius=radius_value))

    end_count = 0
    for row in structural_rows:
        module_len = row["x_end"] - row["x_start"]
        if row["module_type"] == "end" and end_count == 0:
            add_xsec(row["x_start"], 0.0)
            for xi in np.linspace(0.0, module_len, n_cap + 2)[1:-1]:
                add_xsec(row["x_start"] + float(xi), nose_radius(float(xi), tube_radius, module_len))
            add_xsec(row["x_end"], tube_radius)
            end_count += 1
            continue
        if row["module_type"] == "end" and end_count == 1:
            add_xsec(row["x_start"], tube_radius)
            for si in np.linspace(0.0, module_len, n_cap + 2)[1:-1]:
                add_xsec(row["x_start"] + float(si), tail_radius(float(si), tube_radius, module_len))
            add_xsec(row["x_end"], 0.0)
            end_count += 1
            continue
        add_xsec(row["x_start"], tube_radius)
        add_xsec(row["x_end"], tube_radius)

    fuselage = asb.Fuselage(name="Fuselage", xsecs=xsecs)

    main_wings = []
    tail_wings = []
    main_index = 0
    tail_index = 0
    for surface in analysis["surface_sets"]:
        x_le = surface["x_root_le"]
        for panel in surface["panels"]:
            wing = create_asb_surface(
                idx=panel["index"],
                chord_root=surface["chord_root"],
                chord_tip=surface["chord_tip"],
                x_le=x_le,
                y_le=panel["root_y"],
                z_le=panel["root_z"],
                y_tip=panel["tip_y"],
                z_tip=panel["tip_z"],
                aoa_deg=surface["aoa"],
                sweep_deg=math.degrees(math.atan2(surface["sweep_len"], surface["span"])) if surface["span"] > 0.0 else 0.0,
                name_prefix=surface["name"],
                airfoil_name=surface["airfoil"],
            )
            if surface["role"] == "main":
                wing.name = f"{surface['name']}_main_{main_index + 1}"
                main_wings.append(wing)
                main_index += 1
            elif surface["role"] in {"tail", "canard"}:
                wing.name = f"{surface['name']}_{surface['role']}_{tail_index + 1}"
                tail_wings.append(wing)
                tail_index += 1
            else:
                wing.name = f"{surface['name']}_aux_{tail_index + 1}"
                tail_wings.append(wing)
                tail_index += 1

    c_ref = (analysis["wing_chord_root"] + analysis["wing_chord_tip"]) / 2.0 if (analysis["wing_chord_root"] + analysis["wing_chord_tip"]) > 0.0 else analysis["tube_diam"]
    b_ref = analysis["tot_wingspan"]
    return asb.Airplane(
        name=analysis["config_name"],
        xyz_ref=[analysis["cg_x_m"], 0.0, 0.0],
        s_ref=analysis["s_main_planform"],
        c_ref=c_ref,
        b_ref=b_ref,
        wings=main_wings + tail_wings,
        fuselages=[fuselage],
        propulsors=[],
    )


def resolve_config_with_standard(config: dict, standards_data: dict) -> dict:
    resolved = copy.deepcopy(config)
    if resolved["apply_standard"] and resolved["standard_profile"]:
        standard_spec = standards_data["specs"][resolved["standard_profile"]]
        resolved["fuselage_od_mm"] = standard_spec["module_size"]["nominal_od"]
        resolved["fuselage_id_mm"] = standard_spec["module_size"]["nominal_id"]
    return resolved


def load_builder_data() -> tuple[dict, dict, dict, dict, pd.DataFrame, list[str], list[str]]:
    with open(MODULES_PATH, "r") as file:
        modules_data = json.loads(file.read())
    with open(PARTS_PATH, "r") as file:
        parts_data = json.loads(file.read())
    with open(DEFAULT_CONFIG_PATH, "r") as file:
        default_config = json.loads(file.read())
    with open(STANDARDS_PATH, "r") as file:
        standards_data = json.loads(file.read())
    motor_data_df = pd.read_csv(MOTORDATA_PATH)
    motor_data_df["display_name"] = (
        motor_data_df["Brand"].astype(str)
        + " "
        + motor_data_df["Motor"].astype(str)
        + "-"
        + motor_data_df["KV"].astype(int).astype(str)
    )
    motor_names = sorted(motor_data_df["display_name"].dropna().astype(str).unique().tolist())
    propeller_names = sorted(motor_data_df["Prop"].dropna().astype(str).unique().tolist())
    return modules_data, parts_data, default_config, standards_data, motor_data_df, motor_names, propeller_names


def resolve_named_block(block, modules_data: dict) -> tuple[str, dict]:
    if type(block) is dict:
        return block["name"], copy.deepcopy(block)
    return block, copy.deepcopy(modules_data[block])


def parse_motor_name(motor_entry) -> tuple[str, str, str, str]:
    brand_override = None
    if type(motor_entry) is dict:
        brand_override = motor_entry.get("mfg")
        motor_name = motor_entry.get("model", "")
    else:
        motor_name = motor_entry
    tokens = re.split(r"[_\s-]+", str(motor_name).strip())
    brand = None
    model = None
    kv = None
    if len(tokens) >= 3:
        brand = tokens[0]
        kv = tokens[-1]
        model = "_".join(tokens[1:-1])
    else:
        match = re.match(r"([A-Za-zA-Z]+)[\s_-]*(\d+)[\s_-]*(\d+)", motor_name)
        if match:
            brand = match.group(1)
            model = match.group(2)
            kv = match.group(3)
    if brand_override:
        brand = brand_override
    return motor_name, brand, model, kv


def find_motor_group(motor_name: str, brand: str, model: str, kv_val: str, prop_name: str, batt_s: int, motor_data_df: pd.DataFrame) -> pd.DataFrame:
    group = motor_data_df[
        (motor_data_df["Brand"].astype(str).str.lower() == str(brand).lower())
        & (motor_data_df["Motor"].astype(str).str.lower() == str(model).lower())
        & (motor_data_df["KV"].astype(int) == int(kv_val))
        & (motor_data_df["Prop"].astype(str).str.lower() == str(prop_name).lower())
        & (motor_data_df["Data Batt S"].astype(int) == int(batt_s))
    ]
    if group.empty:
        raise RuntimeError(f"No simlib data found for {motor_name} with {prop_name} on {batt_s}S")
    return group.sort_values("T")


def motor_mass_kg(motor_name: str, brand: str, model: str, kv_val: str, prop_name: str, batt_s: int, motor_data_df: pd.DataFrame) -> float:
    group = find_motor_group(motor_name, brand, model, kv_val, prop_name, batt_s, motor_data_df)
    return float(group["Weight g"].iloc[0]) / 1000.0


def motor_bell_geometry_m(motor_name: str, brand: str, model: str, kv_val: str, prop_name: str, batt_s: int, motor_data_df: pd.DataFrame) -> tuple[float, float]:
    group = find_motor_group(motor_name, brand, model, kv_val, prop_name, batt_s, motor_data_df)
    return float(group["Stator Diameter"].iloc[0]) / 1000.0, float(group["Stator height"].iloc[0]) / 1000.0


def parse_prop_diameter_m(prop_name: str) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)", prop_name)
    if match:
        return float(match.group(1)) * 0.0254
    digits_match = re.search(r"(\d{4,5})", prop_name)
    if digits_match:
        digits = digits_match.group(1)
        diameter_in = float(digits[:-3])
        return diameter_in * 0.0254
    raise RuntimeError(f"Could not parse prop diameter from {prop_name}")


def build_report_text(analysis: dict) -> str:
    lines = []
    lines.append(f"Airframe: {analysis['assembly_name']}")
    lines.append("")
    lines.append("Overall Summary")
    lines.append(f"Main wing incidence   : {analysis['wing_aoa']:.3f} deg")
    if analysis["tail_airfoil"] is not None:
        aft_label = "Canard incidence" if analysis["canards"] else "Tail AoA"
        lines.append(f"{aft_label:<21}: {analysis['tail_aoa']:.3f} deg")
    lines.append(f"Fuselage length       : {analysis['fuselage_length']:.4f} m")
    lines.append(f"Main wingspan         : {analysis['tot_wingspan']:.4f} m")
    lines.append(f"Total mass            : {analysis['total_mass_kg'] * 1000.0:.1f} g")
    lines.append(f"Total weight          : {analysis['weight_N']:.4f} N")
    if analysis["total_motor_count"] > 0:
        lines.append(f"Total thrust/weight   : {analysis['static_thrust_to_weight']:.3f}")
    lines.append(f"Total moment          : {analysis['total_moment_kg_m']:.4f} kg*m")
    lines.append(f"CG                    : {analysis['cg_x_m']:.4f} m")
    lines.append("")
    lines.append("Lifting Surfaces")
    lines.append(f"Main leading edge pos : {analysis['le_pos_x']:.4f} m")
    lines.append(f"Reference area        : {analysis['s_main_planform']:.4f} m^2")
    lines.append(f"Tail planform area    : {analysis['s_tail_planform']:.4f} m^2")
    lines.append(f"Wing loading          : {analysis['wing_loading']:.2f} N/m^2")
    lines.append(f"Wing/tail area ratio  : {analysis['wing_to_tail_ratio']:.2f}")
    if analysis["battery_summary"] is not None:
        battery_summary = analysis["battery_summary"]
        lines.append("")
        lines.append("Battery")
        lines.append(f"Type                  : {battery_summary['s']}S{battery_summary['p']}P")
        lines.append(f"Capacity              : {battery_summary['mah']:.0f} mAh {battery_summary['wh']:.2f} Wh")
        lines.append(f"Current               : {battery_summary['curr']:.2f} A")
    if analysis["total_motor_count"] > 0:
        lines.append("")
        lines.append("Motors")
        lines.append(f"Motor count           : {analysis['total_motor_count']}")
        lines.append(f"Static thrust total   : {analysis['total_static_thrust_n']:.2f} N")
        for row in analysis["motor_summary_rows"]:
            lines.append(f"Config                : {row['label']} with {row['propeller']}")
            lines.append(f"Pack voltage          : {row['pack_voltage_v']:.2f} V")
            lines.append(f"Motor KV              : {row['kv']:.0f}")
            lines.append(f"No-load RPM           : {row['no_load_rpm']:.0f}")
            lines.append(f"Static prop RPM       : {row['prop_rpm']:.0f}")
            lines.append(f"Static thrust         : {row['thrust_n_per_motor']:.2f} N each {row['thrust_n_total']:.2f} N total")
            lines.append(f"Static current        : {row['current_a_per_motor']:.2f} A each {row['current_a_total']:.2f} A total")
            lines.append(f"Static input power    : {row['input_power_w_per_motor']:.1f} W each {row['input_power_w_total']:.1f} W total")
            lines.append(f"Pitch speed           : {row['pitch_speed_kph']:.1f} km/h")
            lines.append(f"Efficiency            : {row['efficiency_g_per_w']:.2f} g/W")
            if row["warnings"]:
                lines.append(f"Warnings              : {', '.join(row['warnings'])}")
    lines.append("")
    lines.append("Static Stability")
    lines.append(f"Combined CP           : {analysis['x_cp_total']:.4f} m")
    lines.append(f"Static margin         : {analysis['static_margin_m']:.4f} m")
    lines.append(f"Static margin % MAC   : {analysis['static_margin_pct_mac']:.2f} %")
    lines.append(f"Static stability      : {analysis['stability_verdict']}")
    lines.append("")
    lines.append("Aerodynamic Centers")
    lines.append(f"CG marker             : blue @ {analysis['cg_x_m']:.4f} m")
    lines.append(f"CP marker             : red @ {analysis['x_cp_total']:.4f} m")
    if math.isfinite(analysis["lift_center_x"]):
        lines.append(f"CL marker             : green @ {analysis['lift_center_x']:.4f} m @ AoA {analysis['aoa_eval_deg']:.2f} deg")
        lines.append(f"CN / Cm               : {analysis['aoa_cn']:.4f} / {analysis['aoa_cm']:.4f}")
        lines.append(f"AoA verdict           : {analysis['aoa_verdict']}")
    lines.append("")
    lines.append("Components")
    for row in analysis["component_rows"]:
        lines.append(
            f"{row['component']:<24} role={row['role']:<6} CNa={row['cna']:.3f} x_cp={row['x_cp']:.4f} {row['notes']}"
        )
    lines.append("")
    lines.append("Modules")
    for row in analysis["module_rows"]:
        lines.append(
            f"{row['module_name']:<24} {row['module_type']:<10} x0={row['x_start']:.4f} x1={row['x_end']:.4f} mass={row['mass'] * 1000.0:.1f}g"
        )
    lines.append("")
    lines.append("Notes")
    lines.append("Barrowman-style estimate ignores airfoil nonlinearities, trim, thrust, and wing-body interference.")
    lines.append("Use this as a fast first-pass stability screen, not as a forward-flight trim solver.")
    return "\n".join(lines)


def build_vlm_report_text(vlm_results: dict) -> str:
    lines = []
    settings = vlm_results["settings"]
    lines.append("VLM Sweep")
    lines.append(f"Airframe              : {vlm_results['airframe_name']}")
    lines.append("")
    lines.append("Parameters")
    lines.append(
        f"AoA                   : {settings['aoa_start']:.1f} to {settings['aoa_end']:.1f} deg step {settings['aoa_increment']:.1f}"
    )
    lines.append(
        f"Airspeed              : {settings['airspeed_start']:.1f} to {settings['airspeed_end']:.1f} m/s step {settings['airspeed_increment']:.1f}"
    )
    lines.append(
        f"Altitude              : {settings['altitude_start']:.1f} to {settings['altitude_end']:.1f} m step {settings['altitude_increment']:.1f}"
    )
    lines.append(f"Spanwise resolution   : {settings['vlm_span_res']}")
    lines.append(f"Chordwise resolution  : {settings['vlm_chord_res']}")
    lines.append(f"Worker threads        : {settings['vlm_threads']}")
    lines.append(f"Solved points         : {len(vlm_results['rows'])}")
    lines.append("")
    lines.append("Results")
    header = (
        f"{'Alt m':<8} | "
        f"{'AoA':<5} | "
        f"{'V m/s':<7} | "
        f"{'Lift N':<8} | "
        f"{'Drag_i N':<10} | "
        f"{'CL':<7} | "
        f"{'CDi':<7} | "
        f"{'Cm':<7} | "
        f"{'Load %':<8} | "
        f"{'SM %MAC':<8}"
    )
    lines.append(header)
    lines.append("-" * len(header))
    for row in vlm_results["rows"]:
        lines.append(
            f"{row['altitude_m']:<8.1f} | "
            f"{row['aoa_deg']:<5.1f} | "
            f"{row['airspeed_m_s']:<7.1f} | "
            f"{row['lift_n']:<8.2f} | "
            f"{row['drag_induced_n']:<10.2f} | "
            f"{row['cl']:<7.4f} | "
            f"{row['cdi']:<7.4f} | "
            f"{row['cm']:<7.4f} | "
            f"{row['load_pct']:<8.1f} | "
            f"{row['static_margin_pct_mac']:<8.2f}"
        )
    return "\n".join(lines)


def run_vlm_sweep(
    analysis: dict,
    settings: dict,
    progress_callback=None,
) -> dict:
    airplane = build_asb_airplane(analysis)
    aoa_values = build_range_values(settings["aoa_start"], settings["aoa_end"], settings["aoa_increment"])
    airspeed_values = build_range_values(settings["airspeed_start"], settings["airspeed_end"], settings["airspeed_increment"])
    altitude_values = build_range_values(settings["altitude_start"], settings["altitude_end"], settings["altitude_increment"])
    c_ref = airplane.c_ref
    total_points = len(aoa_values) * len(airspeed_values) * len(altitude_values)
    completed_points = 0
    rows = []

    def solve_point(point: tuple[float, float, float]) -> dict:
        altitude_m, aoa_deg, airspeed_m_s = point
        op_point = asb.OperatingPoint(
            atmosphere=asb.Atmosphere(altitude=altitude_m),
            velocity=airspeed_m_s,
            alpha=aoa_deg,
        )
        vlm = asb.VortexLatticeMethod(
            airplane=airplane,
            op_point=op_point,
            spanwise_resolution=int(settings["vlm_span_res"]),
            chordwise_resolution=int(settings["vlm_chord_res"]),
        )
        vlm_result = vlm.run_with_stability_derivatives()
        lift_n = float(vlm_result["L"])
        drag_induced_n = float(abs(vlm_result["D"]))
        moment_y_n_m = float(vlm_result["M_b"][1])
        x_cp_m = analysis["cg_x_m"] + (-moment_y_n_m / lift_n) if lift_n != 0.0 else float("nan")
        cla_rad = float(vlm_result["CLa"])
        cma_rad = float(vlm_result["Cma"])
        x_np_m = analysis["cg_x_m"] - (cma_rad / cla_rad) * c_ref if cla_rad != 0.0 else float("nan")
        static_margin_pct_mac = ((x_np_m - analysis["cg_x_m"]) / c_ref) * 100.0 if c_ref != 0.0 else float("nan")
        return {
            "altitude_m": altitude_m,
            "aoa_deg": aoa_deg,
            "airspeed_m_s": airspeed_m_s,
            "lift_n": lift_n,
            "drag_induced_n": drag_induced_n,
            "moment_y_n_m": moment_y_n_m,
            "x_cp_m": x_cp_m,
            "x_np_m": x_np_m,
            "cl": float(vlm_result["CL"]),
            "cdi": float(vlm_result["CD"]),
            "cm": float(vlm_result["Cm"]),
            "cla_per_deg": cla_rad * (math.pi / 180.0),
            "cma_per_deg": cma_rad * (math.pi / 180.0),
            "load_pct": lift_n / analysis["weight_N"] * 100.0,
            "static_margin_pct_mac": static_margin_pct_mac,
        }

    def update_progress(row: dict) -> None:
        nonlocal completed_points
        completed_points += 1
        if progress_callback is not None:
            progress_callback(
                completed_points,
                total_points,
                f"Alt {row['altitude_m']:.1f} m  AoA {row['aoa_deg']:.1f} deg  V {row['airspeed_m_s']:.1f} m/s",
            )

    for altitude_m in altitude_values:
        for aoa_deg in aoa_values:
            points = [(altitude_m, aoa_deg, airspeed_m_s) for airspeed_m_s in airspeed_values]
            if int(settings["vlm_threads"]) > 1:
                with concurrent.futures.ThreadPoolExecutor(max_workers=int(settings["vlm_threads"])) as executor:
                    for row in executor.map(solve_point, points):
                        rows.append(row)
                        update_progress(row)
            else:
                for point in points:
                    row = solve_point(point)
                    rows.append(row)
                    update_progress(row)

    rows.sort(key=lambda row: (row["altitude_m"], row["aoa_deg"], row["airspeed_m_s"]))
    vlm_results = {
        "airframe_name": analysis["config_name"],
        "settings": copy.deepcopy(settings),
        "rows": rows,
    }
    vlm_results["report_text"] = build_vlm_report_text(vlm_results)
    return vlm_results


def _static_motor_summary(
    motor_data_df: pd.DataFrame,
    motor_name: str,
    brand: str,
    model: str,
    kv_val: str,
    prop_name: str,
    count: int,
    batt_s: int,
) -> dict:
    group = find_motor_group(motor_name, brand, model, kv_val, prop_name, batt_s, motor_data_df)
    static_row = group.iloc[-1]
    measured_thrust_n = float(static_row["Thrust g"]) / 1000.0 * 9.81
    measured_voltage_v = float(static_row["V"])
    summary = prop_performance_summary(
        {
            "diameter_in": float(static_row["Data P size"]),
            "pitch_in": float(static_row["Data P pitch"]),
            "kv": float(static_row["KV"]),
            "cells": float(static_row["Data Batt S"]),
            "volts_per_cell": measured_voltage_v / float(static_row["Data Batt S"]),
            "measured_rpm": float(static_row["RPM"]),
            "measured_input_w": float(static_row["W"]),
            "measured_thrust_n": measured_thrust_n,
            "measured_v_bus": measured_voltage_v,
        }
    )
    return {
        "label": f"{motor_name} x{count}",
        "count": count,
        "propeller": prop_name,
        "kv": float(static_row["KV"]),
        "pack_voltage_v": measured_voltage_v,
        "no_load_rpm": measured_voltage_v * float(static_row["KV"]),
        "prop_rpm": float(static_row["RPM"]),
        "thrust_n_per_motor": measured_thrust_n,
        "thrust_n_total": measured_thrust_n * count,
        "current_a_per_motor": float(static_row["A"]),
        "current_a_total": float(static_row["A"]) * count,
        "input_power_w_per_motor": float(static_row["W"]),
        "input_power_w_total": float(static_row["W"]) * count,
        "pitch_speed_kph": float(summary["pitch_speed_kmh"]),
        "efficiency_g_per_w": float(static_row["Efficiency"]),
        "warnings": [],
    }


def build_full_assembly_json(
    assembly_name: str,
    config: dict,
    modules_data: dict,
    parts_data: dict,
    standards_data: dict,
    motor_data_df: pd.DataFrame,
) -> dict:
    config = resolve_config_with_standard(config, standards_data)
    tube_diam = config["fuselage_od_mm"] / 1000.0
    assembly = []
    for part in config["parts"]:
        assembly.append(part)
        assembly.append({"module": "joint-ring-c"})
    assembly.pop()

    canards = config["canards"]
    surface_sets = []
    module_rows = []
    fuselage_length = 0.0
    total_mass_kg = 0.0
    total_moment_kg_m = 0.0
    current_x_fuselage_m = 0.0
    batt_s = None
    batt_p = None
    battery_summary = None
    wing_aoa = 0.0
    tail_aoa = 0.0
    wing_airfoil = None
    tail_airfoil = None
    wing_chord_root = 0.0
    wing_chord_tip = 0.0
    wing_span = 0.0
    wing_n = 0
    tail_chord_root = 0.0
    tail_chord_tip = 0.0
    tail_span = 0.0
    tail_n = 0
    le_pos_x = 0.0
    total_motor_count = 0
    nose_length = 0.0
    motor_draw_rows = []
    motor_summary_rows = []

    for item_in_assembly in assembly:
        module_name, module_data = resolve_named_block(item_in_assembly["module"], modules_data)
        module_mass = module_data["mass"]
        module_len = module_data["len"]
        module_cg_offset_from_module_front_m = module_data["cg"] * module_len
        module_structure_cg_global_x_m = current_x_fuselage_m + module_cg_offset_from_module_front_m
        module_rows.append(
            {
                "module_name": module_name,
                "module_type": module_data["type"],
                "x_start": current_x_fuselage_m,
                "x_end": current_x_fuselage_m + module_len,
                "mass": module_mass,
                "cg_x": module_structure_cg_global_x_m,
                "diam": tube_diam,
            }
        )
        total_mass_kg += module_mass
        total_moment_kg_m += module_mass * module_structure_cg_global_x_m
        attached_part_cg_global_x_m = module_structure_cg_global_x_m

        if module_data.get("battery", False):
            battery_name = item_in_assembly["battery"]
            battery_part_data = parts_data["battery"][battery_name]
            batt_s = battery_part_data["s"]
            batt_p = battery_part_data["p"]
            batt_mah = battery_part_data["mah"]
            battery_mass = battery_part_data["mass"]
            batt_curr = (battery_part_data["mah"] / 1000.0) * battery_part_data["c"]
            batt_wh = (batt_mah / 1000.0) * (batt_s * 3.7)
            battery_summary = {
                "s": batt_s,
                "p": batt_p,
                "mah": batt_mah,
                "curr": batt_curr,
                "wh": batt_wh,
            }
            total_mass_kg += battery_mass
            total_moment_kg_m += battery_mass * attached_part_cg_global_x_m
            module_rows.append(
                {
                    "module_name": battery_name,
                    "module_type": "battery",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": battery_mass,
                    "cg_x": attached_part_cg_global_x_m,
                    "diam": tube_diam,
                }
            )

        if module_data.get("payload", False):
            payload_mass = item_in_assembly["payload_mass"]
            total_mass_kg += payload_mass
            total_moment_kg_m += payload_mass * attached_part_cg_global_x_m
            module_rows.append(
                {
                    "module_name": "payload",
                    "module_type": "payload",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": payload_mass,
                    "cg_x": attached_part_cg_global_x_m,
                    "diam": tube_diam,
                }
            )

        if module_data.get("ballast", False):
            ballast_mass = item_in_assembly["ballast_mass"]
            total_mass_kg += ballast_mass
            total_moment_kg_m += ballast_mass * attached_part_cg_global_x_m
            module_rows.append(
                {
                    "module_name": "ballast",
                    "module_type": "ballast",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": ballast_mass,
                    "cg_x": attached_part_cg_global_x_m,
                    "diam": tube_diam,
                }
            )

        if module_data["type"] == "wingmount":
            wing_spec_name, wing_data = resolve_named_block(item_in_assembly["surfaces"], modules_data)
            num_wings = module_data["wings_n"]
            total_wings_mass = num_wings * wing_data["mass"]
            total_mass_kg += total_wings_mass
            total_moment_kg_m += total_wings_mass * attached_part_cg_global_x_m

            if wing_data.get("motor", False):
                motor_name, brand, model, kv = parse_motor_name(item_in_assembly["motors"])
                prop_name = item_in_assembly["props"]
                total_motor_count += num_wings
                motor_mass_each = motor_mass_kg(motor_name, brand, model, kv, prop_name, batt_s, motor_data_df)
                motor_bell_diam_m, motor_bell_len_m = motor_bell_geometry_m(motor_name, brand, model, kv, prop_name, batt_s, motor_data_df)
                total_wing_motors_mass = num_wings * motor_mass_each
                total_mass_kg += total_wing_motors_mass
                total_moment_kg_m += total_wing_motors_mass * attached_part_cg_global_x_m
                module_rows.append(
                    {
                        "module_name": motor_name,
                        "module_type": "motor",
                        "x_start": current_x_fuselage_m,
                        "x_end": current_x_fuselage_m + module_len,
                        "mass": total_wing_motors_mass,
                        "cg_x": attached_part_cg_global_x_m,
                        "diam": tube_diam,
                        "prop_name": prop_name,
                    }
                )
                motor_summary_rows.append(
                    _static_motor_summary(
                        motor_data_df,
                        motor_name,
                        brand,
                        model,
                        kv,
                        prop_name,
                        num_wings,
                        batt_s,
                    )
                )

        if module_data["type"] == "end" and module_data.get("motor", False):
            motor_name, brand, model, kv = parse_motor_name(item_in_assembly["motors"])
            prop_name = item_in_assembly["props"]
            total_motor_count += 1
            motor_mass_each = motor_mass_kg(motor_name, brand, model, kv, prop_name, batt_s, motor_data_df)
            motor_bell_diam_m, motor_bell_len_m = motor_bell_geometry_m(motor_name, brand, model, kv, prop_name, batt_s, motor_data_df)
            motor_cg_offset_from_module_front_m = module_len * 0.5
            end_motor_cg_global_x_m = current_x_fuselage_m + motor_cg_offset_from_module_front_m
            total_mass_kg += motor_mass_each
            total_moment_kg_m += motor_mass_each * end_motor_cg_global_x_m
            module_rows.append(
                {
                    "module_name": motor_name,
                    "module_type": "motor",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": motor_mass_each,
                    "cg_x": end_motor_cg_global_x_m,
                    "diam": tube_diam,
                    "prop_name": prop_name,
                }
            )
            prop_diam_m = parse_prop_diameter_m(prop_name)
            motor_draw_rows.append(
                {
                    "kind": "end",
                    "x_m": current_x_fuselage_m + module_len * 0.5,
                    "y_m": 0.0,
                    "z_m": 0.0,
                    "motor_bell_diam_m": motor_bell_diam_m,
                    "motor_bell_len_m": motor_bell_len_m,
                    "pod_diam_m": motor_bell_diam_m,
                    "pod_len_flat_m": 0.0,
                    "pod_len_curve_m": 0.0,
                    "pod_shape": "elliptical",
                    "prop_diam_m": prop_diam_m,
                    "direction": module_data["motor_dir"],
                }
            )
            motor_summary_rows.append(_static_motor_summary(motor_data_df, motor_name, brand, model, kv, prop_name, 1, batt_s))

        if module_data["type"] == "end" and nose_length == 0.0:
            nose_length = module_len

        fuselage_length += module_len
        current_x_fuselage_m += module_len

    cg_x_m = total_moment_kg_m / total_mass_kg
    weight_n = total_mass_kg * 9.81

    x = 0.0
    module_x_map = []
    for module_entry in assembly:
        _, module_data = resolve_named_block(module_entry["module"], modules_data)
        length = module_data["len"]
        module_x_map.append(x + length / 2.0)
        x += length

    body_x_start = 0.0
    body_x_end = 0.0
    x_cursor = 0.0
    end_count = 0
    for module_entry in assembly:
        _, module_data = resolve_named_block(module_entry["module"], modules_data)
        if module_data["type"] == "end":
            if end_count == 0:
                body_x_start = x_cursor + module_data["len"]
            else:
                body_x_end = x_cursor
            end_count += 1
        x_cursor += module_data["len"]

    n_wing_modules = 0
    main_role_label = "main"
    tail_role_label = "canard" if canards else "tail"
    tot_wingspan = None
    for index, module_entry in enumerate(assembly):
        _, module_data = resolve_named_block(module_entry["module"], modules_data)
        if module_data["type"] != "wingmount":
            continue
        spec_name, wing_data = resolve_named_block(module_entry["surfaces"], modules_data)
        panel_layout = surface_panel_layout(
            count=module_data["wings_n"],
            root_radius=tube_diam / 2.0,
            span=wing_data["semispan"],
            wing_phase_deg=module_data["wing_phase"],
            dihedral_deg=module_data["wings_ang"],
        )
        n_wing_modules += 1
        role = "aux"
        if (n_wing_modules == 1 and not canards) or (n_wing_modules == 2 and canards):
            role = main_role_label
            wing_chord_root = wing_data["chord_root"]
            wing_chord_tip = wing_data["chord_tip"]
            wing_span = wing_data["semispan"]
            wing_n = module_data["wings_n"]
            wing_aoa = wing_data["aoa_ang"]
            wing_airfoil = wing_data["airfoil"]
            min_y = -tube_diam / 2.0
            max_y = tube_diam / 2.0
            for panel in panel_layout:
                min_y = min(min_y, panel["root_y"], panel["tip_y"])
                max_y = max(max_y, panel["root_y"], panel["tip_y"])
            tot_wingspan = max_y - min_y
            le_pos_x = module_x_map[index] - wing_data["chord_root"] / 2.0
        elif (n_wing_modules == 1 and canards) or (n_wing_modules == 2 and not canards):
            role = tail_role_label
            tail_chord_root = wing_data["chord_root"]
            tail_chord_tip = wing_data["chord_tip"]
            tail_span = wing_data["semispan"]
            tail_n = module_data["wings_n"]
            tail_aoa = wing_data["aoa_ang"]
            tail_airfoil = wing_data["airfoil"]
        sweep_len = wing_data["semispan"] * math.tan(math.radians(wing_data["sweep_ang"]))
        surface_sets.append(
            {
                "role": role,
                "name": spec_name,
                "module": module_entry["module"] if type(module_entry["module"]) is str else module_entry["module"]["name"],
                "count": module_data["wings_n"],
                "span": wing_data["semispan"],
                "chord_root": wing_data["chord_root"],
                "chord_tip": wing_data["chord_tip"],
                "sweep_len": sweep_len,
                "x_root_le": module_x_map[index] - wing_data["chord_root"] / 2.0,
                "body_radius": tube_diam / 2.0,
                "wing_phase": module_data["wing_phase"],
                "dihedral_deg": module_data["wings_ang"],
                "aoa": wing_data["aoa_ang"],
                "airfoil": wing_data["airfoil"],
                "planform_area_total": module_data["wings_n"] * wing_data["semispan"] * (wing_data["chord_root"] + wing_data["chord_tip"]) / 2.0,
                "panels": panel_layout,
                "mount_factor": average_mount_factor(panel_layout),
            }
        )
        if wing_data["motor"]:
            prop_name = module_entry["props"]
            prop_diam_m = parse_prop_diameter_m(prop_name)
            x_motor = module_x_map[index] - wing_data["chord_root"] / 2.0 + wing_data["motor_x_pos"]
            for panel in panel_layout:
                motor_draw_rows.append(
                    {
                        "kind": "surface",
                        "x_m": x_motor,
                        "y_m": panel["root_y"] + wing_data["motor_y_pos"] * panel["dir_y"],
                        "z_m": panel["root_z"] + wing_data["motor_y_pos"] * panel["dir_z"],
                        "motor_bell_diam_m": motor_bell_diam_m,
                        "motor_bell_len_m": motor_bell_len_m,
                        "pod_diam_m": wing_data["pod_diam"],
                        "pod_len_flat_m": wing_data["pod_len_flat"],
                        "pod_len_curve_m": wing_data["pod_len_curve"],
                        "pod_shape": wing_data["pod_shape"],
                        "prop_diam_m": prop_diam_m,
                        "direction": wing_data["motor_dir"],
                    }
                )

    if tot_wingspan is None:
        tot_wingspan = tube_diam

    s_main_planform = float(wing_n * wing_span * (wing_chord_root + wing_chord_tip) / 2.0) if wing_n else 0.0
    s_tail_planform = float(tail_n * tail_span * (tail_chord_root + tail_chord_tip) / 2.0) if tail_n else 0.0
    wing_loading = float(weight_n / s_main_planform) if s_main_planform > 0.0 else float("nan")
    wing_to_tail_ratio = float(s_main_planform / s_tail_planform) if s_tail_planform > 0.0 else float("nan")
    total_static_thrust_n = sum(row["thrust_n_total"] for row in motor_summary_rows)
    static_thrust_to_weight = total_static_thrust_n / weight_n if total_motor_count > 0 else float("nan")
    return {
        "assembly_name": assembly_name,
        "config_name": config.get("name", assembly_name),
        "canards": canards,
        "assembly": assembly,
        "module_rows": module_rows,
        "surface_sets": surface_sets,
        "fuselage_length": fuselage_length,
        "body_x_start": body_x_start,
        "body_x_end": body_x_end,
        "tube_diam": tube_diam,
        "total_mass_kg": total_mass_kg,
        "total_moment_kg_m": total_moment_kg_m,
        "weight_N": weight_n,
        "cg_x_m": cg_x_m,
        "wing_aoa": wing_aoa,
        "tail_aoa": tail_aoa,
        "wing_airfoil": wing_airfoil,
        "tail_airfoil": tail_airfoil,
        "le_pos_x": le_pos_x,
        "s_main_planform": s_main_planform,
        "s_tail_planform": s_tail_planform,
        "wing_loading": wing_loading,
        "wing_to_tail_ratio": wing_to_tail_ratio,
        "battery_summary": battery_summary,
        "total_motor_count": total_motor_count,
        "tot_wingspan": tot_wingspan,
        "nose_length": nose_length,
        "motor_draw_rows": motor_draw_rows,
        "motor_summary_rows": motor_summary_rows,
        "total_static_thrust_n": total_static_thrust_n,
        "static_thrust_to_weight": static_thrust_to_weight,
        "wing_chord_root": wing_chord_root,
        "wing_chord_tip": wing_chord_tip,
        "tail_chord_root": tail_chord_root,
        "tail_chord_tip": tail_chord_tip,
        "wing_span": wing_span,
        "tail_span": tail_span,
        "wing_n": wing_n,
        "tail_n": tail_n,
    }


def apply_barrowman_analysis(analysis: dict) -> dict:
    component_rows = []
    nose_cna = 2.0
    nose_cp_x = 0.466 * analysis["nose_length"]
    component_rows.append(
        {
            "component": "nose_tangent_ogive",
            "role": "body",
            "cna": nose_cna,
            "x_cp": nose_cp_x,
            "notes": f"L={analysis['nose_length']:.3f} m",
        }
    )
    for surface in analysis["surface_sets"]:
        cna, x_cp, mid_chord_line = barrowman_finset(
            fin_count=surface["count"],
            span=surface["span"],
            chord_root=surface["chord_root"],
            chord_tip=surface["chord_tip"],
            sweep_len=surface["sweep_len"],
            body_radius=surface["body_radius"],
            ref_diameter=analysis["tube_diam"],
            x_root_le=surface["x_root_le"],
        )
        component_rows.append(
            {
                "component": surface["name"],
                "role": surface["role"],
                "cna": cna,
                "x_cp": x_cp,
                "notes": (
                    f"N={surface['count']} span={surface['span']:.3f} m "
                    f"area={surface['planform_area_total']:.4f} m^2 MCL={mid_chord_line:.3f} m"
                ),
            }
        )

    total_cna = sum(row["cna"] for row in component_rows)
    x_cp_total = sum(row["cna"] * row["x_cp"] for row in component_rows) / total_cna
    main_mac = trapezoid_mac(analysis["wing_chord_root"], analysis["wing_chord_tip"]) if (analysis["wing_chord_root"] + analysis["wing_chord_tip"]) > 0.0 else float("nan")
    static_margin_m = x_cp_total - analysis["cg_x_m"]
    static_margin_pct_mac = static_margin_m / main_mac * 100.0 if math.isfinite(main_mac) and main_mac > 0.0 else float("nan")
    if static_margin_pct_mac <= 0.0:
        stability_verdict = "unstable"
    elif static_margin_pct_mac < 5.0:
        stability_verdict = "marginal"
    else:
        stability_verdict = "stable"

    analysis["component_rows"] = component_rows
    analysis["nose_cna"] = nose_cna
    analysis["nose_cp_x"] = nose_cp_x
    analysis["total_cna"] = total_cna
    analysis["x_cp_total"] = x_cp_total
    analysis["main_mac"] = main_mac
    analysis["static_margin_m"] = static_margin_m
    analysis["static_margin_pct_mac"] = static_margin_pct_mac
    analysis["stability_verdict"] = stability_verdict
    return analysis


def apply_linear_aero_analysis(analysis: dict, aoa_eval_deg: float) -> dict:
    s_ref = analysis["s_main_planform"] if analysis["s_main_planform"] > 0.0 else math.pi * (analysis["tube_diam"] / 2.0) ** 2
    c_ref = analysis["main_mac"] if math.isfinite(analysis["main_mac"]) and analysis["main_mac"] > 0.0 else analysis["fuselage_length"]
    surface_aero = []
    for surface in analysis["surface_sets"]:
        slope = finite_wing_lift_slope(surface["span"], surface["chord_root"], surface["chord_tip"])
        mount_factor = surface["mount_factor"]
        area_ratio = surface["planform_area_total"] / s_ref
        x_ac = trapezoid_ac_x(
            x_root_le=surface["x_root_le"],
            span=surface["span"],
            chord_root=surface["chord_root"],
            chord_tip=surface["chord_tip"],
            sweep_len=surface["sweep_len"],
        )
        surface_aero.append(
            {
                "name": surface["name"],
                "role": surface["role"],
                "x_ac": x_ac,
                "incidence_deg": surface["aoa"],
                "mount_factor": mount_factor,
                "slope": slope,
                "area_ratio": area_ratio,
            }
        )

    body_ref_area = math.pi * (analysis["tube_diam"] / 2.0) ** 2
    body_area_ratio = body_ref_area / s_ref
    body_slope = analysis["nose_cna"] * body_area_ratio
    aoa_rad = math.radians(aoa_eval_deg)
    cn_total = body_slope * aoa_rad
    cm_total = -body_slope * (analysis["nose_cp_x"] - analysis["cg_x_m"]) / c_ref * aoa_rad
    for surface in surface_aero:
        alpha_eff_rad = math.radians(aoa_eval_deg + surface["incidence_deg"])
        cn_i = surface["slope"] * alpha_eff_rad * surface["mount_factor"] * surface["area_ratio"]
        cm_i = -cn_i * (surface["x_ac"] - analysis["cg_x_m"]) / c_ref
        cn_total += cn_i
        cm_total += cm_i

    if abs(cn_total) > 1e-9:
        lift_center_x = analysis["cg_x_m"] - (cm_total / cn_total) * c_ref
    else:
        lift_center_x = float("nan")

    if cm_total > 0.0:
        aoa_verdict = "nose-up"
    elif cm_total < 0.0:
        aoa_verdict = "nose-down"
    else:
        aoa_verdict = "trim"

    analysis["aoa_eval_deg"] = aoa_eval_deg
    analysis["aoa_cn"] = cn_total
    analysis["aoa_cm"] = cm_total
    analysis["aoa_verdict"] = aoa_verdict
    analysis["lift_center_x"] = lift_center_x
    return analysis


def analyze_config(
    assembly_name: str,
    config: dict,
    modules_data: dict,
    parts_data: dict,
    standards_data: dict,
    motor_data_df: pd.DataFrame,
    aoa_eval_deg: float,
) -> dict:
    analysis = build_full_assembly_json(
        assembly_name=assembly_name,
        config=config,
        modules_data=modules_data,
        parts_data=parts_data,
        standards_data=standards_data,
        motor_data_df=motor_data_df,
    )
    analysis = apply_barrowman_analysis(analysis)
    analysis = apply_linear_aero_analysis(analysis, aoa_eval_deg)
    analysis["report_text"] = build_report_text(analysis)
    return analysis


def recompute_analysis(
    assembly_name: str,
    config: dict,
    modules_data: dict,
    parts_data: dict,
    standards_data: dict,
    motor_data_df: pd.DataFrame,
    aoa_eval_deg: float,
) -> dict:
    return analyze_config(
        assembly_name=assembly_name,
        config=config,
        modules_data=modules_data,
        parts_data=parts_data,
        standards_data=standards_data,
        motor_data_df=motor_data_df,
        aoa_eval_deg=aoa_eval_deg,
    )
