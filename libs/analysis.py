"""
Usage:
from libs.analysis import load_builder_data, recompute_analysis, run_vlm_sweep
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
from libs.pointed_shapes import pointed_shape_param_values, sample_pointed_profile
from libs.simlib2 import TestedMotorData

SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR.parent / "data"
MOTORDATA_PATH = SCRIPT_DIR / "motordata.csv"
BATTERYDATA_PATH = SCRIPT_DIR / "batterydata.csv"
MODULES_PATH = DATA_DIR / "modules.json"
PARTS_PATH = DATA_DIR / "parts.json"
DEFAULT_CONFIG_PATH = DATA_DIR / "default.json"
STANDARDS_PATH = DATA_DIR / "specs.json"
PROP_SHAFT_LEN_M = 0.015
STANDARD_GRAVITY_M_S2 = 9.80665
GRAMS_FORCE_PER_NEWTON = 1000.0 / STANDARD_GRAVITY_M_S2
SURFACE_FORCE_N_CRIT = 9.0
SURFACE_FORCE_XTR_UPPER = 1.0
SURFACE_FORCE_XTR_LOWER = 1.0
SURFACE_FORCE_MODEL_SIZE = "large"
SURFACE_FORCE_INCLUDE_360_DEG_EFFECTS = False


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


def layout_wing_count(layout: str) -> int:
    return {"1": 1, "H": 2, "V": 2, "3": 3, "+": 4, "X": 4}.get(layout, 2)


def derive_wings_layout(module_data: dict) -> str:
    """Backward-compat: infer layout from old wings_n / wing_phase fields."""
    n = module_data.get("wings_n", 2)
    phase = module_data.get("wing_phase", 0.0) % 360
    if n == 1:
        return "1"
    if n == 2:
        if 80 <= phase <= 100 or 260 <= phase <= 280:
            return "V"
        return "H"
    if n == 3:
        return "3"
    if n >= 4:
        if 35 <= phase % 90 <= 55:
            return "X"
        return "+"
    return "H"


def surface_panel_layout(
    layout: str,
    root_radius: float,
    span: float,
    dihedral_deg: float,
    y_offset: float = 0.0,
    fin_pos: str = "top",
    centerline_root: bool = False,
) -> list[dict]:
    """
    Layout defines the wing configuration:
      H  — 2 horizontal wings, roots positioned by y_offset (-1=bottom, 0=equator, 1=top)
      V  — 2 vertical fins, fixed at tube top and bottom
      1  — single fin, position set by fin_pos ("top" or "bottom")
      3  — H pair + single fin at fin_pos
      +  — H pair + top and bottom fins
      X  — biplane: upper H pair at +y_offset, lower H pair at -y_offset

    Wing direction is always horizontal (±Y) for H-type panels.
    Fin direction is always vertical (±Z) for fin-type panels.
    Dihedral tilts H-type wings from horizontal; fins are unaffected.
    """
    d = math.radians(dihedral_deg)
    cos_d = math.cos(d)
    sin_d = math.sin(d)
    oz = max(-root_radius, min(root_radius, y_offset * root_radius))
    oy = math.sqrt(max(0.0, root_radius ** 2 - oz ** 2))
    fin_z = root_radius if fin_pos == "top" else -root_radius
    fin_dz = 1.0 if fin_pos == "top" else -1.0

    # (root_y, root_z, dir_y, dir_z) — fully explicit, no angles
    if layout == "1":
        specs = [(0.0, fin_z, 0.0, fin_dz)]
    elif layout == "V":
        specs = [(0.0, +root_radius, 0.0, +1.0), (0.0, -root_radius, 0.0, -1.0)]
    elif layout == "3":
        specs = [(+oy, oz, +cos_d, +sin_d), (-oy, oz, -cos_d, +sin_d), (0.0, fin_z, 0.0, fin_dz)]
    elif layout == "+":
        specs = [(+oy, oz, +cos_d, +sin_d), (-oy, oz, -cos_d, +sin_d),
                 (0.0, +root_radius, 0.0, +1.0), (0.0, -root_radius, 0.0, -1.0)]
    elif layout == "X":
        specs = [(+oy, +oz, +cos_d, +sin_d), (-oy, +oz, -cos_d, +sin_d),
                 (+oy, -oz, +cos_d, -sin_d), (-oy, -oz, -cos_d, -sin_d)]
    else:  # "H" and fallback
        specs = [(+oy, oz, +cos_d, +sin_d), (-oy, oz, -cos_d, +sin_d)]

    panels = []
    for index, (ry, rz, dy, dz) in enumerate(specs):
        norm = math.hypot(dy, dz)
        if norm > 1e-9:
            dy /= norm
            dz /= norm
        if centerline_root:
            ry, rz = 0.0, 0.0
        ty = dy * span if centerline_root else ry + span * dy
        tz = dz * span if centerline_root else rz + span * dz
        panels.append({"index": index, "phase_deg": 0.0, "dihedral_sign": 1.0,
                        "root_y": ry, "root_z": rz, "tip_y": ty, "tip_z": tz,
                        "dir_y": dy, "dir_z": dz})
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
    xsecs = [root, tip]
    if y_tip < y_le or (abs(y_tip - y_le) < 1e-9 and z_tip < z_le):
        xsecs = [tip, root]
    return asb.Wing(
        name=f"{name_prefix}_{idx + 1}",
        symmetric=False,
        xsecs=xsecs,
    )


def build_asb_airplane(analysis: dict) -> asb.Airplane:
    n_cap = 12
    eps = 1e-6
    structural_rows = [row for row in analysis["module_rows"] if row["module_type"] in {"body", "end", "coupler", "wingmount"}]
    tube_radius = analysis["tube_diam"] / 2.0

    xsecs = []

    def add_xsec(x_value: float, radius_value: float) -> None:
        if xsecs and abs(x_value - xsecs[-1].xyz_c[0]) < eps:
            return
        xsecs.append(asb.FuselageXSec(xyz_c=[x_value, 0.0, 0.0], radius=radius_value))

    end_count = 0
    for row in structural_rows:
        module_len = row["x_end"] - row["x_start"]
        if row["module_type"] == "end" and end_count == 0:
            for x_local, radius_value in sample_pointed_profile(
                row["pointed_shape"]["shape"],
                module_len,
                tube_radius,
                n_cap + 2,
                pointed_shape_param_values(row["pointed_shape"]),
            ):
                add_xsec(row["x_start"] + x_local, radius_value)
            end_count += 1
            continue
        if row["module_type"] == "end" and end_count == 1:
            tail_profile = sample_pointed_profile(
                row["pointed_shape"]["shape"],
                module_len,
                tube_radius,
                n_cap + 2,
                pointed_shape_param_values(row["pointed_shape"]),
            )
            for x_local, radius_value in reversed(tail_profile):
                add_xsec(row["x_end"] - x_local, radius_value)
            end_count += 1
            continue
        add_xsec(row["x_start"], tube_radius)
        add_xsec(row["x_end"], tube_radius)

    fuselages = [asb.Fuselage(name="Fuselage", xsecs=xsecs)]

    wings = []
    wing_index = 0
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
            wing.name = f"{surface['name']}_{wing_index + 1}"
            wings.append(wing)
            wing_index += 1

    c_ref = analysis["reference_chord_m"] if math.isfinite(analysis["reference_chord_m"]) and analysis["reference_chord_m"] > 0.0 else analysis["tube_diam"]
    b_ref = analysis["tot_wingspan"]
    propulsors = []
    for index, motor in enumerate(analysis["motor_draw_rows"]):
        front_sign = -1.0 if motor["direction"] == "front" else 1.0
        pod_sign = -front_sign
        motor_radius = motor["motor_bell_diam_m"] / 2.0
        pod_radius = motor["pod_diam_m"] / 2.0
        motor_x0 = motor["x_m"] - motor["motor_bell_len_m"] / 2.0
        motor_x1 = motor["x_m"] + motor["motor_bell_len_m"] / 2.0
        fuselages.append(
            asb.Fuselage(
                name=f"motor_{index + 1}",
                xsecs=[
                    asb.FuselageXSec(xyz_c=[motor_x0, motor["y_m"], motor["z_m"]], radius=motor_radius),
                    asb.FuselageXSec(xyz_c=[motor_x1, motor["y_m"], motor["z_m"]], radius=motor_radius),
                ],
            )
        )
        if motor["pod_len_flat_m"] > 0.0 or motor["pod_len_curve_m"] > 0.0:
            pod_xsecs = []
            pod_x0 = motor["x_m"] + pod_sign * (motor["motor_bell_len_m"] / 2.0)
            pod_xsecs.append(asb.FuselageXSec(xyz_c=[pod_x0, motor["y_m"], motor["z_m"]], radius=pod_radius))
            if motor["pod_len_flat_m"] > 0.0:
                pod_x1 = pod_x0 + pod_sign * motor["pod_len_flat_m"]
                pod_xsecs.append(asb.FuselageXSec(xyz_c=[pod_x1, motor["y_m"], motor["z_m"]], radius=pod_radius))
            else:
                pod_x1 = pod_x0
            if motor["pod_len_curve_m"] > 0.0:
                curve_profile = sample_pointed_profile(motor["pod_shape"], motor["pod_len_curve_m"], pod_radius, n_cap + 2)
                for curve_x, curve_radius in reversed(curve_profile[:-1]):
                    pod_xsecs.append(
                        asb.FuselageXSec(
                            xyz_c=[pod_x1 + pod_sign * (motor["pod_len_curve_m"] - curve_x), motor["y_m"], motor["z_m"]],
                            radius=curve_radius,
                        )
                    )
            fuselages.append(asb.Fuselage(name=f"pod_{index + 1}", xsecs=pod_xsecs))
        propulsors.append(
            asb.Propulsor(
                name=f"propulsor_{index + 1}",
                xyz_c=[
                    motor["x_m"] + front_sign * (motor["motor_bell_len_m"] / 2.0 + PROP_SHAFT_LEN_M),
                    motor["y_m"],
                    motor["z_m"],
                ],
                xyz_normal=[front_sign, 0.0, 0.0],
                radius=motor["prop_diam_m"] / 2.0,
                length=0.0,
            )
        )
    return asb.Airplane(
        name=analysis["config_name"],
        xyz_ref=[analysis["cg_x_m"], 0.0, 0.0],
        s_ref=analysis["surface_planform_total"],
        c_ref=c_ref,
        b_ref=b_ref,
        wings=wings,
        fuselages=fuselages,
        propulsors=propulsors,
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


def part_has_lifting_surfaces(module_data: dict, module_entry: dict) -> bool:
    if module_data["type"] == "wingmount":
        return True
    return module_data["type"] == "end" and module_data.get("fins", False)


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
    group = group.copy()
    group["ThrottlePct"] = group["T"].astype(str).str.rstrip("%").astype(float)
    return group.sort_values("ThrottlePct")


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
        if len(digits) == 4:
            first_two = int(digits[:2])
            if first_two <= 20:
                diameter_in = float(first_two)
            else:
                diameter_in = float(digits[0])
        else:
            diameter_in = float(digits[:2])
        return diameter_in * 0.0254
    raise RuntimeError(f"Could not parse prop diameter from {prop_name}")


def build_report_text(analysis: dict) -> str:
    lines = []
    lines.append(f"Airframe: {analysis['assembly_name']}")
    lines.append("")
    lines.append("Overall Summary")
    lines.append(f"Fuselage length       : {analysis['fuselage_length']:.4f} m")
    lines.append(f"Overall span          : {analysis['tot_wingspan']:.4f} m")
    lines.append(f"Total mass            : {analysis['total_mass_kg'] * 1000.0:.1f} g")
    lines.append(f"Total weight          : {analysis['weight_N']:.4f} N")
    lines.append(f"Total moment          : {analysis['total_moment_kg_m']:.4f} kg*m")
    lines.append(f"CG                    : {analysis['cg_x_m']:.4f} m")
    lines.append("")
    lines.append("Lifting Surfaces")
    lines.append(f"Surface area total    : {analysis['surface_planform_total']:.4f} m^2")
    lines.append(f"Surface loading       : {analysis['surface_loading']:.2f} N/m^2")
    if analysis["surface_force_rows"]:
        lines.append("")
        lines.append("Surface Forces")
        lines.append(
            f"Operating point       : AoA {analysis['force_eval_aoa_deg']:.1f} deg  V {analysis['force_eval_airspeed_m_s']:.1f} m/s  Alt {analysis['force_eval_altitude_m']:.1f} m"
        )
        lines.append(f"Air density           : {analysis['force_eval_density_kg_m3']:.4f} kg/m^3")
        lines.append(f"Dynamic pressure      : {analysis['force_eval_dynamic_pressure_pa']:.2f} Pa")
        lines.append(
            f"Surface vertical total: {analysis['surface_vertical_force_n']:.3f} N  {analysis['surface_vertical_force_g']:.1f} g  {analysis['surface_vertical_load_pct']:.1f} % weight"
        )
        for row in analysis["surface_force_rows"]:
            lines.append(
                f"{row['name']:<18} airfoil={row['airfoil']:<10} alpha={row['alpha_eff_deg']:.2f} deg "
                f"area={row['area_m2']:.4f} m^2 CL={row['cl']:.3f} CD={row['cd']:.4f} "
                f"vertical={row['vertical_force_n']:.3f} N {row['vertical_force_g']:.1f} g "
                f"lift={row['lift_n']:.3f} N drag={row['drag_n']:.3f} N Re={row['reynolds']:.0f} conf={row['confidence']:.3f}"
            )
    if analysis["battery_summary"] is not None:
        battery_summary = analysis["battery_summary"]
        lines.append("")
        lines.append("Battery")
        lines.append(f"Selected              : {battery_summary['name']}")
        lines.append(f"Type                  : {battery_summary['s']}S{battery_summary['p']}P")
        lines.append(f"Capacity              : {battery_summary['mah']:.0f} mAh {battery_summary['wh']:.2f} Wh")
        lines.append(f"Current               : {battery_summary['curr']:.2f} A")
    if analysis["total_motor_count"] > 0:
        lines.append("")
        lines.append("Propulsion")
        lines.append(f"Motor count           : {analysis['total_motor_count']}")
        lines.append(f"Max thrust total      : {analysis['total_static_thrust_n']:.2f} N")
        lines.append(f"Max current total     : {analysis['motor_summary_rows'][0]['current_a_total']:.2f} A")
        lines.append(f"Max input power total : {analysis['motor_summary_rows'][0]['input_power_w_total'] / 1000.0:.2f} kW")
        lines.append(f"Total thrust/weight   : {analysis['static_thrust_to_weight']:.3f}")
        lines.append(f"Hover throttle        : {analysis['hover_throttle_pct']:.1f} %" if math.isfinite(analysis["hover_throttle_pct"]) else "Hover throttle        : n/a")
        lines.append(f"Hover time            : {analysis['hover_time_min']:.1f} min" if math.isfinite(analysis["hover_time_min"]) else "Hover time            : n/a")
        lines.append(f"Full throttle time    : {analysis['full_throttle_time_min']:.1f} min" if math.isfinite(analysis["full_throttle_time_min"]) else "Full throttle time    : n/a")
    lines.append("")
    lines.append("Static Stability")
    lines.append(f"Static CP (Barrowman) : {analysis['x_cp_total']:.4f} m")
    lines.append(f"Static offset         : {analysis['static_margin_m']:.4f} m")
    lines.append(f"Static stability      : {analysis['stability_verdict']}")
    lines.append("")
    lines.append("Markers")
    lines.append(f"CG marker             : blue @ {analysis['cg_x_m']:.4f} m")
    lines.append(f"Static CP marker      : red @ {analysis['x_cp_total']:.4f} m")
    if math.isfinite(analysis["force_center_x"]):
        lines.append(f"Lift marker           : green @ {analysis['force_center_x']:.4f} m @ AoA {analysis['force_eval_aoa_deg']:.2f} deg")
    lines.append("")
    lines.append("AoA Sweep")
    header = f"{'AoA':<5} | {'CN':<9} | {'Cm':<9} | {'Lift x m':<10} | {'Offset m':<8} | {'Verdict':<9}"
    lines.append(header)
    lines.append("-" * len(header))
    for row in analysis["aoa_sweep_rows"]:
        cl_x_text = f"{row['lift_center_x']:.4f}" if math.isfinite(row["lift_center_x"]) else "nan"
        sm_text = f"{row['lift_static_offset_m']:.4f}" if math.isfinite(row["lift_static_offset_m"]) else "nan"
        lines.append(
            f"{row['aoa_eval_deg']:<5.0f} | {row['aoa_cn']:<9.4f} | {row['aoa_cm']:<9.4f} | {cl_x_text:<10} | {sm_text:<8} | {row['aoa_verdict']:<9}"
        )
    lines.append("")
    lines.append("Components")
    for row in analysis["component_rows"]:
        lines.append(
            f"{row['component']:<24} CNa={row['cna']:.3f} x_cp={row['x_cp']:.4f} {row['notes']}"
        )
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


def vlm_point_key(altitude_m: float, aoa_deg: float, airspeed_m_s: float) -> tuple[float, float, float]:
    return (round(float(altitude_m), 6), round(float(aoa_deg), 6), round(float(airspeed_m_s), 6))


def run_vlm_sweep(
    analysis: dict,
    settings: dict,
    progress_callback=None,
    include_vlm_objects: bool = False,
) -> dict:
    airplane = build_asb_airplane(analysis)
    aoa_values = build_range_values(settings["aoa_start"], settings["aoa_end"], settings["aoa_increment"])
    airspeed_values = build_range_values(settings["airspeed_start"], settings["airspeed_end"], settings["airspeed_increment"])
    altitude_values = build_range_values(settings["altitude_start"], settings["altitude_end"], settings["altitude_increment"])
    c_ref = airplane.c_ref
    total_points = len(aoa_values) * len(airspeed_values) * len(altitude_values)
    completed_points = 0
    rows = []
    vlm_cache = {}

    def solve_point(point: tuple[float, float, float]) -> tuple[dict, asb.VortexLatticeMethod]:
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
        }, vlm

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
                    for row, vlm in executor.map(solve_point, points):
                        rows.append(row)
                        if include_vlm_objects:
                            vlm_cache[vlm_point_key(row["altitude_m"], row["aoa_deg"], row["airspeed_m_s"])] = vlm
                        update_progress(row)
            else:
                for point in points:
                    row, vlm = solve_point(point)
                    rows.append(row)
                    if include_vlm_objects:
                        vlm_cache[vlm_point_key(row["altitude_m"], row["aoa_deg"], row["airspeed_m_s"])] = vlm
                    update_progress(row)

    rows.sort(key=lambda row: (row["altitude_m"], row["aoa_deg"], row["airspeed_m_s"]))
    vlm_results = {
        "airframe_name": analysis["config_name"],
        "settings": copy.deepcopy(settings),
        "rows": rows,
        "aoa_values": aoa_values,
        "airspeed_values": airspeed_values,
        "altitude_values": altitude_values,
    }
    if include_vlm_objects:
        vlm_results["vlm_cache"] = vlm_cache
    vlm_results["report_text"] = build_vlm_report_text(vlm_results)
    return vlm_results


def _static_motor_summary(
    motor_name: str,
    brand: str,
    model: str,
    kv_val: str,
    prop_name: str,
    count: int,
    batt_s: int,
) -> dict:
    motor = TestedMotorData(brand=brand, motor=model, kv=int(kv_val), batt_s=batt_s, csv_path=MOTORDATA_PATH)
    zero_op = motor.from_throttle(prop_name, 0.0)
    static_op = motor.from_throttle(prop_name, 100.0)
    return {
        "label": f"{motor_name} x{count}",
        "count": count,
        "propeller": prop_name,
        "kv": float(kv_val),
        "pack_voltage_v": zero_op.voltage,
        "no_load_rpm": zero_op.voltage * float(kv_val),
        "prop_rpm": static_op.rpm,
        "thrust_n_per_motor": static_op.thrust_n,
        "thrust_n_total": static_op.thrust_n * count,
        "current_a_per_motor": static_op.current_a,
        "current_a_total": static_op.current_a * count,
        "input_power_w_per_motor": static_op.power_w,
        "input_power_w_total": static_op.power_w * count,
        "pitch_speed_kph": static_op.pitch_speed_mps * 3.6,
        "efficiency_g_per_w": static_op.efficiency_gw,
        "warnings": [],
    }


def resolve_battery_summary(parts_data: dict, battery_name: str) -> dict:
    battery_part_data = parts_data["battery"][battery_name]
    if battery_part_data["type"] == "liion":
        battery_data_df = pd.read_csv(BATTERYDATA_PATH)
        battery_data = battery_data_df[
            (battery_data_df["Brand"].astype(str) == str(battery_part_data["cell"][0]))
            & (battery_data_df["Name"].astype(str) == str(battery_part_data["cell"][1]))
            & (battery_data_df["Format"].astype(str) == str(battery_part_data["cell"][2]))
        ].iloc[0]
        return {
            "name": battery_name,
            "type": battery_part_data["type"],
            "mass": float(battery_data["Weight"]) / 1000.0 * battery_part_data["s"] * battery_part_data["p"],
            "s": battery_part_data["s"],
            "p": battery_part_data["p"],
            "mah": float(battery_data["CapacityA"]) * 1000.0 * battery_part_data["p"],
            "curr": float(battery_data["C-rating"]) * float(battery_data["CapacityA"]) * battery_part_data["p"],
            "wh": float(battery_data["CapacityWh"]) * battery_part_data["s"] * battery_part_data["p"],
            "dimensions": None,
        }
    batt_mah = battery_part_data["mah"]
    return {
        "name": battery_name,
        "type": battery_part_data["type"],
        "mass": battery_part_data["mass"],
        "s": battery_part_data["s"],
        "p": battery_part_data["p"],
        "mah": batt_mah,
        "curr": (batt_mah / 1000.0) * battery_part_data["c"],
        "wh": (batt_mah / 1000.0) * (battery_part_data["s"] * 3.7),
        "dimensions": {
            "l": battery_part_data["dimensions"]["l"] / 1000.0,
            "h": battery_part_data["dimensions"]["h"] / 1000.0,
            "w": battery_part_data["dimensions"]["w"] / 1000.0,
        },
    }


def build_propulsion_summary(
    motor_name: str,
    prop_name: str,
    battery_summary: dict,
    total_motor_count: int,
    weight_n: float,
) -> dict:
    _motor_name, brand, model, kv = parse_motor_name(motor_name)
    summary = _static_motor_summary(
        motor_name,
        brand,
        model,
        kv,
        prop_name,
        total_motor_count,
        battery_summary["s"],
    )
    motor = TestedMotorData(brand=brand, motor=model, kv=int(kv), batt_s=int(battery_summary["s"]), csv_path=MOTORDATA_PATH)
    curve_ops = [motor.from_throttle(prop_name, float(throttle_pct)) for throttle_pct in range(0, 101)]
    throttle_pct = np.array([op.throttle_pct for op in curve_ops])
    thrust_g = np.array([op.thrust_g for op in curve_ops])
    thrust_n = np.array([op.thrust_n for op in curve_ops])
    current_a = np.array([op.current_a for op in curve_ops])
    rpm = np.array([op.rpm for op in curve_ops])
    watts = np.array([op.power_w for op in curve_ops])
    efficiency = np.array([op.efficiency_gw for op in curve_ops])
    summary["curve"] = {
        "throttle_pct": throttle_pct.tolist(),
        "thrust_n": thrust_n.tolist(),
        "rpm": rpm.tolist(),
        "watts": watts.tolist(),
        "efficiency_g_per_w": efficiency.tolist(),
    }
    if total_motor_count > 0 and weight_n <= summary["thrust_n_total"]:
        hover_thrust_g_per_motor = (weight_n / total_motor_count) / 9.81 * 1000.0
        hover_op = motor.from_thrust(prop_name, hover_thrust_g_per_motor)
        summary["hover_throttle_pct"] = hover_op.throttle_pct
        summary["hover_current_a_total"] = hover_op.current_a * total_motor_count
        summary["hover_power_w_total"] = hover_op.power_w * total_motor_count
        summary["hover_time_min"] = battery_summary["wh"] / summary["hover_power_w_total"] * 60.0
    else:
        summary["hover_throttle_pct"] = float("nan")
        summary["hover_current_a_total"] = float("nan")
        summary["hover_power_w_total"] = float("nan")
        summary["hover_time_min"] = float("nan")
    summary["full_throttle_time_min"] = battery_summary["wh"] / summary["input_power_w_total"] * 60.0
    return summary


def linear_aero_state(analysis: dict, aoa_eval_deg: float) -> dict:
    s_ref = analysis["surface_planform_total"] if analysis["surface_planform_total"] > 0.0 else math.pi * (analysis["tube_diam"] / 2.0) ** 2
    c_ref = analysis["reference_chord_m"] if math.isfinite(analysis["reference_chord_m"]) and analysis["reference_chord_m"] > 0.0 else analysis["fuselage_length"]
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
    if math.isfinite(lift_center_x):
        static_offset_m = lift_center_x - analysis["cg_x_m"]
    else:
        static_offset_m = float("nan")
    return {
        "aoa_eval_deg": aoa_eval_deg,
        "aoa_cn": cn_total,
        "aoa_cm": cm_total,
        "aoa_verdict": aoa_verdict,
        "lift_center_x": lift_center_x,
        "lift_static_offset_m": static_offset_m,
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
    battery_name = config["battery"]
    motor_name = config["motors"]
    prop_name = config["props"]
    assembly = []
    for part in config["parts"]:
        assembly.append(part)
        assembly.append(
            {
                "module": "joint-ring-c",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "roll": 0.0,
                "payload_mass": 0.0,
                "ballast_mass": 0.0,
            }
        )
    assembly.pop()

    surface_sets = []
    module_rows = []
    fuselage_length = 0.0
    total_mass_kg = 0.0
    total_moment_kg_m = 0.0
    current_x_fuselage_m = 0.0
    battery_summary = None
    surface_airfoils = []
    total_motor_count = 0
    nose_length = 0.0
    nose_shape = ""
    motor_draw_rows = []
    internal_draw_rows = []
    motor_summary_rows = []
    if battery_name:
        battery_summary = resolve_battery_summary(parts_data, battery_name)
        batt_s = battery_summary["s"]
    else:
        batt_s = None

    x = 0.0
    module_x_mount_map = []
    for module_entry in assembly:
        _, module_data = resolve_named_block(module_entry["module"], modules_data)
        length = module_data["len"]
        module_x_mount_map.append(x + length * module_data.get("wings_x", 0.5))
        x += length

    for assembly_index, item_in_assembly in enumerate(assembly):
        module_name, module_data = resolve_named_block(item_in_assembly["module"], modules_data)
        module_mass = module_data["mass"]
        module_len = module_data["len"]
        module_cg_offset_from_module_front_m = module_data["cg"] * module_len
        module_structure_cg_global_x_m = current_x_fuselage_m + module_cg_offset_from_module_front_m
        internal_part_x_m = item_in_assembly["x"]
        internal_part_cg_global_x_m = current_x_fuselage_m + module_len * 0.5 + internal_part_x_m
        internal_part_y_m = item_in_assembly["y"]
        internal_part_z_m = item_in_assembly["z"]
        module_rows.append(
            {
                "module_name": module_name,
                "module_type": module_data["type"],
                "x_start": current_x_fuselage_m,
                "x_end": current_x_fuselage_m + module_len,
                "mass": module_mass,
                "cg_x": module_structure_cg_global_x_m,
                "diam": tube_diam,
                "pointed_shape": copy.deepcopy(module_data["pointed_shape"]) if module_data["type"] == "end" else {},
            }
        )
        total_mass_kg += module_mass
        total_moment_kg_m += module_mass * module_structure_cg_global_x_m

        if module_data.get("battery", False):
            battery_mass = battery_summary["mass"]
            total_mass_kg += battery_mass
            total_moment_kg_m += battery_mass * internal_part_cg_global_x_m
            module_rows.append(
                {
                    "module_name": battery_name,
                    "module_type": "battery",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": battery_mass,
                    "cg_x": internal_part_cg_global_x_m,
                    "diam": tube_diam,
                }
            )
            if battery_summary["dimensions"] is not None:
                internal_draw_rows.append(
                    {
                        "kind": "battery",
                        "name": battery_name,
                        "x_m": internal_part_cg_global_x_m,
                        "y_m": internal_part_y_m,
                        "z_m": internal_part_z_m,
                        "len_m": battery_summary["dimensions"]["l"],
                        "height_m": battery_summary["dimensions"]["h"],
                        "width_m": battery_summary["dimensions"]["w"],
                    }
                )

        if module_data.get("payload", False):
            payload_mass = item_in_assembly["payload_mass"]
            total_mass_kg += payload_mass
            total_moment_kg_m += payload_mass * internal_part_cg_global_x_m
            module_rows.append(
                {
                    "module_name": "payload",
                    "module_type": "payload",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": payload_mass,
                    "cg_x": internal_part_cg_global_x_m,
                    "diam": tube_diam,
                }
            )

        if module_data.get("ballast", False):
            ballast_mass = item_in_assembly["ballast_mass"]
            total_mass_kg += ballast_mass
            total_moment_kg_m += ballast_mass * internal_part_cg_global_x_m
            module_rows.append(
                {
                    "module_name": "ballast",
                    "module_type": "ballast",
                    "x_start": current_x_fuselage_m,
                    "x_end": current_x_fuselage_m + module_len,
                    "mass": ballast_mass,
                    "cg_x": internal_part_cg_global_x_m,
                    "diam": tube_diam,
                }
            )

        if part_has_lifting_surfaces(module_data, item_in_assembly):
            wing_spec_name, wing_data = resolve_named_block(item_in_assembly["surfaces"], modules_data)
            num_wings = layout_wing_count(module_data.get("wings_layout") or derive_wings_layout(module_data))
            wing_mount_x_m = module_x_mount_map[assembly_index]
            wing_root_le_x_m = wing_mount_x_m - wing_data["chord_root"] / 2.0
            wing_sweep_len_m = wing_data["semispan"] * math.tan(math.radians(wing_data["sweep_ang"]))
            wing_mac_m = trapezoid_mac(wing_data["chord_root"], wing_data["chord_tip"])
            wing_taper = wing_data["chord_tip"] / wing_data["chord_root"] if wing_data["chord_root"] > 0.0 else 0.0
            wing_y_mac_m = wing_data["semispan"] * (1.0 + 2.0 * wing_taper) / (3.0 * (1.0 + wing_taper))
            wing_x_mac_le_m = wing_root_le_x_m + wing_sweep_len_m * wing_y_mac_m / wing_data["semispan"] if wing_data["semispan"] > 0.0 else wing_root_le_x_m
            wing_surface_cg_global_x_m = wing_x_mac_le_m + wing_data["cg"] * wing_mac_m
            total_wings_mass = num_wings * wing_data["mass"]
            total_mass_kg += total_wings_mass
            total_moment_kg_m += total_wings_mass * wing_surface_cg_global_x_m

            if wing_data.get("motor", False):
                total_motor_count += num_wings
                _motor_name, brand, model, kv = parse_motor_name(motor_name)
                motor_mass_each = motor_mass_kg(motor_name, brand, model, kv, prop_name, batt_s, motor_data_df)
                motor_bell_diam_m, motor_bell_len_m = motor_bell_geometry_m(motor_name, brand, model, kv, prop_name, batt_s, motor_data_df)
                wing_motor_cg_global_x_m = wing_root_le_x_m + wing_data["motor_x_pos"]
                total_wing_motors_mass = num_wings * motor_mass_each
                total_mass_kg += total_wing_motors_mass
                total_moment_kg_m += total_wing_motors_mass * wing_motor_cg_global_x_m
                module_rows.append(
                    {
                        "module_name": motor_name,
                        "module_type": "motor",
                        "x_start": current_x_fuselage_m,
                        "x_end": current_x_fuselage_m + module_len,
                        "mass": total_wing_motors_mass,
                        "cg_x": wing_motor_cg_global_x_m,
                        "diam": tube_diam,
                        "prop_name": prop_name,
                    }
                )

        if module_data["type"] == "end" and module_data.get("motor", False):
            total_motor_count += 1
            _motor_name, brand, model, kv = parse_motor_name(motor_name)
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

        if module_data["type"] == "end" and nose_length == 0.0:
            nose_length = module_len
            nose_shape = module_data["pointed_shape"]["shape"]

        fuselage_length += module_len
        current_x_fuselage_m += module_len

    cg_x_m = total_moment_kg_m / total_mass_kg
    weight_n = total_mass_kg * 9.81

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
    lifting_module_motor_flags = []
    for module_entry in assembly:
        _, module_data = resolve_named_block(module_entry["module"], modules_data)
        if not part_has_lifting_surfaces(module_data, module_entry):
            continue
        _, wing_data = resolve_named_block(module_entry["surfaces"], modules_data)
        lifting_module_motor_flags.append(wing_data["motor"])
    tot_wingspan = None
    overall_min_y = -tube_diam / 2.0
    overall_max_y = tube_diam / 2.0
    for index, module_entry in enumerate(assembly):
        _, module_data = resolve_named_block(module_entry["module"], modules_data)
        if not part_has_lifting_surfaces(module_data, module_entry):
            continue
        spec_name, wing_data = resolve_named_block(module_entry["surfaces"], modules_data)
        if wing_data["airfoil"] not in surface_airfoils:
            surface_airfoils.append(wing_data["airfoil"])
        centerline_root = module_data["type"] == "end"
        _layout = module_data.get("wings_layout") or derive_wings_layout(module_data)
        panel_layout = surface_panel_layout(
            layout=_layout,
            root_radius=tube_diam / 2.0,
            span=wing_data["semispan"],
            dihedral_deg=module_data["wings_ang"],
            y_offset=module_data.get("wings_y_offset", 0.0),
            fin_pos=module_data.get("wings_fin_pos", "top"),
            centerline_root=centerline_root,
        )
        for panel in panel_layout:
            overall_min_y = min(overall_min_y, panel["root_y"], panel["tip_y"])
            overall_max_y = max(overall_max_y, panel["root_y"], panel["tip_y"])
        sweep_len = wing_data["semispan"] * math.tan(math.radians(wing_data["sweep_ang"]))
        surface_sets.append(
            {
                "name": spec_name,
                "module": module_entry["module"] if type(module_entry["module"]) is str else module_entry["module"]["name"],
                "count": len(panel_layout),
                "span": wing_data["semispan"],
                "chord_root": wing_data["chord_root"],
                "chord_tip": wing_data["chord_tip"],
                "sweep_len": sweep_len,
                "x_root_le": module_x_mount_map[index] - wing_data["chord_root"] / 2.0,
                "body_radius": tube_diam / 2.0,
                "wing_phase": 0.0,
                "dihedral_deg": module_data["wings_ang"],
                "aoa": wing_data["aoa_ang"],
                "airfoil": wing_data["airfoil"],
                "planform_area_total": len(panel_layout) * wing_data["semispan"] * (wing_data["chord_root"] + wing_data["chord_tip"]) / 2.0,
                "panels": panel_layout,
                "mount_factor": average_mount_factor(panel_layout),
            }
        )
        if wing_data["motor"]:
            prop_diam_m = parse_prop_diameter_m(prop_name)
            x_motor = module_x_mount_map[index] - wing_data["chord_root"] / 2.0 + wing_data["motor_x_pos"]
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
        tot_wingspan = overall_max_y - overall_min_y

    surface_planform_total = sum(surface["planform_area_total"] for surface in surface_sets)
    reference_chord_area_sum = 0.0
    for surface in surface_sets:
        reference_chord_area_sum += surface["planform_area_total"] * trapezoid_mac(surface["chord_root"], surface["chord_tip"])
    reference_chord_m = reference_chord_area_sum / surface_planform_total if surface_planform_total > 0.0 else float("nan")
    surface_loading = float(weight_n / surface_planform_total) if surface_planform_total > 0.0 else float("nan")
    if total_motor_count > 0:
        motor_summary_rows = [
            build_propulsion_summary(
                motor_name,
                prop_name,
                battery_summary,
                total_motor_count,
                weight_n,
            )
        ]
    total_static_thrust_n = sum(row["thrust_n_total"] for row in motor_summary_rows)
    static_thrust_to_weight = total_static_thrust_n / weight_n if total_motor_count > 0 else float("nan")
    return {
        "assembly_name": assembly_name,
        "config_name": config.get("name", assembly_name),
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
        "surface_airfoils": surface_airfoils,
        "surface_planform_total": surface_planform_total,
        "surface_loading": surface_loading,
        "reference_chord_m": reference_chord_m,
        "battery_summary": battery_summary,
        "total_motor_count": total_motor_count,
        "tot_wingspan": tot_wingspan,
        "nose_length": nose_length,
        "nose_shape": nose_shape,
        "motor_draw_rows": motor_draw_rows,
        "internal_draw_rows": internal_draw_rows,
        "motor_summary_rows": motor_summary_rows,
        "total_static_thrust_n": total_static_thrust_n,
        "static_thrust_to_weight": static_thrust_to_weight,
        "hover_throttle_pct": motor_summary_rows[0]["hover_throttle_pct"] if total_motor_count > 0 else float("nan"),
        "hover_time_min": motor_summary_rows[0]["hover_time_min"] if total_motor_count > 0 else float("nan"),
        "full_throttle_time_min": motor_summary_rows[0]["full_throttle_time_min"] if total_motor_count > 0 else float("nan"),
    }


def apply_barrowman_analysis(analysis: dict) -> dict:
    component_rows = []
    nose_cna = 2.0
    nose_cp_x = 0.466 * analysis["nose_length"]
    component_rows.append(
        {
            "component": f"nose_{analysis['nose_shape']}",
            "cna": nose_cna,
            "x_cp": nose_cp_x,
            "notes": f"shape={analysis['nose_shape']} L={analysis['nose_length']:.3f} m",
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
    static_margin_m = x_cp_total - analysis["cg_x_m"]
    static_margin_pct_mac = (
        static_margin_m / analysis["reference_chord_m"] * 100.0
        if math.isfinite(analysis["reference_chord_m"]) and analysis["reference_chord_m"] > 0.0
        else float("nan")
    )
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
    analysis["static_margin_m"] = static_margin_m
    analysis["static_margin_pct_mac"] = static_margin_pct_mac
    analysis["stability_verdict"] = stability_verdict
    return analysis


def apply_linear_aero_analysis(
    analysis: dict,
    aoa_eval_deg: float,
    force_eval_airspeed_m_s: float,
    force_eval_altitude_m: float,
) -> dict:
    sweep_rows = []
    for sweep_aoa_deg in range(-45, 46):
        sweep_rows.append(linear_aero_state(analysis, float(sweep_aoa_deg)))
    analysis["aoa_sweep_rows"] = sweep_rows
    eval_row = linear_aero_state(analysis, aoa_eval_deg)
    analysis.update(eval_row)
    atmosphere = asb.Atmosphere(altitude=force_eval_altitude_m)
    density_kg_m3 = float(atmosphere.density())
    dynamic_viscosity_pa_s = float(atmosphere.dynamic_viscosity())
    speed_of_sound_m_s = float(atmosphere.speed_of_sound())
    dynamic_pressure_pa = 0.5 * density_kg_m3 * force_eval_airspeed_m_s * force_eval_airspeed_m_s
    mach = force_eval_airspeed_m_s / speed_of_sound_m_s
    body_aoa_rad = math.radians(aoa_eval_deg)
    flow_dir = np.array([math.cos(body_aoa_rad), 0.0, -math.sin(body_aoa_rad)], dtype=float)
    surface_force_rows = []
    surface_vertical_force_n = 0.0
    surface_vertical_moment_n_m = 0.0
    for surface in analysis["surface_sets"]:
        area_m2 = surface["planform_area_total"]
        mac_m = trapezoid_mac(surface["chord_root"], surface["chord_tip"])
        reynolds = density_kg_m3 * force_eval_airspeed_m_s * mac_m / dynamic_viscosity_pa_s
        alpha_eff_deg = aoa_eval_deg + surface["aoa"]
        x_force_m = trapezoid_ac_x(
            x_root_le=surface["x_root_le"],
            span=surface["span"],
            chord_root=surface["chord_root"],
            chord_tip=surface["chord_tip"],
            sweep_len=surface["sweep_len"],
        )
        aero = asb.Airfoil(surface["airfoil"]).get_aero_from_neuralfoil(
            alpha=alpha_eff_deg,
            Re=reynolds,
            mach=mach,
            n_crit=SURFACE_FORCE_N_CRIT,
            xtr_upper=SURFACE_FORCE_XTR_UPPER,
            xtr_lower=SURFACE_FORCE_XTR_LOWER,
            model_size=SURFACE_FORCE_MODEL_SIZE,
            include_360_deg_effects=SURFACE_FORCE_INCLUDE_360_DEG_EFFECTS,
        )
        cl = float(aero["CL"])
        cd = float(aero["CD"])
        confidence = float(aero["analysis_confidence"])
        lift_n = dynamic_pressure_pa * area_m2 * cl
        drag_n = dynamic_pressure_pa * area_m2 * cd
        panel_area_m2 = area_m2 / surface["count"]
        vertical_force_n = 0.0
        for panel in surface["panels"]:
            span_dir = np.array([0.0, panel["dir_y"], panel["dir_z"]], dtype=float)
            lift_dir = np.cross(flow_dir, span_dir)
            lift_dir = lift_dir / np.linalg.norm(lift_dir)
            panel_lift_n = dynamic_pressure_pa * panel_area_m2 * cl
            panel_drag_n = dynamic_pressure_pa * panel_area_m2 * cd
            vertical_force_n += panel_lift_n * abs(lift_dir[2]) + panel_drag_n * flow_dir[2]
        surface_vertical_force_n += vertical_force_n
        surface_vertical_moment_n_m += vertical_force_n * x_force_m
        surface_force_rows.append(
            {
                "name": surface["name"],
                "module": surface["module"],
                "airfoil": surface["airfoil"],
                "alpha_eff_deg": alpha_eff_deg,
                "area_m2": area_m2,
                "x_force_m": x_force_m,
                "reynolds": reynolds,
                "mach": mach,
                "cl": cl,
                "cd": cd,
                "confidence": confidence,
                "lift_n": lift_n,
                "drag_n": drag_n,
                "vertical_force_n": vertical_force_n,
                "vertical_force_g": vertical_force_n * GRAMS_FORCE_PER_NEWTON,
                "load_pct": vertical_force_n / analysis["weight_N"] * 100.0,
            }
        )
    analysis["force_eval_aoa_deg"] = aoa_eval_deg
    analysis["force_eval_airspeed_m_s"] = force_eval_airspeed_m_s
    analysis["force_eval_altitude_m"] = force_eval_altitude_m
    analysis["force_eval_density_kg_m3"] = density_kg_m3
    analysis["force_eval_dynamic_pressure_pa"] = dynamic_pressure_pa
    analysis["surface_force_rows"] = surface_force_rows
    analysis["surface_vertical_force_n"] = surface_vertical_force_n
    analysis["surface_vertical_force_g"] = surface_vertical_force_n * GRAMS_FORCE_PER_NEWTON
    analysis["surface_vertical_load_pct"] = surface_vertical_force_n / analysis["weight_N"] * 100.0
    analysis["force_center_x"] = surface_vertical_moment_n_m / surface_vertical_force_n if abs(surface_vertical_force_n) > 1e-9 else float("nan")
    return analysis


def analyze_config(
    assembly_name: str,
    config: dict,
    modules_data: dict,
    parts_data: dict,
    standards_data: dict,
    motor_data_df: pd.DataFrame,
    aoa_eval_deg: float,
    force_eval_airspeed_m_s: float,
    force_eval_altitude_m: float,
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
    analysis = apply_linear_aero_analysis(analysis, aoa_eval_deg, force_eval_airspeed_m_s, force_eval_altitude_m)
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
    force_eval_airspeed_m_s: float,
    force_eval_altitude_m: float,
) -> dict:
    return analyze_config(
        assembly_name=assembly_name,
        config=config,
        modules_data=modules_data,
        parts_data=parts_data,
        standards_data=standards_data,
        motor_data_df=motor_data_df,
        aoa_eval_deg=aoa_eval_deg,
        force_eval_airspeed_m_s=force_eval_airspeed_m_s,
        force_eval_altitude_m=force_eval_altitude_m,
    )
