"""
Usage:
from libs.airfoil_analysis import analyze_airfoils, load_airfoil_library
"""

from __future__ import annotations

import json
from pathlib import Path

import aerosandbox as asb
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent.parent
AIRFOILS_PATH = SCRIPT_DIR / "data" / "airfoils.json"
DEFAULT_LIBRARY_AIRFOIL_NAMES = [
    "naca0008",
    "naca0012",
    "naca2408",
    "naca2412",
    "naca6408",
]
DEFAULT_SELECTED_AIRFOIL_NAMES = [
    "naca2412",
    "naca0008",
]
NEURALFOIL_MODEL_SIZES = [
    "xxsmall",
    "xsmall",
    "small",
    "medium",
    "large",
    "xlarge",
    "xxlarge",
    "xxxlarge",
]
DEFAULT_AIRFOIL_OPERATING_POINT = {
    "alpha_deg": 4.0,
    "airspeed_m_s": 25.0,
    "altitude_m": 0.0,
    "temperature_c": 15.0,
    "chord_mm": 100.0,
}
DEFAULT_NEURALFOIL_SETTINGS = {
    "n_crit": 9.0,
    "xtr_upper": 1.0,
    "xtr_lower": 1.0,
    "model_size": "large",
    "include_360_deg_effects": True,
}
DEFAULT_SWEEP_ALPHA_MIN_DEG = -20.0
DEFAULT_SWEEP_ALPHA_MAX_DEG = 20.0
DEFAULT_SWEEP_ALPHA_STEP_DEG = 1.0
SUMMARY_CONFIDENCE_MIN = 0.5
SUMMARY_ZERO_LIFT_ALPHA_WINDOW_DEG = 15.0
SUMMARY_BEST_LD_ALPHA_MIN_DEG = -5.0
SUMMARY_BEST_LD_ALPHA_MAX_DEG = 15.0
SUMMARY_STALL_POS_ALPHA_MIN_DEG = 0.0
SUMMARY_STALL_POS_ALPHA_MAX_DEG = 25.0
SUMMARY_STALL_NEG_ALPHA_MIN_DEG = -25.0
SUMMARY_STALL_NEG_ALPHA_MAX_DEG = 0.0


def parse_dat_text(dat_text: str) -> np.ndarray:
    coordinates = []
    for line in dat_text.splitlines():
        stripped = line.strip()
        if stripped == "":
            continue
        parts = stripped.replace(",", " ").split()
        if len(parts) < 2:
            continue
        try:
            x_value = float(parts[0])
            y_value = float(parts[1])
        except ValueError:
            continue
        coordinates.append((x_value, y_value))
    return np.array(coordinates, dtype=float)


def airfoil_record_from_path(dat_path: str | Path) -> dict:
    path = Path(dat_path)
    return {
        "name": path.stem,
        "source": "dat_file",
        "dat_text": path.read_text(),
        "path": str(path),
    }


def load_airfoil_library() -> dict[str, dict]:
    data = json.loads(AIRFOILS_PATH.read_text())
    return {name: {"name": name, **record} for name, record in data.items()}


def airfoil_from_record(record: dict) -> asb.Airfoil:
    coordinates = parse_dat_text(record["dat_text"])
    return asb.Airfoil(name=record["name"], coordinates=coordinates)


def alpha_sweep_values() -> np.ndarray:
    return np.arange(
        DEFAULT_SWEEP_ALPHA_MIN_DEG,
        DEFAULT_SWEEP_ALPHA_MAX_DEG + DEFAULT_SWEEP_ALPHA_STEP_DEG,
        DEFAULT_SWEEP_ALPHA_STEP_DEG,
    )


def compute_flow_conditions(operating_point: dict) -> dict:
    standard_atmosphere = asb.Atmosphere(altitude=operating_point["altitude_m"])
    standard_temperature_c = float(standard_atmosphere.temperature() - 273.15)
    atmosphere = asb.Atmosphere(
        altitude=operating_point["altitude_m"],
        temperature_deviation=operating_point["temperature_c"] - standard_temperature_c,
    )
    chord_m = operating_point["chord_mm"] / 1000.0
    airspeed_m_s = operating_point["airspeed_m_s"]
    density_kg_m3 = float(atmosphere.density())
    dynamic_viscosity_pa_s = float(atmosphere.dynamic_viscosity())
    speed_of_sound_m_s = float(atmosphere.speed_of_sound())
    return {
        "alpha_deg": float(operating_point["alpha_deg"]),
        "airspeed_m_s": float(airspeed_m_s),
        "altitude_m": float(operating_point["altitude_m"]),
        "temperature_c": float(operating_point["temperature_c"]),
        "chord_mm": float(operating_point["chord_mm"]),
        "pressure_pa": float(atmosphere.pressure()),
        "density_kg_m3": density_kg_m3,
        "dynamic_viscosity_pa_s": dynamic_viscosity_pa_s,
        "speed_of_sound_m_s": speed_of_sound_m_s,
        "mach": float(airspeed_m_s / speed_of_sound_m_s),
        "reynolds": float(density_kg_m3 * airspeed_m_s * chord_m / dynamic_viscosity_pa_s),
    }


def compute_zero_lift_alpha(alpha_deg: np.ndarray, cl_values: np.ndarray, confidence_values: np.ndarray) -> float:
    search_mask = (
        (np.abs(alpha_deg) <= SUMMARY_ZERO_LIFT_ALPHA_WINDOW_DEG)
        & (confidence_values >= SUMMARY_CONFIDENCE_MIN)
    )
    sign_change_indices = np.where(np.diff(np.signbit(cl_values)))[0]
    if len(sign_change_indices) == 0:
        return float("nan")
    candidate_indices = [
        int(index)
        for index in sign_change_indices
        if search_mask[int(index)] and search_mask[int(index) + 1]
    ]
    if len(candidate_indices) == 0:
        candidate_indices = [int(index) for index in sign_change_indices]
    crossing_alpha_deg = []
    for index in candidate_indices:
        alpha_pair = alpha_deg[index : index + 2]
        cl_pair = cl_values[index : index + 2]
        crossing_alpha_deg.append(float(np.interp(0.0, cl_pair, alpha_pair)))
    return crossing_alpha_deg[int(np.argmin(np.abs(np.array(crossing_alpha_deg, dtype=float))))]


def first_local_extremum_index(
    alpha_deg: np.ndarray,
    values: np.ndarray,
    confidence_values: np.ndarray,
    alpha_min_deg: float,
    alpha_max_deg: float,
    find_maximum: bool,
) -> int | None:
    valid_indices = np.where(
        (alpha_deg >= alpha_min_deg)
        & (alpha_deg <= alpha_max_deg)
        & (confidence_values >= SUMMARY_CONFIDENCE_MIN)
    )[0]
    if len(valid_indices) < 3:
        return None
    for index in valid_indices[1:-1]:
        if index - 1 not in valid_indices or index + 1 not in valid_indices:
            continue
        if find_maximum:
            if values[index] >= values[index - 1] and values[index] > values[index + 1]:
                return int(index)
            continue
        if values[index] <= values[index - 1] and values[index] < values[index + 1]:
            return int(index)
    return None


def summarize_sweep(alpha_deg: np.ndarray, cl_values: np.ndarray, cd_values: np.ndarray, cm_values: np.ndarray, confidence_values: np.ndarray, cl_cd_values: np.ndarray) -> dict:
    pos_stall_index = first_local_extremum_index(
        alpha_deg,
        cl_values,
        confidence_values,
        SUMMARY_STALL_POS_ALPHA_MIN_DEG,
        SUMMARY_STALL_POS_ALPHA_MAX_DEG,
        True,
    )
    neg_stall_index = first_local_extremum_index(
        alpha_deg,
        cl_values,
        confidence_values,
        SUMMARY_STALL_NEG_ALPHA_MIN_DEG,
        SUMMARY_STALL_NEG_ALPHA_MAX_DEG,
        False,
    )
    best_ld_mask = (
        (alpha_deg >= SUMMARY_BEST_LD_ALPHA_MIN_DEG)
        & (alpha_deg <= SUMMARY_BEST_LD_ALPHA_MAX_DEG)
        & (confidence_values >= SUMMARY_CONFIDENCE_MIN)
        & (cl_values > 0.0)
        & np.isfinite(cl_cd_values)
    )
    min_cd_mask = (
        (alpha_deg >= SUMMARY_BEST_LD_ALPHA_MIN_DEG)
        & (alpha_deg <= SUMMARY_BEST_LD_ALPHA_MAX_DEG)
        & (confidence_values >= SUMMARY_CONFIDENCE_MIN)
        & np.isfinite(cd_values)
        & (cd_values > 0.0)
    )
    best_ld_index = None
    if np.any(best_ld_mask):
        valid_best_ld_indices = np.where(best_ld_mask)[0]
        best_ld_index = int(valid_best_ld_indices[int(np.nanargmax(cl_cd_values[best_ld_mask]))])
    min_cd_index = None
    if np.any(min_cd_mask):
        valid_min_cd_indices = np.where(min_cd_mask)[0]
        min_cd_index = int(valid_min_cd_indices[int(np.argmin(cd_values[min_cd_mask]))])
    return {
        "alpha_zero_lift_deg": compute_zero_lift_alpha(alpha_deg, cl_values, confidence_values),
        "alpha_pos_stall_deg": float(alpha_deg[pos_stall_index]) if pos_stall_index is not None else float("nan"),
        "cl_max": float(cl_values[pos_stall_index]) if pos_stall_index is not None else float("nan"),
        "alpha_neg_stall_deg": float(alpha_deg[neg_stall_index]) if neg_stall_index is not None else float("nan"),
        "cl_min": float(cl_values[neg_stall_index]) if neg_stall_index is not None else float("nan"),
        "alpha_best_ld_deg": float(alpha_deg[best_ld_index]) if best_ld_index is not None else float("nan"),
        "cl_cd_max": float(cl_cd_values[best_ld_index]) if best_ld_index is not None else float("nan"),
        "alpha_cd_min_deg": float(alpha_deg[min_cd_index]) if min_cd_index is not None else float("nan"),
        "cd_min": float(cd_values[min_cd_index]) if min_cd_index is not None else float("nan"),
        "cm_at_cd_min": float(cm_values[min_cd_index]) if min_cd_index is not None else float("nan"),
        "confidence_min": float(np.min(confidence_values)),
        "confidence_mean": float(np.mean(confidence_values)),
    }


def analyze_airfoil_record(record: dict, flow_conditions: dict, neuralfoil_settings: dict) -> dict:
    airfoil = airfoil_from_record(record)
    sweep_alpha_deg = alpha_sweep_values()
    sweep_outputs = airfoil.get_aero_from_neuralfoil(
        alpha=sweep_alpha_deg,
        Re=flow_conditions["reynolds"],
        mach=flow_conditions["mach"],
        n_crit=neuralfoil_settings["n_crit"],
        xtr_upper=neuralfoil_settings["xtr_upper"],
        xtr_lower=neuralfoil_settings["xtr_lower"],
        model_size=neuralfoil_settings["model_size"],
        include_360_deg_effects=neuralfoil_settings["include_360_deg_effects"],
    )
    operating_outputs = airfoil.get_aero_from_neuralfoil(
        alpha=flow_conditions["alpha_deg"],
        Re=flow_conditions["reynolds"],
        mach=flow_conditions["mach"],
        n_crit=neuralfoil_settings["n_crit"],
        xtr_upper=neuralfoil_settings["xtr_upper"],
        xtr_lower=neuralfoil_settings["xtr_lower"],
        model_size=neuralfoil_settings["model_size"],
        include_360_deg_effects=neuralfoil_settings["include_360_deg_effects"],
    )
    cl_values = np.array(sweep_outputs["CL"], dtype=float)
    cd_values = np.array(sweep_outputs["CD"], dtype=float)
    cm_values = np.array(sweep_outputs["CM"], dtype=float)
    confidence_values = np.array(sweep_outputs["analysis_confidence"], dtype=float)
    cl_cd_values = np.full_like(cl_values, np.nan, dtype=float)
    np.divide(cl_values, cd_values, out=cl_cd_values, where=cd_values > 0.0)
    operating_cl_cd = float("nan")
    if float(operating_outputs["CD"]) > 0.0:
        operating_cl_cd = float(operating_outputs["CL"] / operating_outputs["CD"])
    return {
        "name": record["name"],
        "source": record["source"],
        "path": record.get("path", ""),
        "operating_point": {
            **flow_conditions,
            "cl": float(operating_outputs["CL"]),
            "cd": float(operating_outputs["CD"]),
            "cm": float(operating_outputs["CM"]),
            "cl_cd": operating_cl_cd,
            "top_xtr": float(operating_outputs["Top_Xtr"]),
            "bot_xtr": float(operating_outputs["Bot_Xtr"]),
            "confidence": float(operating_outputs["analysis_confidence"]),
        },
        "sweep": {
            "alpha_deg": sweep_alpha_deg,
            "cl": cl_values,
            "cd": cd_values,
            "cm": cm_values,
            "cl_cd": cl_cd_values,
            "confidence": confidence_values,
        },
        "summary": summarize_sweep(
            alpha_deg=sweep_alpha_deg,
            cl_values=cl_values,
            cd_values=cd_values,
            cm_values=cm_values,
            confidence_values=confidence_values,
            cl_cd_values=cl_cd_values,
        ),
    }


def analyze_airfoils(records: list[dict], operating_point: dict, neuralfoil_settings: dict) -> dict:
    flow_conditions = compute_flow_conditions(operating_point)
    results = [analyze_airfoil_record(record, flow_conditions, neuralfoil_settings) for record in records]
    return {
        "operating_point": dict(flow_conditions),
        "neuralfoil_settings": dict(neuralfoil_settings),
        "sweep_alpha_deg": alpha_sweep_values(),
        "results": results,
        "summary_text": build_airfoil_summary_text(results, flow_conditions, neuralfoil_settings),
    }


def build_airfoil_summary_text(results: list[dict], flow_conditions: dict, neuralfoil_settings: dict) -> str:
    lines = [
        f"op.alpha_deg={flow_conditions['alpha_deg']:.3f}",
        f"op.airspeed_m_s={flow_conditions['airspeed_m_s']:.3f}",
        f"op.altitude_m={flow_conditions['altitude_m']:.3f}",
        f"op.temperature_c={flow_conditions['temperature_c']:.3f}",
        f"op.chord_mm={flow_conditions['chord_mm']:.3f}",
        f"flow.reynolds={flow_conditions['reynolds']:.1f}",
        f"flow.mach={flow_conditions['mach']:.5f}",
        f"flow.density_kg_m3={flow_conditions['density_kg_m3']:.6f}",
        f"flow.dynamic_viscosity_pa_s={flow_conditions['dynamic_viscosity_pa_s']:.9f}",
        f"flow.pressure_pa={flow_conditions['pressure_pa']:.3f}",
        f"flow.speed_of_sound_m_s={flow_conditions['speed_of_sound_m_s']:.6f}",
        f"nf.n_crit={neuralfoil_settings['n_crit']:.3f}",
        f"nf.xtr_upper={neuralfoil_settings['xtr_upper']:.3f}",
        f"nf.xtr_lower={neuralfoil_settings['xtr_lower']:.3f}",
        f"nf.model_size={neuralfoil_settings['model_size']}",
        f"nf.include_360_deg_effects={int(bool(neuralfoil_settings['include_360_deg_effects']))}",
        f"sweep.alpha_min_deg={DEFAULT_SWEEP_ALPHA_MIN_DEG:.1f}",
        f"sweep.alpha_max_deg={DEFAULT_SWEEP_ALPHA_MAX_DEG:.1f}",
        f"sweep.alpha_step_deg={DEFAULT_SWEEP_ALPHA_STEP_DEG:.1f}",
        "",
    ]
    for result in results:
        op = result["operating_point"]
        summary = result["summary"]
        lines.extend(
            [
                f"airfoil.name={result['name']}",
                f"airfoil.source={result['source']}",
                f"op.cl={op['cl']:.6f}",
                f"op.cd={op['cd']:.6f}",
                f"op.cm={op['cm']:.6f}",
                f"op.cl_cd={op['cl_cd']:.6f}",
                f"op.top_xtr={op['top_xtr']:.6f}",
                f"op.bot_xtr={op['bot_xtr']:.6f}",
                f"op.confidence={op['confidence']:.6f}",
                f"sweep.alpha_zero_lift_deg={summary['alpha_zero_lift_deg']:.6f}",
                f"sweep.alpha_pos_stall_deg={summary['alpha_pos_stall_deg']:.6f}",
                f"sweep.cl_max={summary['cl_max']:.6f}",
                f"sweep.alpha_neg_stall_deg={summary['alpha_neg_stall_deg']:.6f}",
                f"sweep.cl_min={summary['cl_min']:.6f}",
                f"sweep.alpha_best_ld_deg={summary['alpha_best_ld_deg']:.6f}",
                f"sweep.cl_cd_max={summary['cl_cd_max']:.6f}",
                f"sweep.alpha_cd_min_deg={summary['alpha_cd_min_deg']:.6f}",
                f"sweep.cd_min={summary['cd_min']:.6f}",
                f"sweep.cm_at_cd_min={summary['cm_at_cd_min']:.6f}",
                f"sweep.confidence_min={summary['confidence_min']:.6f}",
                f"sweep.confidence_mean={summary['confidence_mean']:.6f}",
                "",
            ]
        )
    return "\n".join(lines).strip()
