"""Server API for the Aero Analysis view: NeuralFoil 2D airfoil comparison (the retired Tk aero_analysis_view.py).

The compute is libs.airfoil_analysis unchanged (the Tk view imported the same functions). This module is the
adapter: it resolves the library plus the session's imported DATs, seeds the selection from the airframe's
surfaces, and makes analyze_airfoils' result JSON-safe for the browser.

The Tk view's imported DATs lived only in its own in-memory library and were lost on restart
(aero_analysis_view.py 301-325); so do this module's, in _SESSION_AIRFOILS.
"""

from __future__ import annotations

import math
import threading
from typing import Any

import numpy as np

from libs.airfoil_analysis import (
    DEFAULT_AIRFOIL_OPERATING_POINT,
    DEFAULT_NEURALFOIL_SETTINGS,
    DEFAULT_SELECTED_AIRFOIL_NAMES,
    DEFAULT_SWEEP_ALPHA_STEP_DEG,
    NEURALFOIL_MODEL_SIZES,
    alpha_sweep_values,
    analyze_airfoils,
    asb_airfoil_record,
    load_airfoil_library,
    parse_dat_text,
)

# Airfoils this session added: imported DATs (aero_analysis_view.py 301-325) and AeroSandbox-named surfaces the
# library file does not hold (aero_analysis_view.py 257-271). In memory only, like the Tk library additions.
# Every request thread shares the dict, so name resolution and insertion must be atomic under this lock.
_SESSION_AIRFOILS: dict[str, dict] = {}
_SESSION_AIRFOILS_LOCK = threading.Lock()

# Out-of-domain operating points are refused before NeuralFoil runs: below-sea-level and stratospheric margins
# around the AeroSandbox standard-atmosphere fit, and Earth-surface temperature extremes with margin.
_ALTITUDE_MIN_M = -1000.0
_ALTITUDE_MAX_M = 80000.0
_TEMPERATURE_MIN_C = -100.0
_TEMPERATURE_MAX_C = 100.0

# The per-airfoil buckets the browser gets: the operating-point coefficients analyze_airfoil_record computes from
# the flow conditions, and the sweep series. The names are libs.airfoil_analysis' own.
_OPERATING_POINT_KEYS = ("cl", "cd", "cm", "cl_cd", "top_xtr", "bot_xtr", "confidence")
_SWEEP_KEYS = ("alpha_deg", "cl", "cd", "cl_cd", "cm", "confidence")


def _records_locked() -> dict[str, dict]:
    """The library file plus this session's additions, by name. The caller holds _SESSION_AIRFOILS_LOCK."""
    records = load_airfoil_library()
    records.update(_SESSION_AIRFOILS)
    return records


def _records() -> dict[str, dict]:
    """The library file plus this session's additions, by name."""
    with _SESSION_AIRFOILS_LOCK:
        return _records_locked()


def _json_safe(value: Any) -> Any:
    """Plain JSON values: numpy arrays to lists, numpy scalars to float/int, a non-finite float to None.

    Python's json writes NaN/Infinity for non-finite floats and browser JSON.parse rejects those tokens, so
    every NaN (e.g. a summary field NeuralFoil could not fill) must travel as null.
    """
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.generic):  # numpy scalar: np.float64, np.int64, np.bool_, ...
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _number(request: dict, key: str, default: float) -> float:
    """One numeric request field. A JSON boolean is refused: float(True) would silently read as 1.0."""
    value = request.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{key} must be finite")
    return number


def _validate_operating_point(operating_point: dict, neuralfoil_settings: dict) -> None:
    """Refuse inputs NeuralFoil cannot answer meaningfully, before it runs (a 400, not a compute 500)."""
    if operating_point["airspeed_m_s"] <= 0:
        raise ValueError("airspeed_m_s must be > 0")
    if operating_point["chord_mm"] <= 0:
        raise ValueError("chord_mm must be > 0")
    if not _ALTITUDE_MIN_M <= operating_point["altitude_m"] <= _ALTITUDE_MAX_M:
        raise ValueError(f"altitude_m must be within {_ALTITUDE_MIN_M:g}..{_ALTITUDE_MAX_M:g} m")
    if not _TEMPERATURE_MIN_C <= operating_point["temperature_c"] <= _TEMPERATURE_MAX_C:
        raise ValueError(f"temperature_c must be within {_TEMPERATURE_MIN_C:g}..{_TEMPERATURE_MAX_C:g} C")
    if neuralfoil_settings["n_crit"] < 0:
        raise ValueError("n_crit must be >= 0")
    for key in ("xtr_upper", "xtr_lower"):
        if not 0.0 <= neuralfoil_settings[key] <= 1.0:
            raise ValueError(f"{key} must be within [0, 1]")


def library_payload(surfaces: list[str] | None) -> dict[str, Any]:
    """GET /api/airfoils: the library plus session imports, the seeded selection and the form defaults.

    Seeding ports aero_analysis_view.py 257-271 seed_selected_airfoils: each surface name selects its airfoil;
    one the library file does not hold is added for the session through asb_airfoil_record, and a name that is
    not a known AeroSandbox airfoil is skipped. No surfaces falls back to DEFAULT_SELECTED_AIRFOIL_NAMES.
    """
    records = _records()
    selected: list[str] = []
    for name in surfaces if surfaces else DEFAULT_SELECTED_AIRFOIL_NAMES:
        if not isinstance(name, str) or not name:
            continue
        if name not in records:
            # The session store is shared across request threads; re-check under the lock so a concurrent
            # import of the same name wins instead of being overwritten by the surface seed.
            with _SESSION_AIRFOILS_LOCK:
                if name not in _SESSION_AIRFOILS:
                    try:
                        _SESSION_AIRFOILS[name] = asb_airfoil_record(name)
                    except ValueError:
                        continue
                records[name] = _SESSION_AIRFOILS[name]
        if name not in selected:
            selected.append(name)
    sweep_alpha_deg = alpha_sweep_values()
    return {
        "airfoils": [{"name": name, "source": record.get("source", "")} for name, record in sorted(records.items())],
        "selected": selected,
        "defaults": {**DEFAULT_AIRFOIL_OPERATING_POINT, **DEFAULT_NEURALFOIL_SETTINGS},
        "model_sizes": list(NEURALFOIL_MODEL_SIZES),
        "alpha_sweep": {
            "min_deg": float(sweep_alpha_deg[0]),
            "max_deg": float(sweep_alpha_deg[-1]),
            "step_deg": float(DEFAULT_SWEEP_ALPHA_STEP_DEG),
        },
    }


def import_dat(name: str, dat_text: str) -> dict[str, Any]:
    """POST /api/airfoils/import: validate one Selig/Lednicer DAT and keep it for this session.

    Ports aero_analysis_view.py 301-325 import_airfoil_dat (the file dialog is the browser's): the text must
    parse as a unit-chord airfoil, and a name already taken gets the Tk "-2"/"-3" suffix.
    """
    if not isinstance(name, str) or not name.strip():
        raise ValueError("name must be a non-empty string")
    if not isinstance(dat_text, str) or not dat_text.strip():
        raise ValueError(f"Could not import {name}: the DAT text is empty")
    name = name.strip()
    try:
        parse_dat_text(dat_text)
    except ValueError as error:
        raise ValueError(f"Could not import {name}: {error}") from error
    # Resolve the deduplicated name and insert it atomically: two concurrent imports of the same name must not
    # both read the same free suffix. Records are re-read under the lock so a just-added name is seen.
    with _SESSION_AIRFOILS_LOCK:
        records = _records_locked()
        final_name, suffix = name, 2
        while final_name in records:
            final_name = f"{name}-{suffix}"
            suffix += 1
        _SESSION_AIRFOILS[final_name] = {"name": final_name, "source": "dat_file", "dat_text": dat_text}
    return {"name": final_name, "source": "dat_file"}


def analyze(request: dict) -> dict[str, Any]:
    """POST /api/airfoils/analyze: run libs.airfoil_analysis.analyze_airfoils over the selected airfoils.

    The body carries the library payload's `defaults` fields; a missing field keeps its default. Names resolve
    against the library plus session imports; an unknown one is refused. The result is flattened for the wire:
    the flow conditions, the summary text, and one entry per airfoil with its operating-point coefficients,
    sweep series and summarize fields, with numpy and non-finite values made JSON-safe.
    """
    if not isinstance(request, dict):
        raise ValueError("request must be a JSON object")
    names = request.get("airfoils") or []
    if not isinstance(names, list):
        raise ValueError("airfoils must be a list of names")
    if not names:
        raise ValueError("No airfoils selected.")
    records = _records()
    selected: list[dict] = []
    for name in names:
        record = records.get(name) if isinstance(name, str) else None
        if record is None:
            raise ValueError(f"Unknown airfoil {name!r}")
        selected.append(record)
    operating_point = {
        "alpha_deg": _number(request, "alpha_deg", DEFAULT_AIRFOIL_OPERATING_POINT["alpha_deg"]),
        "airspeed_m_s": _number(request, "airspeed_m_s", DEFAULT_AIRFOIL_OPERATING_POINT["airspeed_m_s"]),
        "altitude_m": _number(request, "altitude_m", DEFAULT_AIRFOIL_OPERATING_POINT["altitude_m"]),
        "temperature_c": _number(request, "temperature_c", DEFAULT_AIRFOIL_OPERATING_POINT["temperature_c"]),
        "chord_mm": _number(request, "chord_mm", DEFAULT_AIRFOIL_OPERATING_POINT["chord_mm"]),
    }
    model_size = request.get("model_size", DEFAULT_NEURALFOIL_SETTINGS["model_size"])
    if model_size not in NEURALFOIL_MODEL_SIZES:
        raise ValueError(f"model_size must be one of {NEURALFOIL_MODEL_SIZES}")
    include_360_deg_effects = request.get("include_360_deg_effects", DEFAULT_NEURALFOIL_SETTINGS["include_360_deg_effects"])
    if not isinstance(include_360_deg_effects, bool):
        raise ValueError("include_360_deg_effects must be a boolean")
    neuralfoil_settings = {
        "n_crit": _number(request, "n_crit", DEFAULT_NEURALFOIL_SETTINGS["n_crit"]),
        "xtr_upper": _number(request, "xtr_upper", DEFAULT_NEURALFOIL_SETTINGS["xtr_upper"]),
        "xtr_lower": _number(request, "xtr_lower", DEFAULT_NEURALFOIL_SETTINGS["xtr_lower"]),
        "model_size": model_size,
        "include_360_deg_effects": include_360_deg_effects,
    }
    _validate_operating_point(operating_point, neuralfoil_settings)
    try:
        result = analyze_airfoils(selected, operating_point, neuralfoil_settings)
    except Exception as error:  # noqa: BLE001 - any NeuralFoil/compute failure is a client 400, not a 500
        raise ValueError(f"Analysis failed: {error}") from error
    return {
        "flow": _json_safe(result["operating_point"]),
        "summary_text": result["summary_text"],
        "airfoils": [
            {
                "name": item["name"],
                "source": item["source"],
                "operating_point": _json_safe({key: item["operating_point"][key] for key in _OPERATING_POINT_KEYS}),
                "sweep": _json_safe({key: item["sweep"][key] for key in _SWEEP_KEYS}),
                "summary": _json_safe(dict(item["summary"])),
            }
            for item in result["results"]
        ],
    }
