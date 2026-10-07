"""
Usage:
from libs.presets import apply_preset, preset_names, preset_status, save_preset
apply_preset(definition, "M62")
preset_status(definition)  # "M62" or "M62 (modified)"

Size presets (data/presets/<name>.json, in the definition's own form) on a raw airframe/2 definition. A preset fills
the fuselage blocks it drives (section, joints, rod pattern); the aircraft keeps its own values and the preset name as a
label. The aircraft's own coupler mass and rod extent (from_x_m, to_x_m) are never driven. The preset's spars are what a
surface set starts with when spars are added; they are not driven. Validation is libs/airframe.py's.
"""

import copy
import json

from libs.airframe import PRESETS_DIR, AirframeError, load_presets

DRIVEN_BLOCKS = ("section", "joints", "reinforcement")
DRIVEN_JOINT_KEYS = ("type", "count", "screw", "insertion_depth_m", "coupler_wall_m", "fit_clearance_m", "socket_wall_m", "hole_edge_dist_m", "hole_diameter_m",
                     "hole_angle_offset_deg")
ROD_PATTERN_KEYS = ("type", "material", "diameter_m", "radial_position_m", "polar_n", "polar_offset_angle", "polar_split", "polar_split_angle")
ROD_EXTENT_KEYS = ("from_x_m", "to_x_m")


def read_presets() -> dict[str, dict]:
    """Every preset as stored, by name."""
    return {path.stem: json.loads(path.read_text()) for path in sorted(PRESETS_DIR.glob("*.json"))}


def preset_names() -> list[str]:
    return list(read_presets())


def preset_spars(definition: dict) -> dict | None:
    """The spars block of the definition's preset, without its carry-through; None without a preset or preset spars."""
    spars = read_presets().get(definition.get("preset"), {}).get("spars")
    return None if spars is None else {key: value for key, value in spars.items() if key != "carry_through"}


def preset_carry_through(definition: dict) -> dict | None:
    """The carry-through of the definition's preset spars; None without one."""
    return read_presets().get(definition.get("preset"), {}).get("spars", {}).get("carry_through")


def preset_blocks(definition: dict) -> dict:
    """The values of a definition a preset drives, shaped like a preset entry."""
    fuselage = definition["fuselage"]
    blocks = {"section": dict(fuselage["section"]), "joints": {key: fuselage["joints"][key] for key in DRIVEN_JOINT_KEYS}}
    if "reinforcement" in fuselage:
        blocks["reinforcement"] = {key: fuselage["reinforcement"][key] for key in ROD_PATTERN_KEYS}
    return blocks


def preset_status(definition: dict) -> str:
    """The preset label, marked modified once a driven value differs from the preset; "none" without one."""
    name = definition.get("preset")
    if name is None:
        return "none"
    stored = read_presets().get(name)
    if stored is None:
        return f"{name} (not in data/presets/)"
    return name if preset_blocks(definition) == {key: stored[key] for key in DRIVEN_BLOCKS if key in stored} else f"{name} (modified)"


def default_rod_extent(definition: dict) -> dict:
    """Rods from the end of the nose to the start of the tail."""
    fuselage = definition["fuselage"]
    return {"from_x_m": fuselage["nose"]["length_m"], "to_x_m": round(fuselage["length_m"] - fuselage["tail"]["length_m"], 6)}


def apply_preset(definition: dict, name: str) -> None:
    """Fill the driven blocks from the preset and label the definition with it. Rods keep their extent (the default
    one when the aircraft had none); a preset without rods removes them."""
    preset = read_presets()[name]
    fuselage = definition["fuselage"]
    fuselage["section"] = copy.deepcopy(preset["section"])
    fuselage["joints"].update(copy.deepcopy(preset["joints"]))
    if "reinforcement" in preset:
        extent = {key: fuselage["reinforcement"][key] for key in ROD_EXTENT_KEYS} if "reinforcement" in fuselage else default_rod_extent(definition)
        fuselage["reinforcement"] = copy.deepcopy(preset["reinforcement"]) | extent
    else:
        fuselage.pop("reinforcement", None)
    definition["preset"] = name


def save_preset(name: str, definition: dict, parts_data: dict) -> None:
    """Write the definition's driven values as preset `name` (new or overwritten; an existing preset keeps its spars,
    limits and CAD data) and validate the presets; on AirframeError the file is restored (or removed, if new) and the
    error raised."""
    path = PRESETS_DIR / f"{name}.json"
    before = path.read_text() if path.exists() else None
    kept = {key: value for key, value in (json.loads(before) if before else {}).items() if key not in DRIVEN_BLOCKS}
    path.write_text(json.dumps(preset_blocks(definition) | kept, indent=4) + "\n")
    try:
        load_presets(parts_data)
    except AirframeError:
        if before is None:
            path.unlink()
        else:
            path.write_text(before)
        raise
