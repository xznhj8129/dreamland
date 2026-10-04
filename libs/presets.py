"""
Usage:
from libs.presets import apply_preset, preset_names, preset_status, save_preset
apply_preset(definition, "M62")
preset_status(definition)  # "M62" or "M62 (modified)"

Size presets (data/specs.json presets) on a raw airframe/2 definition. A preset fills the fuselage blocks it drives
(section, joints, rod pattern); the aircraft keeps its own values and the preset name as a label. The aircraft's
own coupler mass and rod extent (from_x_m, to_x_m) are never driven. Validation is libs/airframe.py's.
"""

import copy
import json

from libs.airframe import SPECS_PATH, AirframeError, presets_from_specs

DRIVEN_BLOCKS = ("section", "joints", "reinforcement")
DRIVEN_JOINT_KEYS = ("type", "count", "screw", "insertion_depth_m")
ROD_PATTERN_KEYS = ("type", "material", "diameter_m", "radial_position_m", "polar_n", "polar_offset_angle", "polar_split", "polar_split_angle")
ROD_EXTENT_KEYS = ("from_x_m", "to_x_m")


def read_specs() -> dict:
    with open(SPECS_PATH, "r") as file:
        return json.load(file)


def preset_names() -> list[str]:
    return list(read_specs()["presets"])


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
    stored = read_specs()["presets"].get(name)
    if stored is None:
        return f"{name} (not in data/specs.json)"
    return name if preset_blocks(definition) == {key: stored[key] for key in DRIVEN_BLOCKS if key in stored} else f"{name} (modified)"


def default_rod_extent(definition: dict) -> dict:
    """Rods from the end of the nose to the start of the tail."""
    fuselage = definition["fuselage"]
    return {"from_x_m": fuselage["nose"]["length_m"], "to_x_m": round(fuselage["length_m"] - fuselage["tail"]["length_m"], 6)}


def apply_preset(definition: dict, name: str) -> None:
    """Fill the driven blocks from the preset and label the definition with it. Rods keep their extent (the default
    one when the aircraft had none); a preset without rods removes them."""
    preset = read_specs()["presets"][name]
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
    """Write the definition's driven values as preset `name` (new or overwritten; an existing preset keeps its limits
    and CAD data) and validate the presets; on AirframeError the file is restored and the error raised."""
    before = SPECS_PATH.read_text()
    specs = json.loads(before)
    kept = {key: value for key, value in specs["presets"].get(name, {}).items() if key not in DRIVEN_BLOCKS}
    specs["presets"][name] = preset_blocks(definition) | kept
    SPECS_PATH.write_text(json.dumps(specs, indent=4) + "\n")
    try:
        presets_from_specs(parts_data)
    except AirframeError:
        SPECS_PATH.write_text(before)
        raise
