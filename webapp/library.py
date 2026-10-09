"""Server API for the part-library editor (data/parts/), mirroring the Tk PartEditorFrame.

The Tk editor is libs/views/part_editor_view.py + libs/library_editors.py; that module is never imported (it needs
tkinter), so the field metadata and the save/delete semantics are ported here, with line references to it. The
library itself is read/written through libs.airframe.load_parts_data / save_parts_category, which are the same
functions the Tk panel uses, so a file is always round-tripped the same way.
"""

from __future__ import annotations

import copy
import json
import re
from typing import Any

from libs.airframe import PART_ROLES, load_parts_data, save_parts_category
from webapp.state import AppState

# library_editors.py 22-33: values stored in m, shown in mm.
LENGTH_FIELD_KEYS_M = {
    "len",
    "diam",
    "semispan",
    "chord_root",
    "chord_tip",
    "motor_x_pos",
    "motor_y_pos",
    "pod_diam",
    "pod_len_flat",
    "pod_len_curve",
}
# library_editors.py 34: values stored in kg, shown in g.
MASS_FIELD_KEYS_KG = {"mass"}
# library_editors.py 35-47: the order fields are shown in; keys not listed follow sorted.
PART_LIBRARY_FIELD_ORDER = [
    "type",
    "mass",
    "s",
    "p",
    "mah",
    "c",
    "curr",
    "min_s",
    "max_s",
    "cell",
    "dimensions",
]
# library_editors.py 49-52: fields an entry may leave out (shown blank, written when filled, removed when
# blanked); the Tk values are Python types, the payload needs their JSON names.
OPTIONAL_LIBRARY_FIELDS = {
    "battery": {"connector": "str", "ir_mohm_per_cell": "float", "lead_awg": "int", "lead_length_mm": "float"},
    "esc": {"mosfet_rds_on_ohm": "float"},
}
# library_editors.py 54: a new role part's common fields, filled in by hand.
NEW_ROLE_PART = {"generic": True, "mass": 0.0, "dimensions": {"l": 10.0, "w": 10.0, "h": 10.0}, "mount": None, "wiring_origin_mm": [0.0, 0.0, 0.0], "notes": ""}
# library_editors.py 55-72: what New starts a category's part from.
NEW_PART_LIBRARY_DEFAULTS = {
    "battery": {
        "type": "lipo",
        "mass": 0.0,
        "s": 4,
        "p": 1,
        "mah": 0,
        "c": 0,
        "connector": "",
        "dimensions": {"l": 0, "h": 0, "w": 0},
    },
    **{role: dict(NEW_ROLE_PART) for role in PART_ROLES},
    "esc": dict(NEW_ROLE_PART) | {"curr": 0, "min_s": 0, "max_s": 0},
    "mounting": {"holes_mm": [], "screw": "M3", "outline_mm": [0.0, 0.0], "notes": ""},
    "prop": {"diameter_in": 0, "mass": 0.0},
    "material": {"density_kg_m3": 0.0},
    "connector": {"contact_resistance_ohm": 0.0},
}


# library_editors.py 75-79
def ordered_keys(data: dict, preferred_keys: list[str]) -> list[str]:
    keys = list(data.keys())
    ordered = [key for key in preferred_keys if key in data]
    ordered.extend(sorted(key for key in keys if key not in ordered))
    return ordered


# library_editors.py 82-91
def format_field_value(key: str, value):
    if value is None:
        return ""
    if key in LENGTH_FIELD_KEYS_M:
        return f"{value * 1000.0:g}"
    if key in MASS_FIELD_KEYS_KG:
        return f"{value * 1000.0:g}"
    if type(value) is list:
        return json.dumps(value)
    return str(value)


# library_editors.py 94-127
def field_label(key: str) -> str:
    if key == "wings_layout":
        return "layout (H 1 3 + X)"
    if key == "wings_fin_pos":
        return "fin position (top / bottom)"
    if key == "wings_ang":
        return "dihedral (deg)"
    if key == "wings_x":
        return "fore/aft mount (0=front, 1=back)"
    if key == "wings_y_offset":
        return "y-offset (-1=bottom, 0=mid, 1=top; X: -1=mid, 0=45deg, 1=top)"
    if key == "pointed_shape":
        return "pointed shape"
    if key == "shape":
        return "shape"
    if key == "k_prime":
        return "parabolic K'"
    if key == "rho_factor":
        return "secant rho / tangent rho"
    if key == "exponent":
        return "power n"
    if key == "c":
        return "haack C"
    if key == "break_fraction":
        return "bi-conic L1 / L"
    if key == "break_radius_fraction":
        return "bi-conic R1 / R"
    if key == "nose_radius":
        return "nose radius r_n (mm)"
    if key in LENGTH_FIELD_KEYS_M:
        return f"{key} (mm)"
    if key in MASS_FIELD_KEYS_KG:
        return f"{key} (g)"
    return key


# library_editors.py 130-147
def parse_field_value(key: str, text: str, existing_value):
    if existing_value is None:  # e.g. a part's mount: blank (or none) stays null
        return None if text.strip().lower() in ("", "none", "null") else text.strip()
    if type(existing_value) is int:
        value = int(text)
    elif type(existing_value) is float:
        value = float(text)
    elif type(existing_value) is bool:
        value = text.lower() == "true"
    elif type(existing_value) is list:
        value = json.loads(text)
    else:
        value = text
    if key in LENGTH_FIELD_KEYS_M:
        return value / 1000.0
    if key in MASS_FIELD_KEYS_KG:
        return value / 1000.0
    return value


# library_editors.py 150-155
def text_is_valid_for_existing_value(value_type: str, proposed_text: str) -> bool:
    if value_type == "int":
        return re.fullmatch(r"[+-]?\d*", proposed_text) is not None
    if value_type == "float":
        return re.fullmatch(r"[+-]?((\d+(\.\d*)?)|(\.\d*))?([eE][+-]?\d*)?", proposed_text) is not None
    return True


# library_editors.py 158-162
def set_nested_value(target: dict, path: tuple[str, ...], value) -> None:
    cursor = target
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = value


def library_categories(parts_data: dict) -> list[dict[str, str]]:
    """The categories and their kind: a role (libs.airframe.PART_ROLES, a data/parts/<role>/ directory) or a table
    file. Sorted as the Tk combobox (library_editors.py 270) lists them."""
    return [{"name": name, "kind": "role" if name in PART_ROLES else "table"} for name in sorted(parts_data)]


def library_payload(state: AppState) -> dict[str, Any]:
    """GET /api/library: the whole library plus the form metadata the browser editor needs."""
    return {
        "parts": state.parts_data,
        "categories": library_categories(state.parts_data),
        "field_order": list(PART_LIBRARY_FIELD_ORDER),
        "length_keys": sorted(LENGTH_FIELD_KEYS_M),
        "mass_keys": sorted(MASS_FIELD_KEYS_KG),
        "optional_fields": {category: dict(fields) for category, fields in OPTIONAL_LIBRARY_FIELDS.items()},
        "defaults": NEW_PART_LIBRARY_DEFAULTS,
        "roles": list(PART_ROLES),
    }


def _plain_stem(value: Any) -> bool:
    """A category name is an existing table stem or role directory; a part name is a plain filename stem
    (save_parts_category writes f"{name}.json"). Reject empty names, path separators, any dot and NUL."""
    return isinstance(value, str) and value != "" and not any(char in value for char in (".", "/", "\\", "\x00"))


def _checked_category(state: AppState, value: Any) -> str:
    if not _plain_stem(value) or value not in state.parts_data:
        raise ValueError(f"unknown category {value!r}")
    return value


def _checked_name(value: Any) -> str:
    if not _plain_stem(value):
        raise ValueError(f"invalid part name {value!r}")
    return value


def _original(state: AppState, body: dict) -> tuple[str, str] | None:
    """The entry the save was opened on: {category, name}, or null/absent for a New part."""
    original = body.get("original")
    if original is None:
        return None
    if not isinstance(original, dict):
        raise ValueError("original must be {category, name} or null")
    category = _checked_category(state, original.get("category"))
    name = _checked_name(original.get("name"))
    if name not in state.parts_data[category]:
        raise ValueError(f"no part named {name!r} in category {category!r}")
    return category, name


def _reload_after_write(state: AppState) -> None:
    """Adopt what is on disk, bump the revision so cached editor context invalidates, recompute and push."""
    state.parts_data = load_parts_data()
    state.library_revision += 1
    state.recompute()
    state.notify()


def save_part(state: AppState, body: dict) -> None:
    """The Tk PartEditorFrame.save_current (library_editors.py 318-343) over JSON.

    body: {category, name, original: {category, name} | null, values: {...}}. A rename/move writes both the old
    and the new category, so save_parts_category deletes the old role file; a failure reloads from disk before
    propagating, so state.parts_data never keeps an entry the write did not make durable.
    """
    category = _checked_category(state, body.get("category"))
    name = _checked_name(body.get("name"))
    values = body.get("values")
    if not isinstance(values, dict):
        raise ValueError("values must be a JSON object")
    original = _original(state, body)
    updated = copy.deepcopy(values)
    original_category, original_name = original if original is not None else (None, None)
    try:
        if original_category is None:
            state.parts_data[category][name] = updated
            categories: tuple[str, ...] = (category,)
        elif original_category == category:  # in place, so the entry keeps its order
            state.parts_data[category] = {
                (name if key == original_name else key): (updated if key == original_name else value)
                for key, value in state.parts_data[category].items()
            }
            categories = (category,)
        else:  # renamed into another category: drop the old entry, save both categories
            del state.parts_data[original_category][original_name]
            state.parts_data.setdefault(category, {})[name] = updated
            categories = tuple(dict.fromkeys((original_category, category)))
        for changed in categories:
            save_parts_category(changed, state.parts_data[changed])
    except Exception:
        state.parts_data = load_parts_data()  # in-memory library back in step with disk
        raise
    _reload_after_write(state)


def delete_part(state: AppState, body: dict) -> None:
    """The Tk PartEditorFrame.delete_current (library_editors.py 359-370) over JSON (confirmation is client-side)."""
    category = _checked_category(state, body.get("category"))
    name = _checked_name(body.get("name"))
    if name not in state.parts_data[category]:
        raise ValueError(f"no part named {name!r} in category {category!r}")
    del state.parts_data[category][name]
    try:
        save_parts_category(category, state.parts_data[category])
    except Exception:
        state.parts_data = load_parts_data()
        raise
    _reload_after_write(state)
