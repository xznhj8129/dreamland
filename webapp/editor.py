"""Editor context and structural operations for the browser frontend.

The constants and semantics here mirror the Tk editor, libs/views/editor_view.py, so the browser forms behave
exactly like it; that module is never imported (it needs Tk). Line references are to editor_view.py unless noted.
The structural operations are ported from the Tk methods named in each docstring.
"""

from __future__ import annotations

import copy
from typing import Any

from libs.airframe import (
    AXES, BATTERY_PART_NAME, CONTROL_KINDS, CONTROL_ROLES, CUT_JOINTS, FIN_POSITIONS, INCIDENCE_MODES, JOINT_TYPES,
    LAYOUT_PANEL_COUNTS, LAYOUTS, MOUNT_TARGETS, ORIENTATION_KEYS, PART_CATEGORIES, REINFORCEMENT_TYPES,
    SERVO_OUTPUTS, SINGLE_PROP_SPINS, SURFACE_PROP_SPINS, SURFACE_ROOTS, THRUST_DIRECTIONS, VOLUME_SHAPES,
    AirframeError, cell_format, motor_count, motor_repo,
)
from libs.pointed_shapes import POINTED_SHAPE_PARAM_KEYS
from libs.presets import (
    ROD_EXTENT_KEYS, ROD_PATTERN_KEYS, apply_preset, preset_carry_through, preset_names, preset_spars, read_presets,
    save_preset,
)
from webapp.state import AppState

# editor_view.py 37-42
MM_KEYS = {
    "length_m", "width_m", "height_m", "wall_radial_m", "insertion_depth_m", "diameter_m", "radial_position_m",
    "from_x_m", "to_x_m", "x_root_le_m", "span_m", "chord_root_m", "chord_tip_m", "span_position_m",
    "x_offset_m", "y_offset_m", "z_offset_m", "x_m", "y_m", "z_m", "lead_length_m", "flat_length_m", "curve_length_m", "nose_radius", "size_m", "printer_bed_m", "cuts_m",
    "corner_radius_m", "stem_width_m", "flare_radius_m", "root_length_m", "wall_fillet_m", "coupler_wall_m", "fit_clearance_m", "socket_wall_m", "hole_edge_dist_m", "hole_diameter_m", "spacing_m", "forward_offset_m", "leading_cut_m", "trailing_cut_m",
}
# editor_view.py 43
GRAM_KEYS = {"mass_kg", "coupler_mass_kg", "empty_mass_kg", "complete_mass_kg"}
# editor_view.py 44
UNIT_SUFFIXES = ("_kg_m3", "_m_s", "_deg_s", "_deg", "_kg", "_m")
# editor_view.py 45-55
CHOICES = {
    "layout": list(LAYOUTS),
    "fin_position": list(FIN_POSITIONS),
    "root": list(SURFACE_ROOTS),
    "incidence_mode": list(INCIDENCE_MODES),
    "axis": list(AXES),
    "on": list(MOUNT_TARGETS),
    "direction": list(THRUST_DIRECTIONS),
    "kind": list(CONTROL_KINDS),
    "shape": list(VOLUME_SHAPES),
    "spin": list(SINGLE_PROP_SPINS),  # a surface mount uses SURFACE_PROP_SPINS instead (build_mounts_tab 1039)
    "type": list(JOINT_TYPES),
    "profile": sorted(POINTED_SHAPE_PARAM_KEYS),
}
# editor_view.py 57
PROFILE_PARAM_DEFAULTS = {"k_prime": 1.0, "nose_radius": 0.00465, "break_fraction": 0.5, "break_radius_fraction": 0.5, "rho_factor": 1.25, "exponent": 0.66, "c": 0.0}
# editor_view.py 79
NEW_ROD_PATTERN = {"type": "cf_rod", "material": "carbon_rod", "diameter_m": 0.0025, "polar_n": 4, "polar_offset_angle": 45.0, "polar_split": 0, "polar_split_angle": 0.0}
# editor_view.py 80
NO_PRESET = "(none)"
# editor_view.py 98
TYPE_FLAGS = ("tailsitter", "vtol", "powered")
# editor_view.py 96
NEW_CUT_JOINT = "coupler"
# Every place parse_airframe coerces a definition value to int (libs/airframe.py 527, 561, 563, 696, 946-947,
# 960, 984, 1110), which is what makes type(existing) is int in editor_view.parse_value: those fields take whole
# numbers only. stacking is the battery's per-axis cell counts (editor_view.py 1002 uses it as the unit key).
INTEGER_KEYS = frozenset({"count", "polar_n", "polar_split", "quadrants", "s", "p", "conductors", "stacking"})
# editor_view.py 58-64: the fields each volume shape owns, and the value a newly picked shape starts with.
VOLUME_SHAPE_FIELDS = {
    "box": {"size_m": [0.02, 0.02, 0.02]},
    "rounded_box": {"size_m": [0.02, 0.02, 0.02], "axis": "x", "corner_radius_m": 0.004},
    "cylinder": {"diameter_m": 0.02, "length_m": 0.03, "axis": "x"},
    "bore": {"length_m": 0.05},
    "point": {},
}
# editor_view.py 65-68
NEW_SURFACE = {
    "name": "wing", "x_root_le_m": 0.2, "layout": "H", "fin_position": "top", "dihedral_deg": 0.0, "root_offset": 0.0,
    "root": "body", "span_m": 0.12, "chord_root_m": 0.1, "chord_tip_m": 0.07, "sweep_le_deg": 10.0, "incidence_deg": 0.0, "incidence_mode": "symmetric", "airfoil": "naca2412",
}
# editor_view.py 69
CONTROL_KIND_FIELDS = {"split": {"chord_start": 0.75, "span_start": 0.0, "span_end": 1.0}, "all_moving": {"pivot": 0.25}}
# editor_view.py 70
NEW_CONTROL = {"kind": "split", "roles": [], "deflection_min_deg": -25.0, "deflection_max_deg": 25.0, "rate_deg_s": 300.0}  # + kind fields, servos
# editor_view.py 71
NEW_POD = {"diameter_m": 0.035, "flat_length_m": 0.02, "curve_length_m": 0.05, "profile": "elliptical"}
# editor_view.py 73-75: spars and carry-through for an aircraft without a preset to take them from (the tube62 values).
NEW_SPARS = {"material": "carbon_rod", "diameter_m": 0.0064, "count": 2, "spacing_m": 0.0075, "x_offset_m": 0.02, "y_offset_m": 0.0}
NEW_CARRY_THROUGH = {"stem_width_m": 0.01, "length_m": 0.02, "flare_radius_m": 0.02, "root_length_m": 0.04, "wall_fillet_m": 0.004}
NEW_ROOT_BASE = {"length_m": 0.096, "width_m": 0.018, "height_m": 0.0, "forward_offset_m": 0.0, "leading_cut_m": 0.01, "trailing_cut_m": 0.06}
# editor_view.add_mount 1115-1118
NEW_MOUNT = {"on": "nose", "direction": "tractor", "spin": SINGLE_PROP_SPINS[0], "thrust_axis_tilt_deg": 0.0}
# editor_view.add_volume 1242-1249: name and x_m are set from the definition, then the `box` shape fields.
NEW_VOLUME = {"name": "volume", "shape": "box", "x_m": 0.0, **VOLUME_SHAPE_FIELDS["box"]}
# editor_view.add_part 1227-1234: name, the first library part with a mass and its place are set from the definition.
NEW_PART = {"name": "part", "part": ""}
# editor_view.py 76-93: propulsion and wiring defaults. new_propulsion (597-609) is built client-side from these and
# the definition (its place names a volume or the fuselage midpoint, and the ESC is the first placed esc part).
NEW_MOTOR = {"manufacturer": "DYS", "name": "X2807-1300"}  # motorlib
NEW_PROP = "APC E 7x5"  # motorlib
NEW_BATTERY_RUN = {"gauge_awg": 14, "conductors": 2, "plugs": ["XT60", "XT60"]}  # after the battery's own connector
NEW_MOTOR_RUN = {"gauge_awg": 18, "conductors": 3, "plugs": ["MR60"]}
BATTERY_MODEL_LABELS = {"commercial": "Commercial", "custom_liion": "Custom Li-Ion"}  # libs/airframe.py BATTERY_MODELS
NEW_CUSTOM_PACK = {"model": "custom_liion", "s": 4, "p": 1, "cell": "Molicel INR21700-P45B 7/10C", "connector": "XT60", "cell_axis": "x",
                   "stacking": [4, 1, 1], "lead_awg": 14, "lead_length_m": 0.1}
REMOVE_PLUG = "(remove)"
ADD_PLUG = "+"
# editor_view.build_wiring_tab 639
WIRING_COLUMNS = ("run", "length mm", "AWG", "wires", "plugs, along the run")
# editor_view.py 81 and 92
NO_VOLUME = "(none)"
PLACEMENT_KEYS = ("volume", "x_offset_m", "y_offset_m", "z_offset_m", "x_m", "y_m", "z_m")
# editor_view.py 99-100
MOVE_TARGETS = ("cut", "spars", "surface", "volume", "part", "battery", "rod")  # the drag handle targets
MIN_SECTION_LENGTH_M = 0.005
MOVE_QUANTUM_M = 1e-5  # 0.01 mm: dragged dimensions snap to this grid and are rounded to this accuracy


def _quantize(value: float) -> float:
    """Round a dragged dimension to 0.01 mm."""
    return round(value * 1e5) / 1e5


def motorlib_lists(parts_data: dict) -> dict[str, Any]:
    """The motorlib combo lists the Propulsion and Wiring forms need (editor_view.build_propulsion_tab 952-965,
    build_battery_group 972-1005, build_wiring_tab 641-643): manufacturers with a stator-sized motor and their
    motors, all props, the stackable cylindrical cells, the connectors with a contact resistance, and wire gauges."""
    repo = motor_repo()
    motors_by_manufacturer: dict[str, list[str]] = {}
    for code, name in repo.manufacturer_by_idx.items():
        names = sorted(motor.name for motor in repo.motors_for(code) if motor.stator_diameter_mm is not None and motor.stator_height_mm is not None)
        if names:
            motors_by_manufacturer[name] = names
    return {
        "motor_manufacturers": sorted(motors_by_manufacturer, key=str.lower),
        "motors_by_manufacturer": motors_by_manufacturer,
        "props": sorted(repo.prop_by_name, key=str.lower),
        "cells": sorted(name for name in repo.cell_by_name if cell_format(name) is not None),
        "connectors": sorted(name for name, connector in parts_data["connector"].items() if "contact_resistance_ohm" in connector),
        "wire_gauges": sorted(wire.gauge for wire in repo.wires),
    }


def context(state: AppState) -> dict[str, Any]:
    """Everything the editor needs that is not in the state snapshot: part library, presets and the ported Tk
    constants/templates. Presets supply their rod pattern (editor_view.toggle_reinforcement 781-784).

    Only global data belongs here: parts_data and the preset names/rod patterns are read from data/parts/ and
    data/presets/, the motorlib combo lists (motors by manufacturer, props, cells, connectors, gauges) come from
    libs.motorlib, and the rest are code constants and the add templates (NEW_SURFACE 65-68, NEW_CONTROL 70,
    NEW_SPARS 73, NEW_CARRY_THROUGH 74, NEW_ROOT_BASE 75, NEW_MOUNT from add_mount 1115-1118, NEW_VOLUME from
    add_volume 1242-1249, NEW_PART from add_part 1227-1234, NEW_MOTOR/NEW_PROP/NEW_BATTERY_RUN/NEW_MOTOR_RUN
    76-87). Per-definition state (preset_status, preset spars, a reinforcement pattern computed from the fuselage,
    new_propulsion 597-609) comes from the snapshot or is built client-side from these, so a stale context cannot
    override it. The client caches this per browser session and refreshes it after a preset op; a part-library
    save/delete bumps the snapshot's library_revision, which invalidates the cache the same way.
    """
    rods = {
        name: {key: preset["reinforcement"][key] for key in ROD_PATTERN_KEYS}
        for name, preset in read_presets().items()
        if "reinforcement" in preset
    }
    return {
        "parts_data": state.parts_data,
        "preset_names": preset_names(),
        "mm_keys": sorted(MM_KEYS),
        "gram_keys": sorted(GRAM_KEYS),
        "unit_suffixes": list(UNIT_SUFFIXES),
        "choices": CHOICES,
        "type_flags": list(TYPE_FLAGS),
        "profile_param_keys": POINTED_SHAPE_PARAM_KEYS,
        "profile_param_defaults": PROFILE_PARAM_DEFAULTS,
        "new_rod_pattern": NEW_ROD_PATTERN,
        "preset_airframe_rods": rods,
        "rod_pattern_keys": list(ROD_PATTERN_KEYS),
        "rod_extent_keys": list(ROD_EXTENT_KEYS),
        "integer_keys": sorted(INTEGER_KEYS),
        "cut_joints": list(CUT_JOINTS),
        "reinforcement_types": list(REINFORCEMENT_TYPES),
        "no_preset": NO_PRESET,
        "no_volume": NO_VOLUME,
        # Surfaces, mounts, volumes and parts (editor_view.build_surfaces_tab 852-942, build_mounts_tab 1021-1042,
        # build_volumes_tab 1124-1172, build_parts_tab 1174-1234).
        "servo_outputs": list(SERVO_OUTPUTS),
        "control_kinds": list(CONTROL_KINDS),
        "control_roles": list(CONTROL_ROLES),
        "control_kind_fields": CONTROL_KIND_FIELDS,
        "surface_prop_spins": list(SURFACE_PROP_SPINS),
        "single_prop_spins": list(SINGLE_PROP_SPINS),
        "volume_shapes": list(VOLUME_SHAPES),
        "volume_shape_fields": VOLUME_SHAPE_FIELDS,
        "part_categories": list(PART_CATEGORIES),
        "layout_panel_counts": LAYOUT_PANEL_COUNTS,
        "placement_keys": list(PLACEMENT_KEYS),
        "orientation_keys": list(ORIENTATION_KEYS),
        "new_surface": NEW_SURFACE,
        "new_control": NEW_CONTROL,
        "new_spars": NEW_SPARS,
        "new_carry_through": NEW_CARRY_THROUGH,
        "new_root_base": NEW_ROOT_BASE,
        "new_mount": NEW_MOUNT,
        "new_volume": NEW_VOLUME,
        "new_pod": NEW_POD,
        # Propulsion and wiring (editor_view.build_propulsion_tab 944-1019, build_wiring_tab 627-682): the model
        # labels and the add templates, plus the global motorlib lists the combos read (motorlib_lists).
        "battery_model_labels": BATTERY_MODEL_LABELS,
        "new_motor": NEW_MOTOR,
        "new_prop": NEW_PROP,
        "new_battery_run": NEW_BATTERY_RUN,
        "new_motor_run": NEW_MOTOR_RUN,
        "new_custom_pack": NEW_CUSTOM_PACK,
        "remove_plug": REMOVE_PLUG,
        "add_plug": ADD_PLUG,
        "wiring_columns": list(WIRING_COLUMNS),
        **motorlib_lists(state.parts_data),
    }


class OpFailed(Exception):
    """The operation was refused and the reason is already in the Log; the handler answers with the snapshot."""


class SessionMismatch(Exception):
    """The command was aimed at another project session; the handler answers 409 with the current snapshot.

    A project switch or a new project bumps AppState.session_id. Every editor command carries the session the
    browser rendered from, so a request left over from the old project is rejected instead of editing the new
    one (the client's serialized queue keeps these rare, but a tab with several in-flight commands can produce
    them). Nothing is mutated or logged for a mismatch.
    """


def check_session(state: AppState, body: dict[str, Any]) -> None:
    """Reject a command whose session_id is not the current one; bool is not an int (True == 1)."""
    session_id = body.get("session_id")
    if type(session_id) is not int or session_id != state.session_id:
        raise SessionMismatch()


def apply_set(state: AppState, body: dict[str, Any]) -> None:
    """One atomic path edit on the session definition (POST /api/editor/set), then recompute and push.

    The path walks the CURRENT definition and exactly one edit is applied: a dict key is set or (remove)
    deleted, as Tk's optional entry pops the key; a list entry is set or becomes None, as editor_view.make_entry
    437-439 keeps optional list positions. A path that does not resolve raises ValueError before anything is
    mutated, so the handler can answer 400 with the untouched snapshot.
    """
    check_session(state, body)
    path = body.get("path")
    remove = body.get("remove") is True
    if not isinstance(path, list) or not path:
        raise ValueError("path must be a non-empty list of keys and indices")
    if not remove and "value" not in body:
        raise ValueError("a set needs a value, or remove: true")
    node: Any = state.definition
    for step in path[:-1]:
        if type(step) is str and isinstance(node, dict) and step in node:
            node = node[step]
        elif type(step) is int and isinstance(node, list) and 0 <= step < len(node):
            node = node[step]
        else:
            raise ValueError(f"no {step!r} on the way to {path!r}")
    last = path[-1]
    if type(last) is str and isinstance(node, dict):
        if remove:
            node.pop(last, None)  # absent optional key: Tk's data.pop(key, None)
        else:
            node[last] = body["value"]
    elif type(last) is int and isinstance(node, list) and 0 <= last < len(node):
        node[last] = None if remove else body["value"]
    else:
        raise ValueError(f"cannot {'remove' if remove else 'set'} {last!r} in {path[:-1]!r}")
    sync_motor_runs(state.definition)  # editor_view.changed 389-390: the runs follow the motor count before the recompute
    state.set_definition(state.definition)  # bumped revision, recompute and push, as editor_view.changed


def _valid_preset_name(name: str) -> bool:
    """A preset name is a single plain file name (save_preset writes data/presets/<name>.json)."""
    if not name or name in (".", ".."):
        return False
    return not any(char in name for char in ("/", "\\", "\x00"))


def add_cut(definition: dict) -> None:
    """editor_view.add_cut (825-834): split the longest section in two; both halves keep its solid flag and lose
    its weighed masses."""
    sections = definition["sections"]
    cuts, rows = sections["cuts_m"], sections["per_section"]
    bounds = [0.0, *cuts, definition["fuselage"]["length_m"]]
    longest = max(range(len(bounds) - 1), key=lambda index: bounds[index + 1] - bounds[index])
    cuts.insert(longest, round((bounds[longest] + bounds[longest + 1]) / 2.0, 4))
    sections["cut_joints"].insert(longest, NEW_CUT_JOINT)
    rows[longest:longest + 1] = [{"solid": rows[longest]["solid"]}, {"solid": rows[longest]["solid"]}]


def delete_cut(definition: dict, index: int) -> None:
    """editor_view.delete_cut (836-847): merge the sections either side of a cut: the left one's solid flag,
    each weighed mass summed when both have one."""
    sections = definition["sections"]
    if not 0 <= index < len(sections["cuts_m"]):
        raise ValueError(f"cut index {index} is out of range")
    sections["cuts_m"].pop(index)
    sections["cut_joints"].pop(index)
    left, right = sections["per_section"][index:index + 2]
    merged = {"solid": left["solid"]}
    for key in ("empty_mass_kg", "complete_mass_kg"):
        if key in left and key in right:
            merged[key] = left[key] + right[key]
    sections["per_section"][index:index + 2] = [merged]


def reset_profile(definition: dict, end_name: Any, profile: Any) -> None:
    """editor_view.reset_profile_params (849-850), the profile combobox's on_change (make_choice 492-499): the end
    keeps a parameter value that exists under the picked profile and starts its missing keys at
    PROFILE_PARAM_DEFAULTS, so profile and params are one definition mutation (the parser needs exactly the new
    profile's params, libs/airframe.py 496-507). Tk's on_change runs after the combobox stored the new profile, and
    the defaults come from the old params, hence params of the end as stored."""
    if end_name not in ("nose", "tail"):
        raise ValueError(f"fuselage end must be 'nose' or 'tail', got {end_name!r}")
    if profile not in POINTED_SHAPE_PARAM_KEYS:
        raise ValueError(f"profile must be one of {sorted(POINTED_SHAPE_PARAM_KEYS)}, got {profile!r}")
    end = definition["fuselage"][end_name]
    params = end.get("params")
    if not isinstance(params, dict):
        raise ValueError(f"fuselage.{end_name}.params must be an object to reset the profile")
    end["profile"] = profile
    end["params"] = {key: params.get(key, PROFILE_PARAM_DEFAULTS[key]) for key in POINTED_SHAPE_PARAM_KEYS[profile]}


def load_preset(definition: dict, name: str) -> None:
    """editor_view.load_preset (721-727): fill section, joints and rods from the preset and label the aircraft."""
    if name == NO_PRESET:
        definition.pop("preset", None)
        return
    if name not in preset_names():
        raise ValueError(f"no preset named {name!r} in data/presets/")
    apply_preset(definition, name)


def save_preset_as(state: AppState, name: str, overwrite: bool) -> None:
    """editor_view.save_preset_as (729-743): write the definition's driven values as preset `name`; a validation
    failure leaves data/presets/ unchanged (libs.presets.save_preset restores the file)."""
    if not _valid_preset_name(name):
        raise ValueError("preset name must be a plain file name")
    if name == NO_PRESET:
        state.log_error("Save preset", "Pick a preset to save over, or New")
        raise OpFailed()
    if not overwrite and name in preset_names():
        state.log_error("New preset", f"A preset named {name} already exists.")
        raise OpFailed()
    try:
        save_preset(name, state.definition, state.parts_data)
    except AirframeError as error:
        state.log_error("Save preset", f"Not saved, data/presets/ unchanged:\n{error}")
        raise OpFailed() from error
    state.definition["preset"] = name


def _indexed(items: list, index: Any, what: str) -> Any:
    """The item a structural op names; a bool is not an index and an out-of-range one fails before any mutation."""
    if type(index) is not int or not 0 <= index < len(items):
        raise ValueError(f"{what} index {index!r} is out of range")
    return items[index]


def add_surface(definition: dict) -> None:
    """editor_view.add_surface (933-938): NEW_SURFACE under the next free `wing n` name."""
    surface = copy.deepcopy(NEW_SURFACE)
    names = {item["name"] for item in definition["surfaces"]}
    surface["name"] = next(f"wing {n}" for n in range(1, len(names) + 2) if f"wing {n}" not in names)
    definition["surfaces"].append(surface)


def free_servos(definition: dict, count: int) -> list[str]:
    """editor_view.free_servos (899-902): the lowest outputs no control surface uses yet, then S1 (outputs may be shared)."""
    used = {servo for surface in definition["surfaces"] if "control" in surface for servo in surface["control"]["servos"]}
    return ([servo for servo in SERVO_OUTPUTS if servo not in used] + [SERVO_OUTPUTS[0]] * count)[:count]


def toggle_control(definition: dict, surface: dict, enabled: bool) -> None:
    """editor_view.toggle_control (904-912): NEW_CONTROL with its kind fields and one free servo per panel."""
    if enabled:
        control = copy.deepcopy(NEW_CONTROL)
        control.update(copy.deepcopy(CONTROL_KIND_FIELDS[control["kind"]]))
        control["servos"] = free_servos(definition, LAYOUT_PANEL_COUNTS[surface["layout"]])
        surface["control"] = control
    else:
        surface.pop("control", None)


def fit_servos(definition: dict, surface: dict) -> None:
    """editor_view.fit_servos (920-926): a layout change keeps one servo per panel."""
    if "control" not in surface:
        return
    servos = surface["control"]["servos"]
    count = LAYOUT_PANEL_COUNTS[surface["layout"]]
    surface["control"]["servos"] = servos[:count] + free_servos(definition, count - len(servos)) if count > len(servos) else servos[:count]


def reset_control_kind(control: dict, kind: str) -> None:
    """editor_view.reset_control_kind (914-918): the other kind's fields go, this kind's come back at their defaults."""
    if kind not in CONTROL_KINDS:
        raise ValueError(f"control kind must be one of {CONTROL_KINDS}, got {kind!r}")
    control["kind"] = kind
    for fields in CONTROL_KIND_FIELDS.values():
        for key in fields:
            control.pop(key, None)
    control.update(copy.deepcopy(CONTROL_KIND_FIELDS[kind]))


def toggle_role(control: dict, role: str, on: bool) -> None:
    """editor_view.toggle_role (928-931): roles stored in CONTROL_ROLES order."""
    if role not in CONTROL_ROLES:
        raise ValueError(f"control role must be one of {CONTROL_ROLES}, got {role!r}")
    chosen = (set(control["roles"]) - {role}) | ({role} if on else set())
    control["roles"] = [item for item in CONTROL_ROLES if item in chosen]


def toggle_block(data: dict, key: str, template: dict, enabled: bool) -> None:
    """editor_view.toggle_block (1062-1067): an optional block comes back as the template (the current preset's spars,
    or NEW_SPARS) and goes without one."""
    if enabled:
        data[key] = copy.deepcopy(template)
    else:
        data.pop(key, None)


def normalize_mount(definition: dict, mount: dict) -> None:
    """editor_view.normalize_mount (1100-1113): a surface mount carries surface, span_position_m and x_offset_m and
    spins out|in; a nose or tail mount has none of those, its direction (tractor/pusher) and spins cw|ccw."""
    surfaces = definition["surfaces"]
    if mount["on"] == "surface":
        mount.setdefault("surface", surfaces[0]["name"] if surfaces else "")
        mount.setdefault("span_position_m", 0.0)
        mount.setdefault("x_offset_m", 0.0)
        if mount["spin"] not in SURFACE_PROP_SPINS:
            mount["spin"] = SURFACE_PROP_SPINS[0]
    else:
        for key in ("surface", "span_position_m", "x_offset_m"):
            mount.pop(key, None)
        mount["direction"] = "tractor" if mount["on"] == "nose" else "pusher"
        if mount["spin"] not in SINGLE_PROP_SPINS:
            mount["spin"] = SINGLE_PROP_SPINS[0]


def add_mount(definition: dict) -> None:
    """editor_view.add_mount (1115-1118)."""
    if "propulsion" not in definition:
        raise ValueError("not powered: turn on the `powered` type on the Airframe tab first")
    definition["propulsion"]["mounts"].append(copy.deepcopy(NEW_MOUNT))


def sync_motor_runs(definition: dict) -> None:
    """editor_view.sync_motor_runs (611-625), run from Tk's changed() on every edit (389-391), before the
    recompute: propulsion.wiring.motors follows the motor count, an added motor getting a copy of the last run and
    a removed one losing its own. Left alone while a mount names a surface that does not exist (the parser's
    definition error says so), exactly Tk's skip guard, so no run is invented for a missing surface."""
    propulsion = definition.get("propulsion")
    if propulsion is None:
        return
    layouts = {surface["name"]: surface["layout"] for surface in definition["surfaces"]}
    mounts = [(mount["on"], mount.get("surface")) for mount in propulsion["mounts"]]
    if any(on == "surface" and surface not in layouts for on, surface in mounts):
        return
    count = motor_count(mounts, layouts)
    runs = propulsion["wiring"]["motors"]
    while len(runs) < count:
        runs.append(copy.deepcopy(runs[-1] if runs else NEW_MOTOR_RUN))
    del runs[count:]


def set_type_flag(definition: dict, key: str, checked: bool, propulsion: dict | None) -> None:
    """editor_view.set_type_flag (577-593) on the whole definition, so one recompute sees the flags and the
    propulsion block together (adding it while powered is false, or removing it while powered is true, is a
    definition error in between). propulsion is the block to install (the client's stash or new_propulsion
    597-609) when powered is absent; removing one leaves stashing it to the client."""
    if key not in TYPE_FLAGS:
        raise ValueError(f"type flag must be one of {TYPE_FLAGS}, got {key!r}")
    if type(checked) is not bool:
        raise ValueError("flags.set needs a boolean checked")
    chosen = {flag: definition.get(flag) is True for flag in TYPE_FLAGS} | {key: checked}
    if chosen[key]:
        for implied in TYPE_FLAGS[TYPE_FLAGS.index(key) + 1:]:
            chosen[implied] = True
    else:
        for dependent in TYPE_FLAGS[:TYPE_FLAGS.index(key)]:
            chosen[dependent] = False
    if chosen["powered"] and "propulsion" not in definition:
        if not isinstance(propulsion, dict):
            raise ValueError("turning powered on needs a propulsion block")
        definition["propulsion"] = propulsion
    elif not chosen["powered"] and "propulsion" in definition:
        definition.pop("propulsion")
    definition.update(chosen)


def add_volume(definition: dict) -> None:
    """editor_view.add_volume (1242-1249): a `box` at the middle of the fuselage under the next free `volume n` name."""
    volumes = definition["volumes"]
    names = {volume["name"] for volume in volumes}
    name = next(f"volume {n}" for n in range(1, len(names) + 2) if f"volume {n}" not in names)
    volume = copy.deepcopy(NEW_VOLUME)
    volume["name"] = name
    volume["x_m"] = round(definition["fuselage"]["length_m"] / 2.0, 4)
    volumes.append(volume)


def delete_volume(definition: dict, index: int) -> None:
    """editor_view.delete_volume (1151-1167): the parts (and the battery) placed in it stay where they are, frozen at
    their resolved x, y, z (volume centre plus offset); nothing moves."""
    volume = _indexed(definition["volumes"], index, "volume")
    centre = (volume["x_m"], volume.get("y_m", 0.0), volume.get("z_m", 0.0))
    held = list(definition["parts"])
    if "propulsion" in definition:
        held.append(definition["propulsion"]["battery"])
    for item in held:
        if item.get("volume") != volume["name"]:
            continue
        position = [c + item.get(f"{axis}_offset_m", 0.0) for c, axis in zip(centre, AXES)]
        for key in PLACEMENT_KEYS:
            item.pop(key, None)
        item.update({f"{axis}_m": round(value, 6) for axis, value in zip(AXES, position)})
    definition["volumes"].pop(index)


def rename_volume(definition: dict, index: int, name: str) -> None:
    """editor_view.rename_volume (1169-1172): parts placed in it keep pointing at it under the new name."""
    if not isinstance(name, str):
        raise ValueError("volume.rename needs a string name")
    volume = _indexed(definition["volumes"], index, "volume")
    old = volume["name"]
    volume["name"] = name
    for part in definition["parts"]:
        if part.get("volume") == old:
            part["volume"] = name


def reset_volume_shape(volume: dict, shape: str) -> None:
    """editor_view.reset_volume_shape (1236-1240): every shape's fields go, this shape's come back at their defaults."""
    if shape not in VOLUME_SHAPES:
        raise ValueError(f"volume shape must be one of {VOLUME_SHAPES}, got {shape!r}")
    volume["shape"] = shape
    for fields in VOLUME_SHAPE_FIELDS.values():
        for key in fields:
            volume.pop(key, None)
    for key, value in VOLUME_SHAPE_FIELDS[shape].items():
        volume[key] = copy.deepcopy(value)


def add_part(definition: dict, parts_data: dict) -> None:
    """editor_view.add_part (1227-1234): `part n`, the first library entry that has a mass, into the first volume or at
    the middle of the fuselage."""
    names = {part["name"] for part in definition["parts"]}
    name = next(f"part {n}" for n in range(1, len(names) + 2) if f"part {n}" not in names)
    library = [f"{category}:{entry}" for category in PART_CATEGORIES for entry, data in sorted(parts_data[category].items()) if data.get("mass")]
    if not library:
        raise ValueError("no part in data/parts/ has a mass to place")
    place = {"volume": definition["volumes"][0]["name"]} if definition["volumes"] else {"x_m": round(definition["fuselage"]["length_m"] / 2.0, 4)}
    part = copy.deepcopy(NEW_PART)
    part["name"] = name
    part["part"] = library[0]
    part.update(place)
    definition["parts"].append(part)


def rename_part(definition: dict, index: int, name: str) -> None:
    """editor_view.rename_part (1222-1225): a propulsion ESC reference follows the part it names."""
    if not isinstance(name, str):
        raise ValueError("part.rename needs a string name")
    part = _indexed(definition["parts"], index, "part")
    old = part["name"]
    part["name"] = name
    propulsion = definition.get("propulsion")
    if propulsion is not None and propulsion["esc"] == old:
        propulsion["esc"] = name


def set_item_volume(state: AppState, item: dict, volume: str, placed_name: str) -> None:
    """editor_view.set_part_volume (1209-1220): into a volume it sits at its centre (offsets cleared); out of one
    (NO_VOLUME) it stays where it is, frozen at its resolved position. placed_name: the item among the resolved
    parts (the battery is BATTERY_PART_NAME)."""
    if not isinstance(volume, str) or (volume != NO_VOLUME and volume not in {entry["name"] for entry in state.definition["volumes"]}):
        raise ValueError(f"volume {volume!r} is not a defined volume")
    resolved = state.resolved
    placed = None if resolved is None else next((entry for entry in resolved.airframe.parts if entry.name == placed_name), None)
    for key in ("volume", "x_offset_m", "y_offset_m", "z_offset_m", "x_m", "y_m", "z_m"):
        item.pop(key, None)
    if volume != NO_VOLUME:
        item["volume"] = volume
    else:
        position = placed.position_m if placed is not None else (round(state.definition["fuselage"]["length_m"] / 2.0, 4), 0.0, 0.0)
        item.update(x_m=round(position[0], 4), y_m=round(position[1], 4), z_m=round(position[2], 4))


def set_part_volume(state: AppState, part: dict, volume: str) -> None:
    """editor_view.set_part_volume (1209-1220) for a placed part."""
    set_item_volume(state, part, volume, part["name"])


def apply_op(state: AppState, body: dict[str, Any]) -> None:
    """Run one structural edit on the session definition, then recompute and push the new state.

    A command from another session is refused before anything runs (SessionMismatch -> 409); unknown or
    malformed ops raise ValueError (the handler answers 400); flags are read strictly so a truthy string cannot
    count as the overwrite confirmation. The ported ops cover the Tk editors: the Airframe type flags (set_type_flag
    577-593), section cuts and presets as before, the nose/tail profile pick (reset_profile_params 849-850), and the
    surfaces, motor mounts, volumes and parts forms (editor_view.build_surfaces_tab 852-942, build_mounts_tab
    1021-1042, build_volumes_tab 1124-1172, build_parts_tab 1174-1234)."""
    check_session(state, body)
    op = body.get("op")
    definition = state.definition
    if op == "section.add_cut":
        add_cut(definition)
    elif op == "section.delete_cut":
        index = body.get("index")
        if type(index) is not int:
            raise ValueError("section.delete_cut needs an integer index")
        delete_cut(definition, index)
    # The nose/tail profile pick (build_fuselage_tab 696-698, reset_profile_params 849-850): profile and its params
    # are one mutation, so the definition never holds the old profile's params under the new profile.
    elif op == "fuselage.profile":
        reset_profile(definition, body.get("end"), body.get("profile"))
    elif op == "preset.load":
        name = body.get("name")
        if not isinstance(name, str):
            raise ValueError("preset.load needs a preset name")
        load_preset(definition, name)
    elif op == "preset.save":
        name = body.get("name")
        if not isinstance(name, str):
            raise ValueError("preset.save needs a preset name")
        save_preset_as(state, name, overwrite=body.get("overwrite") is True)
    elif op == "preset.new":
        name = body.get("name")
        if not isinstance(name, str):
            raise ValueError("preset.new needs a preset name")
        save_preset_as(state, name, overwrite=False)
    # Airframe type flags (set_type_flag 577-593): one atomic update of the flags and the propulsion block.
    elif op == "flags.set":
        set_type_flag(definition, body.get("key"), body.get("checked"), body.get("propulsion"))
    # Surfaces (build_surfaces_tab 852-942).
    elif op == "surface.add":
        add_surface(definition)
    elif op == "surface.delete":
        _indexed(definition["surfaces"], body.get("index"), "surface")
        definition["surfaces"].pop(body["index"])
    elif op == "surface.layout":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        layout = body.get("layout")
        if layout not in LAYOUTS:
            raise ValueError(f"layout must be one of {LAYOUTS}, got {layout!r}")
        surface["layout"] = layout
        fit_servos(definition, surface)
    elif op == "surface.control.toggle":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        toggle_control(definition, surface, enabled=body.get("enabled") is True)
    elif op == "surface.control.kind":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        if "control" not in surface:
            raise ValueError("the surface has no control surface to change")
        reset_control_kind(surface["control"], body.get("kind"))
    elif op == "surface.control.role":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        if "control" not in surface:
            raise ValueError("the surface has no control surface to change")
        toggle_role(surface["control"], body.get("role"), body.get("on") is True)
    elif op == "surface.spars.toggle":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        toggle_block(surface, "spars", preset_spars(definition) or NEW_SPARS, enabled=body.get("enabled") is True)
    elif op == "surface.carry_through.toggle":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        spars = surface.get("spars")
        if not isinstance(spars, dict):
            raise ValueError("add spars before a carry-through")
        toggle_block(spars, "carry_through", preset_carry_through(definition) or NEW_CARRY_THROUGH, enabled=body.get("enabled") is True)
    elif op == "surface.root_base.toggle":
        surface = _indexed(definition["surfaces"], body.get("index"), "surface")
        toggle_block(surface, "root_base", NEW_ROOT_BASE, enabled=body.get("enabled") is True)
    # Motor mounts (build_mounts_tab 1021-1042, normalize_mount 1100-1113, add_mount 1115-1118).
    elif op == "mount.add":
        add_mount(definition)
    elif op == "mount.delete":
        propulsion = definition.get("propulsion")
        if not isinstance(propulsion, dict):
            raise ValueError("the aircraft is not powered")
        _indexed(propulsion["mounts"], body.get("index"), "mount")
        propulsion["mounts"].pop(body["index"])
    elif op == "mount.on":
        propulsion = definition.get("propulsion")
        if not isinstance(propulsion, dict):
            raise ValueError("the aircraft is not powered")
        mount = _indexed(propulsion["mounts"], body.get("index"), "mount")
        on = body.get("on")
        if on not in MOUNT_TARGETS:
            raise ValueError(f"mount on must be one of {MOUNT_TARGETS}, got {on!r}")
        mount["on"] = on
        normalize_mount(definition, mount)
    elif op == "mount.pod.toggle":
        propulsion = definition.get("propulsion")
        if not isinstance(propulsion, dict):
            raise ValueError("the aircraft is not powered")
        mount = _indexed(propulsion["mounts"], body.get("index"), "mount")
        toggle_block(mount, "pod", NEW_POD, enabled=body.get("enabled") is True)
    # The propulsion battery's place (build_battery_group 1003 calls add_placement 1190-1207 with BATTERY_PART_NAME).
    elif op == "propulsion.battery.volume":
        propulsion = definition.get("propulsion")
        if not isinstance(propulsion, dict):
            raise ValueError("the aircraft is not powered")
        battery = propulsion.get("battery")
        if not isinstance(battery, dict):
            raise ValueError("the definition has no propulsion battery")
        set_item_volume(state, battery, body.get("volume"), BATTERY_PART_NAME)
    # Volumes (build_volumes_tab 1124-1172).
    elif op == "volume.add":
        add_volume(definition)
    elif op == "volume.delete":
        delete_volume(definition, body.get("index"))
    elif op == "volume.shape":
        volume = _indexed(definition["volumes"], body.get("index"), "volume")
        reset_volume_shape(volume, body.get("shape"))
    elif op == "volume.rename":
        rename_volume(definition, body.get("index"), body.get("name"))
    # Parts (build_parts_tab 1174-1234).
    elif op == "part.add":
        add_part(definition, state.parts_data)
    elif op == "part.delete":
        _indexed(definition["parts"], body.get("index"), "part")
        definition["parts"].pop(body["index"])
    elif op == "part.rename":
        rename_part(definition, body.get("index"), body.get("name"))
    elif op == "part.volume":
        part = _indexed(definition["parts"], body.get("index"), "part")
        set_part_volume(state, part, body.get("volume"))
    else:
        raise ValueError(f"unknown op {op!r}")
    sync_motor_runs(state.definition)  # editor_view.changed 389-390: the runs follow the motor count before the recompute
    state.set_definition(state.definition)  # editor_view.changed(rebuild=True): recompute and repaint


def _move_delta(body: dict[str, Any]) -> dict[str, float]:
    """The move body's delta as metres: absent axes stay absent, a present non-number is refused."""
    delta = body.get("delta")
    if delta is None:
        return {}
    if not isinstance(delta, dict):
        raise ValueError("delta must be an object with optional x, y and z metres")
    values: dict[str, float] = {}
    for axis in AXES:
        if axis not in delta or delta[axis] is None:
            continue
        value = delta[axis]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"delta.{axis} must be a number of metres")
        values[axis] = float(value)
    return values


def _move_placement(item: dict, dx: float, dy: float, dz: float, what: str) -> bool:
    """Move a free placement by its own x_m/y_m/z_m (absent y_m/z_m start at 0.0, as the drawing uses them).

    True when it moved, False for an all-zero delta. A placement inside a volume is not movable through its owner:
    the caller drags the volume, so this is an unknown path (ValueError -> 400)."""
    if "volume" in item:
        raise ValueError(f"{what} sits in a volume; drag the volume instead")
    if dx == 0.0 and dy == 0.0 and dz == 0.0:
        return False
    if "x_m" not in item:
        raise ValueError(f"{what} has no own position to move")
    if dx:
        item["x_m"] = _quantize(item["x_m"] + dx)
    if dy:
        item["y_m"] = _quantize(item.get("y_m", 0.0) + dy)
    if dz:
        item["z_m"] = _quantize(item.get("z_m", 0.0) + dz)
    return True


def apply_move(state: AppState, body: dict[str, Any], notify: bool = True) -> None:
    """One drag motion on the session definition (POST /api/editor/move), the port of on_view_drag_motion 363-373.

    The delta is in metres (the client divided the screen pixels by metrics["scale"]) rather than screen pixels.
    target and index name the movable element; unknown ones raise ValueError before anything is mutated (the
    handler answers 400 with a Log entry). A delta with no component for the target is a no-op. An applied move
    recomputes geometry only and pushes, exactly as the Tk motion does; the release runs the full /api/recompute.
    notify=False skips the websocket push for the fast drag path, which answers with the drawing instead.
    """
    check_session(state, body)
    target = body.get("target")
    if target not in MOVE_TARGETS:
        raise ValueError(f"unknown move target {target!r}")
    index = body.get("index")
    if type(index) is not int:
        raise ValueError("move needs an integer index")
    delta = _move_delta(body)
    dx, dy, dz = delta.get("x", 0.0), delta.get("y", 0.0), delta.get("z", 0.0)
    definition = state.definition
    if target == "cut":
        cuts = definition["sections"]["cuts_m"]
        _indexed(cuts, index, "cut")
        if dx == 0.0:
            return
        # on_view_drag_motion 366-371: max(0, ...) then the section-length clamp; the first cut's lower bound is 0.0.
        # 0.01 mm quantization: the bound wins when quantizing would cross it.
        value = max(0.0, cuts[index] + dx)
        low = cuts[index - 1] if index > 0 else 0.0
        high = cuts[index + 1] if index + 1 < len(cuts) else definition["fuselage"]["length_m"]
        cuts[index] = min(max(_quantize(value), low + MIN_SECTION_LENGTH_M), high - MIN_SECTION_LENGTH_M)
    elif target == "surface":
        surface = _indexed(definition["surfaces"], index, "surface")
        if "x_root_le_m" not in surface:
            raise ValueError(f"surface {index!r} has no x_root_le_m")
        if dx == 0.0:
            return
        surface["x_root_le_m"] = _quantize(max(0.0, surface["x_root_le_m"] + dx))
    elif target == "spars":
        surface = _indexed(definition["surfaces"], index, "surface")
        spars = surface.get("spars")
        if not isinstance(spars, dict) or "x_offset_m" not in spars:
            raise ValueError(f"surface {index!r} has no spars to move")
        if dx == 0.0:
            return
        spars["x_offset_m"] = _quantize(spars["x_offset_m"] + dx)
    elif target == "volume":
        volume = _indexed(definition["volumes"], index, "volume")
        if "x_m" not in volume:
            raise ValueError(f"volume {index!r} has no x_m")
        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return
        if dx:
            volume["x_m"] = _quantize(max(0.0, volume["x_m"] + dx))
        if dy:
            volume["y_m"] = _quantize(volume.get("y_m", 0.0) + dy)
        if dz:
            volume["z_m"] = _quantize(volume.get("z_m", 0.0) + dz)
    elif target == "part":
        part = _indexed(definition["parts"], index, "part")
        if not _move_placement(part, dx, dy, dz, f"part {part.get('name', index)!r}"):
            return
    elif target == "battery":
        propulsion = definition.get("propulsion")
        battery = propulsion.get("battery") if isinstance(propulsion, dict) else None
        if not isinstance(battery, dict):
            raise ValueError("the definition has no propulsion battery")
        if not _move_placement(battery, dx, dy, dz, "battery"):
            return
    else:  # rod
        reinforcement = definition["fuselage"].get("reinforcement")
        if not isinstance(reinforcement, dict):
            raise ValueError("the fuselage has no reinforcement to move")
        if dx == 0.0:
            return
        span = reinforcement["to_x_m"] - reinforcement["from_x_m"]
        length = definition["fuselage"]["length_m"]
        from_x = min(max(_quantize(reinforcement["from_x_m"] + dx), 0.0), max(length - span, 0.0))
        reinforcement["from_x_m"] = from_x
        reinforcement["to_x_m"] = min(_quantize(from_x + span), length)
    state.set_definition(state.definition, run_aero=False, notify=notify)  # on_view_drag_motion 372: geometry only; the release does the full recompute. The revision bump lets the 3D view refresh when shown.
