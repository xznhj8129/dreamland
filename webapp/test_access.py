"""Headless integration for the saved Access schema, its resolved frames and the editor operations.

Run: uv run python -m webapp.test_access

Exercises the real path in process, exactly as webapp/server.py calls it: airframe/2 parse and resolve, the editor's
queued set/op handlers (add, rename, kind/profile/extent/radial switches), resizing through the station map, and save
plus reload, on a throwaway copy of aircraft/example. The physical cut and retained hatch geometry are shared with CAD and the views.
"""

from __future__ import annotations

import copy
import json
import math
import shutil
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent  # aircraft/example, run as `uv run python -m webapp.test_access`

from libs.airframe import AirframeError, load_parts_data, parse_airframe  # noqa: E402
from libs.geometry import body_section_at, resolve_airframe  # noqa: E402
from libs.project import Project  # noqa: E402
from webapp import editor  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"


def make_state(output: Path) -> AppState:
    """AppState on a persistent validation definition from the example, without the aero thread."""
    root = output / "example"
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(EXAMPLE / "aircraft.json", root / "aircraft.json")
    state = AppState(Project(root=root))
    original = state.recompute
    state.recompute = lambda run_aero=True: original(run_aero=False)
    state.recompute()
    assert state.definition_valid
    return state


def run(state: AppState, body: dict) -> None:
    editor.apply_op(state, {"session_id": state.session_id, **body})
    assert state.definition_valid, [(entry.title, entry.text) for entry in state.log[-2:]]


def set_path(state: AppState, path: list, value: object) -> None:
    editor.apply_set(state, {"session_id": state.session_id, "path": path, "value": value})
    assert state.definition_valid, [(entry.title, entry.text) for entry in state.log[-2:]]


def valid_entry(**changes: object) -> dict:
    entry = {"name": "Vent", "kind": "hole", "profile": "slot", "width_m": 0.01, "length_m": 0.02, "x_m": 0.2, "extent": "wall"}
    entry.update(changes)
    return entry


def rejected(parts_data: dict, entry: dict, what: str) -> None:
    data = json.loads((EXAMPLE / "aircraft.json").read_text())
    data["access"] = [entry]
    try:
        parse_airframe(data, parts_data, path="access-check")
    except AirframeError:
        return
    raise AssertionError(f"{what}: accepted an invalid access entry")


def check_schema(parts_data: dict) -> None:
    """Missing access, entry or radial values, unknown fields and impossible profiles fail visibly."""
    data = json.loads((EXAMPLE / "aircraft.json").read_text())
    del data["access"]
    try:
        parse_airframe(data, parts_data, path="no-access")
        raise AssertionError("a definition without access[] parsed")
    except AirframeError as error:
        assert "access" in str(error), error
    for wrong in ({}, None, "Hole 1"):
        data = json.loads((EXAMPLE / "aircraft.json").read_text())
        data["access"] = wrong
        try:
            parse_airframe(data, parts_data, path="bad-access-shape")
            raise AssertionError(f"access = {wrong!r} parsed")
        except AirframeError as error:
            assert "list" in str(error), error
    rejected(parts_data, valid_entry(length_m=0.005), "slot length < width")
    rejected(parts_data, valid_entry(profile="rounded_rectangle"), "rounded rectangle without corner_radius_m")
    rejected(parts_data, valid_entry(profile="rounded_rectangle", corner_radius_m=0.006), "corner radius over half the extent")
    rejected(parts_data, valid_entry(corner_radius_m=0.003), "corner_radius_m on a slot")
    rejected(parts_data, valid_entry(kind="hatch"), "hatch without gap_m")
    rejected(parts_data, valid_entry(gap_m=0.0003), "gap_m on a hole")
    rejected(parts_data, valid_entry(kind="hatch", gap_m=0.005), "gap with no retained material")
    rejected(parts_data, valid_entry(extent="depth"), "depth extent without depth_m")
    rejected(parts_data, valid_entry(depth_m=0.002), "depth_m on a wall extent")
    rejected(parts_data, valid_entry(extent="depth", depth_m=-0.001), "non-positive depth_m")
    rejected(parts_data, valid_entry(radial={"count": 0, "axis": "x", "x_m": 0.2, "y_m": 0.0, "z_m": 0.0}), "radial count < 1")
    rejected(parts_data, valid_entry(radial={"count": 8, "axis": "x", "x_m": 0.2, "y_m": 0.0}), "radial point without z_m")
    rejected(parts_data, valid_entry(radial={"count": 8.5, "axis": "x", "x_m": 0.2, "y_m": 0.0, "z_m": 0.0}), "fractional radial count")
    rejected(parts_data, valid_entry(slot_depth_m=0.002), "unknown entry field")
    rejected(parts_data, valid_entry(radial={"count": 8, "axis": "x", "x_m": 0.2, "y_m": 0.0, "z_m": 0.0, "radius_m": 0.1}), "unknown radial field")
    second = valid_entry(name="Vent")
    data["access"] = [valid_entry(), second]
    try:
        parse_airframe(data, parts_data, path="duplicates")
        raise AssertionError("duplicate access names parsed")
    except AirframeError:
        pass
    print("ok   access schema: missing/unknown fields and bad values fail visibly")


def main() -> None:
    parts_data = load_parts_data()
    check_schema(parts_data)
    with tempfile.TemporaryDirectory() as folder:
        state = make_state(Path(folder))
        state.set_definition({**state.definition, "access": []}, run_aero=False)  # the example may carry its own entries
        baseline_mass = state.resolved.mass_kg
        assert state.resolved.access == ()

        run(state, {"op": "access.add"})
        entry = state.definition["access"][0]
        x_mid = state.definition["fuselage"]["length_m"] / 2.0
        station = body_section_at(state.resolved.body, x_mid)
        assert (entry["name"], entry["kind"], entry["profile"], entry["extent"]) == ("Hole 1", "hole", "slot", "wall")
        assert math.isclose(entry["x_m"], x_mid) and entry["y_m"] == 0.0 and math.isclose(entry["z_m"], -station.height_m / 2.0)
        seed = state.resolved.access[0]
        assert (seed.instance_id, seed.index, seed.copy, seed.count) == ("Hole 1", 0, 0, 1)
        assert seed.cut_direction == (0.0, 0.0, 1.0), "the lower-skin seed must cut inward (+z)"
        assert state.resolved.mass_kg == baseline_mass, "live access edit ran physical mass operations"
        print("ok   access.add: lower-skin seed at mid-body, frames resolved without CAD")

        set_path(state, ["access", 0, "name"], "Vent")
        run(state, {"op": "access.kind", "index": 0, "kind": "hatch"})
        assert entry["name"] == "Vent" and entry["gap_m"] == editor.NEW_ACCESS_GAP_M
        run(state, {"op": "access.profile", "index": 0, "profile": "rounded_rectangle"})
        assert entry["corner_radius_m"] == editor.NEW_ACCESS_CORNER_RADIUS_M and entry["width_m"] == editor.NEW_ACCESS["width_m"]
        run(state, {"op": "access.extent", "index": 0, "extent": "depth"})
        assert entry["depth_m"] == editor.NEW_ACCESS_DEPTH_M
        set_path(state, ["access", 0, "width_m"], 0.03)
        set_path(state, ["access", 0, "length_m"], 0.04)
        run(state, {"op": "access.radial", "index": 0, "enabled": True})
        radial = entry["radial"]
        assert radial == {"count": 8, "axis": "x", "x_m": entry["x_m"], "y_m": entry["y_m"], "z_m": entry["z_m"]}
        print("ok   kind/profile/extent/radial switches add exactly their fields and keep existing values")

        frames = state.resolved.access
        ids = [frame.instance_id for frame in frames]
        assert len(frames) == 8 and len(set(ids)) == 8 and ids[0] == "Vent 1/8" and ids[1] == "Vent 2/8"
        assert all(frame.index == 0 for frame in frames)
        axis_point = (radial["x_m"], radial["y_m"], radial["z_m"])
        seed_center, turned = frames[0].center, frames[2].center
        relative = (seed_center[1] - axis_point[1], seed_center[2] - axis_point[2])
        expected = (seed_center[0], axis_point[1] - relative[1], axis_point[2] + relative[0])  # 90 deg about +x
        assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(turned, expected)), (turned, expected)
        assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(frames[2].cut_direction, (0.0, -1.0, 0.0))), (frames[2].instance_id, frames[2].cut_direction)
        state.recompute(run_aero=False)
        assert [frame.instance_id for frame in state.resolved.access] == ids, "instance IDs are not deterministic"
        print("ok   count-8 radial pattern: a full circle of frames with turned axes, deterministic IDs")

        # The pattern line may sit off the seed: a nonzero perpendicular axis-point component rotates about it.
        axis_y, axis_z = radial["y_m"] + 0.005, radial["z_m"] - 0.010
        set_path(state, ["access", 0, "radial", "y_m"], axis_y)
        set_path(state, ["access", 0, "radial", "z_m"], axis_z)
        turned_frames = state.resolved.access
        seed_centre = turned_frames[0].center
        expected_90 = (seed_centre[0], axis_y - (seed_centre[2] - axis_z), axis_z + (seed_centre[1] - axis_y))
        assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(turned_frames[2].center, expected_90))
        radius = math.hypot(seed_centre[1] - axis_y, seed_centre[2] - axis_z)
        for frame in turned_frames:
            assert math.isclose(math.hypot(frame.center[1] - axis_y, frame.center[2] - axis_z), radius, rel_tol=1e-9)
        print("ok   off-axis radial point: every copy keeps its distance to the line and the 90 deg copy turns about it")

        run(state, {"op": "access.extent", "index": 0, "extent": "wall"})
        run(state, {"op": "access.profile", "index": 0, "profile": "slot"})
        run(state, {"op": "access.kind", "index": 0, "kind": "hole"})
        assert "depth_m" not in entry and "corner_radius_m" not in entry and "gap_m" not in entry
        set_path(state, ["access", 0, "radial", "count"], 4)
        set_path(state, ["access", 0, "radial", "axis"], "z")
        assert len(state.resolved.access) == 4 and state.resolved.access[1].count == 4
        print("ok   switching back drops only the inapplicable fields; the pattern follows its own values")

        run(state, {"op": "access.add", "kind": "hatch"})
        assert state.definition["access"][1]["name"] == "Hatch 2" and "gap_m" in state.definition["access"][1]
        run(state, {"op": "access.delete", "index": 1})
        assert len(state.definition["access"]) == 1
        print("ok   Add Hatch and delete: kind defaults, unique names, clean removal")

        before = copy.deepcopy(state.definition["access"][0])
        length = state.definition["fuselage"]["length_m"]
        set_path(state, ["fuselage", "length_m"], length * 1.5)
        after = state.definition["access"][0]
        assert math.isclose(after["x_m"], before["x_m"] * 1.5, rel_tol=1e-9)
        assert math.isclose(after["radial"]["x_m"], before["radial"]["x_m"] * 1.5, rel_tol=1e-9)
        for key in ("width_m", "length_m", "roll_deg", "pitch_deg", "yaw_deg"):
            assert after.get(key) == before.get(key), key
        assert (after["radial"]["count"], after["radial"]["axis"]) == (4, "z")
        print("ok   resizing maps seed and pattern axis x; dimensions and angles stay physical")

        state.save()
        saved = json.loads(state.project.build_path.read_text())
        assert saved["access"] == state.definition["access"]
        reloaded, _ = resolve_airframe(parse_airframe(saved, parts_data, path="reload"), parts_data)
        assert [frame.instance_id for frame in reloaded.access] == [frame.instance_id for frame in state.resolved.access]
        assert reloaded.mass_kg == state.resolved.mass_kg and reloaded.cg_m == state.resolved.cg_m
        print("ok   save/reload: access[] and its mass properties round-trip")


if __name__ == "__main__":
    main()
