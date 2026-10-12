"""Log-clean assertions for the browser editor's structural ops and field sets.

Run: /home/anon/miniconda3/envs/drone/bin/python webapp/test_editor_sync.py

The ops run on a throwaway copy of aircraft/example, in process, against AppState exactly as webapp/server.py
calls them. Every accepted edit must leave definition_valid true with no new "Definition error" (or unexpected
"Error") entry in the Log, and propulsion.wiring.motors must match the motor count the parser requires.
"""

from __future__ import annotations

import copy
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from libs.airframe import AirframeError, blank_definition, motor_count, parse_airframe  # noqa: E402
from libs.geometry import body_section_at, resolve_airframe  # noqa: E402
from libs.project import Project  # noqa: E402
from webapp import editor  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"


def make_state(output: Path) -> AppState:
    """AppState on a persistent validation definition from the example, recompute without the aero thread."""
    root = output / "example"
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(EXAMPLE / "aircraft.json", root / "aircraft.json")
    state = AppState(Project(root=root))
    original = state.recompute
    state.recompute = lambda run_aero=True: original(run_aero=False)
    state.recompute()
    return state


def expected_motors(definition: dict) -> int:
    """libs.airframe.motor_count over the definition's mounts and surface layouts."""
    layouts = {surface["name"]: surface["layout"] for surface in definition["surfaces"]}
    mounts = [(mount["on"], mount.get("surface")) for mount in definition["propulsion"]["mounts"]]
    return motor_count(mounts, layouts)


def assert_valid_clean(state: AppState, before: int) -> None:
    new = state.log[before:]
    assert state.definition_valid, f"definition became invalid: {[(e.title, e.text) for e in new]}"
    bad = [entry for entry in new if entry.title in ("Definition error", "Error")]
    assert not bad, [f"{entry.title}: {entry.text}" for entry in bad]


def run_op(state: AppState, body: dict) -> None:
    before = len(state.log)
    editor.apply_op(state, {"session_id": state.session_id, **body})
    assert_valid_clean(state, before)


def run_set(state: AppState, path: list, value: object) -> None:
    before = len(state.log)
    editor.apply_set(state, {"session_id": state.session_id, "path": path, "value": value})
    assert_valid_clean(state, before)


def check_motor_sync(state: AppState) -> None:
    runs = state.definition["propulsion"]["wiring"]["motors"]
    assert len(runs) == expected_motors(state.definition) == len(state.resolved.motors) == 1

    run_op(state, {"op": "mount.add"})
    assert len(runs) == expected_motors(state.definition) == 2, "mount.add lost the new run"
    run_op(state, {"op": "mount.delete", "index": 1})
    assert len(runs) == expected_motors(state.definition) == 1, "mount.delete kept the removed run"

    run_op(state, {"op": "mount.on", "index": 0, "on": "surface"})
    assert state.definition["propulsion"]["mounts"][0]["surface"] == "main-wing"
    assert len(runs) == expected_motors(state.definition) == len(state.resolved.motors) == 2, "mount.on (H) did not gain its panel runs"
    # On the mounted surface, even layouts exercise the sync: H -> X -> H is 2 -> 4 -> 2 runs, all valid.
    run_op(state, {"op": "surface.layout", "index": 0, "layout": "X"})
    assert len(runs) == expected_motors(state.definition) == len(state.resolved.motors) == 4, "surface.layout H->X did not gain the panel runs"
    run_op(state, {"op": "surface.layout", "index": 0, "layout": "H"})
    assert len(runs) == expected_motors(state.definition) == len(state.resolved.motors) == 2, "surface.layout X->H did not drop the panel runs"
    # The literal H -> 1 -> H on an unmounted surface (elevators) is log-clean and leaves the motor count alone.
    run_op(state, {"op": "surface.layout", "index": 1, "layout": "1"})
    run_op(state, {"op": "surface.layout", "index": 1, "layout": "H"})
    assert len(runs) == expected_motors(state.definition) == 2
    # A surface mount needs an even motor count (libs/geometry.py:999-1000), so layout 1 on the MOUNTED surface
    # is a real geometry error, not a sync artefact: the runs must still be synced to 1 before that recompute.
    before = len(state.log)
    editor.apply_op(state, {"session_id": state.session_id, "op": "surface.layout", "index": 0, "layout": "1"})
    assert len(runs) == expected_motors(state.definition) == 1, "surface.layout H->1 did not drop the panel runs"
    assert not state.definition_valid and state.log[before].title == "Definition error" and "cannot spin in pairs" in state.log[before].text
    run_op(state, {"op": "surface.layout", "index": 0, "layout": "H"})
    assert len(runs) == expected_motors(state.definition) == len(state.resolved.motors) == 2, "surface.layout 1->H did not restore the panel runs"
    print("ok   mount.add / mount.delete / mount.on / surface.layout: runs follow the motor count, no spurious Log error")


def check_apply_set_syncs(state: AppState) -> None:
    """A set on any field syncs the runs before the recompute, as Tk's changed() does on every edit."""
    runs = state.definition["propulsion"]["wiring"]["motors"]
    del runs[1:]  # the mismatch the old client-side sync left behind between snapshot and recompute
    run_set(state, ["fuselage", "length_m"], state.definition["fuselage"]["length_m"])
    assert len(runs) == expected_motors(state.definition) == 2, "apply_set did not sync the runs before the recompute"
    print("ok   apply_set syncs wiring.motors before the recompute")


def check_profile(state: AppState) -> None:
    defaults = editor.PROFILE_PARAM_DEFAULTS
    run_op(state, {"op": "fuselage.profile", "end": "nose", "profile": "parabolic"})
    assert state.definition["fuselage"]["nose"]["profile"] == "parabolic"
    assert state.definition["fuselage"]["nose"]["params"] == {"k_prime": defaults["k_prime"]}
    run_op(state, {"op": "fuselage.profile", "end": "nose", "profile": "elliptical"})
    assert state.definition["fuselage"]["nose"]["params"] == {}
    run_op(state, {"op": "fuselage.profile", "end": "nose", "profile": "spherically_blunted_conic"})
    assert state.definition["fuselage"]["nose"]["params"] == {"nose_radius": defaults["nose_radius"]}
    run_op(state, {"op": "fuselage.profile", "end": "nose", "profile": "parabolic"})
    assert state.definition["fuselage"]["nose"]["params"] == {"k_prime": defaults["k_prime"]}
    print("ok   fuselage.profile: one mutation per switch, no Log error (elliptical -> parabolic -> back)")


def check_missing_surface_guard(state: AppState) -> None:
    """Port of Tk's skip guard: a mount naming a missing surface leaves the runs untouched, invents none."""
    definition = copy.deepcopy(state.definition)
    definition["propulsion"]["mounts"][0] = {
        "on": "surface", "surface": "ghost", "span_position_m": 0.0, "x_offset_m": 0.0,
        "spin": "in", "thrust_axis_tilt_deg": 0.0,
    }
    runs = definition["propulsion"]["wiring"]["motors"]
    before = copy.deepcopy(runs)
    editor.sync_motor_runs(definition)
    assert runs == before, "sync_motor_runs invented runs for a missing surface"
    definition["propulsion"]["mounts"][0]["surface"] = "main-wing"
    editor.sync_motor_runs(definition)
    assert len(runs) == 2
    print("ok   missing-surface skip guard preserves wiring.motors (no invented runs)")


def check_resizing(state: AppState) -> None:
    """Exercise the same queued editor mutations against real parsed/resolved aircraft geometry."""
    run_set(state, ["propulsion", "battery", "x_offset_m"], 0.02)
    original = copy.deepcopy(state.definition)
    length = original["fuselage"]["length_m"]
    battery_x = state.resolved.airframe.propulsion.battery.placed.position_m[0]
    run_set(state, ["fuselage", "length_m"], length * 1.5)
    enlarged = state.definition
    for kind, key in (("parts", "x_m"), ("volumes", "x_m"), ("surfaces", "x_root_le_m")):
        for old, new in zip(original[kind], enlarged[kind]):
            assert math.isclose(new[key], old[key] * 1.5)
    assert enlarged["surfaces"][0]["chord_root_m"] == original["surfaces"][0]["chord_root_m"]
    assert enlarged["volumes"][0]["size_m"] == original["volumes"][0]["size_m"]
    assert math.isclose(state.resolved.airframe.propulsion.battery.placed.position_m[0], battery_x * 1.5)
    run_set(state, ["fuselage", "length_m"], length)

    run_set(state, ["sections", "per_section", 0, "fixed_length"], True)
    fixed_end = state.definition["sections"]["cuts_m"][0]
    run_set(state, ["fuselage", "length_m"], length * 1.2)
    assert math.isclose(state.definition["sections"]["cuts_m"][0], fixed_end)
    run_set(state, ["fuselage", "length_m"], length)
    before = copy.deepcopy(state.definition)
    cuts = before["sections"]["cuts_m"]
    run_op(state, {"op": "section.length", "index": 1, "value": (cuts[1] - cuts[0]) * 1.2})
    assert math.isclose(state.definition["fuselage"]["length_m"], length)
    assert state.definition["sections"]["cuts_m"][0] == cuts[0]
    assert state.definition["sections"]["cuts_m"][1] > cuts[1]
    assert state.definition["parts"][1] == before["parts"][1], "part ahead of edit moved"
    assert state.definition["parts"] == before["parts"], "cut edit moved components"
    assert state.definition["fuselage"] == before["fuselage"], "cut edit changed the fuselage"
    run_op(state, {"op": "section.length", "index": len(cuts), "value": 0.1})
    assert math.isclose(state.definition["fuselage"]["length_m"] - state.definition["sections"]["cuts_m"][-1], 0.1)
    assert math.isclose(state.definition["fuselage"]["length_m"], length)
    refused = copy.deepcopy(state.definition)
    for body in ({"op": "section.length", "index": 1, "value": length * 2},
                 {"op": "section.length", "index": len(cuts), "value": 0}):
        try:
            editor.apply_op(state, {"session_id": state.session_id, **body})
        except ValueError:
            pass
        else:
            raise AssertionError(f"impossible resize accepted: {body}")
        assert state.definition == refused, "rejected resize changed geometry"
    print("ok   airframe/section resizing: resolved placements, physical sizes, fixed lengths and atomic refusal")


def check_section_loft(state: AppState) -> None:
    original = copy.deepcopy(state.definition)
    row_path = ["fuselage", "loft_stations", 2]
    run_set(state, [*row_path, "width_m"], 0.09)
    run_set(state, [*row_path, "height_m"], 0.075)
    run_set(state, [*row_path, "shape"], 4.0)
    body = state.resolved.body
    x = state.definition["fuselage"]["loft_stations"][2]["x_m"]
    local = body_section_at(body, x)
    assert (local.width_m, local.height_m, local.shape) == (0.09, 0.075, 4.0)
    step = 1e-6
    left, right = body_section_at(body, x - step), body_section_at(body, x + step)
    assert abs(local.width_m - left.width_m) / step < 1e-4
    assert abs(local.width_m - right.width_m) / step < 1e-4
    assert abs(local.shape - left.shape) / step < 1e-3
    assert abs(local.shape - right.shape) / step < 1e-3
    assert not any("inner width" in problem or "inner height" in problem for problem in state.problems)
    # Cut removal, insertion, motion and length editing must leave the complete body identical.
    guides = copy.deepcopy(state.definition["fuselage"]["loft_stations"])
    editor.apply_set(state, {"session_id": state.session_id, "path": ["sections", "per_section", 3, "empty_mass_kg"], "remove": True})
    run_op(state, {"op": "section.delete_cut", "index": 2})
    assert state.resolved.body == body
    assert state.definition["fuselage"]["loft_stations"] == guides
    run_op(state, {"op": "section.add_cut", "index": 2})
    assert state.resolved.body == body
    editor.apply_move(state, {"session_id": state.session_id, "target": "cut", "index": 2, "delta": {"x": 0.002}})
    assert state.resolved.body == body
    run_op(state, {"op": "section.length", "index": 2, "value": 0.1})
    assert state.resolved.body == body
    cuts = copy.deepcopy(state.definition["sections"])
    run_op(state, {"op": "loft.add", "x_m": x + 0.001})
    assert state.definition["sections"] == cuts
    added = next(i for i, guide in enumerate(state.definition["fuselage"]["loft_stations"]) if guide["x_m"] == x + 0.001)
    run_op(state, {"op": "loft.position", "index": added, "x_m": 0.1})
    assert state.definition["sections"] == cuts
    moved = next(i for i, guide in enumerate(state.definition["fuselage"]["loft_stations"]) if guide["x_m"] == 0.1)
    run_op(state, {"op": "loft.delete", "index": moved})
    assert state.resolved.body == body
    state.save()
    restored = AppState(state.project)
    restored.recompute(run_aero=False)
    assert restored.definition_valid
    assert restored.resolved.body == body
    editor.apply_set(state, {"session_id": state.session_id, "path": [*row_path, "width_m"], "remove": True})
    assert math.isclose(body_section_at(state.resolved.body, x).width_m, original["fuselage"]["section"]["width_m"])
    for invalid in ({"width_m": 0.0}, {"height_m": 0.001}, {"shape": 0.9}, {"shape": None}, {"width": 0.1}):
        definition = copy.deepcopy(original)
        definition["fuselage"]["loft_stations"][2] = {"x_m": x, **invalid}
        try:
            parse_airframe(definition, state.parts_data)
        except AirframeError:
            pass
        else:
            raise AssertionError(f"invalid loft override accepted: {invalid}")
    state.set_definition(original)
    state.save()
    print("ok   independent loft guides/cuts: exact profiles, smooth slopes, cut edits preserve the whole body, guide add/move/delete preserves cuts, inheritance and save/load")


def check_section_insertion(state: AppState) -> None:
    original = copy.deepcopy(state.definition)
    for side in editor.SECTION_INSERT_SIDES:
        for index in (0, 2, len(original["sections"]["per_section"]) - 1):
            definition = copy.deepcopy(original)
            row = definition["sections"]["per_section"][index]
            row.pop("empty_mass_kg", None)
            row.pop("complete_mass_kg", None)
            state.set_definition(definition)
            before = copy.deepcopy(state.definition)
            body = state.resolved.body
            bounds = [0.0, *before["sections"]["cuts_m"], before["fuselage"]["length_m"]]
            run_op(state, {"op": "section.add_cut", "index": index, "side": side})
            added = index if side == "before" else index + 1
            assert len(state.definition["sections"]["per_section"]) == len(before["sections"]["per_section"]) + 1
            assert state.definition["sections"]["cuts_m"][index] == (bounds[index] + bounds[index + 1]) / 2.0
            assert state.definition["sections"]["per_section"][added] == row
            remaining = copy.deepcopy(state.definition["sections"]["per_section"])
            remaining.pop(added)
            assert remaining == before["sections"]["per_section"]
            assert state.resolved.body == body
            for key in before.keys() - {"sections"}:
                assert state.definition[key] == before[key]
    state.set_definition(copy.deepcopy(original))
    before = copy.deepcopy(state.definition)
    for command in ({"op": "section.add_cut", "index": 0, "side": "before"},
                    {"op": "section.add_cut", "index": 2, "side": "unknown"}):
        try:
            editor.apply_op(state, {"session_id": state.session_id, **command})
        except ValueError:
            pass
        else:
            raise AssertionError(f"invalid or measured section split accepted: {command}")
        assert state.definition == before
    print("ok   section insertion before/after at first, middle and last spans: body and existing data preserved; invalid/measured splits refused atomically")


def check_local_end_radius(state: AppState) -> None:
    definition = blank_definition("local-end-radius")
    definition["fuselage"]["section"]["width_m"] = 0.08
    definition["fuselage"]["nose"] = {"profile": "tangent_ogive", "length_m": 0.02, "params": {}}
    definition["fuselage"]["loft_stations"] = [{"x_m": 0.1, "width_m": 0.03}]
    frame = parse_airframe(definition, state.parts_data)
    resolved, _ = resolve_airframe(frame, state.parts_data)
    assert math.isclose(body_section_at(resolved.body, 0.02).width_m, 0.03)
    print("ok   end profile validates against the actual loft shoulder, independently of default width")


def check_surface_sections(state: AppState) -> None:
    # Start unsplit: the bundled loft example already has wing print cuts.
    while state.definition["surfaces"][0]["sections"]["cuts_m"]:
        run_op(state, {"op": "surface.section.delete", "index": 0, "section_index": 0})
    panels = state.resolved.surfaces[0].panels
    controls = state.resolved.controls
    fuselage_cuts = copy.deepcopy(state.definition["sections"])
    run_op(state, {"op": "surface.section.add", "index": 0, "section_index": 0, "side": "after"})
    run_op(state, {"op": "surface.section.add", "index": 0, "section_index": 1, "side": "before"})
    surface = state.definition["surfaces"][0]
    cuts = surface["sections"]["cuts_m"]
    span = surface["span_m"]
    assert len(cuts) == 2 and math.isclose(cuts[0], editor.NEW_SURFACE_ROOT_LENGTH_M) and cuts[0] < cuts[1] < span
    # A cut boundary is a plain set: it must move only that cut and leave the span and the aero alone.
    run_set(state, ["surfaces", 0, "sections", "cuts_m", 1], cuts[0] + 0.04)
    assert math.isclose(cuts[1] - cuts[0], 0.04)
    assert math.isclose(surface["span_m"], span), "a cut edit changed the wing span"
    assert state.resolved.surfaces[0].panels == panels and state.resolved.controls == controls
    # An edit that crosses its neighbouring cut fails visibly (definition error), like the fuselage boundaries.
    before = copy.deepcopy(state.definition)
    editor.apply_set(state, {"session_id": state.session_id, "path": ["surfaces", 0, "sections", "cuts_m", 1], "value": 0.001})
    assert not state.definition_valid
    state.set_definition(before)
    surface = state.definition["surfaces"][0]
    cuts = surface["sections"]["cuts_m"]
    # The span is the last section's End: editing it moves the wing tip and never a cut.
    run_set(state, ["surfaces", 0, "span_m"], span - 0.02)
    assert surface["sections"]["cuts_m"] == cuts, "a span edit moved a cut"
    assert math.isclose(surface["span_m"], span - 0.02)
    assert state.resolved.surfaces[0].panels != panels and state.resolved.controls != controls
    assert state.definition["sections"] == fuselage_cuts
    state.save()
    reloaded = parse_airframe(state.definition, state.parts_data)
    assert reloaded.surfaces[0].sections.cuts_m == tuple(cuts)
    run_op(state, {"op": "surface.section.delete", "index": 0, "section_index": 1})
    run_op(state, {"op": "surface.section.delete", "index": 0, "section_index": 0})
    assert surface["sections"]["cuts_m"] == []
    print("ok   surface print sections: cuts add/remove, boundary edits keep cuts and span independent, crossing refused visibly, round trip and fuselage cuts preserved")


def main() -> None:
    output = EXAMPLE / "cad" / "editor-validation"
    state = make_state(output)
    assert state.definition_valid and not state.log, "the example project did not start clean"
    check_motor_sync(state)
    check_apply_set_syncs(state)
    check_profile(state)
    check_missing_surface_guard(state)
    check_resizing(state)
    check_section_loft(state)
    check_local_end_radius(state)
    check_section_insertion(state)
    check_surface_sections(make_state(output / "surface-sections"))
    print("editor sync suite OK")


if __name__ == "__main__":
    main()
