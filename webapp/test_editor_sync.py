"""Log-clean assertions for the browser editor's structural ops and field sets.

Run: /home/anon/miniconda3/envs/drone/bin/python webapp/test_editor_sync.py

The ops run on a throwaway copy of aircraft/example, in process, against AppState exactly as webapp/server.py
calls them. Every accepted edit must leave definition_valid true with no new "Definition error" (or unexpected
"Error") entry in the Log, and propulsion.wiring.motors must match the motor count the parser requires.
"""

from __future__ import annotations

import copy
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from libs.airframe import motor_count  # noqa: E402
from libs.project import Project  # noqa: E402
from webapp import editor  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"


def make_state(tmp: Path) -> AppState:
    """AppState on a copy of the example project, recompute without the aero thread."""
    root = tmp / "example"
    shutil.copytree(EXAMPLE, root)
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


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="qrb-editor-sync-") as tmp:
        state = make_state(Path(tmp))
        assert state.definition_valid and not state.log, "the example project did not start clean"
        check_motor_sync(state)
        check_apply_set_syncs(state)
        check_profile(state)
        check_missing_surface_guard(state)
    print("editor sync suite OK")


if __name__ == "__main__":
    main()
