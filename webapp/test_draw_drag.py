"""Headless assertions for the 2D draw handles and the drag move op.

Run: /home/anon/miniconda3/envs/drone/bin/python webapp/test_draw_drag.py

The state is a throwaway copy of aircraft/example, in process, exercised exactly as webapp/server.py calls it:
draw at 900x600 in both modes for the handle list, then apply_move for the clamps and the session fence, including
one pass over the real aiohttp routes.
"""

from __future__ import annotations

import asyncio
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

from libs.project import Project  # noqa: E402
from webapp import drawing, editor  # noqa: E402
from webapp.server import create_app  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"
AXIS_X_TARGETS = ("cut", "spars", "surface", "rod")


def make_state(tmp: Path, name: str) -> AppState:
    """AppState on a copy of the example project; the recompute never starts the aero thread."""
    root = tmp / name
    shutil.copytree(EXAMPLE, root)
    state = AppState(Project(root=root))
    original = state.recompute
    state.recompute = lambda run_aero=True: original(run_aero=False)
    state.recompute()
    return state


def targets(payload: dict) -> set[tuple[str, int]]:
    return {(handle["target"], handle["index"]) for handle in payload["handles"]}


def draw(state: AppState, mode: str) -> dict:
    view = drawing.ensure_view(state)
    view.set_mode(mode)
    payload = drawing.draw(state, 900, 600, {})
    json.dumps(payload)  # the payload the browser receives must be JSON-safe
    return payload


def move_body(state: AppState, target: str, index: int, delta: dict) -> dict:
    return {"session_id": state.session_id, "target": target, "index": index, "delta": delta}


def check_handles(state: AppState) -> None:
    side = draw(state, "Longitudinal")
    assert side["session_id"] == state.session_id
    present = targets(side)
    for target in (("cut", 0), ("spars", 0), ("surface", 0), ("volume", 0), ("part", 0)):
        assert target in present, f"side view lost its {target[0]} handle"
    for handle in side["handles"]:
        assert set(handle) == {"target", "index", "kind", "coords", "horizontal", "vertical", "order"}
        assert handle["kind"] in ("polygon", "line") and len(handle["coords"]) >= 4 and len(handle["coords"]) % 2 == 0
        axes = (handle["horizontal"], handle["vertical"])
        assert axes == ("x", None) if handle["target"] in AXIS_X_TARGETS else axes == ("x", "z"), handle
    print("ok   side view: cut/spars/surface/volume/part handles (x, z axes; JSON-safe)")

    front = draw(state, "Front")
    present = targets(front)
    for target in (("spars", 0), ("surface", 0), ("volume", 0), ("part", 0)):
        assert target in present, f"front view lost its {target[0]} handle"
    assert ("cut", 0) not in present, "cuts are not drawn in the front view, so they have no handle there"
    for handle in front["handles"]:
        axes = (handle["horizontal"], handle["vertical"])
        assert axes == ("x", None) if handle["target"] in AXIS_X_TARGETS else axes == ("y", "z"), handle
    print("ok   front view: spars/surface/volume/part handles (y, z axes)")

    bound = state.definition["parts"][1]
    for key in ("x_m", "y_m", "z_m"):
        bound.pop(key, None)
    bound["volume"] = state.definition["volumes"][0]["name"]
    state.recompute()
    present = targets(draw(state, "Longitudinal"))
    assert ("part", 1) not in present, "a part in a volume must not have its own handle"
    assert ("battery", 0) not in present, "a battery in a volume must not have its own handle"
    print("ok   bound part and bound battery have no handles")

    battery = state.definition["propulsion"]["battery"]
    battery.pop("volume", None)
    battery["x_m"], battery["y_m"], battery["z_m"] = 0.2, 0.0, 0.0
    state.recompute()
    assert ("battery", 0) in targets(draw(state, "Longitudinal")), "a free battery must have a handle"
    print("ok   free battery has a handle")

    view = drawing.ensure_view(state)
    view.set_roll(30.0)
    rolled = draw(state, "Longitudinal")
    for handle in rolled["handles"]:
        if handle["target"] not in AXIS_X_TARGETS:
            assert handle["vertical"] is None, "a rolled side view has no unambiguous vertical axis"
    view.set_roll(0.0)
    print("ok   rolled side view: volume/part vertical is null (ambiguous)")


def check_moves(state: AppState) -> None:
    definition = state.definition

    editor.apply_move(state, move_body(state, "cut", 0, {"x": -10.0}))
    assert definition["sections"]["cuts_m"][0] == 0.005, definition["sections"]["cuts_m"][0]
    print("ok   move cut: clamped to MIN_SECTION_LENGTH_M from the neighbour")

    editor.apply_move(state, move_body(state, "surface", 1, {"x": -10.0}))
    assert definition["surfaces"][1]["x_root_le_m"] == 0.0
    print("ok   move surface: x_root_le_m floored at 0")

    definition["volumes"][0].pop("z_m", None)
    state.recompute()
    editor.apply_move(state, move_body(state, "volume", 0, {"z": 0.02}))
    assert definition["volumes"][0]["z_m"] == 0.02, "a missing z_m must start at 0.0"
    print("ok   move volume: missing z_m created from 0.0")

    length = definition["fuselage"]["length_m"]
    definition["fuselage"]["reinforcement"] = {
        "pattern": {"type": "cf_rod", "material": "carbon_rod", "diameter_m": 0.0025, "radial_position_m": 0.027375,
                    "polar_n": 8, "polar_offset_angle": 30, "polar_split": 2, "polar_split_angle": 15},
        "from_x_m": 0.1, "to_x_m": 0.2,
    }
    state.recompute()
    editor.apply_move(state, move_body(state, "rod", 0, {"x": 10.0}))
    reinforcement = definition["fuselage"]["reinforcement"]
    assert length - reinforcement["to_x_m"] <= editor.MOVE_QUANTUM_M, reinforcement
    assert 0.0 <= reinforcement["from_x_m"] <= reinforcement["to_x_m"] <= length, reinforcement
    print("ok   move rod: clamped to 0 <= from <= to <= fuselage length")

    part_x = definition["parts"][0]["x_m"]
    q0 = editor._quantize(part_x)
    editor.apply_move(state, move_body(state, "part", 0, {"x": 0.000004}))  # below half a quantum: snaps back
    assert definition["parts"][0]["x_m"] == q0, "a sub-quantum drag must not move"
    editor.apply_move(state, move_body(state, "part", 0, {"x": 0.000006}))  # past half a quantum: one step
    assert definition["parts"][0]["x_m"] == editor._quantize(q0 + 0.000006), "0.01 mm quantization"
    before = definition["parts"][0]["x_m"]
    editor.apply_move(state, move_body(state, "part", 0, {"x": 0.01}))
    assert definition["parts"][0]["x_m"] == editor._quantize(before + 0.01)
    battery_y = definition["propulsion"]["battery"]["y_m"]
    editor.apply_move(state, move_body(state, "battery", 0, {"y": 0.01}))
    assert definition["propulsion"]["battery"]["y_m"] == editor._quantize(battery_y + 0.01)
    for bad in (move_body(state, "part", 1, {"x": 0.01}),  # part 1 sits in a volume since check_handles
                move_body(state, "cut", 999, {"x": 0.01}), move_body(state, "spars", 1, {"x": 0.01})):
        try:
            editor.apply_move(state, bad)
            raise AssertionError(f"move accepted an unknown path: {bad}")
        except ValueError:
            pass
    print("ok   move part/battery by own x/y/z; bound part, unknown index and missing spars refused")

    resolved_before = state.resolved
    editor.apply_move(state, move_body(state, "volume", 0, {}))
    editor.apply_move(state, move_body(state, "cut", 0, {"y": 1.0}))
    assert state.resolved is resolved_before, "a delta with no component for the target must be a no-op"
    print("ok   missing/zero deltas: no-op (no recompute)")


async def check_routes(state: AppState) -> None:
    app = create_app(state, ROOT / "web" / "dist")
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
        stale = state.session_id + 1
        response = await client.post("/api/editor/move", json={"session_id": stale, "target": "cut", "index": 0, "delta": {"x": -1.0}})
        assert response.status == 409, response.status

        response = await client.post("/api/editor/move", json=move_body(state, "ghost", 0, {"x": 0.01}))
        assert response.status == 400, response.status
        payload = await response.json()
        assert payload["log"][-1]["title"] == "Move" and "ghost" in payload["log"][-1]["text"], payload["log"][-1]

        response = await client.post("/api/editor/move", json=move_body(state, "surface", 2, {"x": 0.01}))
        assert response.status == 200, response.status

        response = await client.post("/api/draw", json={"session_id": stale, "width": 900, "height": 600, "action": {"kind": "redraw"}})
        assert response.status == 409, response.status
        response = await client.post("/api/draw", json={"session_id": state.session_id, "width": 900, "height": 600, "action": {"kind": "redraw"}})
        assert response.status == 200, response.status
        payload = await response.json()
        assert payload["session_id"] == state.session_id and payload["handles"], "the draw echo must carry session_id and handles"

        response = await client.post("/api/recompute", json={"run_aero": False})
        assert response.status == 200, response.status
    finally:
        await client.close()
    print("ok   routes: stale move/draw 409, unknown target 400 with Log, move 200, draw echo, recompute body")


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="qrb-draw-drag-") as tmp:
        temp = Path(tmp)
        state = make_state(temp, "example")
        assert state.definition_valid and not state.log, "the example project did not start clean"
        check_handles(state)
        check_moves(state)
        asyncio.run(check_routes(make_state(temp, "http")))
    print("draw/drag suite OK")


if __name__ == "__main__":
    main()
