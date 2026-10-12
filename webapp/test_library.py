"""Headless assertions for the part-library API (webapp/library.py, the /api/library routes).

Run: /home/anon/miniconda3/envs/drone/bin/python webapp/test_library.py

libs.airframe.PARTS_DIR is pointed at a persistent validation copy of data/parts in aircraft/example/cad (both libs load/save look the module global up at
call time); the real data/parts is hashed before and after and must be byte-identical.
"""

from __future__ import annotations

import asyncio
import hashlib
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

import libs.airframe as airframe  # noqa: E402
from libs.project import Project  # noqa: E402
from webapp import library  # noqa: E402
from webapp.serialize import snapshot  # noqa: E402
from webapp.server import create_app  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"
PARTS = ROOT / "data" / "parts"


def hash_tree(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def make_state(output: Path, parts: Path) -> AppState:
    root = output / "example"
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(EXAMPLE / "aircraft.json", root / "aircraft.json")
    airframe.PARTS_DIR = parts  # libs.airframe.load_parts_data/save_parts_category read this global at call time
    state = AppState(Project(root=root))
    original = state.recompute
    state.recompute = lambda run_aero=True: original(run_aero=False)
    state.recompute()
    return state


def expect_value_error(state: AppState, body: dict) -> None:
    before = state.library_revision
    try:
        library.save_part(state, body)
    except ValueError:
        assert state.library_revision == before, "a refused save must not bump the revision"
        return
    raise AssertionError(f"save accepted bad input: {body}")


def check_payload(state: AppState) -> None:
    payload = library.library_payload(state)
    assert set(payload) == {"parts", "categories", "field_order", "length_keys", "mass_keys", "optional_fields", "defaults", "roles"}
    kinds = {category["name"]: category["kind"] for category in payload["categories"]}
    assert kinds["battery"] == "table" and kinds["esc"] == "role" and kinds["prop"] == "table", kinds
    assert payload["optional_fields"]["battery"]["lead_awg"] == "int" and payload["optional_fields"]["esc"]["mosfet_rds_on_ohm"] == "float"
    assert "mass" in payload["mass_keys"] and "chord_root" in payload["length_keys"]
    assert payload["defaults"]["esc"]["curr"] == 0 and payload["defaults"]["fc"]["generic"] is True
    assert payload["roles"][0] == "fc" and payload["parts"]["prop"], "payload lost the library"
    json.dumps(payload)
    print("ok   payload: parts, categories with kinds, field metadata and defaults are JSON-safe")


def check_saves(state: AppState, parts: Path) -> None:
    before = state.library_revision
    library.save_part(state, {"category": "prop", "name": "bench-prop", "original": None, "values": {"diameter_in": 10, "mass": 0.01}})
    assert (parts / "prop.json").is_file()
    assert state.parts_data["prop"]["bench-prop"] == {"diameter_in": 10, "mass": 0.01}
    assert airframe.load_parts_data()["prop"]["bench-prop"] == {"diameter_in": 10, "mass": 0.01}
    assert state.library_revision == before + 1 and snapshot(state)["library_revision"] == before + 1
    print("ok   save into a table category: prop.json written, load_parts_data sees it, revision bumped")

    library.save_part(state, {"category": "fc", "name": "bench-fc", "original": None, "values": {"mass": 0.01, "dimensions": {"l": 20.0, "w": 20.0, "h": 5.0}}})
    assert (parts / "fc" / "bench-fc.json").is_file()
    assert airframe.load_parts_data()["fc"]["bench-fc"]["mass"] == 0.01
    print("ok   save into a role category: fc/bench-fc.json written and load_parts_data sees it")


def check_rename_move(state: AppState, parts: Path) -> None:
    library.save_part(state, {"category": "esc", "name": "moved-esc", "original": {"category": "fc", "name": "bench-fc"},
                              "values": {"mass": 0.02, "dimensions": {"l": 20.0, "w": 20.0, "h": 5.0}}})
    data = airframe.load_parts_data()
    assert not (parts / "fc" / "bench-fc.json").exists() and "bench-fc" not in data["fc"], "the old role file survived the move"
    assert (parts / "esc" / "moved-esc.json").is_file() and data["esc"]["moved-esc"]["mass"] == 0.02
    library.save_part(state, {"category": "prop", "name": "renamed-prop", "original": {"category": "prop", "name": "bench-prop"},
                              "values": {"diameter_in": 10, "mass": 0.02}})
    data = airframe.load_parts_data()
    assert "bench-prop" not in data["prop"] and data["prop"]["renamed-prop"]["mass"] == 0.02
    assert "bench-prop" not in json.loads((parts / "prop.json").read_text()), "a table rename left the old key"
    print("ok   rename + move: old file/entry gone, new file/entry present, both categories round-tripped")


def check_delete(state: AppState, parts: Path) -> None:
    before = state.library_revision
    library.delete_part(state, {"category": "esc", "name": "moved-esc"})
    assert not (parts / "esc" / "moved-esc.json").exists() and "moved-esc" not in airframe.load_parts_data()["esc"]
    assert state.library_revision == before + 1 and snapshot(state)["library_revision"] == state.library_revision
    before = len(state.log)
    try:
        library.delete_part(state, {"category": "esc", "name": "moved-esc"})
    except ValueError:
        pass
    else:
        raise AssertionError("deleting a missing part was accepted")
    assert len(state.log) == before
    print("ok   delete: entry/file gone, revision bumped and in the snapshot, missing part refused")


def check_bad_input(state: AppState) -> None:
    for body in (
        {"category": "ghost", "name": "x", "values": {}},
        {"category": "prop", "name": "a/b", "values": {}},
        {"category": "prop", "name": "..", "values": {}},
        {"category": "prop", "name": "bad\\name", "values": {}},
        {"category": "prop", "name": "ok"},
        {"category": "prop", "name": "ok", "values": []},
        {"category": "prop", "name": "ok", "values": {}, "original": {"category": "prop", "name": "absent"}},
        {"category": "prop", "name": "ok", "values": {}, "original": "nope"},
    ):
        expect_value_error(state, body)
    print("ok   bad input: unknown category, separators/dots/NUL names, missing/non-object values, bad original -> ValueError")


async def check_routes(state: AppState) -> None:
    app = create_app(state, ROOT / "web" / "dist")
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
        response = await client.get("/api/library")
        assert response.status == 200
        payload = await response.json()
        assert payload["parts"] and any(category["kind"] == "role" for category in payload["categories"])

        response = await client.post("/api/library/save", json={"category": "ghost", "name": "x", "values": {}})
        assert response.status == 400, response.status
        payload = await response.json()
        assert payload["log"][-1]["title"] == "Part Library" and "ghost" in payload["log"][-1]["text"]

        response = await client.post("/api/library/save", json={"category": "battery", "name": "http-pack", "original": None, "values": {"type": "lipo"}})
        assert response.status == 200, response.status
        payload = await response.json()
        assert payload["library_revision"] > 0

        response = await client.post("/api/library/delete", json={"category": "battery", "name": "http-pack"})
        assert response.status == 200, response.status

        response = await client.post("/api/library/delete", json={"category": "battery", "name": "http-pack"})
        assert response.status == 400, response.status
    finally:
        await client.close()
    print("ok   routes: GET payload, save 200/400 with Log entry, delete 200/400")


def main() -> None:
    real_before = hash_tree(PARTS)
    output = EXAMPLE / "cad" / "library-validation"
    parts = output / "parts"
    shutil.copytree(PARTS, parts, dirs_exist_ok=True)
    state = make_state(output, parts)
    check_payload(state)
    check_saves(state, parts)
    check_rename_move(state, parts)
    check_delete(state, parts)
    check_bad_input(state)
    asyncio.run(check_routes(state))
    real_after = hash_tree(PARTS)
    assert real_before == real_after, "the real data/parts changed"
    print("ok   real data/parts byte-identical before/after")
    print("library suite OK")


if __name__ == "__main__":
    main()
