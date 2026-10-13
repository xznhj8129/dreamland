"""Headless assertions for the CAD export progress record (webapp/cad.py, GET /api/cad/progress).

Run: uv run python -m webapp.test_cad_progress

The route set runs on a throwaway copy of aircraft/example and exercises the cheapest real export (varsets), so no
CadQuery build runs: the endpoint's idle shape, the record helpers behind it, and that a finished or failed export
clears the record. libs.cad's own "building"/"writing" callbacks are exercised by the export script, not here.
"""

from __future__ import annotations

import asyncio
import shutil
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

from libs.project import Project  # noqa: E402
from webapp import cad as cad_backend  # noqa: E402
from webapp.server import create_app  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"
IDLE = {"running": False, "task": None, "label": None, "done": 0, "total": 0}


def make_state(root: Path) -> AppState:
    """AppState on a throwaway copy of the example definition, recomputed without the aero thread."""
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(EXAMPLE / "aircraft.json", root / "aircraft.json")
    state = AppState(Project(root=root))
    state.log_path = root / "webapp.log"  # keep the failure test's tracebacks out of the repo's logs/webapp.log
    original = state.recompute
    state.recompute = lambda run_aero=True: original(run_aero=False)
    state.recompute()
    assert state.definition_valid, [(entry.title, entry.text) for entry in state.log]
    return state


async def get_progress(client: TestClient) -> dict:
    response = await client.get("/api/cad/progress")
    assert response.status == 200, response.status
    return await response.json()


async def check_idle_endpoint(client: TestClient) -> None:
    assert await get_progress(client) == IDLE, await get_progress(client)
    print("ok   idle: GET /api/cad/progress answers the idle shape")


def check_record() -> None:
    """The exact shape the endpoint serves, including the fields the view reads while an export runs."""
    assert cad_backend.current() == IDLE, cad_backend.current()
    cad_backend.begin("sections")
    cad_backend.report("building S01", 1, 12)
    assert cad_backend.current() == {"running": True, "task": "sections", "label": "building S01", "done": 1, "total": 12}, cad_backend.current()
    cad_backend.end()
    assert cad_backend.current() == IDLE, cad_backend.current()
    print("ok   record: begin/report/current/end carry task, label and done/total, then clear")


async def check_finished_export_clears(client: TestClient) -> None:
    """The varsets export is real but fast: no CAD geometry, just the JSON files."""
    response = await client.post("/api/cad/varsets", json={})
    assert response.status == 200, (response.status, await response.text())
    detail = (await response.json())["detail"]
    assert "varsets written" in detail, detail
    assert cad_backend.current() == IDLE, cad_backend.current()
    assert await get_progress(client) == IDLE
    print(f"ok   varsets export: {detail}; the record is idle after it")


async def check_running_export(client: TestClient) -> None:
    """The browser's poll (GET while the POST is open) sees the export's task, label and done/total."""
    import libs.varsets as varsets_lib

    original = varsets_lib.export_varsets
    entered = threading.Event()

    def slow(*_args: object, progress=None, **_kwargs: object) -> list:
        entered.set()
        assert progress is not None
        progress("writing varsets", 1, 2)
        time.sleep(0.5)
        progress("writing varsets", 2, 2)
        return []

    varsets_lib.export_varsets = slow
    try:
        post = asyncio.create_task(client.post("/api/cad/varsets", json={}))
        assert await asyncio.to_thread(entered.wait, 5.0), "the export never reached the worker thread"
        seen = None
        for _ in range(20):  # the 0.5 s sleep in the stub leaves room for the poll
            snapshot = await get_progress(client)
            if snapshot["running"]:
                seen = snapshot
                break
            await asyncio.sleep(0.05)
        assert seen is not None, "the endpoint never reported the running export"
        assert seen["task"] == "varsets" and seen["label"] == "writing varsets", seen
        assert seen["done"] == 1 and seen["total"] == 2, seen
        response = await post
        assert response.status == 200, response.status
    finally:
        varsets_lib.export_varsets = original
    assert cad_backend.current() == IDLE, cad_backend.current()
    print("ok   running: a mid-export poll carries task/label/done/total, and the record clears after")


async def check_failed_export_clears(client: TestClient, state: AppState) -> None:
    """A raising libs call must still clear the record: the wrapper brackets it with begin/end in a finally.

    The 500 snapshot's last Log entry is the CAD failure in full — the traceback with the task/label context
    webapp.cad.ExportFailed carries and the original error as its cause — and its summary is the one-line
    headline the views show instead. The same traceback goes to state.log_path as the durable copy; make_state
    points that at the throwaway project, so this stays self-contained instead of touching the repo-level
    logs/webapp.log. The failure must also notify the open websockets, since nothing recomputed: the spy
    records what the Log held at push time.
    """
    import libs.varsets as varsets_lib

    original = varsets_lib.export_varsets
    original_notify = state.notify
    pushed: list[str] = []

    def spying_notify() -> None:
        pushed.append(state.log[-1].text if state.log else "")
        original_notify()

    def explode(*_args: object, **_kwargs: object) -> list:
        raise RuntimeError("boom")

    varsets_lib.export_varsets = explode
    state.notify = spying_notify
    try:
        response = await client.post("/api/cad/varsets", json={})
        assert response.status == 500, response.status
        payload = await response.json()
    finally:
        varsets_lib.export_varsets = original
        state.notify = original_notify
    last = payload["log"][-1]
    assert last["title"] == "CAD export", last
    assert "Traceback (most recent call last)" in last["text"], last
    assert "webapp/cad.py" in last["text"], last
    assert "RuntimeError: boom" in last["text"], last  # the original error, via the cause chain
    assert "CAD varsets export failed while starting" in last["text"], last
    assert "\n" not in last["summary"] and "Traceback" not in last["summary"], last
    assert "CAD varsets export failed while starting" in last["summary"], last
    assert len(pushed) == 1, f"the failed export must push the Log entry exactly once (pushes: {pushed})"
    assert pushed[0] == last["text"], pushed
    record = state.log_path.read_text(encoding="utf-8")
    assert "Traceback (most recent call last)" in record, record
    assert "RuntimeError: boom" in record, record
    assert "CAD varsets export failed while starting" in record, record
    assert cad_backend.current() == IDLE, cad_backend.current()
    print("ok   failure: a raising export answers 500 with the full CAD traceback in the Log (summary one line), notifies the open websockets, and the durable copy is in the log file")


async def check_routes(state: AppState) -> None:
    app = create_app(state, ROOT / "web" / "dist")
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
        await check_idle_endpoint(client)
        check_record()
        await check_finished_export_clears(client)
        await check_running_export(client)
        await check_failed_export_clears(client, state)
    finally:
        await client.close()


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="cad-progress-test-") as temp:
        state = make_state(Path(temp) / "example")
        asyncio.run(check_routes(state))
    print("cad progress suite OK")


if __name__ == "__main__":
    main()
