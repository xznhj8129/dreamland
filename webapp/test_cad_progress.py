"""Headless assertions for the CAD run record (webapp/cad.py, GET /api/cad/state, POST /api/cad/worker).

Run: uv run python -m webapp.test_cad_progress

The route set runs on a throwaway copy of aircraft/example and exercises the cheapest real export (varsets), so no
CadQuery build runs: the endpoint's idle shape, the record helpers behind it, that a finished or failed export ends
the run and keeps its outcome, that an export goes on when its request is dropped (the browser left the tab or
reloaded), and the worker CPU setting. libs.cad's own "building"/"writing" callbacks are exercised by the export
script, not here.
"""

from __future__ import annotations

import asyncio
import contextlib
import os
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
from webapp.cad_worker import WORKER_CPUS, WORKER_NICE  # noqa: E402
from webapp.server import create_app  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"


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


async def get_state(client: TestClient) -> dict:
    response = await client.get("/api/cad/state")
    assert response.status == 200, response.status
    return await response.json()


async def check_idle_endpoint(client: TestClient) -> None:
    answer = await get_state(client)
    assert answer["run"] is None and answer["outcomes"] == {}, answer
    assert answer["worker"]["cpus"] == WORKER_CPUS and answer["worker"]["max_cpus"] == WORKER_CPUS, answer["worker"]
    assert len(answer["worker"]["cpu_ids"]) == WORKER_CPUS and answer["worker"]["nice"] == min(19, os.nice(0) + WORKER_NICE), answer["worker"]
    print(f"ok   idle: GET /api/cad/state has no run, no outcomes, the worker on {WORKER_CPUS} CPUs at nice {answer['worker']['nice']}")


def check_record() -> None:
    """The run shape the endpoint serves: steps with arrival times, the elapsed time, and a pace estimate only once a
    phase has ESTIMATE_MIN_STEPS steps."""
    cad_backend.begin("sections", {"format": "step"})
    cad_backend.report("building S01", 1, 12)
    run = cad_backend.current()["run"]
    assert run["task"] == "sections" and run["state"] == "running" and run["options"] == {"format": "step"}, run
    assert [(step["label"], step["done"], step["total"]) for step in run["steps"]] == [("building S01", 1, 12)], run
    assert run["estimate"] is None, run
    time.sleep(0.05)
    cad_backend.report("building S02", 2, 12)
    time.sleep(0.05)
    cad_backend.report("building S03", 3, 12)
    run = cad_backend.current()["run"]
    estimate = run["estimate"]
    assert estimate is not None and estimate["phase"] == "building", run
    assert 0.03 < estimate["per_step_s"] < 0.5 and estimate["left_s"] <= estimate["per_step_s"] * 9, estimate
    cad_backend.report("writing S01", 1, 12)
    assert cad_backend.current()["run"]["estimate"] is None, "a new phase has no pace yet"
    assert cad_backend.running_task() == "sections"
    cad_backend.end("done", "12 STEP parts written")
    run = cad_backend.current()["run"]
    assert run["state"] == "done" and run["message"] == "12 STEP parts written" and run["estimate"] is None, run
    assert len(run["steps"]) == 4 and run["elapsed_s"] >= 0.1, run
    assert cad_backend.running_task() is None
    print("ok   record: steps carry label, done/total and arrival time; the estimate needs 3 steps of one phase; end keeps the run")


async def check_finished_export(client: TestClient, state: AppState) -> None:
    """The varsets export is real but fast: no CAD geometry, just the JSON files."""
    response = await client.post("/api/cad/varsets", json={})
    assert response.status == 200, (response.status, await response.text())
    detail = (await response.json())["detail"]
    assert "varsets written" in detail, detail
    answer = await get_state(client)
    assert answer["run"]["task"] == "varsets" and answer["run"]["state"] == "done", answer["run"]
    assert answer["run"]["steps"][-1]["label"] == "writing varsets", answer["run"]
    outcome = answer["outcomes"]["varsets"]
    assert outcome["ok"] and outcome["message"] == detail and outcome["project"] == state.project.name, outcome
    assert outcome["files"] and all(file.startswith("cad/varsets/") for file in outcome["files"]), outcome
    print(f"ok   varsets export: {detail}; the run is done and its outcome kept for the card")


def _slow_varsets(entered: threading.Event, release: threading.Event):
    def slow(*_args: object, progress=None, **_kwargs: object) -> list:
        entered.set()
        assert progress is not None
        progress("writing varsets", 1, 2)
        assert release.wait(5.0), "the test never released the export"
        progress("writing varsets", 2, 2)
        return []

    return slow


async def check_running_export(client: TestClient) -> None:
    """The browser's poll (GET while the POST is open) sees the export's task, latest step and elapsed time."""
    import libs.varsets as varsets_lib

    original = varsets_lib.export_varsets
    entered, release = threading.Event(), threading.Event()
    varsets_lib.export_varsets = _slow_varsets(entered, release)
    try:
        post = asyncio.create_task(client.post("/api/cad/varsets", json={}))
        assert await asyncio.to_thread(entered.wait, 5.0), "the export never reached the worker thread"
        run = (await get_state(client))["run"]
        assert run["task"] == "varsets" and run["state"] == "running", run
        assert run["steps"][-1] == {"label": "writing varsets", "done": 1, "total": 2, "at_s": run["steps"][-1]["at_s"]}, run
        release.set()
        response = await post
        assert response.status == 200, response.status
    finally:
        release.set()
        varsets_lib.export_varsets = original
    run = (await get_state(client))["run"]
    assert run["state"] == "done" and [step["done"] for step in run["steps"]] == [1, 2], run
    print("ok   running: a mid-export poll carries the task, its latest step and done/total; the run ends done")


async def check_dropped_request(client: TestClient) -> None:
    """Leaving the tab or reloading drops the export's request; the export must go on and keep its outcome."""
    import libs.varsets as varsets_lib

    original = varsets_lib.export_varsets
    entered, release = threading.Event(), threading.Event()
    varsets_lib.export_varsets = _slow_varsets(entered, release)
    try:
        post = asyncio.create_task(client.post("/api/cad/varsets", json={}))
        assert await asyncio.to_thread(entered.wait, 5.0), "the export never reached the worker thread"
        post.cancel()  # the client goes away mid-export
        with contextlib.suppress(asyncio.CancelledError):
            await post
        await asyncio.sleep(0.1)
        assert (await get_state(client))["run"]["state"] == "running", "the dropped request stopped the export"
        before = (await get_state(client))["outcomes"]["varsets"]["finished_at"]
        release.set()
        for _ in range(50):
            answer = await get_state(client)
            if answer["run"]["state"] != "running":
                break
            await asyncio.sleep(0.05)
    finally:
        release.set()
        varsets_lib.export_varsets = original
    assert answer["run"]["state"] == "done", answer["run"]
    assert answer["outcomes"]["varsets"]["finished_at"] > before, answer["outcomes"]["varsets"]
    print("ok   dropped request: the export runs to the end after its request is cancelled, and its outcome is kept")


async def check_worker_setting(client: TestClient) -> None:
    for refused in (0, WORKER_CPUS + 1, "2"):
        response = await client.post("/api/cad/worker", json={"cpus": refused})
        assert response.status == 400, (refused, response.status)
    response = await client.post("/api/cad/worker", json={"cpus": 1})
    assert response.status == 200, response.status
    worker = (await response.json())["worker"]
    assert worker["cpus"] == 1 and len(worker["cpu_ids"]) == 1, worker
    response = await client.post("/api/cad/worker", json={"cpus": WORKER_CPUS})
    assert (await response.json())["worker"]["cpus"] == WORKER_CPUS
    print(f"ok   worker: POST /api/cad/worker takes 1..{WORKER_CPUS} CPUs and refuses anything else")


async def check_failed_export(client: TestClient, state: AppState) -> None:
    """A raising libs call must still end the run (failed) and keep the failure as the card's outcome.

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
    answer = await get_state(client)
    assert answer["run"]["state"] == "failed" and answer["run"]["message"] == "CAD varsets export failed while starting", answer["run"]
    failed = answer["outcomes"]["varsets"]
    assert not failed["ok"] and failed["files"] == [] and failed["message"] == "starting", failed
    print("ok   failure: a raising export answers 500 with the full CAD traceback in the Log (summary one line), notifies the open websockets, the durable copy is in the log file, and the run and the card show it failed")


async def check_routes(state: AppState) -> None:
    app = create_app(state, ROOT / "web" / "dist")
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
        await check_idle_endpoint(client)
        check_record()
        await check_finished_export(client, state)
        await check_running_export(client)
        await check_dropped_request(client)
        await check_worker_setting(client)
        await check_failed_export(client, state)
    finally:
        await client.close()


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="cad-progress-test-") as temp:
        state = make_state(Path(temp) / "example")
        asyncio.run(check_routes(state))
    print("cad run record suite OK")


if __name__ == "__main__":
    main()
