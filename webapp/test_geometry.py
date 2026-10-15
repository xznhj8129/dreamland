"""Headless integration for Compute Geometry (POST /api/geometry/compute) and the build-once solid store.

Run: uv run python -m webapp.test_geometry   (builds every solid of the example once: minutes)

Exercises the real path on a throwaway copy of aircraft/example: the route in its worker threads while editor
commands go on, the exact mass applied to the report, cad/exact_mass.json, an edit making the measurement stale and
undoing it making it current again without building, exports of the computed design only writing files, the headless
resolve the sim and analyze_airframe.py use, and mass conservation of the attribution.
"""

from __future__ import annotations

import asyncio
import copy
import math
import shutil
import tempfile
import time
from pathlib import Path

from aiohttp import ClientTimeout
from aiohttp.test_utils import TestClient, TestServer

from libs.airframe import load_airframe
from libs.cad import build_printable_parts
from libs.computed_geometry import EXACT_MASS_FILE, exact_structure, resolve_project
from libs.geometry import resolve_airframe
from libs.project import Project
from webapp.server import create_app
from webapp.state import AppState

ROOT = Path(__file__).resolve().parent.parent  # aircraft/example, run as `uv run python -m webapp.test_geometry`
EXAMPLE = ROOT / "aircraft" / "example"
EDIT_LATENCY_S = 1.0  # an editor command while Compute Geometry runs answers within this
COMPUTE_TIMEOUT_S = 3600.0  # the compute request: every solid built on the CAD worker's two CPUs takes minutes (aiohttp's own 300 s default cut it off)


def make_state(root: Path) -> AppState:
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(EXAMPLE / "aircraft.json", root / "aircraft.json")
    state = AppState(Project(root=root))
    state.log_path = root / "webapp.log"
    original = state.recompute
    state.recompute = lambda run_aero=True, notify=True: original(run_aero=False, notify=notify)
    state.recompute()
    assert state.definition_valid, [(entry.title, entry.text) for entry in state.log]
    return state


def exact_points(state: AppState) -> list[str]:
    return [point.name for point in state.resolved.mass_points if point.exact]


async def set_name(client: TestClient, state: AppState, name: str) -> float:
    started = time.monotonic()
    response = await client.post("/api/editor/set", json={"session_id": state.session_id, "path": ["name"], "value": name})
    assert response.status == 200, (response.status, await response.text())
    return time.monotonic() - started


async def check_compute(client: TestClient, state: AppState) -> None:
    assert state.geometry_status == "none" and not exact_points(state), state.geometry_status
    estimate_kg = state.resolved.mass_kg
    original = state.definition["name"]
    compute = asyncio.ensure_future(client.post("/api/geometry/compute", json={}))
    while not state.geometry_computing:
        await asyncio.sleep(0.05)
    latencies = [await set_name(client, state, original + " edited"), await set_name(client, state, original)]
    assert max(latencies) < EDIT_LATENCY_S, latencies
    refused = await client.post("/api/geometry/compute", json={})
    assert refused.status == 409, refused.status
    response = await compute
    assert response.status == 200, (response.status, await response.text())
    snapshot = await response.json()
    assert snapshot["geometry"] == "current", snapshot["geometry"]
    names = exact_points(state)
    sections = [point for point in state.resolved.mass_points if point.name.startswith("section ")]
    assert sections and all(point.exact for point in sections), names
    assert all(point.exact for point in state.resolved.mass_points if point.kind == "surface"), names
    assert not any(point.name.startswith("carry-through") for point in state.resolved.mass_points)
    assert (state.project.cad_dir / EXACT_MASS_FILE).is_file()
    assert any("Printed structure: exact" in getattr(line, "text", line) for line in state.report), state.report[:12]
    print(f"ok   compute: edits answered in {max(latencies) * 1000:.0f} ms while it ran, a second request refused (409); "
          f"exact mass applied ({estimate_kg[0] * 1000:.1f} g estimated -> {state.resolved.mass_kg[0] * 1000:.1f} g), saved")


async def check_exports_reuse(client: TestClient) -> None:
    for route, body in (("/api/cad/sections", {"format": "step"}), ("/api/cad/step", {"mode": "fused"}), ("/api/cad/stl", {}),
                        ("/api/cad/drawings", {})):
        started = time.monotonic()
        response = await client.post(route, json=body)
        assert response.status == 200, (route, response.status, await response.text())
        print(f"ok   {route} after compute: {time.monotonic() - started:.1f} s (files only)")


async def check_stale_and_back(client: TestClient, state: AppState) -> None:
    original = state.definition["name"]
    await set_name(client, state, original + " edited")
    assert state.geometry_status == "stale" and not exact_points(state), state.geometry_status
    await set_name(client, state, original)
    assert state.geometry_status == "current" and exact_points(state), state.geometry_status
    print("ok   an edit falls back to the estimates (stale); undoing it applies the exact mass again without building")


def check_headless(state: AppState) -> None:
    airframe = load_airframe(state.project.build_path, state.parts_data)
    resolved, _problems, exact = resolve_project(airframe, state.parts_data, state.project.cad_dir)
    assert exact and resolved.mass_kg == state.resolved.mass_kg and resolved.cg_m == state.resolved.cg_m
    restarted = AppState(state.project)
    assert restarted.measured is not None and restarted.measured.digest == state.measured.digest
    print("ok   headless resolve (sim, analyze_airframe.py) and a fresh server use the saved exact mass")


def check_conservation(state: AppState) -> None:
    """With material and bulk density equal, every printed solid counts once: the exact printed mass is that density
    times the printable parts' total volume (an independent sum over the built parts, not the attribution)."""
    density = state.resolved.airframe.print.bulk_density_kg_m3
    parts_data = copy.deepcopy(state.parts_data)
    parts_data["material"][state.resolved.airframe.print.material]["density_kg_m3"] = density
    estimate, _problems = resolve_airframe(state.resolved.airframe, parts_data)
    exact = exact_structure(estimate, parts_data)
    measured = sum(piece.mass_kg for piece in (*exact.sections, *exact.surfaces.values(), *exact.couplers.values()))
    volume_m3 = sum(shape.Volume() for _name, shape, _notes in build_printable_parts(estimate)) * 1e-9
    assert math.isclose(measured, volume_m3 * density, rel_tol=1e-6), (measured, volume_m3 * density)
    print(f"ok   conservation: exact printed mass {measured * 1000:.2f} g = density x printable volume {volume_m3 * 1e6:.1f} cm^3")


async def run(state: AppState) -> None:
    async with TestClient(TestServer(create_app(state, ROOT / "web" / "dist")), timeout=ClientTimeout(total=COMPUTE_TIMEOUT_S)) as client:
        await check_compute(client, state)
        await check_exports_reuse(client)
        await check_stale_and_back(client, state)
    check_headless(state)
    check_conservation(state)


def main() -> None:
    with tempfile.TemporaryDirectory() as folder:
        state = make_state(Path(folder) / "example")
        asyncio.run(run(state))
    print("geometry suite OK")


if __name__ == "__main__":
    main()
