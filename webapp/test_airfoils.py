"""Headless assertions for the Aero Analysis backend (webapp/airfoils.py, the /api/airfoils routes).

Run: /home/anon/miniconda3/envs/drone/bin/python webapp/test_airfoils.py

The session store is process-local (imported DATs are lost on restart, as in the retired Tk view), so this suite
imports its own foils and checks them through the real routes. The airfoil routes never touch the project state;
the example project is only what an AppState needs to exist.
"""

from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # runnable from anywhere
    sys.path.insert(0, str(ROOT))

from aiohttp.test_utils import TestClient, TestServer  # noqa: E402

from libs.project import Project  # noqa: E402
from webapp import airfoils as airfoils_backend  # noqa: E402
from webapp.server import create_app  # noqa: E402
from webapp.state import AppState  # noqa: E402

EXAMPLE = ROOT / "aircraft" / "example"
LIBRARY_NAMES = ["naca0008", "naca0012", "naca2408", "naca2412", "naca6408"]  # data/airfoils.json
DEFAULTS = {
    "alpha_deg": 4.0,
    "airspeed_m_s": 25.0,
    "altitude_m": 0.0,
    "temperature_c": 15.0,
    "chord_mm": 100.0,
    "n_crit": 9.0,
    "xtr_upper": 1.0,
    "xtr_lower": 1.0,
    "model_size": "large",
    "include_360_deg_effects": True,
}
MODEL_SIZES = ["xxsmall", "xsmall", "small", "medium", "large", "xlarge", "xxlarge", "xxxlarge"]
SUMMARY_KEYS = {
    "alpha_zero_lift_deg", "alpha_pos_stall_deg", "cl_max", "alpha_neg_stall_deg", "cl_min",
    "alpha_best_ld_deg", "cl_cd_max", "alpha_cd_min_deg", "cd_min", "cm_at_cd_min",
    "confidence_min", "confidence_mean",
}
ANALYZE_BODY = {"airfoils": ["naca2412", "naca0008"], **DEFAULTS}


def _reject_json_constant(name: str) -> None:
    raise AssertionError(f"non-JSON constant {name!r} reached the payload")


async def get_library(client: TestClient, surfaces: str | None = None) -> dict:
    response = await client.get("/api/airfoils", params={"surfaces": surfaces} if surfaces is not None else None)
    assert response.status == 200, response.status
    return await response.json()


async def check_library(client: TestClient) -> None:
    payload = await get_library(client)
    assert [item["name"] for item in payload["airfoils"]] == LIBRARY_NAMES, payload["airfoils"]
    assert all(item["source"] == "builtin" for item in payload["airfoils"]), payload["airfoils"]
    assert payload["selected"] == ["naca2412", "naca0008"], payload["selected"]
    assert payload["defaults"] == DEFAULTS, payload["defaults"]
    assert payload["model_sizes"] == MODEL_SIZES, payload["model_sizes"]
    assert payload["alpha_sweep"] == {"min_deg": -20.0, "max_deg": 20.0, "step_deg": 1.0}, payload["alpha_sweep"]
    print("ok   library: the five library airfoils, default selection, defaults, model sizes and sweep bounds")

    payload = await get_library(client, "naca2412,naca0008,not-a-real-foil")
    assert payload["selected"] == ["naca2412", "naca0008"], payload["selected"]
    assert "not-a-real-foil" not in [item["name"] for item in payload["airfoils"]], payload["airfoils"]
    print("ok   surfaces: two library names select, an unknown AeroSandbox name is skipped")


async def check_asb_seed(client: TestClient) -> None:
    """A surface name the library file does not hold is seeded from AeroSandbox and then analyzes."""
    payload = await get_library(client, "naca4412")
    seeded = {item["name"]: item["source"] for item in payload["airfoils"]}
    assert seeded.get("naca4412") == "asb", payload["airfoils"]
    assert payload["selected"] == ["naca4412"], payload["selected"]
    response = await client.post("/api/airfoils/analyze", json={"airfoils": ["naca4412"], **DEFAULTS})
    assert response.status == 200, (response.status, await response.text())
    body = await response.json()
    assert [item["name"] for item in body["airfoils"]] == ["naca4412"], body["airfoils"]
    assert body["airfoils"][0]["source"] == "asb", body["airfoils"][0]
    print("ok   ASB seed: ?surfaces=naca4412 adds it for the session and it resolves in analyze")


async def check_import(client: TestClient) -> None:
    dat_text = json.loads((ROOT / "data" / "airfoils.json").read_text())["naca0012"]["dat_text"]
    response = await client.post("/api/airfoils/import", json={"name": "test-import", "dat_text": dat_text})
    assert response.status == 200, (response.status, await response.text())
    assert await response.json() == {"name": "test-import", "source": "dat_file"}
    response = await client.post("/api/airfoils/import", json={"name": "test-import", "dat_text": dat_text})
    assert response.status == 200 and (await response.json())["name"] == "test-import-2"
    payload = await get_library(client)
    imported = {item["name"]: item["source"] for item in payload["airfoils"]}
    assert imported["test-import"] == "dat_file" and imported["test-import-2"] == "dat_file", imported
    print("ok   import: a valid Selig DAT is listed with source dat_file; a duplicate name gets -2")

    response = await client.post("/api/airfoils/import", json={"name": "naca2412", "dat_text": dat_text})
    assert response.status == 200 and (await response.json())["name"] == "naca2412-2"
    payload = await get_library(client)
    sources = {item["name"]: item["source"] for item in payload["airfoils"]}
    assert sources["naca2412"] == "builtin" and sources["naca2412-2"] == "dat_file", sources
    print("ok   import collision: the library's naca2412 is kept and the import lands as naca2412-2")

    response = await client.post("/api/airfoils/analyze", json={"airfoils": ["test-import"], **DEFAULTS})
    assert response.status == 200, (response.status, await response.text())
    body = await response.json()
    assert [item["name"] for item in body["airfoils"]] == ["test-import"], body["airfoils"]
    assert body["airfoils"][0]["source"] == "dat_file", body["airfoils"][0]
    print("ok   import analyze: a session-imported DAT resolves and runs through NeuralFoil")

    response = await client.post("/api/airfoils/import", json={"name": "broken", "dat_text": "not an airfoil"})
    assert response.status == 400, response.status
    message = (await response.json())["message"]
    assert message.startswith("Could not import broken: "), message
    print(f"ok   bad import: 400 with the refusal message ({message})")


async def check_import_atomicity() -> None:
    """Concurrent same-name imports resolve under the module lock: no two get the same suffix."""
    dat_text = json.loads((ROOT / "data" / "airfoils.json").read_text())["naca0012"]["dat_text"]
    answers = await asyncio.gather(
        *(asyncio.to_thread(airfoils_backend.import_dat, "race-import", dat_text) for _ in range(8))
    )
    names = sorted(answer["name"] for answer in answers)
    expected = sorted(["race-import"] + [f"race-import-{suffix}" for suffix in range(2, 9)])
    assert names == expected, names
    print("ok   import atomicity: 8 concurrent imports of one name get 8 distinct suffixed names")


async def check_analyze(client: TestClient) -> None:
    response = await client.post("/api/airfoils/analyze", json=ANALYZE_BODY)
    assert response.status == 200, (response.status, await response.text())
    text = await response.text()
    assert "NaN" not in text and "Infinity" not in text, "a non-finite JSON constant reached the browser"
    payload = json.loads(text, parse_constant=_reject_json_constant)
    assert "op.alpha_deg" in payload["summary_text"], payload["summary_text"][:200]
    assert {"reynolds", "mach", "density_kg_m3", "pressure_pa", "speed_of_sound_m_s", "dynamic_viscosity_pa_s"} <= set(payload["flow"]), payload["flow"]
    assert [item["name"] for item in payload["airfoils"]] == ["naca2412", "naca0008"], payload["airfoils"]
    for item in payload["airfoils"]:
        assert item["source"] == "builtin", item
        sweep = item["sweep"]
        assert set(sweep) == {"alpha_deg", "cl", "cd", "cl_cd", "cm", "confidence"}, sorted(sweep)
        assert len(sweep["alpha_deg"]) == 41 and sweep["alpha_deg"][0] == -20.0 and sweep["alpha_deg"][-1] == 20.0, sweep["alpha_deg"][:3]
        assert all(len(sweep[key]) == 41 for key in sweep), {key: len(sweep[key]) for key in sweep}
        assert set(item["operating_point"]) == {"cl", "cd", "cm", "cl_cd", "top_xtr", "bot_xtr", "confidence"}, sorted(item["operating_point"])
        assert set(item["summary"]) == SUMMARY_KEYS, sorted(item["summary"])
    # naca0008's stall extrema sit below NeuralFoil's confidence threshold at this Re, so those summary fields are
    # non-finite in analyze_airfoils and must arrive as null; the finite ones stay numbers.
    summary = payload["airfoils"][1]["summary"]
    assert summary["alpha_pos_stall_deg"] is None and summary["cl_max"] is None, summary
    assert summary["alpha_neg_stall_deg"] is None and summary["cl_min"] is None, summary
    assert isinstance(summary["alpha_zero_lift_deg"], float), summary
    assert isinstance(payload["airfoils"][0]["summary"]["cd_min"], float), payload["airfoils"][0]["summary"]
    print("ok   analyze: flow, summary text, 41-point sweeps, per-airfoil summary; NaN fields arrive as null")


async def refusal_message(client: TestClient, body: dict) -> str:
    response = await client.post("/api/airfoils/analyze", json=body)
    assert response.status == 400, (response.status, await response.text())
    return (await response.json())["message"]


async def check_refusals(client: TestClient) -> None:
    response = await client.post("/api/airfoils/analyze", json={"airfoils": ["nope"]})
    assert response.status == 400, response.status
    assert "nope" in (await response.json())["message"]
    response = await client.post("/api/airfoils/analyze", json={"airfoils": []})
    assert response.status == 400 and (await response.json())["message"] == "No airfoils selected."
    print("ok   refusals: an unknown airfoil and an empty selection are 400")

    # Out-of-domain and wrong-typed operating points are 400 with the offending field named, before NeuralFoil.
    assert (await refusal_message(client, {"airfoils": ["naca2412"], "model_size": "gigantic"})).startswith("model_size must be one of")
    assert await refusal_message(client, {"airfoils": ["naca2412"], "airspeed_m_s": "fast"}) == "airspeed_m_s must be a number"
    assert await refusal_message(client, {"airfoils": ["naca2412"], "n_crit": True}) == "n_crit must be a number"
    assert await refusal_message(client, {"airfoils": ["naca2412"], "airspeed_m_s": 0.0}) == "airspeed_m_s must be > 0"
    assert await refusal_message(client, {"airfoils": ["naca2412"], "chord_mm": -1.0}) == "chord_mm must be > 0"
    assert await refusal_message(client, {"airfoils": ["naca2412"], "n_crit": -0.5}) == "n_crit must be >= 0"
    assert await refusal_message(client, {"airfoils": ["naca2412"], "xtr_upper": 1.5}) == "xtr_upper must be within [0, 1]"
    assert await refusal_message(client, {"airfoils": ["naca2412"], "xtr_lower": -0.1}) == "xtr_lower must be within [0, 1]"
    assert (await refusal_message(client, {"airfoils": ["naca2412"], "altitude_m": 1e6})).startswith("altitude_m must be within")
    assert (await refusal_message(client, {"airfoils": ["naca2412"], "temperature_c": 1000.0})).startswith("temperature_c must be within")
    print("ok   refusals: booleans, strings, bad model_size and out-of-domain values are 400 with field messages")

    # A compute failure anywhere inside analyze_airfoils becomes the same 400 shape, not a 500 snapshot.
    original = airfoils_backend.analyze_airfoils

    def explode(*_args: object, **_kwargs: object) -> dict:
        raise RuntimeError("boom")

    airfoils_backend.analyze_airfoils = explode
    try:
        message = await refusal_message(client, ANALYZE_BODY)
    finally:
        airfoils_backend.analyze_airfoils = original
    assert message == "Analysis failed: boom", message
    print("ok   compute failure: a NeuralFoil exception is a 400 'Analysis failed: ...'")


async def check_routes(state: AppState) -> None:
    app = create_app(state, ROOT / "web" / "dist")
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
        await check_library(client)
        await check_asb_seed(client)
        await check_import(client)
        await check_import_atomicity()
        await check_analyze(client)
        await check_refusals(client)
    finally:
        await client.close()


def main() -> None:
    # The airfoil routes never read the project; the example is only what an AppState needs to exist.
    state = AppState(Project(root=EXAMPLE))
    asyncio.run(check_routes(state))
    print("airfoil suite OK")


if __name__ == "__main__":
    main()
