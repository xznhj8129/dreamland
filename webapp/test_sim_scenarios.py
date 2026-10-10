"""Real HTTP -> Python scenario -> INAV SITL flights on a temporary copy of example.

Run with the drone Python: python -m webapp.test_sim_scenarios
The shared harness SITL must support stepped time. No mock bench or controller is used.
"""

import asyncio
import json
import tempfile
import time
from pathlib import Path

from aiohttp.test_utils import TestClient, TestServer
from inavbench.msp import Msp
from pymavlink import mavutil

from webapp.server import create_app
from webapp.test_sim import make_state
from webapp import sim

TIMEOUT_S = 600
ROOT = Path(__file__).resolve().parent.parent
STOP_SOURCE = '''from controllers import inav as controller

START = {"start": "air"}

def stop(flight):
    return flight.elapsed >= 2

def run(flight):
    controller.mode("ACRO")
    controller.takeoff(throttle=0.4)
    flight.stop_when(stop, "two seconds")
    flight.wait(10)
    flight.measure("must not execute")
'''
TIMEOUT_SOURCE = '''from controllers import inav as controller

START = {"start": "air"}

def never(flight):
    return False

def run(flight):
    controller.mode("ACRO")
    controller.takeoff(throttle=0.4)
    flight.wait(1, until=never)
    flight.measure("must not execute")
'''


async def result(client, running=False):
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        response = await client.get("/api/sim/state")
        payload = await response.json()
        if running:
            if any("Configurator session:" in line for line in payload["stdout"]["lines"]):
                return payload
        elif not payload["running"]:
            return payload
        await asyncio.sleep(0.2)
    raise TimeoutError("INAV scenario did not finish/boot within the test deadline")


async def save(client, name, source, kind="scenario"):
    response = await client.post("/api/sim/resource", json={"controller": "inav", "kind": kind, "name": name, "source": source, "create": True})
    assert response.status == 200, await response.text()


async def run(client, scenario, **options):
    response = await client.post("/api/sim/run", json={"controller": "inav", "scenario": scenario, "clock": "stepped-faster", **options})
    assert response.status == 200, await response.text()
    return await result(client)


async def main():
    with tempfile.TemporaryDirectory(prefix="quad-scenarios-") as temp:
        state = make_state(Path(temp))
        client = TestClient(TestServer(create_app(state, ROOT / "web/dist")))
        await client.start_server()
        try:
            source = (ROOT / "data/scenarios/inav/acro_climb.py").read_text()
            options = await (await client.get("/api/sim/options")).json()
            assert {"acro_climb", "controls", "efficiency", "glide", "top-speed"} <= set(options["scenarios"]["inav"]), options
            # A different aircraft-local binary must never change the harness firmware selection.
            state.project.sitl_dir.joinpath("SITL.elf").write_text("obsolete per-aircraft firmware")
            await save(client, "climb.py", source)
            payload = await run(client, "climb")
            assert payload["exit_code"] == 0, payload["stdout"]["lines"]
            assert "--start catapult" in payload["command"], payload["command"]
            points = json.loads((state.project.logs_dir / "inav_climb_catapult.measurements.json").read_text())
            assert [point["label"] for point in points] == ["off launcher", "200 m", "finished"], points
            assert points[1]["agl_m"] >= 200 and abs(points[1]["pitch"] - 20) < 3 and abs(points[1]["roll"]) < 2, points[1]
            header, rows = payload["log"]["header"], payload["log"]["rows"]
            assert len(rows) > 100 and any(row[header.index("armed")] == 1 for row in rows)
            assert all(row[header.index("mode_ANGLE")] == 0 for row in rows), "ACRO flight switched to ANGLE"
            print("ok   edited Python scenario: START catapult, RC ACRO climb to 200 m, 20° pitch / level roll, three measurement points")
            print(next(line for line in payload["stdout"]["lines"] if line.startswith("Flight clock:")))

            await save(client, "stop.py", STOP_SOURCE)
            payload = await run(client, "stop")
            assert payload["exit_code"] == 0 and "Scenario stopped: two seconds" in payload["stdout"]["lines"], payload
            assert not any("must not execute" in line for line in payload["stdout"]["lines"])
            print("ok   global stop condition interrupts a longer wait in simulated time")

            await save(client, "timeout.py", TIMEOUT_SOURCE)
            payload = await run(client, "timeout")
            assert payload["exit_code"] == 1 and any("TimeoutError: scenario condition not reached" in line for line in payload["stdout"]["lines"]), payload
            assert not any("must not execute" in line for line in payload["stdout"]["lines"])
            print("ok   unmet goal fails visibly at its timeout; next manoeuvre never runs")

            for name in ("glide", "controls", "top-speed", "efficiency"):
                payload = await run(client, name)
                assert payload["exit_code"] == 0, payload["stdout"]["lines"]
                measurements = state.project.logs_dir / f"inav_{name}_air.measurements.json"
                assert measurements.is_file() and json.loads(measurements.read_text()), payload["stdout"]["lines"]
                print(f"ok   shared {name} scenario: actual INAV flight and named measurements")

            response = await client.post("/api/sim/run", json={"controller": "inav", "scenario": "configurator", "start": "air"})
            assert response.status == 200, await response.text()
            payload = await result(client, running=True)
            msp = Msp(5761)
            try:
                assert len(msp.request(1)) >= 3, "Configurator port did not reply to MSP_API_VERSION"
            finally:
                msp.close()
            mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
            try:
                assert mavlink.recv_match(type="HEARTBEAT", blocking=True, timeout=5) is not None, "No real MAVLink heartbeat"
            finally:
                mavlink.close()
            await asyncio.sleep(0.5)
            payload = await (await client.get("/api/sim/state")).json()
            header, rows = payload["log"]["header"], payload["log"]["rows"]
            assert rows and all(row[header.index("armed")] == 0 for row in rows)
            assert max(row[header.index("north_m")] for row in rows) - min(row[header.index("north_m")] for row in rows) < 0.01
            await client.post("/api/sim/abort")
            payload = await result(client)
            assert payload["status"] == "Aborted" and payload["exit_code"] == 130, payload
            print("ok   configurator-only session: real MSP connection, disarmed, physics held, Abort stops SITL")

            response = await client.post("/api/sim/run", json={"controller": "inav", "scenario": "configurator", "start": "air", "msp": False, "mavlink": False})
            assert response.status == 200, await response.text()
            payload = await result(client, running=True)
            assert "GCS: MSP disabled, MAVLink disabled" in payload["stdout"]["lines"]
            for port in (5761, 5762):
                try:
                    link = Msp(port, timeout=0.5)
                except ConnectionRefusedError:
                    continue
                try:
                    try:
                        link.request(1)
                    except (TimeoutError, ConnectionError):
                        pass
                    else:
                        raise AssertionError(f"Disabled GCS link {port} still replied")
                finally:
                    link.close()
            await client.post("/api/sim/abort")
            payload = await result(client)
            assert payload["exit_code"] == 130
            print("ok   external MSP/MAVLink disabled while bench physics and FC logging continue")
        finally:
            sim.shutdown()
            await client.close()
    print("scenario integration suite OK")


if __name__ == "__main__":
    asyncio.run(main())
