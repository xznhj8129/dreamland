"""MCU hardware connection: PWM outputs and simulated launcher release."""

from inavbench import Catapult
from inavbench.vehicles import Plant

from libs.direct_flight import DirectTransport
from libs.scenario import CONTROL_PERIOD_S
from libs.simrun import build_vehicle

engine = None
esc = None
servo = None


def connect(project, resolved, export, args, state, hardware, log_path):
    plant = Plant(build_vehicle(export, "", args.start, state, resolved.airframe.tailsitter),
                  heading_deg=state.heading_deg, home=(state.lat_deg, state.lon_deg), launcher=state.launcher)
    plant.hover_frame = False
    return DirectTransport(plant, export, log_path, hardware, args.rate, args.clock == "realtime")


def bind(flight):
    global engine, esc, servo
    engine = flight._engine
    esc, servo = flight.sensors.esc, flight.sensors.servo


def arm():
    """Enable the simulated PWM outputs; no flight-control logic runs here."""
    engine.arm()


def release():
    if engine.plant.rail:
        if isinstance(engine.plant.rail.launcher, Catapult):
            engine.run(1.0)
            engine.plant.fire()
            print(f"catapult fired at {engine.flight.elapsed:.1f} s simulated", flush=True)
        while engine.plant.on_rail:
            engine.run(CONTROL_PERIOD_S)


def outputs(*, motors_us, servos_us):
    engine.outputs(motors_us, servos_us)
