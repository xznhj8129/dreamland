"""Hold the aircraft disarmed while an external configurator is connected."""

from controllers import inav as controller

START = {"start": "air"}


def run(flight):
    controller.engine.hold = True
    controller.engine.disarm()
    print("Configurator session: physics held, aircraft disarmed. Abort to stop.", flush=True)
    while True:
        flight.wait(1.0)
