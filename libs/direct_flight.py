"""PWM transport to JSBSim, with shared sensor sampling, clock and logging."""

import csv
import math
import time
from pathlib import Path

from libs.flightlog import flight_snapshot

LOG_PERIOD_S = 0.05


class DirectTransport:
    """JSBSim transport ticks feed the common flight scheduler."""

    def __init__(self, plant, export, log_path: Path, hardware, rate_hz, realtime):
        self.plant, self.export, self.hardware = plant, export, hardware
        self.step_s, self.realtime = 1 / rate_hz, realtime
        self.flight = None
        self._wall_start = time.monotonic()
        self.elapsed = 0.0
        self.armed = False
        self.motors = [1000] * len(export.motor_properties)
        self.servos = [1500] * len(export.servo_properties)
        self.next_log_s = 0.0
        hardware._sample(plant, 0)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = log_path.open("w", newline="")
        self.writer = csv.DictWriter(self.log_file, fieldnames=list(self.row()))
        self.writer.writeheader()

    def truth(self):
        return self.plant.truth()

    def arm(self):
        self.armed = True

    def active_modes(self):
        return {"ARM"} if self.armed else set()

    def outputs(self, motors_us, servos_us):
        if len(motors_us) != len(self.motors) or len(servos_us) != len(self.servos):
            raise ValueError("outputs must specify every motor and servo")
        if any(not math.isfinite(value) or not 1000 <= value <= 2000 for value in (*motors_us, *servos_us)):
            raise ValueError("motor and servo outputs must be finite PWM values in 1000..2000 us")
        self.hardware.esc[:], self.hardware.servo[:] = motors_us, servos_us

    def row(self):
        return flight_snapshot(self, self.export) | self.hardware._sensor_columns() | self.hardware._logged

    def run(self, seconds):
        end = self.elapsed + seconds
        while self.elapsed < end - 1e-9:
            if self.armed:
                self.motors, self.servos = self.hardware._outputs_us()
            self.plant.set_outputs(self.motors, self.servos)
            self.plant.advance(min(self.step_s, end - self.elapsed))
            self.elapsed = self.plant.time
            self.hardware._sample(self.plant, round(self.elapsed * 1e6))
            if self.flight is not None:
                self.flight._tick()
            if self.realtime:
                delay = self.elapsed - (time.monotonic() - self._wall_start)
                if delay > 0:
                    time.sleep(delay)
            if self.elapsed >= self.next_log_s:
                self.writer.writerow(self.row())
                self.log_file.flush()
                self.next_log_s += LOG_PERIOD_S

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.log_file.close()
