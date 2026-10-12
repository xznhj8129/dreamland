"""Flight snapshots and optional selected telemetry for every scheduled scenario."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

from libs.fdm import pitch_rate_deg_s, roll_rate_deg_s, yaw_rate_deg_s
from libs.simrun import FT, electrical_columns

MAX_TELEMETRY_HZ = 50.0


@dataclass(frozen=True)
class TelemetrySpec:
    fields: tuple[str, ...]
    rate_hz: float

    @classmethod
    def read(cls, value):
        if not isinstance(value, dict) or set(value) != {"fields", "rate_hz"}:
            raise ValueError("TELEMETRY requires fields and rate_hz")
        fields, rate = value["fields"], value["rate_hz"]
        if not isinstance(fields, (list, tuple)) or not fields or any(not isinstance(field, str) for field in fields):
            raise ValueError("telemetry fields must be a nonempty list of column names")
        if len(set(fields)) != len(fields) or set(fields) & {"t", "phase"}:
            raise ValueError("telemetry fields must be unique; t and phase are included automatically")
        if not isinstance(rate, (int, float)) or not math.isfinite(rate) or not 0 < rate <= MAX_TELEMETRY_HZ:
            raise ValueError(f"telemetry rate_hz must be in (0, {MAX_TELEMETRY_HZ:g}]")
        return cls(tuple(fields), float(rate))


def telemetry_path(log_path: Path) -> Path:
    return log_path.with_suffix(".telemetry.csv")


def flight_log_path(project, controller, scenario, start):
    return project.logs_dir / f"{controller}_{scenario}_{start}.csv"


def flight_snapshot(engine, export):
    """The same simulator state and units for every controller."""
    fdm = engine.plant.fdm
    row = {"t": engine.flight.elapsed if engine.flight is not None else engine.elapsed, "armed": int("ARM" in engine.active_modes())} | engine.truth()
    row |= {"airspeed_m_s": fdm["velocities/vt-fps"] * FT, "aoa_deg": fdm["aero/alpha-deg"],
            "sideslip_deg": fdm["aero/beta-deg"],
            "p_deg_s": roll_rate_deg_s(fdm), "q_deg_s": pitch_rate_deg_s(fdm),
            "r_deg_s": yaw_rate_deg_s(fdm)}
    motor_count = len(export.motor_properties)
    row |= {f"motor{index + 1}": value for index, value in enumerate(engine.motors[:motor_count])}
    row |= {f"servo{index}": value for index, value in enumerate(engine.servos[:len(export.servo_properties)])}
    if export.motors:
        row |= electrical_columns(fdm, export, motor_count)
    return row


class TelemetryRecorder:
    def __init__(self, flight, spec, path):
        self.flight, self.spec = flight, spec
        self.next_s = flight.elapsed
        available = flight.state
        missing = set(spec.fields) - available.keys()
        if missing:
            raise ValueError(f"Unknown/unavailable telemetry fields: {sorted(missing)}; available: {sorted(available)}")
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=["t", "phase", *spec.fields])
        self.writer.writeheader()

    def sample(self):
        flight = self.flight
        if flight.elapsed < self.next_s - 1e-8:
            return
        values = flight.state
        self.writer.writerow({"t": flight.elapsed, "phase": flight.current_phase,
                              **{field: values[field] for field in self.spec.fields}})
        self.file.flush()
        self.next_s = flight.elapsed + 1 / self.spec.rate_hz

    def close(self):
        self.file.close()
