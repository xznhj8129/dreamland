"""
Usage:
from libs.rocketmotors import rocket_motor, rocket_motor_names
motor = rocket_motor("AeroTech_F42T")

Solid rocket motors: one RASP (.eng) file each in data/rocket_motors/, named by the file stem. RASP is the format
thrustcurve.org serves and OpenRocket and RockSim read: ';' comment lines, one header line
(designation, diameter mm, length mm, delays, propellant kg, total kg, manufacturer), then time (s) / thrust (N)
pairs from ignition, the last at 0 N. Ignition at (0 s, 0 N) is implied.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

ROCKET_MOTORS_DIR = Path(__file__).resolve().parent.parent / "data" / "rocket_motors"
RASP_SUFFIX = ".eng"
MM_M = 0.001


class RocketMotorError(ValueError):
    """A motor file is missing or not a valid single-motor RASP file."""


@dataclass(frozen=True)
class RocketMotor:
    name: str  # file stem, how a definition refers to it
    designation: str  # e.g. F42T
    manufacturer: str
    diameter_m: float
    length_m: float
    delays: str  # as the file gives them, e.g. "4-8"; "P" = plugged
    propellant_kg: float
    total_kg: float  # loaded: casing + propellant
    curve: tuple[tuple[float, float], ...]  # (s, N) from (0, 0) at ignition to burnout at 0 N

    @property
    def casing_kg(self) -> float:
        return self.total_kg - self.propellant_kg

    @property
    def burn_time_s(self) -> float:
        return self.curve[-1][0]

    @property
    def peak_thrust_n(self) -> float:
        return max(thrust for _t, thrust in self.curve)

    def cumulative_impulse_ns(self) -> tuple[float, ...]:
        """Impulse delivered by each curve point's time (trapezoids between the points)."""
        total, impulses = 0.0, [0.0]
        for (t0, f0), (t1, f1) in zip(self.curve, self.curve[1:]):
            total += (f0 + f1) / 2.0 * (t1 - t0)
            impulses.append(total)
        return tuple(impulses)

    @property
    def total_impulse_ns(self) -> float:
        return self.cumulative_impulse_ns()[-1]

    @property
    def average_thrust_n(self) -> float:
        return self.total_impulse_ns / self.burn_time_s


def parse_rasp(text: str, name: str) -> RocketMotor:
    lines = [line.strip() for line in text.splitlines() if line.strip() and not line.strip().startswith(";")]
    if not lines:
        raise RocketMotorError(f"{name}: no header line")
    header = lines[0].split()
    if len(header) != 7:
        raise RocketMotorError(f"{name}: header needs 7 fields (designation, diameter, length, delays, propellant kg, total kg, manufacturer), got {lines[0]!r}")
    designation, diameter, length, delays, propellant, total, manufacturer = header
    try:
        diameter_m, length_m, propellant_kg, total_kg = float(diameter) * MM_M, float(length) * MM_M, float(propellant), float(total)
        points = [tuple(float(value) for value in line.split()) for line in lines[1:]]
    except ValueError as error:
        raise RocketMotorError(f"{name}: {error}") from error
    if any(len(point) != 2 for point in points):
        raise RocketMotorError(f"{name}: a data line is not one time and one thrust; one motor per file")
    if min(diameter_m, length_m, propellant_kg) <= 0.0 or total_kg <= propellant_kg:
        raise RocketMotorError(f"{name}: diameter, length and propellant must be positive and the total mass above the propellant mass")
    curve = ((0.0, 0.0), *points)
    if len(points) < 2 or any(t1 <= t0 for (t0, _), (t1, _) in zip(curve, curve[1:])) or any(f < 0.0 for _, f in points) or points[-1][1] != 0.0:
        raise RocketMotorError(f"{name}: thrust points must have increasing times after 0, thrust >= 0, and end at 0 N")
    return RocketMotor(name=name, designation=designation, manufacturer=manufacturer, diameter_m=diameter_m, length_m=length_m,
                       delays=delays, propellant_kg=propellant_kg, total_kg=total_kg, curve=curve)


def rocket_motor_path(name: str) -> Path:
    return ROCKET_MOTORS_DIR / f"{name}{RASP_SUFFIX}"


def rocket_motor_names() -> list[str]:
    return sorted(path.stem for path in ROCKET_MOTORS_DIR.glob(f"*{RASP_SUFFIX}"))


def rocket_motor(name: str) -> RocketMotor:
    if not name or Path(name).name != name or not rocket_motor_path(name).is_file():
        raise RocketMotorError(f"{name!r} is not in data/rocket_motors/ {rocket_motor_names()}")
    return parse_rasp(rocket_motor_path(name).read_text(), name)
