"""
Usage:
from libs.propulsion import motor_model, propulsion_summary
model = motor_model(airframe, parts_data)
summary = propulsion_summary(resolved, parts_data)

The propulsion model shared by the GUI, the report and the JSBSim export: a DC motor + prop fitted to the published
bench rows (libs/motordata.csv) for the definition's motor, prop and battery (fit_motor). Throttle is ESC duty, as
INAV's motor output (DShot): duty = motor voltage / pack voltage, duty 1 = the bench's full-throttle row. The bench
data's own "throttle %" is not used: the real m62v1 hover (log 2025-08-22) only fits INAV output as duty.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from libs.airframe import MOTORDATA_PATH, Airframe
from libs.geometry import STANDARD_GRAVITY_M_S2, ResolvedAirframe
from libs.simlib2 import TestedMotorData

INCH_M = 0.0254
BENCH_AIR_DENSITY_KG_M3 = 1.225  # ISA sea level; the bench conditions were not recorded
NOMINAL_CELL_V = 3.7
CURVE_POINTS = 101
RPM_SEARCH_FACTOR = 1.5  # duty is inverted on an RPM grid up to this x the highest bench RPM


@dataclass(frozen=True)
class MotorFit:
    """DC motor + prop fitted to bench rows: motor current = shaft power / back-EMF + no-load current, motor voltage =
    back-EMF + current x resistance, battery power = motor power (lossless ESC), duty 1 at full throttle."""
    kv_rpm_v: float
    resistance_ohm: float
    no_load_current_a: float
    full_voltage_v: float  # pack voltage under load at full throttle
    ct0: float
    cp0: float
    battery_current_error: float  # largest relative battery-current misfit over the rows


def fit_motor(points: dict[str, np.ndarray], kv_rpm_v: float, diameter_m: float, rho: float) -> MotorFit:
    revs = points["rpm"] / 60.0
    back_emf = points["rpm"] / kv_rpm_v
    voltage, current = points["voltage_v"], points["current_a"]
    ct0 = float(np.mean(points["thrust_g"] / 1000.0 * STANDARD_GRAVITY_M_S2 / (rho * revs ** 2 * diameter_m ** 4)))
    resistance = float((voltage[-1] - back_emf[-1]) / current[-1])

    def motor_current(no_load_a: float) -> tuple[np.ndarray, float]:
        cp0 = (current[-1] - no_load_a) * back_emf[-1] / (rho * revs[-1] ** 3 * diameter_m ** 5)
        return cp0 * rho * revs ** 3 * diameter_m ** 5 / back_emf + no_load_a, cp0

    def battery_current_misfit(no_load: np.ndarray) -> np.ndarray:
        motor_a, _cp0 = motor_current(float(no_load[0]))
        return ((back_emf + motor_a * resistance) * motor_a / voltage - current) / current

    no_load = float(least_squares(battery_current_misfit, [1.0], bounds=([0.0], [float(current[-1])])).x[0])
    _motor_a, cp0 = motor_current(no_load)
    return MotorFit(
        kv_rpm_v=kv_rpm_v, resistance_ohm=resistance, no_load_current_a=no_load, full_voltage_v=float(voltage[-1]),
        ct0=ct0, cp0=float(cp0), battery_current_error=float(np.max(np.abs(battery_current_misfit(np.array([no_load]))))),
    )


@dataclass(frozen=True)
class MotorModel:
    label: str
    prop: str
    cells: int
    pack_wh: float  # nominal, NOMINAL_CELL_V per cell
    diameter_m: float
    pitch_m: float
    rho: float
    fit: MotorFit
    bench: dict[str, np.ndarray]  # the measured rows (TestedMotorData.measured_points)


@dataclass(frozen=True)
class MotorPoints:
    """One motor, static, at a pack voltage; arrays or 0-d arrays. Battery side of a lossless ESC."""
    duty: np.ndarray
    rpm: np.ndarray
    thrust_n: np.ndarray
    current_a: np.ndarray
    power_w: np.ndarray


def motor_model(airframe: Airframe, parts_data: dict) -> MotorModel:
    propulsion = airframe.propulsion
    motor = propulsion.motor
    battery_volume = next(volume for volume in airframe.volumes if volume.name == propulsion.battery_volume)
    battery = parts_data["battery"][battery_volume.part.partition(":")[2]]
    cells = int(battery["s"])
    tested = TestedMotorData(brand=motor.brand, motor=motor.model, kv=motor.kv, batt_s=cells, csv_path=MOTORDATA_PATH)
    bench = tested.measured_points(motor.prop)
    info = tested.prop_info(motor.prop)
    diameter_m = info["diameter_in"] * INCH_M
    return MotorModel(
        label=f"{motor.brand} {motor.model} {motor.kv}KV", prop=motor.prop, cells=cells,
        pack_wh=battery["mah"] / 1000.0 * cells * NOMINAL_CELL_V * battery["p"], diameter_m=diameter_m,
        pitch_m=info["pitch_in"] * INCH_M, rho=BENCH_AIR_DENSITY_KG_M3,
        fit=fit_motor(bench, float(motor.kv), diameter_m, BENCH_AIR_DENSITY_KG_M3), bench=bench,
    )


def static_points(model: MotorModel, rpm, pack_voltage_v: float) -> MotorPoints:
    fit = model.fit
    rpm = np.asarray(rpm, dtype=float)
    revs = rpm / 60.0
    back_emf = rpm / fit.kv_rpm_v
    with np.errstate(divide="ignore", invalid="ignore"):
        motor_a = np.where(rpm > 0.0, fit.cp0 * model.rho * revs ** 3 * model.diameter_m ** 5 / back_emf + fit.no_load_current_a, 0.0)
    voltage = np.where(rpm > 0.0, back_emf + motor_a * fit.resistance_ohm, 0.0)
    power = voltage * motor_a
    return MotorPoints(duty=voltage / pack_voltage_v, rpm=rpm, thrust_n=fit.ct0 * model.rho * revs ** 2 * model.diameter_m ** 4,
                       current_a=power / pack_voltage_v, power_w=power)


def rpm_for_thrust(model: MotorModel, thrust_n):
    return 60.0 * np.sqrt(np.asarray(thrust_n, dtype=float) / (model.fit.ct0 * model.rho * model.diameter_m ** 4))


def rpm_for_duty(model: MotorModel, duty, pack_voltage_v: float):
    grid = np.linspace(0.0, RPM_SEARCH_FACTOR * float(model.bench["rpm"].max()), 4000)
    return np.interp(duty, static_points(model, grid, pack_voltage_v).duty, grid)


@dataclass(frozen=True)
class PropulsionSummary:
    model: MotorModel
    motor_count: int
    pack_voltage_v: float  # under load at full throttle (the bench's)
    weight_n: float  # nominal mass (ranged masses at the middle)
    curve: MotorPoints  # duty 0..1
    bench_duty: np.ndarray  # the model's duty at each measured row's RPM, to compare the rows with the curve
    full: MotorPoints  # duty 1
    thrust_to_weight: float
    vtol: bool
    hover: MotorPoints | None  # VTOL only; None when full thrust cannot lift the weight
    hover_time_min: float | None
    full_throttle_time_min: float


def propulsion_summary(resolved: ResolvedAirframe, parts_data: dict) -> PropulsionSummary:
    model = motor_model(resolved.airframe, parts_data)
    pack = model.fit.full_voltage_v
    count = len(resolved.motors)
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    full = static_points(model, rpm_for_duty(model, 1.0, pack), pack)
    vtol = resolved.airframe.vtol
    hover = static_points(model, rpm_for_thrust(model, weight_n / count), pack) if vtol and full.thrust_n * count >= weight_n else None
    return PropulsionSummary(
        model=model, motor_count=count, pack_voltage_v=pack, weight_n=weight_n,
        curve=static_points(model, rpm_for_duty(model, np.linspace(0.0, 1.0, CURVE_POINTS), pack), pack),
        bench_duty=static_points(model, model.bench["rpm"], pack).duty, full=full,
        thrust_to_weight=float(full.thrust_n) * count / weight_n, vtol=vtol, hover=hover,
        hover_time_min=None if hover is None else model.pack_wh / (float(hover.power_w) * count) * 60.0,
        full_throttle_time_min=model.pack_wh / (float(full.power_w) * count) * 60.0,
    )


def summary_lines(summary: PropulsionSummary) -> list[str]:
    """Plain-text static propulsion, totals over all motors; throttle as INAV motor output (duty)."""
    model, full, hover, count = summary.model, summary.full, summary.hover, summary.motor_count
    grams = 1000.0 / STANDARD_GRAVITY_M_S2

    def efficiency(points: MotorPoints) -> float:
        return float(points.thrust_n) * grams / float(points.power_w)

    lines = [
        f"{count} x {model.label}, {model.prop}, {model.cells}S {model.pack_wh:.1f} Wh nominal, {summary.pack_voltage_v:.2f} V under load",
        f"Full throttle : {float(full.thrust_n) * count * grams:.0f} g, {float(full.current_a) * count:.1f} A, {float(full.power_w) * count:.0f} W; "
        f"{float(full.rpm):.0f} RPM, pitch speed {float(full.rpm) / 60.0 * model.pitch_m * 3.6:.0f} km/h, {efficiency(full):.2f} g/W, "
        f"{summary.full_throttle_time_min:.1f} min",
        f"Thrust/weight : {summary.thrust_to_weight:.2f} at {summary.weight_n * grams:.0f} g",
    ]
    if not summary.vtol:
        pass
    elif hover is None:
        lines.append("Hover         : not possible, full thrust is below the weight")
    else:
        lines.append(f"Hover         : motor output {float(hover.duty):.3f}, {float(hover.current_a) * count:.1f} A, {float(hover.power_w) * count:.0f} W, "
                     f"{efficiency(hover):.2f} g/W, {summary.hover_time_min:.1f} min")
    return lines
