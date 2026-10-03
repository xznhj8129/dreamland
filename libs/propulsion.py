"""
Usage:
from libs.propulsion import motor_model, propulsion_summary
model = motor_model(resolved, parts_data)
summary = propulsion_summary(resolved, parts_data)

The propulsion model shared by the GUI, the report and the JSBSim export: motorlib (libs/motorlib, the rcplanes.online
multirotor calculator: its motor and prop data and formulas) at full throttle, static, on the definition's battery, with
the resistance between the cells and each motor worked out from the design (wiring_resistance). Throttle is ESC duty,
as INAV's motor output (DShot): motor voltage = duty x pack voltage.

Below full throttle, thrust and shaft power follow motorlib's power laws in RPM (the prop's texp and pexp); current and
duty come from the motor constants that reproduce motorlib's full-throttle point (MotorModel), as in the JSBSim motor.
Not modelled: pack voltage under load other than its internal resistance (nominal cell voltage), ESC switching losses,
and anything a part's data leaves out (named in WiringResistance.not_modelled).
"""

import json
from dataclasses import dataclass
from functools import cache

import numpy as np

from libs.airframe import INCH_M, SPECS_PATH, motor_repo
from libs.geometry import STANDARD_GRAVITY_M_S2, ResolvedAirframe, route_length_m
from libs.motorlib import MotorSim
from libs.motorlib.rcplanes_multirotor import COPPER_RHO, G_ACCEL

NOMINAL_CELL_V = {"lipo": 3.7, "liion": 3.6}
AIR_TEMP_C = 15.0  # ISA sea level: motorlib's density is then 1.225 kg/m^3
MOTOR_TEMP_C = 20.0  # motorlib's reference for the listed winding resistance
CURVE_POINTS = 101
RPM_SEARCH_FACTOR = 1.5  # duty is inverted on an RPM grid up to this x the full-throttle RPM
PLUG_ABSENT = ("soldered", "")  # wire_paths plug_* values that are not a connector
CONTACTS_PER_PLUG = 2  # in the current path: + and - (battery plugs), in and out (breaker plug), 2 of 3 pins (motor plug)
MOTOR_LEAD_CONDUCTORS = 2  # six-step commutation: one phase wire in, one out, the third floating
ESC_SWITCHES_IN_PATH = 2  # one high-side and one low-side MOSFET


@cache
def wire_specs() -> dict:
    with open(SPECS_PATH, "r") as file:
        specs = json.load(file)
    return {"paths": specs["wire_paths"], "connectors": specs["connectors"]}


def library_part(resolved: ResolvedAirframe, parts_data: dict, name: str) -> dict:
    """data/parts.json entry of a placed part, by the placed part's name."""
    placed = next(part for part in resolved.airframe.parts if part.name == name)
    category, _, part_name = placed.part.partition(":")
    return parts_data[category][part_name]


def copper_ohm(length_m: float, gauge: int, conductors: int) -> float:
    """Copper at 20 C, motorlib's resistivity and AWG areas."""
    return COPPER_RHO * conductors * length_m / motor_repo().wire(gauge).area_m2


def plugs_ohm(path: dict) -> float:
    connectors = wire_specs()["connectors"]
    return sum(CONTACTS_PER_PLUG * connectors[value]["contact_resistance_ohm"]
               for key, value in path.items() if key.startswith("plug_") and value not in PLUG_ABSENT)


@dataclass(frozen=True)
class WiringResistance:
    """Resistance between the cells and one motor, ohm. The pack, battery leads and battery plugs carry every motor's
    current, so a motor's voltage drop sees them times the motor count (equal currents)."""
    motor_count: int
    pack_ohm: float  # s x per-cell / p
    battery_leads_ohm: float  # batt_to_esc, + and -
    battery_plugs_ohm: float
    motor_leads_ohm: float  # esc_to_motor, per motor
    motor_plugs_ohm: float
    esc_ohm: float
    battery_route_m: float
    motor_routes_m: tuple[float, ...]  # per motor; motor_leads_ohm is for their mean (one motor model for all)
    not_modelled: tuple[str, ...]

    @property
    def shared_ohm(self) -> float:
        return self.pack_ohm + self.battery_leads_ohm + self.battery_plugs_ohm

    @property
    def per_motor_ohm(self) -> float:
        return self.motor_leads_ohm + self.motor_plugs_ohm + self.esc_ohm + self.motor_count * self.shared_ohm


def wiring_resistance(resolved: ResolvedAirframe, parts_data: dict) -> WiringResistance:
    """The wire runs of resolved.wires (battery part to ESC part, ESC part to each motor) with the gauges and plugs of
    data/specs.json wire_paths batt_to_esc and esc_to_motor. One motor model serves all motors, so the motor leads are
    taken at the mean route (an off-centre ESC makes them differ)."""
    propulsion = resolved.airframe.propulsion
    paths = wire_specs()["paths"]
    battery = library_part(resolved, parts_data, propulsion.battery)
    esc = library_part(resolved, parts_data, propulsion.esc)
    battery_route = route_length_m(resolved.wires.battery)
    motor_routes = tuple(route_length_m(route) for route in resolved.wires.motors)
    not_modelled = []
    if "ir_mohm_per_cell" in battery:
        pack_ohm = battery["s"] * battery["ir_mohm_per_cell"] / 1000.0 / battery["p"]
    else:
        pack_ohm = 0.0
        not_modelled.append("pack internal resistance (no ir_mohm_per_cell)")
    if "mosfet_rds_on_ohm" in esc:
        esc_ohm = ESC_SWITCHES_IN_PATH * esc["mosfet_rds_on_ohm"]
    else:
        esc_ohm = 0.0
        not_modelled.append("ESC resistance (no mosfet_rds_on_ohm)")
    battery_path, motor_path = paths["batt_to_esc"], paths["esc_to_motor"]
    return WiringResistance(
        motor_count=len(resolved.motors), pack_ohm=pack_ohm,
        battery_leads_ohm=copper_ohm(battery_route, battery_path["wires_ga"], battery_path["wires_n"]), battery_plugs_ohm=plugs_ohm(battery_path),
        motor_leads_ohm=copper_ohm(sum(motor_routes) / len(motor_routes), motor_path["wires_ga"], MOTOR_LEAD_CONDUCTORS),
        motor_plugs_ohm=plugs_ohm(motor_path), esc_ohm=esc_ohm, battery_route_m=battery_route, motor_routes_m=motor_routes,
        not_modelled=tuple(not_modelled),
    )


@dataclass(frozen=True)
class MotorModel:
    """motorlib's full-throttle static point, and the DC motor constants that reproduce it (V = rpm / kv + I x R,
    shaft power = (I - I0) x rpm / kv): R is the wiring plus motorlib's loaded winding resistance (Rm with its
    current and Krpm factors) at that point."""
    label: str
    prop: str
    cells: int
    parallel: int
    pack_mah: float
    pack_wh: float  # nominal
    pack_voltage_v: float  # nominal
    diameter_m: float
    pitch_m: float
    rho: float
    kv_rpm_v: float
    resistance_ohm: float
    no_load_current_a: float
    full_rpm: float
    full_thrust_n: float
    full_current_a: float  # motor current
    full_shaft_w: float  # (I - I0) x rpm / kv
    ct0: float
    cp0: float
    texp: float  # thrust ~ rpm^texp
    pexp: float  # shaft power ~ rpm^pexp
    wiring: WiringResistance
    warnings: tuple[str, ...]  # motorlib's


@dataclass(frozen=True)
class MotorPoints:
    """One motor, static, at a pack voltage; arrays or 0-d arrays. Battery side of a lossless ESC."""
    duty: np.ndarray
    rpm: np.ndarray
    thrust_n: np.ndarray
    current_a: np.ndarray
    power_w: np.ndarray


def motor_model(resolved: ResolvedAirframe, parts_data: dict) -> MotorModel:
    airframe = resolved.airframe
    propulsion = airframe.propulsion
    battery = library_part(resolved, parts_data, propulsion.battery)
    cell_v = NOMINAL_CELL_V[battery["type"]]
    wiring = wiring_resistance(resolved, parts_data)
    sim = MotorSim(
        repo=motor_repo(), cell=None, cell_volt=cell_v, controller=None, wire_gauge=None, wire_length_m=0.0,
        external_resistance_ohm=wiring.per_motor_ohm, motor_mfg=propulsion.motor.manufacturer, motor_type=propulsion.motor.name,
        propeller=propulsion.prop, cells=int(battery["s"]), parallel=int(battery["p"]), motors=len(resolved.motors),
        altitude_m=0.0, temp_c=AIR_TEMP_C, wire_temp_c=AIR_TEMP_C, temp_motor_c=MOTOR_TEMP_C,
    )
    static = dict(sim.static)
    pack_v = float(static["pack_v"])
    rpm, current, io = float(static["motor_rpm"]), float(static["motor_current_a"]), float(static["motor_io_eff"])
    kv = sim.motor.kv
    shaft = (current - io) * rpm / kv
    rho, diameter = float(static["alt_density"]), sim.prop.diam_in * INCH_M
    revs = rpm / 60.0
    thrust = float(static["thrust_g"]) / len(resolved.motors) / 1000.0 * G_ACCEL
    return MotorModel(
        label=f"{propulsion.motor.manufacturer} {propulsion.motor.name} ({kv:g} KV)", prop=propulsion.prop, cells=int(battery["s"]),
        parallel=int(battery["p"]), pack_mah=battery["mah"] * battery["p"], pack_wh=battery["mah"] * battery["p"] / 1000.0 * pack_v,
        pack_voltage_v=pack_v, diameter_m=diameter, pitch_m=sim.prop.pitch_in * INCH_M, rho=rho, kv_rpm_v=kv,
        resistance_ohm=(pack_v - rpm / kv) / current, no_load_current_a=io, full_rpm=rpm, full_thrust_n=thrust, full_current_a=current,
        full_shaft_w=shaft, ct0=thrust / (rho * revs ** 2 * diameter ** 4), cp0=shaft / (rho * revs ** 3 * diameter ** 5),
        texp=sim.prop.texp, pexp=sim.prop.pexp, wiring=wiring, warnings=tuple(static["warnings"]),
    )


def static_points(model: MotorModel, rpm, pack_voltage_v: float) -> MotorPoints:
    rpm = np.asarray(rpm, dtype=float)
    ratio = rpm / model.full_rpm
    back_emf = rpm / model.kv_rpm_v
    with np.errstate(divide="ignore", invalid="ignore"):
        motor_a = np.where(rpm > 0.0, model.full_shaft_w * ratio ** model.pexp / back_emf + model.no_load_current_a, 0.0)
    voltage = np.where(rpm > 0.0, back_emf + motor_a * model.resistance_ohm, 0.0)
    power = voltage * motor_a
    return MotorPoints(duty=voltage / pack_voltage_v, rpm=rpm, thrust_n=model.full_thrust_n * ratio ** model.texp,
                       current_a=power / pack_voltage_v, power_w=power)


def rpm_for_thrust(model: MotorModel, thrust_n):
    return model.full_rpm * (np.asarray(thrust_n, dtype=float) / model.full_thrust_n) ** (1.0 / model.texp)


def rpm_for_duty(model: MotorModel, duty, pack_voltage_v: float):
    grid = np.linspace(0.0, RPM_SEARCH_FACTOR * model.full_rpm, 4000)
    return np.interp(duty, static_points(model, grid, pack_voltage_v).duty, grid)


@dataclass(frozen=True)
class PropulsionSummary:
    model: MotorModel
    motor_count: int
    pack_voltage_v: float  # nominal
    weight_n: float  # nominal mass (ranged masses at the middle)
    curve: MotorPoints  # duty 0..1
    full: MotorPoints  # duty 1
    thrust_to_weight: float
    vtol: bool
    hover: MotorPoints | None  # VTOL only; None when full thrust cannot lift the weight
    hover_time_min: float | None
    full_throttle_time_min: float


def propulsion_summary(resolved: ResolvedAirframe, parts_data: dict) -> PropulsionSummary:
    model = motor_model(resolved, parts_data)
    pack = model.pack_voltage_v
    count = len(resolved.motors)
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    full = static_points(model, rpm_for_duty(model, 1.0, pack), pack)
    vtol = resolved.airframe.vtol
    hover = static_points(model, rpm_for_thrust(model, weight_n / count), pack) if vtol and full.thrust_n * count >= weight_n else None
    return PropulsionSummary(
        model=model, motor_count=count, pack_voltage_v=pack, weight_n=weight_n,
        curve=static_points(model, rpm_for_duty(model, np.linspace(0.0, 1.0, CURVE_POINTS), pack), pack), full=full,
        thrust_to_weight=float(full.thrust_n) * count / weight_n, vtol=vtol, hover=hover,
        hover_time_min=None if hover is None else model.pack_wh / (float(hover.power_w) * count) * 60.0,
        full_throttle_time_min=model.pack_wh / (float(full.power_w) * count) * 60.0,
    )


def summary_lines(summary: PropulsionSummary) -> list[str]:
    """Plain-text static propulsion, totals over all motors; throttle as INAV motor output (duty)."""
    model, full, hover, count = summary.model, summary.full, summary.hover, summary.motor_count
    wiring = model.wiring
    grams = 1000.0 / STANDARD_GRAVITY_M_S2
    milli = 1000.0
    routes = sorted(wiring.motor_routes_m)
    motor_routes = f"{routes[0]:.3f} m" if routes[-1] - routes[0] < 0.0005 else f"mean of {routes[0]:.3f}..{routes[-1]:.3f} m"

    def efficiency(points: MotorPoints) -> float:
        return float(points.thrust_n) * grams / float(points.power_w)

    lines = [
        f"{count} x {model.label}, {model.prop} (motorlib); {model.cells}S{model.parallel}P {model.pack_mah:.0f} mAh, {model.pack_wh:.1f} Wh, "
        f"{summary.pack_voltage_v:.1f} V nominal",
        f"Resistance    : {wiring.per_motor_ohm * milli:.1f} mOhm per motor = motor leads {wiring.motor_leads_ohm * milli:.1f} "
        f"({motor_routes}) + motor plug {wiring.motor_plugs_ohm * milli:.1f} + ESC {wiring.esc_ohm * milli:.1f} + {count} x shared "
        f"{wiring.shared_ohm * milli:.1f} (pack {wiring.pack_ohm * milli:.1f}, battery leads {wiring.battery_leads_ohm * milli:.1f} "
        f"({wiring.battery_route_m:.3f} m), battery plugs {wiring.battery_plugs_ohm * milli:.1f})",
        f"Full throttle : {float(full.thrust_n) * count * grams:.0f} g, {float(full.current_a) * count:.1f} A, {float(full.power_w) * count:.0f} W; "
        f"{float(full.rpm):.0f} RPM, pitch speed {float(full.rpm) / 60.0 * model.pitch_m * 3.6:.0f} km/h, {efficiency(full):.2f} g/W, "
        f"{summary.full_throttle_time_min:.1f} min",
        f"Thrust/weight : {summary.thrust_to_weight:.2f} at {summary.weight_n * grams:.0f} g",
    ]
    if wiring.not_modelled:
        lines.append(f"Not modelled  : {'; '.join(wiring.not_modelled)}")
    if model.warnings:
        lines.append(f"motorlib      : {'; '.join(model.warnings)}")
    if not summary.vtol:
        pass
    elif hover is None:
        lines.append("Hover         : not possible, full thrust is below the weight")
    else:
        lines.append(f"Hover         : motor output {float(hover.duty):.3f}, {float(hover.current_a) * count:.1f} A, {float(hover.power_w) * count:.0f} W, "
                     f"{efficiency(hover):.2f} g/W, {summary.hover_time_min:.1f} min")
    return lines
