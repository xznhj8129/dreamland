"""
Usage:
from libs.propulsion import motor_models, propulsion_summary
models = motor_models(resolved, parts_data)  # one per motor, in motor order
summary = propulsion_summary(resolved, parts_data)

The propulsion model shared by the GUI, the report and the JSBSim export: motorlib (libs/motorlib, the rcplanes.online
multirotor calculator: its motor and prop data and formulas) at full throttle, static, on the definition's battery, with
the resistance between the cells and each motor worked out from the design: its own wire run (propulsion.wiring, the
route from geometry) and the shared battery run (wiring_resistances). Throttle is ESC duty, as INAV's motor output
(DShot): motor voltage = duty x pack voltage.

Below full throttle, thrust and shaft power follow motorlib's power laws in RPM (the prop's texp and pexp); current and
duty come from the motor constants that reproduce motorlib's full-throttle point (MotorModel), as in the JSBSim motor.
Not modelled: pack voltage under load other than its internal resistance (nominal cell voltage), ESC switching losses,
and anything a part's data leaves out (named in WiringResistance.not_modelled).
"""

from dataclasses import dataclass

import numpy as np

from libs.airframe import INCH_M, WireRun, motor_repo
from libs.geometry import STANDARD_GRAVITY_M_S2, ResolvedAirframe, route_length_m
from libs.motorlib import MotorSim
from libs.motorlib.rcplanes_multirotor import COPPER_RHO, G_ACCEL
from libs.reportline import ReportLine

AIR_TEMP_C = 15.0  # ISA sea level: motorlib's density is then 1.225 kg/m^3
MOTOR_TEMP_C = 20.0  # motorlib's reference for the listed winding resistance
CURVE_POINTS = 101
RPM_SEARCH_FACTOR = 1.5  # duty is inverted on an RPM grid up to this x the full-throttle RPM
CONTACTS_PER_PLUG = 2  # in the current path: + and - (battery plugs), 2 of 3 pins (motor plug)
CONDUCTORS_IN_PATH = 2  # battery leads + and -; motor leads under six-step commutation: one phase in, one out, the third floating
ESC_SWITCHES_IN_PATH = 2  # one high-side and one low-side MOSFET


def library_part(resolved: ResolvedAirframe, parts_data: dict, name: str) -> dict:
    """Part library entry of a placed part, by the placed part's name."""
    placed = next(part for part in resolved.airframe.parts if part.name == name)
    category, _, part_name = placed.part.partition(":")
    return parts_data[category][part_name]


def leads_ohm(run: WireRun, length_m: float) -> float:
    """The run's copper in the current path at 20 C, motorlib's resistivity and AWG areas."""
    return COPPER_RHO * CONDUCTORS_IN_PATH * length_m / motor_repo().wire(run.gauge_awg).area_m2


def plugs_ohm(plugs: tuple[str, ...], parts_data: dict) -> float:
    return sum(CONTACTS_PER_PLUG * parts_data["connector"][plug]["contact_resistance_ohm"] for plug in plugs)


@dataclass(frozen=True)
class WiringResistance:
    """Resistance between the cells and one motor, ohm. The pack, battery leads and battery plugs carry every motor's
    current, so a motor's voltage drop sees them times the motor count (equal currents)."""
    motor_count: int
    pack_ohm: float  # s x per-cell / p
    pack_leads_ohm: float  # the pack's own leads, + and -
    battery_leads_ohm: float  # the battery run, + and -
    battery_plugs_ohm: float
    motor_leads_ohm: float  # this motor's run
    motor_plugs_ohm: float
    esc_ohm: float
    battery_route_m: float
    motor_route_m: float
    not_modelled: tuple[str, ...]

    @property
    def shared_ohm(self) -> float:
        return self.pack_ohm + self.pack_leads_ohm + self.battery_leads_ohm + self.battery_plugs_ohm

    @property
    def per_motor_ohm(self) -> float:
        return self.motor_leads_ohm + self.motor_plugs_ohm + self.esc_ohm + self.motor_count * self.shared_ohm


def wiring_resistances(resolved: ResolvedAirframe, parts_data: dict) -> tuple[WiringResistance, ...]:
    """Per motor, in motor order: the routes of resolved.wires (battery part to ESC part, ESC part to each motor) with
    the gauges and plugs of propulsion.wiring, the battery's own connector first on the battery run."""
    propulsion = resolved.airframe.propulsion
    battery = propulsion.battery
    esc = library_part(resolved, parts_data, propulsion.esc)
    battery_route = route_length_m(resolved.wires.battery)
    not_modelled = []
    if battery.ir_ohm_per_cell is not None:
        pack_ohm = battery.series * battery.ir_ohm_per_cell / battery.parallel
    else:
        pack_ohm = 0.0
        not_modelled.append("pack internal resistance (no ir_mohm_per_cell)")
    if battery.lead_awg is not None:
        pack_leads_ohm = COPPER_RHO * CONDUCTORS_IN_PATH * battery.lead_length_m / motor_repo().wire(battery.lead_awg).area_m2
    else:
        pack_leads_ohm = 0.0
        not_modelled.append("pack leads (no lead_awg, lead_length_mm)")
    if "mosfet_rds_on_ohm" in esc:
        esc_ohm = ESC_SWITCHES_IN_PATH * esc["mosfet_rds_on_ohm"]
    else:
        esc_ohm = 0.0
        not_modelled.append("ESC resistance (no mosfet_rds_on_ohm)")
    battery_run = propulsion.wiring.battery
    battery_plugs = (battery.connector, *battery_run.plugs)
    return tuple(
        WiringResistance(
            motor_count=len(resolved.motors), pack_ohm=pack_ohm, pack_leads_ohm=pack_leads_ohm, battery_leads_ohm=leads_ohm(battery_run, battery_route),
            battery_plugs_ohm=plugs_ohm(battery_plugs, parts_data), motor_leads_ohm=leads_ohm(run, route_length_m(route)),
            motor_plugs_ohm=plugs_ohm(run.plugs, parts_data), esc_ohm=esc_ohm, battery_route_m=battery_route, motor_route_m=route_length_m(route),
            not_modelled=tuple(not_modelled),
        )
        for run, route in zip(propulsion.wiring.motors, resolved.wires.motors, strict=True)
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


def motor_models(resolved: ResolvedAirframe, parts_data: dict) -> tuple[MotorModel, ...]:
    """One per motor, in motor order, each with its own wiring; motors wired alike share one solve."""
    solved: dict[float, MotorModel] = {}
    models = []
    for wiring in wiring_resistances(resolved, parts_data):
        if wiring.per_motor_ohm not in solved:
            solved[wiring.per_motor_ohm] = motor_model(resolved, parts_data, wiring)
        models.append(solved[wiring.per_motor_ohm])
    return tuple(models)


def motor_model(resolved: ResolvedAirframe, parts_data: dict, wiring: WiringResistance) -> MotorModel:
    """motorlib's full-throttle point of one motor wired as given, the other motors drawing the same."""
    airframe = resolved.airframe
    propulsion = airframe.propulsion
    battery = propulsion.battery
    sim = MotorSim(
        repo=motor_repo(), cell=None, cell_volt=battery.cell_voltage_v, controller=None, wire_gauge=None, wire_length_m=0.0,
        external_resistance_ohm=wiring.per_motor_ohm, motor_mfg=propulsion.motor.manufacturer, motor_type=propulsion.motor.name,
        propeller=propulsion.prop, cells=battery.series, parallel=battery.parallel, motors=len(resolved.motors),
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
        label=f"{propulsion.motor.manufacturer} {propulsion.motor.name} ({kv:g} KV)", prop=propulsion.prop, cells=battery.series,
        parallel=battery.parallel, pack_mah=battery.capacity_mah, pack_wh=battery.capacity_mah / 1000.0 * pack_v,
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
    models: tuple[MotorModel, ...]  # per motor (motor_models); motor, prop and pack are the same in each
    motor_count: int
    pack_voltage_v: float  # nominal
    weight_n: float  # nominal mass (ranged masses at the middle)
    curve: MotorPoints  # duty 0..1, mean per motor
    full: MotorPoints  # duty 1, mean per motor
    thrust_to_weight: float
    vtol: bool
    hover: MotorPoints | None  # VTOL only, mean per motor, each motor holding its share of the weight; None when full thrust cannot lift it
    hover_time_min: float | None
    full_throttle_time_min: float


def mean_points(points: list[MotorPoints]) -> MotorPoints:
    return MotorPoints(*(np.mean([getattr(item, field) for item in points], axis=0) for field in ("duty", "rpm", "thrust_n", "current_a", "power_w")))


def propulsion_summary(resolved: ResolvedAirframe, parts_data: dict) -> PropulsionSummary:
    models = motor_models(resolved, parts_data)
    pack = models[0].pack_voltage_v
    count = len(models)
    weight_n = sum(resolved.mass_kg) / 2.0 * STANDARD_GRAVITY_M_S2
    full = mean_points([static_points(model, rpm_for_duty(model, 1.0, pack), pack) for model in models])
    duties = np.linspace(0.0, 1.0, CURVE_POINTS)
    curve = mean_points([static_points(model, rpm_for_duty(model, duties, pack), pack) for model in models])
    vtol = resolved.airframe.vtol
    hover = None
    if vtol and float(full.thrust_n) * count >= weight_n:
        hover = mean_points([static_points(model, rpm_for_thrust(model, weight_n / count), pack) for model in models])
    pack_wh = models[0].pack_wh
    return PropulsionSummary(
        models=models, motor_count=count, pack_voltage_v=pack, weight_n=weight_n, curve=curve, full=full,
        thrust_to_weight=float(full.thrust_n) * count / weight_n, vtol=vtol, hover=hover,
        hover_time_min=None if hover is None else pack_wh / (float(hover.power_w) * count) * 60.0,
        full_throttle_time_min=pack_wh / (float(full.power_w) * count) * 60.0,
    )


def summary_lines(summary: PropulsionSummary) -> list[ReportLine]:
    """Static propulsion as report lines, totals over all motors; throttle as INAV motor output (duty)."""
    model, full, hover, count = summary.models[0], summary.full, summary.hover, summary.motor_count
    wirings = [item.wiring for item in summary.models]
    wiring = wirings[0]  # the battery run, pack and ESC are shared
    grams = 1000.0 / STANDARD_GRAVITY_M_S2
    milli = 1000.0

    def spread(values: list[float], unit: str, scale: float = 1.0, digits: int = 1) -> str:
        low, high = min(values) * scale, max(values) * scale
        return f"{low:.{digits}f}{unit}" if round(high - low, digits) == 0 else f"{low:.{digits}f}..{high:.{digits}f}{unit}"

    def efficiency(points: MotorPoints) -> float:
        return float(points.thrust_n) * grams / float(points.power_w)

    lines = [ReportLine(text) for text in (
        f"{count} x {model.label}, {model.prop} (motorlib); {model.cells}S{model.parallel}P {model.pack_mah:.0f} mAh, {model.pack_wh:.1f} Wh, "
        f"{summary.pack_voltage_v:.1f} V nominal",
        f"Resistance    : {spread([w.per_motor_ohm for w in wirings], '', milli)} mOhm per motor = motor leads {spread([w.motor_leads_ohm for w in wirings], '', milli)} "
        f"({spread([w.motor_route_m for w in wirings], ' m', digits=3)}) + motor plugs {spread([w.motor_plugs_ohm for w in wirings], '', milli)} "
        f"+ ESC {wiring.esc_ohm * milli:.1f} + {count} x shared "
        f"{wiring.shared_ohm * milli:.1f} (pack {wiring.pack_ohm * milli:.1f}, pack leads {wiring.pack_leads_ohm * milli:.1f}, battery leads {wiring.battery_leads_ohm * milli:.1f} "
        f"({wiring.battery_route_m:.3f} m), battery plugs {wiring.battery_plugs_ohm * milli:.1f})",
        f"Full throttle : {float(full.thrust_n) * count * grams:.0f} g, {float(full.current_a) * count:.1f} A, {float(full.power_w) * count:.0f} W; "
        f"{float(full.rpm):.0f} RPM, pitch speed {float(full.rpm) / 60.0 * model.pitch_m * 3.6:.0f} km/h, {efficiency(full):.2f} g/W, "
        f"{summary.full_throttle_time_min:.1f} min",
        f"Thrust/weight : {summary.thrust_to_weight:.2f} at {summary.weight_n * grams:.0f} g",
    )]
    if wiring.not_modelled:
        lines.append(ReportLine(f"Not modelled  : {'; '.join(wiring.not_modelled)}", "warning"))
    warnings = list(dict.fromkeys(warning for item in summary.models for warning in item.warnings))
    if warnings:
        lines.append(ReportLine(f"motorlib      : {'; '.join(warnings)}", "warning"))
    if not summary.vtol:
        pass
    elif hover is None:
        lines.append(ReportLine("Hover         : not possible, full thrust is below the weight", "problem"))
    else:
        lines.append(ReportLine(f"Hover         : motor output {float(hover.duty):.3f}, {float(hover.current_a) * count:.1f} A, {float(hover.power_w) * count:.0f} W, "
                                f"{efficiency(hover):.2f} g/W, {summary.hover_time_min:.1f} min"))
    return lines
