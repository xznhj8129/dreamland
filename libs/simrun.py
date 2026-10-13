"""
Usage:
from libs.simrun import StartState, add_start_arguments, resolve_start, build_vehicle, write_plots

What every simulated run shares, whoever flies it (the selected controller in sim_flight.py):
start conditions, the JSBSim vehicle at its start, the ground rule, the electrical model and the flight-log plots.
"""

import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from inavbench import Catapult, LaunchRail
from inavbench.vehicles import HOME, Vehicle

from libs.fdm import pitch_deg, roll_deg


@dataclass(frozen=True)
class StartOption:
    """One --option of a start: its help text and the starts that use it. A launcher option on air or ground is
    refused; launch-speed is catapult-only (the rail is unpowered)."""
    help: str
    air_only: bool = False
    launcher_only: bool = False
    catapult_only: bool = False

    def applies_to(self, start: str) -> bool:
        return not (self.air_only and start != "air" or self.launcher_only and start not in ("catapult", "rail")
                    or self.catapult_only and start != "catapult")


# Start conditions a run may set (StartState); those marked air-only are fixed by a ground or launcher start.
START_OPTIONS = {  # option: help and the starts that use it
    "alt": StartOption("start height above ground, m (default START_ALT_M; glide GLIDE_START_ALT_M)", air_only=True),
    "speed": StartOption("start airspeed, m/s (default: the operating airspeed; a tailsitter starts from rest)", air_only=True),
    "heading": StartOption("start heading, deg (default 0, north)"),
    "lat": StartOption("start latitude, deg (default the bench's home)"),
    "lon": StartOption("start longitude, deg (default the bench's home)"),
    "pitch": StartOption("start pitch, deg (default 0)", air_only=True),
    "roll": StartOption("start roll, deg (default 0)", air_only=True),
    "wind-from": StartOption("direction the wind blows from, deg (default 0)"),
    "wind-speed": StartOption("wind speed, m/s (default 0)"),
    "launch-pitch": StartOption("launcher pitch, deg (default: 10 catapult, 20 rail)", launcher_only=True),
    "launch-length": StartOption("launcher length, m (default: 2.0 catapult, 3.0 rail)", launcher_only=True),
    "launch-speed": StartOption("catapult speed, m/s (default 15.0)", launcher_only=True, catapult_only=True),
    "launch-height": StartOption("launcher height above ground, m (default: 1.5 catapult, 1.0 rail)", launcher_only=True),
}
STARTS = ("air", "ground", "catapult", "rail")
DEFAULT_START = "air"
# The launch starts use the bench's own plane recipe (inav-jsbsim-bench examples/launch.py): the Rascal's catapult,
# and its rail. The LaunchRail dataclass default (85 deg) is a vertical rocket rail, wrong for a fixed wing.
CATAPULT = Catapult()  # examples/launch.py:37
RAIL = LaunchRail(pitch_deg=20.0, length_m=3.0, height_m=1.0)  # examples/launch.py:41
START_ALT_M = 150.0  # forward-flight scenarios started in the air, held level here
FT = 0.3048
KNOT_M_S = 0.514444
LBF_N = 4.4482216152605
SLUG_KG = 14.59390294
STANCE_CLEARANCE_M = 0.0005
HOVER_MIXER_PROFILE = 0  # INAV mixer profile 1

# JSBSim's default rotational integration (rectangular Euler) diverges on a stiff, lightly damped pitch mode at the
# bench's 1 ms step: the minirocket near 290 m/s (about 47 Hz, damping ratio ~0.015) blew up to 4000 deg/s, while
# 0.5 ms, 0.25 ms and 0.1 ms steps and Adams-Bashforth 2 at 1 ms all agree on a steady flight. Every vehicle flies AB2.
JSBSIM_INTEGRATORS = {"simulation/integrator/rate/rotational": 3, "simulation/integrator/position/rotational": 3}  # 3 = Adams-Bashforth 2

GROUND_AIRBORNE_M = 2.0  # once higher than this (tailsitters start standing on the ground) ...
GROUND_MIN_AGL_M = 0.5  # ... the run ends at or below this altitude above ground


class GroundHit(Exception):
    pass


@dataclass(frozen=True)
class StartState:
    """Where and how a run starts, and (a launch start's) resolved launcher."""
    alt_m: float
    speed_m_s: float
    heading_deg: float
    lat_deg: float
    lon_deg: float
    pitch_deg: float
    roll_deg: float
    wind_from_deg: float
    wind_m_s: float
    launcher: Catapult | LaunchRail | None = None

    def wind_ned_m_s(self) -> tuple[float, float]:
        """(north, east) of the air's motion: it blows towards wind_from_deg + 180."""
        towards = math.radians(self.wind_from_deg + 180.0)
        return self.wind_m_s * math.cos(towards), self.wind_m_s * math.sin(towards)


def launcher_defaults() -> dict[str, dict[str, float]]:
    """What each launch start's panel fields show as their defaults, in START_OPTIONS keys (the bench's own recipe;
    a rail has no launch speed)."""
    return {
        "catapult": {"launch-pitch": CATAPULT.pitch_deg, "launch-length": CATAPULT.length_m,
                     "launch-speed": CATAPULT.speed_ms, "launch-height": CATAPULT.height_m},
        "rail": {"launch-pitch": RAIL.pitch_deg, "launch-length": RAIL.length_m, "launch-height": RAIL.height_m},
    }


LAUNCHER_FIELDS = {  # the Catapult/LaunchRail dataclass field each launcher option overrides
    "launch-pitch": "pitch_deg",
    "launch-length": "length_m",
    "launch-speed": "speed_ms",
    "launch-height": "height_m",
}


def resolve_launcher(parser, start: str, given: dict[str, float | None]) -> Catapult | LaunchRail | None:
    """The launcher a launch start leaves, built from the given --launch-* overrides on the bench's own recipe.

    Air and ground use none and refuse the launcher options; a rail is unpowered and refuses launch-speed; every
    given value must be positive.
    """
    launcher_given = [option for option, spec in START_OPTIONS.items() if spec.launcher_only and given[option] is not None]
    if start not in ("catapult", "rail"):
        if launcher_given:
            parser.error(f"{', '.join(f'--{option}' for option in launcher_given)}: only a catapult or rail start uses it")
        return None
    if start == "rail" and given["launch-speed"] is not None:
        parser.error("--launch-speed: the rail is unpowered")
    overrides: dict[str, float] = {}
    for option in launcher_given:
        value = given[option]
        if value <= 0.0:
            parser.error(f"--{option}: must be positive, got {value:g}")
        overrides[LAUNCHER_FIELDS[option]] = value
    return replace(CATAPULT if start == "catapult" else RAIL, **overrides)


def start_reasons(airframe) -> dict[str, str | None]:
    """Each start of the forward-flight scenarios: None when this airframe can use it, otherwise why not."""
    return {
        "air": None,
        "ground": None if airframe.vtol else "a fixed-wing cannot roll: use a catapult or rail launch",
        "catapult": None,
        "rail": None,
    }


def add_start_arguments(parser) -> None:
    """--start and the START_OPTIONS, for any runner."""
    parser.add_argument("--start", choices=list(STARTS), default=None,
                        help="air (held in the air), ground (standing on its tail, VTOL), catapult or rail (including tailsitters); "
                             "default: the scenario's START, else air")
    for option, spec in START_OPTIONS.items():
        if spec.air_only:
            suffix = "; air start only"
        elif spec.catapult_only:
            suffix = "; catapult start only"
        elif spec.launcher_only:
            suffix = "; catapult or rail start only"
        else:
            suffix = ""
        parser.add_argument(f"--{option}", type=float, default=None, help=spec.help + suffix)


def apply_start_defaults(args, defaults: dict) -> None:
    """Fill what the command line left out from a scenario's START: --start, else DEFAULT_START, and each option
    START names that applies to that start. Options given on the command line always win."""
    if args.start is None:
        args.start = defaults.get("start", DEFAULT_START)
    for option, value in defaults.items():
        attribute = option.replace("-", "_")
        if option != "start" and getattr(args, attribute) is None and START_OPTIONS[option].applies_to(args.start):
            setattr(args, attribute, float(value))


def resolve_start(parser, args, start: str, default_alt_m: float, default_speed_m_s: float) -> StartState:
    """The StartState from the --options given and the runner's defaults; any start but air refuses the air-only ones."""
    given = {option: getattr(args, option.replace("-", "_")) for option in START_OPTIONS}
    fixed = [f"--{option}" for option, spec in START_OPTIONS.items() if spec.air_only and given[option] is not None]
    if fixed and start != "air":
        reason = "a ground start stands on its tail, at rest" if start == "ground" else "a launcher start sits on the rail, at rest"
        parser.error(f"{', '.join(fixed)}: {reason}")

    def chosen(option: str, default: float) -> float:
        return default if given[option] is None else given[option]

    return StartState(
        alt_m=chosen("alt", default_alt_m), speed_m_s=chosen("speed", default_speed_m_s), heading_deg=chosen("heading", 0.0), lat_deg=chosen("lat", HOME[0]),
        lon_deg=chosen("lon", HOME[1]), pitch_deg=chosen("pitch", 0.0), roll_deg=chosen("roll", 0.0), wind_from_deg=chosen("wind-from", 0.0),
        wind_m_s=chosen("wind-speed", 0.0), launcher=resolve_launcher(parser, start, given),
    )


def start_line(start: str, state: StartState) -> str:
    """The run's start as one line; a launch start prints the launcher's own geometry and, for a catapult, its speed."""
    launcher = state.launcher
    if isinstance(launcher, Catapult):
        return (f"start: catapult, {launcher.pitch_deg:g} deg, {launcher.length_m:.1f} m rail, {launcher.speed_ms:.1f} m/s, "
                f"{launcher.height_m:.1f} m above ground")
    if isinstance(launcher, LaunchRail):
        return f"start: rail, {launcher.pitch_deg:g} deg, {launcher.length_m:.1f} m long, {launcher.height_m:.1f} m above ground"
    return (f"start: {start}" + ("" if start == "ground" else f", {state.alt_m:g} m, {state.speed_m_s:g} m/s, pitch {state.pitch_deg:g}, roll {state.roll_deg:g}")
            + f", heading {state.heading_deg:g}, {state.lat_deg:.6f} {state.lon_deg:.6f}, wind {state.wind_m_s:g} m/s from {state.wind_from_deg:g}")


def build_vehicle(export, profile: str, start: str, state: StartState, tailsitter: bool) -> Vehicle:
    """The JSBSim vehicle at its start: standing on the tail contacts, nose up (ground), held in the air as the
    StartState says, or on its launcher (the Plant sets the rail's pitch and rest speed). profile is the INAV CLI
    file (empty for a script controller)."""
    if start == "ground":
        return Vehicle(
            model=export.model_name, profile=profile, motors=export.motor_properties, servos=export.servo_properties,
            airborne=True, alt_m=export.stance_height_m + STANCE_CLEARANCE_M, speed_kts=0.0,
            init={"ic/theta-deg": 90.0, **JSBSIM_INTEGRATORS}, model_dir=str(export.model_dir), running_engines=export.electric_engines, tailsitter=True, hover_mixer_profile=HOVER_MIXER_PROFILE,
        )
    launcher = state.launcher
    if launcher is not None:
        return Vehicle(
            model=export.model_name, profile=profile, motors=export.motor_properties, servos=export.servo_properties,
            airborne=True, alt_m=launcher.height_m, speed_kts=0.0, init=dict(JSBSIM_INTEGRATORS), model_dir=str(export.model_dir), running_engines=export.electric_engines,
            tailsitter=tailsitter, hover_mixer_profile=HOVER_MIXER_PROFILE,
        )
    return Vehicle(
        model=export.model_name, profile=profile, motors=export.motor_properties, servos=export.servo_properties,
        airborne=True, alt_m=state.alt_m, speed_kts=state.speed_m_s / KNOT_M_S, init={"ic/theta-deg": state.pitch_deg, "ic/phi-deg": state.roll_deg, **JSBSIM_INTEGRATORS},
        model_dir=str(export.model_dir), running_engines=export.electric_engines, tailsitter=tailsitter, hover_mixer_profile=HOVER_MIXER_PROFILE,
    )


def hit_ground(fdm, highest_agl_m: float) -> bool:
    """The ground rule: down to GROUND_MIN_AGL_M after having flown above GROUND_AIRBORNE_M."""
    return highest_agl_m > GROUND_AIRBORNE_M and fdm["position/h-agl-ft"] * FT <= GROUND_MIN_AGL_M


def ground_hit_text(elapsed_s: float, fdm) -> str:
    return (f"at {elapsed_s:.1f} s: airspeed {fdm['velocities/vt-fps'] * FT:.1f} m/s, descending {fdm['velocities/v-down-fps'] * FT:.1f} m/s, "
            f"pitch {pitch_deg(fdm):+.1f}, roll {roll_deg(fdm):+.1f} deg")


def motor_electrics(fdm, export, index: int) -> tuple[float, float]:
    """(volts, amps) of one motor from its JSBSim motor's constants: V = throttle position x pack voltage (lossless ESC),
    I = (V - rpm / Kv) / R, never negative."""
    motor = export.motors[index]
    volts = fdm[f"fcs/throttle-pos-norm[{index}]"] * export.pack_voltage_v
    return volts, max(0.0, (volts - fdm[f"propulsion/engine[{index}]/propeller-rpm"] / motor.kv_rpm_v) / motor.resistance_ohm)


def battery_power_w(fdm, export, motor_count: int) -> float:
    """Battery power of all motors (lossless ESC)."""
    return sum(volts * amps for volts, amps in (motor_electrics(fdm, export, index) for index in range(motor_count)))


def electrical_columns(fdm, export, motor_count: int) -> dict[str, float]:
    """Battery power, current at the pack voltage and the pack voltage (nominal: no sag modelled), then each motor's
    voltage, current and prop rpm."""
    motors = [motor_electrics(fdm, export, index) for index in range(motor_count)]
    power = sum(volts * amps for volts, amps in motors)
    row = {"battery_w": power, "battery_a": power / export.pack_voltage_v, "battery_v": export.pack_voltage_v}
    for index, (volts, amps) in enumerate(motors):
        row |= {f"motor{index + 1}_v": volts, f"motor{index + 1}_a": amps, f"motor{index + 1}_rpm": fdm[f"propulsion/engine[{index}]/propeller-rpm"]}
    return row


def rocket_columns(fdm, export) -> dict[str, float]:
    """Each rocket motor's thrust, their total, the vehicle's mass (propellant burning off) and its acceleration along
    the body x axis (specific force, g; 1 at rest nose up)."""
    thrusts = [fdm[rocket.thrust_property] * LBF_N for rocket in export.rockets]
    row = {f"rocket{index + 1}_thrust_n": thrust for index, thrust in enumerate(thrusts)}
    return row | {"rocket_thrust_n": sum(thrusts), "mass_kg": fdm["inertia/mass-slugs"] * SLUG_KG, "accel_x_g": fdm["accelerations/Nx"]}


def plot_flight_path(log_path: Path, png_path: Path, title: str) -> None:
    """Altitude above ground against distance flown over the ground, both axes to the same scale."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(log_path) as file:
        rows = list(csv.DictReader(file))
    north, east, agl, armed = (np.array([float(row[key]) for row in rows]) for key in ("north_m", "east_m", "agl_m", "armed"))
    distance = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(north), np.diff(east)))))
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(distance, agl, color="C0", linewidth=1.5)
    if armed.any():
        index = int(np.argmax(armed > 0))
        ax.plot(distance[index], agl[index], "o", color="C2", label="armed")
    ax.plot(distance[-1], agl[-1], "x", color="C3", markersize=9, label=f"end: {distance[-1]:.0f} m flown, {agl[-1]:.0f} m AGL")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("distance over ground (m)")
    ax.set_ylabel("altitude above ground (m)")
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    plt.close(fig)


@dataclass(frozen=True)
class Elevator:
    """A pitch-role control's INAV servo and its deflection limits, to turn the logged servo output into degrees."""
    label: str
    servo: int  # INAV servo index (S1 = 0)
    deflection_min_deg: float
    deflection_max_deg: float


def plot_pitch_time(log_path: Path, png_path: Path, title: str, elevators: list[Elevator]) -> None:
    """Airspeed, altitude, pitch, pitch stick (when the log has RC) and elevator deflection against time, each on its own
    axis (normalised, scales shown)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(log_path) as file:
        rows = list(csv.DictReader(file))

    def column(key: str) -> np.ndarray:
        return np.array([float(row[key]) for row in rows])

    t = column("t")
    axes_series = [  # one y-axis each: (axis label, [(trace label, values)], colour)
        ("airspeed (m/s)", [("airspeed", column("ias_kts") * KNOT_M_S)], "C0"),
        ("altitude above ground (m)", [("altitude", column("agl_m"))], "C4"),
        ("pitch (deg)", [("pitch", column("pitch"))], "C1"),
    ]
    if "rc2" in rows[0]:  # an RC pilot (INAV runs); a script controller has no sticks
        axes_series.append(("pitch stick (+ = nose up)", [("pitch stick", -(column("rc2") - 1500.0) / 500.0)], "C2"))
    if elevators:
        traces = []
        for elevator in elevators:
            norm = np.clip((column(f"servo{elevator.servo}") - 1500.0) / 500.0, -1.0, 1.0)
            traces.append((f"{elevator.label} S{elevator.servo + 1}", np.where(norm > 0.0, norm * elevator.deflection_max_deg, -norm * elevator.deflection_min_deg)))
        axes_series.append(("elevator (deg, + = TE down)", traces, "C3"))
    fig, host = plt.subplots(figsize=(14, 5))
    axes = [host] + [host.twinx() for _ in axes_series[1:]]
    for index, (ax, (label, traces, color)) in enumerate(zip(axes, axes_series)):
        if index > 1:
            ax.spines["right"].set_position(("axes", 1.0 + 0.08 * (index - 1)))
        for trace, (trace_label, values) in enumerate(traces):
            ax.plot(t, values, color=color, linewidth=1.2, linestyle=("-", "--", ":")[trace % 3], label=trace_label)
        ax.set_ylabel(label, color=color)
        ax.tick_params(axis="y", colors=color)
    if elevators and len(elevators) > 1:
        axes[-1].legend(loc="lower left", fontsize=8)
    armed = column("armed")
    if armed.any():
        host.axvline(t[int(np.argmax(armed > 0))], color="0.5", linestyle="--", linewidth=1.0)
    host.set_xlabel("time (s); dashed: armed")
    host.set_title(title)
    host.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    plt.close(fig)


def write_plots(run_log: Path, resolved, title: str) -> None:
    """The flight-path and pitch-over-time plots next to a flight log."""
    airframe = resolved.airframe
    png_path = run_log.with_name(f"{run_log.stem}_path.png")
    plot_flight_path(run_log, png_path, title)
    print(f"flight path: {png_path.name}")
    definitions = {surface_set.name: surface_set.definition.control for surface_set in resolved.surfaces}
    elevators = [Elevator(f"C{control.number} {control.surface}", int(control.servo[1:]) - 1, definitions[control.surface].deflection_min_deg,
                          definitions[control.surface].deflection_max_deg)
                 for control in resolved.controls if "pitch" in definitions[control.surface].roles and not airframe.tailsitter]
    time_path = run_log.with_name(f"{run_log.stem}_pitch.png")
    plot_pitch_time(run_log, time_path, f"{title}, pitch", elevators)
    print(f"pitch over time: {time_path.name}")
