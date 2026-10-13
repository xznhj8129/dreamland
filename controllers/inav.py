"""INAV SITL transport, settings, and controller commands."""

import re
import time
from pathlib import Path

from inavbench import Bench

from libs.fdm import pitch_deg, pitch_rate_deg_s, roll_deg, roll_rate_deg_s
from libs.project import Project
from libs.inav_flight import ForwardFlight
from libs.simrun import battery_power_w, build_vehicle, electrical_columns

REPO_DIR = Path(__file__).resolve().parent.parent
ANGLE_GAIN = 0.02
RATE_GAIN = 0.002
DEFAULT_SITL_PATH = REPO_DIR / "vendor" / "inav" / "build" / "build_SITL" / "bin" / "SITL.elf"  # scripts/setup_vendor.sh
INAV_TEMPLATES_DIR = REPO_DIR / "data" / "inav"
TAILSITTER_TEMPLATE = INAV_TEMPLATES_DIR / "tailsitter.txt"  # switches and modes of a generated diff, after the design's mixer
PLANE_TEMPLATE = INAV_TEMPLATES_DIR / "plane.txt"
INAV_10_RENAMED_SETTINGS = {"mixer_pid_profile_linking": "mixer_control_profile_linking"}
SITL_TEMPLATE = INAV_TEMPLATES_DIR / "sitl.txt"  # the bench's simulated hardware, in every generated diff
REAL_FC_BANNER = re.compile(r"# INAV/\S+ \d+\.\d+")  # the version line of a real FC's CLI diff
# From a real (INAV 8) diff only what defines the aircraft goes into the INAV 10 SITL: the mixer profiles (platform,
# motor mix, motor direction, tailsitter offset, profile linking) and the mode switches on AUX1-4 (all MSP_SIMULATOR
# carries). Tuning, filters, hardware and calibration stay at INAV 10 defaults.
REAL_CONFIG_KEPT = re.compile(
    r"(mixer_profile \d|mmix reset|mmix .*|aux \d+ \d+ [0-3] .*|"
    r"set (platform_type|motor_direction_inverted|tailsitter_orientation_offset|mixer_pid_profile_linking) = .*)$"
)
class FlightBench(Bench):
    """INAV transport ticks feed the common flight scheduler."""
    flight = None
    hardware = None

    def _tick(self):
        super()._tick()
        if self.flight is not None:
            self.flight._tick()

    def _log_row(self, now):
        if self.hardware is not None:
            self.hardware._sample(self.plant, self.flight.micros() if self.flight is not None else 0)
        super()._log_row(now)

    def __enter__(self):
        super().__enter__()
        self.flight_wall_start = time.monotonic()
        self.flight_sim_start = self.elapsed
        return self

    def __exit__(self, exc_type, *args):
        wall = time.monotonic() - self.flight_wall_start
        simulated = self.elapsed - self.flight_sim_start
        print(f"Flight clock: {simulated:.2f} simulated seconds / {wall:.2f} wall seconds ({simulated / wall:.2f}x realtime)")
        return super().__exit__(exc_type, *args)


def real_config_lines(path: Path) -> tuple[list[str], int]:
    """(kept lines with INAV 10 names, number of lines left out) of a real INAV CLI diff."""
    kept, left_out = [], 0
    for line in path.read_text().splitlines():
        text = line.strip()
        if not REAL_CONFIG_KEPT.match(text):
            left_out += bool(text)
            continue
        for old, new in INAV_10_RENAMED_SETTINGS.items():
            text = text.replace(f"set {old} ", f"set {new} ")
        kept.append(text)
    return kept, left_out


def is_real_fc_diff(path: Path) -> bool:
    """A real FC's CLI diff (it carries INAV's version banner), as opposed to a SITL diff flown as written."""
    return any(REAL_FC_BANNER.match(line.strip()) for line in path.read_text().splitlines())


def flown_diff(project: Project, export, tailsitter: bool, inav_config: Path | None) -> Path:
    """The diff INAV flies. Generated from the design (no inav_config) or from a real FC diff into the project's
    inav/generated/, rewritten on every such run; any other diff is flown as written."""
    if inav_config is not None and not is_real_fc_diff(inav_config):
        return inav_config.resolve()  # the bench opens it from its own directory
    if inav_config is None:
        source, path = "the design", project.generated_inav_dir / "design.txt"
        lines = [*export.inav_cli_path.read_text().splitlines(), *(TAILSITTER_TEMPLATE if tailsitter else PLANE_TEMPLATE).read_text().splitlines()]
    else:
        source, path = inav_config.name, project.generated_inav_dir / inav_config.name
        lines, left_out = real_config_lines(inav_config)
        print(f"{inav_config.name}: {len(lines)} aircraft-defining lines kept (INAV 10 names), {left_out} left at INAV 10 defaults")
    battery_lines = battery_config_lines(export) if export.motors else []
    path.parent.mkdir(parents=True, exist_ok=True)
    header = f"# SITL diff from {source}, rewritten by controllers/inav.py on every run from it; to edit, copy it into inav/"
    path.write_text("\n".join([header, *lines, *SITL_TEMPLATE.read_text().splitlines(), *battery_lines]) + "\n")
    return path


def pack_state(export, motor_count: int):
    """Bench battery feed: (pack voltage, pack current) for INAV's battery and current meters. The voltage is nominal
    (no sag modelled); the current is the motors' battery power at that voltage."""
    def state(bench) -> tuple[float, float]:
        return export.pack_voltage_v, battery_power_w(bench.plant.fdm, export, motor_count) / export.pack_voltage_v
    return state


def battery_config_lines(export) -> list[str]:
    """INAV CLI lines so its battery meters read the bench's pack feed (pack_state) and know the pack's capacity."""
    return ["feature VBAT", "feature CURRENT_METER", f"set battery_capacity = {round(export.motors[0].pack_mah)}"]  # one pack: the same in every motor model


def electrical_log(export, motor_count: int):
    """The electrical columns (libs.simrun.electrical_columns) as Bench log_extra."""
    def columns(bench) -> dict[str, float]:
        if bench.hardware.truth is None:
            bench.hardware._sample(bench.plant, 0)
        row = bench.hardware._sensor_columns() | bench.hardware._logged
        if export.motors:
            row |= electrical_columns(bench.plant.fdm, export, motor_count)
        return row
    return columns

engine = None
forward = None


def connect(project, resolved, export, args, state, hardware, log_path):
    sitl_path = args.sitl or DEFAULT_SITL_PATH
    if not sitl_path.is_file():
        raise FileNotFoundError(f"INAV SITL build missing: {sitl_path}; run scripts/setup_vendor.sh (or pass --sitl)")
    diff = flown_diff(project, export, resolved.airframe.tailsitter, args.inav_config)
    project.sitl_dir.mkdir(parents=True, exist_ok=True)
    print(f"INAV firmware: {sitl_path.relative_to(REPO_DIR) if sitl_path.is_relative_to(REPO_DIR) else sitl_path}")
    print(f"INAV settings: {diff.relative_to(project.root) if diff.is_relative_to(project.root) else diff.name}")
    vehicle = build_vehicle(export, str(diff), args.start, state, resolved.airframe.tailsitter)
    motor_count = len(export.motor_properties)
    bench = FlightBench(vehicle, stepped=args.clock != "realtime", faster=args.clock == "stepped-faster",
                        rate_hz=args.rate, gcs_msp=not args.no_msp, gcs_mavlink=not args.no_mavlink,
                        sitl=str(sitl_path), workdir=str(project.sitl_dir), log=str(log_path), log_fc=True,
                        heading_deg=state.heading_deg, home=(state.lat_deg, state.lon_deg),
                        log_extra=electrical_log(export, motor_count),
                        battery=pack_state(export, motor_count) if export.motors else None, launcher=state.launcher)
    bench.hardware = hardware
    return bench


def bind(flight):
    global engine, forward
    engine = flight._engine
    forward = ForwardFlight(flight.airframe)


def mode(name, on=True):
    if name == "ACRO":
        for active in sorted(engine.active_modes() - {"ARM"}):
            engine.mode(active, False)
    else:
        engine.mode(name, on)


def sticks(**channels):
    engine.sticks(**channels)


def arm():
    engine.arm()


def release():
    engine.release()


def takeoff(throttle=0.7, modes=()):
    for name in modes:
        mode(name)
    if engine.plant.rail:
        engine.launch(throttle, modes=modes)
    else:
        engine.arm()
        engine.sticks(throttle=throttle)
        engine.release()


def attitude(*, pitch=0.0, roll=0.0, throttle=None):
    fdm = engine.plant.fdm
    pitch_stick = -(ANGLE_GAIN * (pitch - pitch_deg(fdm)) - RATE_GAIN * pitch_rate_deg_s(fdm))
    roll_stick = ANGLE_GAIN * (roll - roll_deg(fdm)) - RATE_GAIN * roll_rate_deg_s(fdm)
    engine.sticks(pitch=pitch_stick, roll=roll_stick, throttle=throttle)


def forward_start():
    if engine.plant.rail:
        forward.launch_off(engine)
    elif engine.flight.start == "ground":
        forward.take_off(engine, engine.flight.export)
    else:
        forward.start_airborne(engine)
