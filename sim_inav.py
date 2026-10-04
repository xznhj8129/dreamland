"""
Usage:
python sim_inav.py efficiency aircraft/m62v1
python sim_inav.py efficiency aircraft/m62v1 --start ground      # take off and transition first
python sim_inav.py top-speed aircraft/simpleplane
python sim_inav.py controls aircraft/quadlancer62 --inav-config aircraft/quadlancer62/inav/<diff>.txt
python sim_inav.py hover aircraft/m62v1 --sitl <SITL.elf>        # another INAV build than the project's pinned one

Exports the aircraft project's airframe to JSBSim (libs/jsbsim.py, into the project's jsbsim/) and flies it in INAV
SITL on inav-jsbsim-bench (installed in the drone env). The SITL runs the project's pinned sitl/SITL.elf, copied in
from DEFAULT_SITL_PATH on the first run, with sitl/ as its workdir (eeprom.bin, sitl.log); the flight log and plots
go to logs/. The flight log also carries what INAV reports (battery, altitude, GPS, navigation, modes) and the
electrical model (battery power and current, each motor's voltage, current and rpm); INAV's battery and current
meters are fed the modelled pack voltage (nominal) and current. The FC settings are generated from the design, or a
real FC's CLI diff (--inav-config) with the real board's hardware and calibration lines removed (listed when run).

Scenarios, each on every aircraft that can physically fly it (scenario_reasons):
hover:      VTOL. Standing on its tail, arm in ANGLE, climb, then NAV POSHOLD (NAV ALTHOLD if the FC settings have no
            POSHOLD switch); compares the hover motor command with the export's prediction. JSBSim pitch is +90 deg in
            hover, so tilt is 90 - pitch; INAV's roll/pitch are in the hover frame.
efficiency: powered. Accelerates to the top speed, then a throttle loop holds each target airspeed from just under it
            (or --sweep-from) down until level flight cannot be
            held: speed, AoA, motor, battery power and current, Wh/km, slowest level speed, best range and endurance.
            Battery power uses the JSBSim motor's own constants (V = throttle position x pack voltage, I = (V - rpm/Kv) / R).
top-speed:  powered. Full throttle in level flight until the airspeed stops rising.
controls:   any. Right roll, pull up and right yaw on the sticks in turn; each passes when the true attitude moves at
            least CONTROLS_RESPONSE_MIN_DEG the commanded way. Prints the control-surface deflections, so the INAV servo
            mix is checked end to end. Runs until done or the ground.
glide:      any. Motors off from GLIDE_START_ALT_M; a pilot aims at airspeeds with pitch from fast to slow until the
            ground: sink rate and glide ratio per step, the smallest sink rate and the best glide of the steady steps.
The forward-flight scenarios start held in the air (--start air: level at START_ALT_M, a fixed-wing at its operating
airspeed, a tailsitter from rest), or take off and transition first (--start ground, VTOL); controls holds the operating
airspeed. --alt, --speed, --heading, --lat, --lon, --pitch, --roll, --wind-from and --wind-speed set the start (START_OPTIONS). Level flight is held by ForwardFlight, the only airframe-type-specific part:
a fixed-wing flies INAV ANGLE + NAV ALTHOLD, a tailsitter its forward mixer profile in ACRO with the script pilot.
"""

import argparse
import json
import math
import re
import shutil
from dataclasses import dataclass
from pathlib import Path

from inavbench import Bench

from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe
from libs.jsbsim import INAV_THROTTLE_IDLE_PCT, reuse_or_export
from libs.project import Project, open_project
from libs.simrun import (
    FT, START_ALT_M, GroundHit, StartState, add_start_arguments, battery_power_w, build_vehicle,
    electrical_columns, ground_hit_text, hit_ground, resolve_start, start_line, start_reasons, write_plots,
)

REPO_DIR = Path(__file__).resolve().parent
DEFAULT_SITL_PATH = REPO_DIR.parent / "inav_development" / "inav" / "build_SITL" / "bin" / "SITL.elf"
SCENARIOS = ("hover", "efficiency", "top-speed", "controls", "glide")
# Flying on the bench: RC, baro and GPS come in over MSP_SIMULATOR.
SITL_SENSOR_LINES = (
    "set receiver_type = SIM (SITL)",
    "set baro_hardware = FAKE",
    "set pitot_hardware = NONE",
    "feature GPS",
    "set gps_provider = MSP",
    "set init_gyro_cal = OFF",
)
# Switches for the generated config (a real config brings its own).
GENERATED_CONFIG_LINES = (
    "set mag_hardware = FAKE",
    "set nav_mc_althold_throttle = MID_STICK    # centre stick holds altitude",
    "# switches: AUX1 arm, AUX2 angle, AUX3 mid althold / high poshold, AUX4 mid mixer transition / high mixer profile 2",
    "aux 0 0 0 1700 2100",
    "aux 1 1 1 1300 2100",
    "aux 2 3 2 1300 1700",
    "aux 3 11 2 1700 2100",
    "aux 4 63 3 1300 1700",
    "aux 5 62 3 1700 2100",
)
# From a real (INAV 8) diff only what defines the aircraft goes into the INAV 10 SITL: the mixer profiles (platform,
# motor mix, motor direction, tailsitter offset, profile linking) and the mode switches on AUX1-4 (all MSP_SIMULATOR
# carries). Tuning, filters, hardware and calibration stay at INAV 10 defaults.
REAL_CONFIG_KEPT = re.compile(
    r"(mixer_profile \d|mmix reset|mmix .*|aux \d+ \d+ [0-3] .*|"
    r"set (platform_type|motor_direction_inverted|tailsitter_orientation_offset|mixer_pid_profile_linking) = .*)$"
)
INAV_10_RENAMED_SETTINGS = {"mixer_pid_profile_linking": "mixer_control_profile_linking"}
STAND_S = 2.0
CLIMB_S = 2.5
CLIMB_STICK_ABOVE_HOVER = 0.12  # throttle stick above the predicted hover stick for the climb
ALTHOLD_STICK = 0.5  # stick centre: hold altitude
FORWARD_MIXER_PROFILE = 1  # INAV mixer profile 2
TAKEOFF_ALT_M = 60.0  # --start ground: the climb ends here, then the transition
GLIDE_START_ALT_M = 1000.0  # glide starts in the air this high: the more height, the more speeds it measures
GLIDE_SWEEP_FROM_FACTOR = 2.0  # the glide sweep's first target airspeed, times the operating airspeed
GLIDE_SPEED_STEP_M_S = 2.0
GLIDE_STEP_S = 8.0
GLIDE_MEASURE_S = 3.0  # each step's sink rate and glide ratio are means over its last this many seconds
GLIDE_STEADY_M_S = 0.5  # a step counts when its airspeed changed less than this over the measuring window (no zoom or dive)
GLIDE_SPEED_GAIN_DEG_PER_M_S = 3.0  # the gliding pilot's pitch per m/s of airspeed above its target
TAKEOFF_CLIMB_STICK_ABOVE_HOVER = 0.15
TAKEOFF_MAX_S = 30.0
CLIMB_STOP_S = 1.0  # hover throttle before the transition, so it does not carry the climb into the pitch-over
TRANSITION_DONE_PITCH_DEG = 20.0
TRANSITION_DONE_CLIMB_M_S = 1.0
TRANSITION_MOTOR = 0.70  # motor command the throttle blends to as the nose comes down in the transition
USABLE_BATTERY_FRACTION = 0.8  # of the nominal pack energy (libs/propulsion.py)
ATTITUDE_SETTLE_DEG = 3.0  # arm once INAV's roll/pitch agree with truth this closely after the profile switch
ATTITUDE_SETTLE_MAX_S = 20.0
TOP_SPEED_MAX_S = 60.0
TOP_SPEED_MIN_S = 15.0
TOP_SPEED_PLATEAU_S = 5  # stop once airspeed rose less than TOP_SPEED_PLATEAU_M_S over this many seconds
TOP_SPEED_PLATEAU_M_S = 0.3
LEVEL_SPEED_STEP_M_S = 2.0
LEVEL_STEP_MIN_S = 8.0
LEVEL_STEP_MAX_S = 30.0
LEVEL_SPEED_TOLERANCE_M_S = 0.5  # a step is held once airspeed is this close to its target and the climb settled
LEVEL_SINK_LIMIT_M_S = 0.5  # ... climbing or sinking slower than this
LEVEL_STEADY_S = 3  # ... for this many seconds in a row
# The sweep also ends when one step loses this much altitude. Per step, not from the sweep's start: the script pilot's
# altitude hold is proportional, so it settles lower the more pitch a speed needs (2 m per degree of AltitudePilot gain).
LEVEL_ALT_LOSS_LIMIT_M = 15.0
LEVEL_THROTTLE_START = 0.5  # throttle stick the speed loop starts from
LEVEL_THROTTLE_P_PER_M_S = 0.05
LEVEL_THROTTLE_I_PER_M_S_S = 0.02
LEVEL_THROTTLE_RAMP_PER_S = 1.0  # the throttle stick moves at most this much per second (0 to full in 1 s)
# The script pilot's pitch range. A tailsitter can hang on its thrust at any pitch, so nose-up goes nearly to vertical;
# it stops short of 90 deg because the pilot steers by Euler roll and heading, which are undefined there.
PILOT_PITCH_LIMITS_DEG = (-15.0, 80.0)
PLANE_CONFIG_LINES = (
    "set mag_hardware = FAKE",
    "# switches: AUX1 arm, AUX2 angle, AUX3 nav althold",
    "aux 0 0 0 1700 2100",
    "aux 1 1 1 1300 2100",
    "aux 2 3 2 1300 2100",
    "# NAV ALTHOLD holds height with pitch and adds throttle stick above cruise: cruise and min at idle, so the stick sets the power",
    "# (cruise 1001: INAV divides by cruise - 1000 in ANGLE mode, and 1000 crashes the SITL)",
    "set nav_fw_allow_manual_thr_increase = ON",
    "set nav_fw_cruise_thr = 1001",
    "set nav_fw_min_thr = 1000",
)
CONTROLS_LEVEL_S = 10.0
CONTROLS_STEP_S = 3.0
CONTROLS_RECOVER_S = 5.0
CONTROLS_RESPONSE_MIN_DEG = 5.0
# label, stick, stick deflection, attitude and the sign it should change by (RC pitch + is nose down)
CONTROL_STEPS = (("roll right", "roll", 0.5, "attitude/phi-deg", 1.0), ("pull up", "pitch", -0.5, "attitude/theta-deg", 1.0),
               ("yaw right", "yaw", 0.5, "attitude/psi-deg", 1.0))


class FlightBench(Bench):
    """The bench, ending the run on the ground rule (libs.simrun.hit_ground)."""
    highest_agl_m = 0.0

    def run(self, seconds, until=None, check_every=0.05):
        hit = False

        def stop(bench) -> bool:
            nonlocal hit
            fdm = bench.plant.fdm
            self.highest_agl_m = max(self.highest_agl_m, fdm["position/h-agl-ft"] * FT)
            hit = hit_ground(fdm, self.highest_agl_m)
            return hit or (until is not None and until(bench))

        fired = super().run(seconds, until=stop, check_every=check_every)
        if hit:
            raise GroundHit(ground_hit_text(self.elapsed, self.plant.fdm))
        return fired


@dataclass
class AltitudePilot:
    """Script pilot on the rate sticks (ACRO): altitude via pitch, wings level, heading hold. Stick pitch + is nose
    down, roll + right, yaw + nose right (RC conventions); JSBSim q is nose-up positive. Yaw is a rate request from the
    heading error alone: INAV's yaw rate loop does the damping, and a rate term of the pilot's own on top of it, through
    the bench's loop delay, sustains a 5 Hz yaw oscillation once something kicks it (the transition does)."""
    target_alt_m: float
    target_heading_deg: float
    alt_gain_deg_per_m: float = 2.0
    climb_gain_deg_per_m_s: float = 2.0
    pitch_limits_deg: tuple[float, float] = PILOT_PITCH_LIMITS_DEG
    angle_gain_per_deg: float = 0.02
    rate_gain_per_deg_s: float = 0.002

    def pitch_command_deg(self, fdm) -> float:
        climb_m_s = -fdm["velocities/v-down-fps"] * FT
        return self.alt_gain_deg_per_m * (self.target_alt_m - fdm["position/h-agl-ft"] * FT) - self.climb_gain_deg_per_m_s * climb_m_s

    def sticks(self, fdm) -> dict[str, float]:
        pitch_cmd = min(max(self.pitch_command_deg(fdm), self.pitch_limits_deg[0]), self.pitch_limits_deg[1])
        heading_error = (self.target_heading_deg - fdm["attitude/psi-deg"] + 180.0) % 360.0 - 180.0
        pitch = -(self.angle_gain_per_deg * (pitch_cmd - fdm["attitude/theta-deg"]) - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/q-rad_sec"]))
        roll = self.angle_gain_per_deg * -fdm["attitude/phi-deg"] - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/p-rad_sec"])
        yaw = self.angle_gain_per_deg * heading_error
        return {axis: min(max(value, -1.0), 1.0) for axis, value in (("pitch", pitch), ("roll", roll), ("yaw", yaw))}


@dataclass
class SpeedPilot(AltitudePilot):
    """AltitudePilot that holds an airspeed with pitch instead of an altitude (gliding): nose up when fast, down when slow."""
    target_speed_m_s: float = 0.0
    speed_gain_deg_per_m_s: float = GLIDE_SPEED_GAIN_DEG_PER_M_S

    def pitch_command_deg(self, fdm) -> float:
        return self.speed_gain_deg_per_m_s * (fdm["velocities/vt-fps"] * FT - self.target_speed_m_s)


def scenario_reasons(airframe) -> dict[str, str | None]:
    """Each scenario: None when this airframe can fly it, otherwise why not (physics only)."""
    return {
        "hover": None if airframe.vtol else "needs a VTOL",
        "efficiency": None if airframe.powered else "needs motors",
        "top-speed": None if airframe.powered else "needs motors",
        "controls": None,
        "glide": None,
    }


def tests_for(airframe) -> list[str]:
    """The scenarios this airframe can fly."""
    return [scenario for scenario, reason in scenario_reasons(airframe).items() if reason is None]


def flight_log_path(project: Project, scenario: str, start: str) -> Path:
    """The bench's flight log (CSV) of a run; a forward-flight scenario started on the ground gets its own."""
    return project.logs_dir / f"sim_inav_{scenario}{'_ground' if start == 'ground' and scenario != 'hover' else ''}.csv"


def pinned_sitl(project: Project) -> Path:
    """The project's SITL.elf, copied in from DEFAULT_SITL_PATH the first time."""
    if not project.sitl_path.exists():
        project.sitl_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(DEFAULT_SITL_PATH, project.sitl_path)
        print(f"{project.name}: pinned SITL {DEFAULT_SITL_PATH} -> {project.sitl_path}")
    return project.sitl_path


def motor_fraction(us: int) -> float:
    return (us - 1000) / 1000.0


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


def report(bench: Bench, phase: str, motor_count: int) -> None:
    truth = bench.truth()
    altitude_m, _ = bench.altitude()
    motors = [motor_fraction(us) for us in bench.motors[:motor_count]]
    print(
        f"{bench.elapsed:5.1f}s {phase:<9} agl {truth['agl_m']:6.2f} m (FC {altitude_m:6.2f})  tilt {90.0 - truth['pitch']:5.1f} deg  "
        f"drift {math.hypot(truth['north_m'], truth['east_m']):5.2f} m  climb {-truth['vd']:+5.2f} m/s  "
        f"FC roll {bench.fc.roll:+6.1f} pitch {bench.fc.pitch:+6.1f}  motors {' '.join(f'{m:.3f}' for m in motors)}  "
        f"{'ARMED' if bench.fc.armed else 'disarmed'}"
    )


def fly(bench: Bench, seconds: float, phase: str, motor_count: int) -> list[float]:
    """Run, reporting once a second; returns the mean motor command per report."""
    means = []
    for _ in range(int(round(seconds))):
        bench.run(1.0)
        report(bench, phase, motor_count)
        means.append(sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count)
    return means


def hover_test(bench: Bench, export, motor_count: int, hold_s: float) -> None:
    hover_stick = (export.hover_stick_us - 1000) / 1000.0
    bench.release()  # stand on the tail contacts
    fly(bench, STAND_S, "standing", motor_count)
    bench.mode("ANGLE")
    bench.arm()
    bench.sticks(throttle=min(1.0, hover_stick + CLIMB_STICK_ABOVE_HOVER))
    fly(bench, CLIMB_S, "climb", motor_count)
    bench.sticks(throttle=ALTHOLD_STICK)
    try:
        bench.mode("NAV POSHOLD")
        hold = "poshold"
    except KeyError:  # the config has no POSHOLD switch
        bench.mode("NAV ALTHOLD")
        hold = "althold"
    hover = fly(bench, hold_s, hold, motor_count)
    settled = hover[len(hover) // 2:]
    mean = sum(settled) / len(settled)
    print(f"\nHover motor command (second half of {hold}): {mean:.3f}; export predicted {export.hover_motor_fraction:.3f} "
          f"({(mean / export.hover_motor_fraction - 1.0) * 100.0:+.1f} %)")


def pack_state(export, motor_count: int):
    """Bench battery feed: (pack voltage, pack current) for INAV's battery and current meters. The voltage is nominal
    (no sag modelled); the current is the motors' battery power at that voltage."""
    def state(bench) -> tuple[float, float]:
        return export.pack_voltage_v, battery_power_w(bench.plant.fdm, export, motor_count) / export.pack_voltage_v
    return state


def battery_config_lines(export) -> list[str]:
    """INAV CLI lines so its battery meters read the bench's pack feed (pack_state) and know the pack's capacity."""
    return ["feature VBAT", "feature CURRENT_METER", f"set battery_capacity = {round(export.motor.pack_mah)}"]


def electrical_log(export, motor_count: int):
    """The electrical columns (libs.simrun.electrical_columns) as Bench log_extra."""
    def columns(bench) -> dict[str, float]:
        return electrical_columns(bench.plant.fdm, export, motor_count)
    return columns


@dataclass(frozen=True)
class LevelPoint:
    target_m_s: float
    motor: float
    airspeed_m_s: float
    aoa_deg: float
    pitch_deg: float
    power_w: float
    altitude_error_m: float
    climb_m_s: float


@dataclass
class SpeedHold:
    """Throttle stick (PI) holding a target airspeed, from LEVEL_THROTTLE_START at release, its changes limited to
    LEVEL_THROTTLE_RAMP_PER_S."""
    target_m_s: float
    integral: float = 0.0
    stick: float = LEVEL_THROTTLE_START

    def throttle(self, airspeed_m_s: float, dt: float) -> float:
        error = self.target_m_s - airspeed_m_s
        wanted = LEVEL_THROTTLE_START + LEVEL_THROTTLE_P_PER_M_S * error + LEVEL_THROTTLE_I_PER_M_S_S * self.integral
        step = LEVEL_THROTTLE_RAMP_PER_S * dt
        stick = min(max(wanted, self.stick - step, 0.0), self.stick + step, 1.0)
        if stick == wanted:  # no wind-up while the ramp or the stops hold the stick back
            self.integral += error * dt
        self.stick = stick
        return stick


def enter_forward_profile(bench: Bench) -> None:
    """Tailsitter: switch to mixer profile 2 and wait for INAV's attitude estimate to swing round to its sensor frame."""
    bench.mode("MIXER PROFILE 2")
    if not bench.run(5.0, until=lambda b: b.mixer_profile == FORWARD_MIXER_PROFILE):
        raise RuntimeError(f"INAV did not switch to mixer profile 2 (active profile index {bench.mixer_profile})")

    def attitude_settled(b: Bench) -> bool:
        truth = b.truth()
        return abs(b.fc.roll - truth["roll"]) < ATTITUDE_SETTLE_DEG and abs(b.fc.pitch - truth["pitch"]) < ATTITUDE_SETTLE_DEG

    # the sensor frame changes with the profile; INAV's attitude estimate has to swing round to it
    if not bench.run(ATTITUDE_SETTLE_MAX_S, until=attitude_settled):
        raise RuntimeError(f"INAV attitude did not settle: FC roll {bench.fc.roll:+.1f} pitch {bench.fc.pitch:+.1f}, truth {bench.truth()}")
    print(f"profile 2 active, INAV attitude settled (roll {bench.fc.roll:+.1f}, pitch {bench.fc.pitch:+.1f}) after {bench.elapsed:.1f} s")


class ForwardFlight:
    """Level forward flight: the one part of a scenario that depends on the airframe type. Scenarios set the throttle
    (and in controls one stick) through sticks(), every tick.
    Fixed-wing: INAV ANGLE, plus NAV ALTHOLD when powered, sticks centred; the throttle stick sets the power
    (PLANE_CONFIG_LINES). Tailsitter: its forward mixer profile in ACRO, the script pilot holding altitude, wings level
    and heading."""

    def __init__(self, airframe) -> None:
        self.tailsitter = airframe.tailsitter
        self.powered = airframe.powered
        self.pilot: AltitudePilot | None = None

    def air_start_speed_m_s(self, operating_m_s: float) -> float:
        """Airspeed of the held air start. A tailsitter starts from rest: held at speed, INAV gets moving GPS with an
        IMU at rest, and its multirotor attitude estimate (the forward profile is a multirotor platform) drifts. A
        fixed-wing starts at the operating airspeed."""
        return 0.0 if self.tailsitter else operating_m_s

    def hands_on(self, bench: Bench, on: bool) -> None:
        """A stick step flies the aircraft directly: a fixed-wing leaves NAV ALTHOLD (it turns the pitch stick into a
        climb-rate request) for plain ANGLE while on; a tailsitter's pilot is overridden on the stepped axis by sticks()."""
        if not self.tailsitter and self.powered:
            bench.mode("NAV ALTHOLD", on=not on)

    def start_airborne(self, bench: Bench) -> None:
        """From the held start (level at START_ALT_M and the start airspeed): arm, take up the hold, release."""
        if self.tailsitter:
            enter_forward_profile(bench)
            bench.arm()
            fdm = bench.plant.fdm
            self.pilot = AltitudePilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"])
        else:
            bench.mode("ANGLE")
            bench.arm()
            if self.powered:
                bench.mode("NAV ALTHOLD")
        bench.release()

    def take_off(self, bench: Bench, export) -> None:
        """Tailsitter, standing: arm, climb in ANGLE straight to TAKEOFF_ALT_M, switch to the forward profile in ACRO;
        the pilot pitches over and holds that altitude while the throttle blends from hover to TRANSITION_MOTOR as the
        nose comes down (vertical thrust stays near the weight), until level and settled."""
        fdm = bench.plant.fdm
        hover_stick = (export.hover_stick_us - 1000) / 1000.0
        idle = INAV_THROTTLE_IDLE_PCT / 100.0
        bench.release()  # stand on the tail contacts
        bench.run(STAND_S)
        bench.mode("ANGLE")
        bench.arm()
        start = bench.elapsed
        bench.sticks(throttle=min(1.0, hover_stick + TAKEOFF_CLIMB_STICK_ABOVE_HOVER))
        max_tilt = 0.0

        def climbed(b: Bench) -> bool:
            nonlocal max_tilt
            max_tilt = max(max_tilt, 90.0 - b.truth()["pitch"])
            return b.truth()["agl_m"] >= TAKEOFF_ALT_M

        if not bench.run(TAKEOFF_MAX_S, until=climbed):
            raise RuntimeError(f"takeoff did not reach {TAKEOFF_ALT_M} m in {TAKEOFF_MAX_S} s: agl {bench.truth()['agl_m']:.1f} m")
        print(f"takeoff: {TAKEOFF_ALT_M:.0f} m in {bench.elapsed - start:.1f} s, max tilt {max_tilt:.1f} deg")
        bench.sticks(throttle=hover_stick)
        bench.run(CLIMB_STOP_S)

        # transition: forward profile, ACRO; the pilot pitches over gently and holds this altitude while the throttle
        # blends from hover to TRANSITION_MOTOR as the nose comes down (vertical thrust stays near the weight)
        pilot = AltitudePilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"])
        cruise_stick = (TRANSITION_MOTOR - idle) / (1.0 - idle)
        bench.mode("ANGLE", on=False)
        bench.mode("MIXER PROFILE 2")
        start = bench.elapsed
        lowest = highest = bench.truth()["agl_m"]
        while True:
            nose_down = min(max(1.0 - fdm["attitude/theta-deg"] / 90.0, 0.0), 1.0)
            bench.sticks(throttle=hover_stick + (cruise_stick - hover_stick) * nose_down, **pilot.sticks(fdm))
            bench.run(0.02)
            altitude = bench.truth()["agl_m"]
            lowest, highest = min(lowest, altitude), max(highest, altitude)
            if (bench.mixer_profile == FORWARD_MIXER_PROFILE and abs(fdm["attitude/theta-deg"]) < TRANSITION_DONE_PITCH_DEG
                    and abs(bench.truth()["vd"]) < TRANSITION_DONE_CLIMB_M_S):
                break
            if bench.elapsed - start > TAKEOFF_MAX_S:
                raise RuntimeError(f"transition did not settle: pitch {fdm['attitude/theta-deg']:.1f} deg, climb {-bench.truth()['vd']:+.1f} m/s, profile {bench.mixer_profile}")
        print(f"transition: level and settled after {bench.elapsed - start:.1f} s at {fdm['velocities/vt-fps'] * FT:.1f} m/s; "
              f"altitude {lowest - pilot.target_alt_m:+.1f} / {highest - pilot.target_alt_m:+.1f} m around {pilot.target_alt_m:.1f} m")
        # the heading taken nose-up at the start of the transition is meaningless (Euler heading near 90 deg pitch): the
        # pilot would hunt for it with the yaw stick; hold the heading the aircraft levelled out on instead
        self.pilot = AltitudePilot(target_alt_m=pilot.target_alt_m, target_heading_deg=fdm["attitude/psi-deg"])

    def glide(self, bench: Bench, speed_m_s: float) -> SpeedPilot:
        """Rate mode for both types with the gliding pilot holding speed_m_s: a fixed-wing leaves ANGLE (and NAV
        ALTHOLD); motors are left at idle by the caller's throttle."""
        if not self.tailsitter:
            bench.mode("ANGLE", on=False)
            if self.powered:
                bench.mode("NAV ALTHOLD", on=False)
        self.pilot = SpeedPilot(target_alt_m=0.0, target_heading_deg=bench.plant.fdm["attitude/psi-deg"], target_speed_m_s=speed_m_s)
        return self.pilot

    def sticks(self, bench: Bench, throttle: float, override: dict[str, float] | None = None) -> None:
        axes = self.pilot.sticks(bench.plant.fdm) if self.pilot is not None else {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        bench.sticks(throttle=throttle, **(axes | (override or {})))


def level_line(bench: Bench, motor_count: int) -> str:
    """One status line of forward flight: speed, height, attitude, AoA, motors, rpm, profile."""
    fdm = bench.plant.fdm
    motors = " ".join(f"{motor_fraction(us):.2f}" for us in bench.motors[:motor_count])
    rpm = f"  rpm {fdm['propulsion/engine[0]/propeller-rpm']:6.0f}" if motor_count else ""
    return (f"{bench.elapsed:5.1f}s airspeed {fdm['velocities/vt-fps'] * FT:5.1f} m/s  agl {fdm['position/h-agl-ft'] * FT:6.1f} m  "
            f"climb {-fdm['velocities/v-down-fps'] * FT:+5.1f}  pitch {fdm['attitude/theta-deg']:+5.1f}  roll {fdm['attitude/phi-deg']:+6.1f}  "
            f"heading {fdm['attitude/psi-deg']:5.1f}  AoA {fdm['aero/alpha-deg']:+5.1f}  motors {motors}{rpm}  "
            f"profile {bench.mixer_profile + 1 if bench.mixer_profile is not None else 1}  {'ARMED' if bench.fc.armed else 'disarmed'}")


def efficiency(bench: Bench, flight: ForwardFlight, export, motor_count: int, sweep_from_m_s: float | None) -> None:
    """A throttle loop holds each target airspeed, from sweep_from_m_s (default: the top speed, rounded down to the
    LEVEL_SPEED_STEP_M_S grid) down in LEVEL_SPEED_STEP_M_S steps, until level flight cannot be held: per step speed, AoA,
    motor, battery power and current, Wh/km (means over its last LEVEL_STEADY_S seconds); then the slowest level speed, best range and best endurance on
    USABLE_BATTERY_FRACTION of the pack."""
    fdm = bench.plant.fdm
    if sweep_from_m_s is None:
        top = accelerate(bench, flight, motor_count)
        sweep_from_m_s = math.floor(top / LEVEL_SPEED_STEP_M_S) * LEVEL_SPEED_STEP_M_S
        print(f"top speed {top:.1f} m/s: the sweep starts at {sweep_from_m_s:.0f} m/s")
    start_speed_m_s = sweep_from_m_s
    target_alt_m = bench.truth()["agl_m"]
    speed = SpeedHold(target_m_s=start_speed_m_s)
    points = []
    print(f"\n{'target':>7} {'airspeed':>9} {'AoA':>6} {'pitch':>6} {'motor':>6} {'battery W':>10} {'A @pack':>8} {'Wh/km':>6} {'alt err':>8}")
    while speed.target_m_s > 0.0:
        step_start, step_alt_m = bench.elapsed, bench.truth()["agl_m"]
        next_sample = step_start + 1.0
        held = []
        samples = []  # (time, motor, airspeed, AoA, pitch, battery W) every tick: a step reports the mean of its steady window
        while bench.elapsed - step_start < LEVEL_STEP_MAX_S:
            flight.sticks(bench, speed.throttle(fdm["velocities/vt-fps"] * FT, 0.02))
            bench.run(0.02)
            samples.append((bench.elapsed, sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count, fdm["velocities/vt-fps"] * FT,
                            fdm["aero/alpha-deg"], fdm["attitude/theta-deg"], battery_power_w(fdm, export, motor_count)))
            if bench.elapsed < next_sample:
                continue
            next_sample += 1.0
            held.append(abs(fdm["velocities/vt-fps"] * FT - speed.target_m_s) < LEVEL_SPEED_TOLERANCE_M_S and abs(bench.truth()["vd"]) < LEVEL_SINK_LIMIT_M_S)
            if bench.elapsed - step_start >= LEVEL_STEP_MIN_S and len(held) >= LEVEL_STEADY_S and all(held[-LEVEL_STEADY_S:]):
                break
            if bench.truth()["agl_m"] - step_alt_m < -LEVEL_ALT_LOSS_LIMIT_M:  # level flight lost: no point flying the step out
                break
        window = [sample[1:] for sample in samples if sample[0] >= bench.elapsed - LEVEL_STEADY_S]
        motor, airspeed, aoa, pitch, power = (sum(column) / len(window) for column in zip(*window))
        point = LevelPoint(
            target_m_s=speed.target_m_s, motor=motor, airspeed_m_s=airspeed, aoa_deg=aoa, pitch_deg=pitch, power_w=power,
            altitude_error_m=bench.truth()["agl_m"] - target_alt_m, climb_m_s=-bench.truth()["vd"],
        )
        print(f"{point.target_m_s:5.1f} m/s {point.airspeed_m_s:6.1f} m/s {point.aoa_deg:+5.1f} {point.pitch_deg:+5.1f} {point.motor:6.3f} {point.power_w:9.0f}  "
              f"{point.power_w / export.pack_voltage_v:7.1f} {point.power_w / (3.6 * point.airspeed_m_s):6.2f} {point.altitude_error_m:+7.1f} m")
        if not all(held[-LEVEL_STEADY_S:]) or bench.truth()["agl_m"] - step_alt_m < -LEVEL_ALT_LOSS_LIMIT_M:
            print(f"  level flight at {speed.target_m_s:.1f} m/s not held (airspeed {point.airspeed_m_s:.1f} m/s, climb {point.climb_m_s:+.1f} m/s, "
                  f"altitude {point.altitude_error_m:+.1f} m): the sweep ends")
            break
        points.append(point)
        speed.target_m_s -= LEVEL_SPEED_STEP_M_S
    if not points:
        print(f"\nLevel flight not held at the start speed {start_speed_m_s:.1f} m/s.")
        return
    battery_wh = export.motor.pack_wh
    usable_wh = USABLE_BATTERY_FRACTION * battery_wh
    best_range = min(points, key=lambda p: p.power_w / p.airspeed_m_s)
    best_endurance = min(points, key=lambda p: p.power_w)
    print(f"\nSlowest level flight held: {points[-1].airspeed_m_s:.1f} m/s ({points[-1].airspeed_m_s * 3.6:.0f} km/h) at AoA {points[-1].aoa_deg:+.1f} deg; "
          f"not held at {speed.target_m_s:.1f} m/s")
    print(f"On {USABLE_BATTERY_FRACTION:.0%} of {battery_wh:.1f} Wh (no battery sag modelled):")
    print(f"  best range of the steps:     {best_range.airspeed_m_s * 3.6:.0f} km/h at {best_range.power_w:.0f} W -> "
          f"{best_range.power_w / (3.6 * best_range.airspeed_m_s):.2f} Wh/km, {usable_wh / (best_range.power_w / (3.6 * best_range.airspeed_m_s)):.1f} km")
    print(f"  best endurance of the steps: {best_endurance.airspeed_m_s * 3.6:.0f} km/h at {best_endurance.power_w:.0f} W -> "
          f"{usable_wh / best_endurance.power_w * 60.0:.1f} min")


def top_speed(bench: Bench, flight: ForwardFlight, motor_count: int) -> None:
    top = accelerate(bench, flight, motor_count)
    print(f"\nTop speed at full throttle, level: {top:.1f} m/s = {top * 3.6:.0f} km/h")


def accelerate(bench: Bench, flight: ForwardFlight, motor_count: int) -> float:
    """Full throttle in level flight until the airspeed stops rising (TOP_SPEED_PLATEAU_M_S over TOP_SPEED_PLATEAU_S,
    after TOP_SPEED_MIN_S); returns the top speed."""
    fdm = bench.plant.fdm
    speeds = []
    start = bench.elapsed
    next_report = start + 1.0
    while bench.elapsed - start < TOP_SPEED_MAX_S:
        flight.sticks(bench, 1.0)
        bench.run(0.02)
        if bench.elapsed < next_report:
            continue
        next_report += 1.0
        speeds.append(fdm["velocities/vt-fps"] * FT)
        print(level_line(bench, motor_count))
        if bench.elapsed - start >= TOP_SPEED_MIN_S and len(speeds) > TOP_SPEED_PLATEAU_S and speeds[-1] - speeds[-1 - TOP_SPEED_PLATEAU_S] < TOP_SPEED_PLATEAU_M_S:
            break
    return max(speeds)


def controls(bench: Bench, flight: ForwardFlight, motor_count: int, control_count: int, speed_m_s: float | None) -> None:
    """Level for CONTROLS_LEVEL_S, then a right roll, a pull up and right yaw on the sticks in turn, each for
    CONTROLS_STEP_S and followed by CONTROLS_RECOVER_S; an axis passes when the true attitude moves at least
    CONTROLS_RESPONSE_MIN_DEG the commanded way during its step, flown hands on (ForwardFlight.hands_on) through the step
    and its recovery. A powered aircraft holds speed_m_s with the throttle
    loop throughout; an unpowered one glides. Prints the control-surface deflections, so the INAV servo mix is checked
    end to end."""
    fdm = bench.plant.fdm
    speed = None if speed_m_s is None else SpeedHold(target_m_s=speed_m_s)
    results = []

    def hold(seconds: float, phase: str, override: dict[str, float] | None = None, attitude: str | None = None) -> float:
        """Fly for seconds; returns how far the attitude moved (unwrapped, deg) at its furthest from the start."""
        start, previous, moved, extreme = bench.elapsed, fdm[attitude] if attitude else 0.0, 0.0, 0.0
        next_report = start + 1.0
        while bench.elapsed - start < seconds:
            flight.sticks(bench, 0.0 if speed is None else speed.throttle(fdm["velocities/vt-fps"] * FT, 0.02), override)
            bench.run(0.02)
            if attitude:
                moved += (fdm[attitude] - previous + 180.0) % 360.0 - 180.0
                previous = fdm[attitude]
                extreme = max(extreme, moved, key=abs)
            if bench.elapsed >= next_report:
                next_report += 1.0
                deflections = " ".join(f"C{number} {fdm[f'fcs/control-{number}-pos-deg']:+5.1f}" for number in range(1, control_count + 1))
                print(f"{phase:<10} {level_line(bench, motor_count)}  {deflections}")
        return extreme

    try:
        hold(CONTROLS_LEVEL_S, "level")
        for label, stick, deflection, attitude, expected_sign in CONTROL_STEPS:
            flight.hands_on(bench, True)
            moved = hold(CONTROLS_STEP_S, label, {stick: deflection}, attitude)
            hold(CONTROLS_RECOVER_S, "recover")
            flight.hands_on(bench, False)
            passed = moved * expected_sign >= CONTROLS_RESPONSE_MIN_DEG
            results.append(passed)
            print(f"  {label}: {attitude.split('/')[1]} moved {moved:+.1f} deg -> {'PASS' if passed else 'FAIL'}")
    except GroundHit as hit:
        print(f"\nHit the ground {hit}")
    print(f"\nControls: {sum(results)}/{len(CONTROL_STEPS)} axes respond the commanded way")


def glide(bench: Bench, flight: ForwardFlight, cruise_m_s: float) -> None:
    """Motors off; the gliding pilot aims at each target airspeed with pitch, from GLIDE_SWEEP_FROM_FACTOR x the operating
    airspeed down in GLIDE_SPEED_STEP_M_S steps, until the ground: per step the sink rate and glide ratio (means over its
    last GLIDE_MEASURE_S); then the smallest sink rate and the best glide of the steady steps, whatever speed they settled at."""
    fdm = bench.plant.fdm
    pilot = flight.glide(bench, GLIDE_SWEEP_FROM_FACTOR * cruise_m_s)
    steady = []
    print(f"\n{'target':>7} {'airspeed':>9} {'sink':>9} {'glide':>6} {'AoA':>6} {'pitch':>6} {'agl':>7}")
    try:
        while pilot.target_speed_m_s > 0.0:
            step_start = bench.elapsed
            samples = []  # (time, airspeed, sink, ground speed, AoA, pitch)
            while bench.elapsed - step_start < GLIDE_STEP_S:
                flight.sticks(bench, 0.0)
                bench.run(0.02)
                truth = bench.truth()
                samples.append((bench.elapsed, fdm["velocities/vt-fps"] * FT, truth["vd"], math.hypot(truth["vn"], truth["ve"]), fdm["aero/alpha-deg"], truth["pitch"]))
            window = [sample[1:] for sample in samples if sample[0] >= bench.elapsed - GLIDE_MEASURE_S]
            airspeed, sink, ground_speed, aoa, pitch = (sum(column) / len(window) for column in zip(*window))
            ratio = ground_speed / sink if sink > 0.0 else math.inf
            settled = abs(window[-1][0] - window[0][0]) < GLIDE_STEADY_M_S
            print(f"{pilot.target_speed_m_s:5.1f} m/s {airspeed:6.1f} m/s {sink:5.2f} m/s {ratio:6.1f} {aoa:+5.1f} {pitch:+5.1f} {bench.truth()['agl_m']:6.0f} m"
                  f"{'' if settled else '  not steady'}")
            if settled:
                steady.append((pilot.target_speed_m_s, airspeed, sink, ratio))
            pilot.target_speed_m_s -= GLIDE_SPEED_STEP_M_S
    except GroundHit as hit:
        print(f"\nHit the ground {hit}")
    if not steady:
        print("\nNo steady glide step.")
        return
    least_sink = min(steady, key=lambda point: point[2])
    best_glide = max(steady, key=lambda point: point[3])
    print(f"\nSmallest sink rate: {least_sink[2]:.2f} m/s at {least_sink[1]:.1f} m/s ({least_sink[1] * 3.6:.0f} km/h)")
    print(f"Best glide: {best_glide[3]:.1f} : 1 ({math.degrees(math.atan2(1.0, best_glide[3])):.1f} deg) at {best_glide[1]:.1f} m/s "
          f"({best_glide[1] * 3.6:.0f} km/h), sinking {best_glide[2]:.2f} m/s")


def main() -> None:
    parser = argparse.ArgumentParser(description="Fly an airframe/2 aircraft in INAV SITL on JSBSim (inav-jsbsim-bench).")
    parser.add_argument("scenario", choices=list(SCENARIOS))
    parser.add_argument("project", type=Path, help="aircraft project directory (aircraft/<name>)")
    parser.add_argument("--sitl", type=Path, default=None, help="INAV SITL.elf (default: the project's pinned sitl/SITL.elf)")
    parser.add_argument("--inav-config", type=Path, default=None, help="real INAV CLI diff to fly (default: FC settings generated from the design)")
    parser.add_argument("--hover-s", type=float, default=15.0, help="hover: hold time in seconds")
    parser.add_argument("--sweep-from", type=float, default=None, help="efficiency: first target airspeed in m/s (default: the top speed, measured first)")
    parser.add_argument("--pack-voltage", type=float, default=None, help="battery voltage for the motors (default: nominal)")
    add_start_arguments(parser)  # forward-flight scenarios; hover always starts standing
    args = parser.parse_args()

    project = open_project(args.project)
    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    resolved, _problems = resolve_airframe(load_airframe(project.build_path, parts_data), parts_data)
    airframe = resolved.airframe
    reason = scenario_reasons(airframe)[args.scenario]
    if reason is not None:
        parser.error(f"{args.project}: {args.scenario} {reason}")
    if args.scenario == "hover":
        if args.start == "air":
            parser.error("hover starts standing on the ground")
        start = "ground"
    else:
        start = args.start or "air"
        reason = start_reasons(airframe)[start]
        if reason is not None:
            parser.error(f"{args.project}: start {start}: {reason}")
    flight = ForwardFlight(airframe)
    cruise_m_s = airframe.operating_point.airspeed_m_s
    state = resolve_start(parser, args, start, GLIDE_START_ALT_M if args.scenario == "glide" else START_ALT_M, flight.air_start_speed_m_s(cruise_m_s))
    sitl_path = pinned_sitl(project) if args.sitl is None else args.sitl
    out_dir = project.jsbsim_dir
    export, reused = reuse_or_export(resolved, parts_data, project.build_path, out_dir, project.name, args.pack_voltage)
    if args.inav_config is None:
        config_lines = [*export.inav_cli_path.read_text().splitlines(), *(GENERATED_CONFIG_LINES if airframe.tailsitter else PLANE_CONFIG_LINES)]
    else:
        config_lines, left_out = real_config_lines(args.inav_config)
        print(f"{args.inav_config}: {len(config_lines)} aircraft-defining lines kept (INAV 10 names), {left_out} left at INAV 10 defaults:")
        for line in config_lines:
            print(f"  {line}")
    project.sitl_dir.mkdir(parents=True, exist_ok=True)
    profile_path = project.sitl_dir / "inav_sitl.cli"
    battery_lines = battery_config_lines(export) if export.motor is not None else []
    profile_path.write_text("\n".join([*config_lines, *SITL_SENSOR_LINES, *battery_lines]) + "\n")
    motor_count = len(export.motor_properties)
    print(f"{airframe.name}: {sum(resolved.mass_kg) / 2.0 * 1000:.0f} g, {motor_count} motors, {len(resolved.controls)} control surfaces on "
          f"{len(export.servo_properties)} servo outputs, JSBSim model in {out_dir} ({'reused: nothing it depends on changed' if reused else 'exported'})")
    if export.motor is not None:
        print(f"  pack {export.pack_voltage_v:.2f} V, throttle_idle {INAV_THROTTLE_IDLE_PCT} %"
              + (f", predicted hover motor command {export.hover_motor_fraction:.3f} (stick {export.hover_stick_us} us)" if airframe.vtol else ""))

    print(start_line(start, state))
    vehicle = build_vehicle(export, str(profile_path), start, state, airframe.tailsitter)
    run_log = flight_log_path(project, args.scenario, start)
    run_log.parent.mkdir(parents=True, exist_ok=True)
    run_log.unlink(missing_ok=True)  # a run that fails before the bench logs must not plot the previous run
    title = f"{airframe.name}: {run_log.stem.removeprefix('sim_inav_')}"
    try:
        run_scenario(args.scenario, start, state, args, flight, vehicle, sitl_path, project.sitl_dir, run_log, resolved, export, motor_count, cruise_m_s)
    except GroundHit as hit:
        print(f"\nHit the ground {hit}; the run ends.")
    finally:
        if run_log.exists():
            write_plots(run_log, resolved, title)


def run_scenario(scenario: str, start: str, state: StartState, args, flight: ForwardFlight, vehicle, sitl_path: Path, workdir: Path, log_path: Path,
                 resolved, export, motor_count: int, cruise_m_s: float) -> None:
    powered = export.motor is not None
    with FlightBench(vehicle, sitl=str(sitl_path), workdir=str(workdir), log=str(log_path), log_fc=True, heading_deg=state.heading_deg,
                     home=(state.lat_deg, state.lon_deg), log_extra=electrical_log(export, motor_count) if powered else None,
                     battery=pack_state(export, motor_count) if powered else None) as bench:
        bench.wind(*state.wind_ned_m_s())
        if scenario == "hover":
            hover_test(bench, export, motor_count, args.hover_s)
            return
        if start == "ground":
            flight.take_off(bench, export)
        else:
            flight.start_airborne(bench)
        if scenario == "efficiency":
            efficiency(bench, flight, export, motor_count, args.sweep_from)
        elif scenario == "top-speed":
            top_speed(bench, flight, motor_count)
        elif scenario == "glide":
            glide(bench, flight, cruise_m_s)
        else:
            controls(bench, flight, motor_count, len(resolved.controls), cruise_m_s if powered else None)


if __name__ == "__main__":
    main()
