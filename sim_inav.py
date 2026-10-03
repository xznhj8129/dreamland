"""
Usage:
python sim_inav.py hover aircraft/m62v1.json --sitl ../inav_development/inav/build_SITL/bin/SITL.elf
python sim_inav.py dash aircraft/m62v1.json --sitl <SITL.elf> --inav-config aircraft/inav/m62v1_2025-08-20.txt

Exports the airframe to JSBSim (libs/jsbsim.py) and flies it in INAV SITL on inav-jsbsim-bench (installed in the
drone env). The INAV config is the export's generated hover profile, or a real CLI diff (--inav-config) with the
real board's hardware and calibration lines removed (listed when run).

hover: standing on its tail, arm in ANGLE, climb, then NAV POSHOLD (NAV ALTHOLD if the config has no POSHOLD) hover;
       compares the hover motor command with the export's prediction. JSBSim pitch is +90 deg in hover, so tilt is
       90 - pitch; INAV's roll/pitch are in the hover frame.
cruise: tailsitter takeoff and cruise efficiency, flown like the VTOL plane it is: standing, arm, climb in ANGLE straight to
       TAKEOFF_ALT_M, switch to the forward profile in ACRO, the script pilot pitches over and holds altitude; then level flight at descending motor
       commands, each until the airspeed settles: speed, AoA, battery power, Wh/km, endurance and range. Battery power
       uses the JSBSim motor's own constants (V = throttle position x pack voltage, I = (V - rpm/Kv) / R).
dash:  full-throttle level dash in forward flight: airborne, level, from rest, in the forward mixer profile
       (MIXER PROFILE 2) without ANGLE (ACRO, as flown). The script is the pilot: it holds altitude, wings level and heading with the
       rate sticks, and runs until the airspeed stops rising.
level: level-flight efficiency and slowest level speed, any powered airframe: starts in level flight at LEVEL_ALT_M and
       --start-speed (a guess; default the operating airspeed). A fixed-wing flies INAV ANGLE + NAV ALTHOLD with the
       sticks centred; a tailsitter its forward profile in ACRO with the script pilot holding altitude and heading. A
       throttle loop holds each target airspeed, stepping down LEVEL_SPEED_STEP_M_S
       until level flight cannot be held: speed, AoA, motor, battery power, Wh/km, best range and endurance.
plane: fixed-wing or glider (hover, cruise and dash fly tailsitters): airborne at PLANE_ALT_M and the operating
       airspeed, armed in ANGLE, sticks centred for PLANE_LEVEL_S, then a right roll, a pull up and right yaw on the
       sticks in turn; each passes when the true attitude moves at least PLANE_RESPONSE_MIN_DEG the commanded way.
       Prints the control-surface deflections, so the INAV servo mix is checked end to end.
"""

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np
from inavbench import Bench
from inavbench.vehicles import Vehicle

from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe
from libs.jsbsim import INAV_THROTTLE_IDLE_PCT, export_jsbsim

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (after choosing the non-GUI backend)

EXPORT_ROOT = Path("exports/jsbsim")
FT = 0.3048
KNOT_M_S = 0.514444
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
STANCE_CLEARANCE_M = 0.0005
HOVER_MIXER_PROFILE = 0  # INAV mixer profile 1
FORWARD_MIXER_PROFILE = 1  # INAV mixer profile 2
DASH_ALT_M = 100.0
TAKEOFF_ALT_M = 60.0
TAKEOFF_CLIMB_STICK_ABOVE_HOVER = 0.15
TAKEOFF_MAX_S = 30.0
CLIMB_STOP_S = 1.0  # hover throttle before the transition, so it does not carry the climb into the pitch-over
TRANSITION_DONE_PITCH_DEG = 20.0
TRANSITION_DONE_CLIMB_M_S = 1.0
CRUISE_MOTOR_STEPS = (0.70, 0.60, 0.55, 0.52, 0.50, 0.48, 0.46, 0.44, 0.42, 0.40, 0.38)  # throttle targets, descending (mean motor runs lower: pitch control)
CRUISE_STEP_MIN_S = 8.0
CRUISE_STEP_MAX_S = 25.0
CRUISE_STEADY_S = 3  # settled when airspeed changed less than CRUISE_STEADY_M_S over this many seconds
CRUISE_STEADY_M_S = 0.2
CRUISE_SINK_LIMIT_M_S = 0.5  # a step still sinking faster than this at its end cannot hold altitude: the sweep ends
CRUISE_ALT_LOSS_LIMIT_M = 15.0  # or this far below the transition altitude (the script pilot sags ~1 m per slower step)
USABLE_BATTERY_FRACTION = 0.8  # of the nominal pack energy (libs/propulsion.py)
ATTITUDE_SETTLE_DEG = 3.0  # arm once INAV's roll/pitch agree with truth this closely after the profile switch
ATTITUDE_SETTLE_MAX_S = 20.0
DASH_START_SPEED_M_S = 0.0  # a held start at speed feeds INAV moving GPS with an IMU at rest: its multirotor attitude estimate drifts
DASH_MAX_S = 60.0
DASH_MIN_S = 15.0
DASH_PLATEAU_S = 5  # stop once airspeed rose less than DASH_PLATEAU_M_S over this many seconds
DASH_PLATEAU_M_S = 0.3
LEVEL_ALT_M = 150.0
LEVEL_SPEED_STEP_M_S = 2.0
LEVEL_STEP_MIN_S = 8.0
LEVEL_STEP_MAX_S = 30.0
LEVEL_SPEED_TOLERANCE_M_S = 0.5  # a step is held once airspeed is this close to its target and the climb settled
LEVEL_THROTTLE_START = 0.5  # throttle stick the speed loop starts from
LEVEL_THROTTLE_P_PER_M_S = 0.05
LEVEL_THROTTLE_I_PER_M_S_S = 0.02
LEVEL_THROTTLE_RAMP_PER_S = 1.0  # the throttle stick moves at most this much per second (0 to full in 1 s)
GROUND_AIRBORNE_M = 2.0  # once higher than this (tailsitters start standing on the ground) ...
GROUND_MIN_AGL_M = 0.5  # ... the run ends at or below this altitude above ground
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
PLANE_ALT_M = 150.0
PLANE_THROTTLE = 0.6  # throttle stick for powered planes in the plane test
PLANE_LEVEL_S = 10.0
PLANE_STEP_S = 3.0
PLANE_RECOVER_S = 5.0
PLANE_RESPONSE_MIN_DEG = 5.0
PLANE_MIN_AGL_M = 30.0  # below this the test stops: the plane could not hold its height
# label, stick, stick deflection, attitude and the sign it should change by (RC pitch + is nose down)
PLANE_STEPS = (("roll right", "roll", 0.5, "attitude/phi-deg", 1.0), ("pull up", "pitch", -0.5, "attitude/theta-deg", 1.0),
               ("yaw right", "yaw", 0.5, "attitude/psi-deg", 1.0))


class GroundHit(Exception):
    pass


class FlightBench(Bench):
    """The bench, ending the run when the aircraft comes down to GROUND_MIN_AGL_M after it has flown."""
    highest_agl_m = 0.0

    def run(self, seconds, until=None, check_every=0.05):
        hit = False

        def stop(bench) -> bool:
            nonlocal hit
            fdm = bench.plant.fdm
            self.highest_agl_m = max(self.highest_agl_m, fdm["position/h-agl-ft"] * FT)
            hit = self.highest_agl_m > GROUND_AIRBORNE_M and fdm["position/h-agl-ft"] * FT <= GROUND_MIN_AGL_M
            return hit or (until is not None and until(bench))

        fired = super().run(seconds, until=stop, check_every=check_every)
        if hit:
            fdm = self.plant.fdm
            raise GroundHit(f"at {self.elapsed:.1f} s: airspeed {fdm['velocities/vt-fps'] * FT:.1f} m/s, descending {fdm['velocities/v-down-fps'] * FT:.1f} m/s, "
                            f"pitch {fdm['attitude/theta-deg']:+.1f}, roll {fdm['attitude/phi-deg']:+.1f} deg")
        return fired


@dataclass
class DashPilot:
    """Script pilot on the rate sticks (ACRO): altitude via pitch, wings level, heading hold. Stick pitch + is nose
    down, roll + right, yaw + nose right (RC conventions); JSBSim q is nose-up positive."""
    target_alt_m: float
    target_heading_deg: float
    alt_gain_deg_per_m: float = 2.0
    climb_gain_deg_per_m_s: float = 2.0
    pitch_limits_deg: tuple[float, float] = (-15.0, 25.0)
    angle_gain_per_deg: float = 0.02
    rate_gain_per_deg_s: float = 0.002
    log: list = field(default_factory=list)

    def sticks(self, fdm) -> dict[str, float]:
        climb_m_s = -fdm["velocities/v-down-fps"] * FT
        pitch_cmd = self.alt_gain_deg_per_m * (self.target_alt_m - fdm["position/h-agl-ft"] * FT) - self.climb_gain_deg_per_m_s * climb_m_s
        pitch_cmd = min(max(pitch_cmd, self.pitch_limits_deg[0]), self.pitch_limits_deg[1])
        heading_error = (self.target_heading_deg - fdm["attitude/psi-deg"] + 180.0) % 360.0 - 180.0
        pitch = -(self.angle_gain_per_deg * (pitch_cmd - fdm["attitude/theta-deg"]) - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/q-rad_sec"]))
        roll = self.angle_gain_per_deg * -fdm["attitude/phi-deg"] - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/p-rad_sec"])
        yaw = self.angle_gain_per_deg * heading_error - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/r-rad_sec"])
        return {axis: min(max(value, -1.0), 1.0) for axis, value in (("pitch", pitch), ("roll", roll), ("yaw", yaw))}


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


def battery_power_w(fdm, export, motor_count: int) -> float:
    """Battery power of all motors from the JSBSim motor's constants and pack voltage (lossless ESC)."""
    motor = export.motor
    total = 0.0
    for index in range(motor_count):
        volts = fdm[f"fcs/throttle-pos-norm[{index}]"] * export.pack_voltage_v
        amps = max(0.0, (volts - fdm[f"propulsion/engine[{index}]/propeller-rpm"] / motor.kv_rpm_v) / motor.resistance_ohm)
        total += volts * amps
    return total


@dataclass(frozen=True)
class CruisePoint:
    motor: float
    airspeed_m_s: float
    aoa_deg: float
    pitch_deg: float
    power_w: float
    altitude_error_m: float
    climb_m_s: float


def cruise_test(bench: Bench, export, motor_count: int, battery_wh: float, motor_steps: tuple[float, ...]) -> None:
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
    # blends from hover to the first cruise step as the nose comes down (vertical thrust stays near the weight)
    pilot = DashPilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"])
    cruise_stick = (motor_steps[0] - idle) / (1.0 - idle)
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

    cruise_sweep(bench, export, motor_count, battery_wh, motor_steps, pilot)


def cruise_sweep(bench: Bench, export, motor_count: int, battery_wh: float, motor_steps: tuple[float, ...], pilot: DashPilot) -> None:
    """Level flight at each motor command in turn, the pilot holding altitude and heading; prints the table and the
    best range and endurance of the steps."""
    fdm = bench.plant.fdm
    idle = INAV_THROTTLE_IDLE_PCT / 100.0
    points = []
    print(f"\n{'motor':>6} {'airspeed':>9} {'AoA':>6} {'pitch':>6} {'battery W':>10} {'A @pack':>8} {'Wh/km':>6} {'alt err':>8}")
    for motor_target in motor_steps:
        bench.sticks(throttle=(motor_target - idle) / (1.0 - idle))
        step_start = bench.elapsed
        next_sample = step_start + 1.0
        speeds = []
        while bench.elapsed - step_start < CRUISE_STEP_MAX_S:
            bench.sticks(**pilot.sticks(fdm))
            bench.run(0.02)
            if bench.elapsed < next_sample:
                continue
            next_sample += 1.0
            speeds.append(fdm["velocities/vt-fps"] * FT)
            if bench.elapsed - step_start >= CRUISE_STEP_MIN_S and len(speeds) > CRUISE_STEADY_S and abs(speeds[-1] - speeds[-1 - CRUISE_STEADY_S]) < CRUISE_STEADY_M_S:
                break
        power = battery_power_w(fdm, export, motor_count)
        point = CruisePoint(
            motor=sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count, airspeed_m_s=speeds[-1],
            aoa_deg=fdm["aero/alpha-deg"], pitch_deg=fdm["attitude/theta-deg"], power_w=power,
            altitude_error_m=bench.truth()["agl_m"] - pilot.target_alt_m, climb_m_s=-bench.truth()["vd"],
        )
        print(f"{point.motor:6.3f} {point.airspeed_m_s:7.1f} m/s {point.aoa_deg:+5.1f} {point.pitch_deg:+5.1f} {point.power_w:9.0f}  "
              f"{point.power_w / export.pack_voltage_v:7.1f} {point.power_w / (3.6 * point.airspeed_m_s):6.2f} {point.altitude_error_m:+7.1f} m")
        if point.climb_m_s < -CRUISE_SINK_LIMIT_M_S or point.altitude_error_m < -CRUISE_ALT_LOSS_LIMIT_M:
            print(f"  cannot hold altitude at motor {point.motor:.3f} (sinking {-point.climb_m_s:.1f} m/s): sweep ends")
            break
        points.append(point)

    if not points:
        print("\nNo step held altitude.")
        return
    usable_wh = USABLE_BATTERY_FRACTION * battery_wh
    best_range = min(points, key=lambda p: p.power_w / p.airspeed_m_s)
    best_endurance = min(points, key=lambda p: p.power_w)
    print(f"\nOn {USABLE_BATTERY_FRACTION:.0%} of {battery_wh:.1f} Wh (no battery sag modelled):")
    print(f"  best range of the steps:     {best_range.airspeed_m_s * 3.6:.0f} km/h at {best_range.power_w:.0f} W -> "
          f"{best_range.power_w / (3.6 * best_range.airspeed_m_s):.2f} Wh/km, {usable_wh / (best_range.power_w / (3.6 * best_range.airspeed_m_s)):.1f} km")
    print(f"  best endurance of the steps: {best_endurance.airspeed_m_s * 3.6:.0f} km/h at {best_endurance.power_w:.0f} W -> "
          f"{usable_wh / best_endurance.power_w * 60.0:.1f} min")


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


def level_test(bench: Bench, export, motor_count: int, battery_wh: float, start_speed_m_s: float, tailsitter: bool) -> None:
    fdm = bench.plant.fdm
    if tailsitter:
        enter_forward_profile(bench)
    bench.arm()
    # Tailsitters fly their forward (multirotor-platform) profile in ACRO with the script pilot; fixed-wings let INAV
    # hold attitude and height (ANGLE + NAV ALTHOLD, sticks centred) and the test sets only the throttle.
    pilot = DashPilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"]) if tailsitter else None
    if not tailsitter:
        bench.mode("ANGLE")
        bench.mode("NAV ALTHOLD")
        bench.sticks(roll=0.0, pitch=0.0, yaw=0.0)
    target_alt_m = bench.truth()["agl_m"]
    speed = SpeedHold(target_m_s=start_speed_m_s)
    bench.release()
    points = []
    print(f"\n{'target':>7} {'airspeed':>9} {'AoA':>6} {'pitch':>6} {'motor':>6} {'battery W':>10} {'A @pack':>8} {'Wh/km':>6} {'alt err':>8}")
    while speed.target_m_s > 0.0:
        step_start = bench.elapsed
        next_sample = step_start + 1.0
        held = []
        while bench.elapsed - step_start < LEVEL_STEP_MAX_S:
            airspeed = fdm["velocities/vt-fps"] * FT
            bench.sticks(throttle=speed.throttle(airspeed, 0.02), **(pilot.sticks(fdm) if pilot else {}))
            bench.run(0.02)
            if bench.elapsed < next_sample:
                continue
            next_sample += 1.0
            held.append(abs(airspeed - speed.target_m_s) < LEVEL_SPEED_TOLERANCE_M_S and abs(bench.truth()["vd"]) < CRUISE_SINK_LIMIT_M_S)
            if bench.elapsed - step_start >= LEVEL_STEP_MIN_S and len(held) >= CRUISE_STEADY_S and all(held[-CRUISE_STEADY_S:]):
                break
        power = battery_power_w(fdm, export, motor_count)
        point = CruisePoint(
            motor=sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count, airspeed_m_s=fdm["velocities/vt-fps"] * FT,
            aoa_deg=fdm["aero/alpha-deg"], pitch_deg=fdm["attitude/theta-deg"], power_w=power,
            altitude_error_m=bench.truth()["agl_m"] - target_alt_m, climb_m_s=-bench.truth()["vd"],
        )
        print(f"{speed.target_m_s:5.1f} m/s {point.airspeed_m_s:6.1f} m/s {point.aoa_deg:+5.1f} {point.pitch_deg:+5.1f} {point.motor:6.3f} {point.power_w:9.0f}  "
              f"{point.power_w / export.pack_voltage_v:7.1f} {point.power_w / (3.6 * point.airspeed_m_s):6.2f} {point.altitude_error_m:+7.1f} m")
        if not all(held[-CRUISE_STEADY_S:]) or point.altitude_error_m < -CRUISE_ALT_LOSS_LIMIT_M:
            print(f"  level flight at {speed.target_m_s:.1f} m/s not held (airspeed {point.airspeed_m_s:.1f} m/s, climb {point.climb_m_s:+.1f} m/s, "
                  f"altitude {point.altitude_error_m:+.1f} m): the sweep ends")
            break
        points.append(point)
        speed.target_m_s -= LEVEL_SPEED_STEP_M_S
    if not points:
        print(f"\nLevel flight not held at the start speed {start_speed_m_s:.1f} m/s.")
        return
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


def dash_test(bench: Bench, motor_count: int) -> None:
    enter_forward_profile(bench)
    bench.arm()
    fdm = bench.plant.fdm
    pilot = DashPilot(target_alt_m=fdm["position/h-agl-ft"] * FT, target_heading_deg=fdm["attitude/psi-deg"])
    bench.sticks(throttle=1.0)
    bench.release()
    speeds = []
    next_report = bench.elapsed
    start = bench.elapsed
    while bench.elapsed - start < DASH_MAX_S:
        bench.sticks(**pilot.sticks(fdm))
        bench.run(0.02)
        if bench.elapsed < next_report:
            continue
        next_report += 1.0
        airspeed = fdm["velocities/vt-fps"] * FT
        speeds.append(airspeed)
        motors = [motor_fraction(us) for us in bench.motors[:motor_count]]
        thrust_g = sum(fdm[f"propulsion/engine[{i}]/thrust-lbs"] for i in range(motor_count)) / motor_count * 453.59237
        print(
            f"{bench.elapsed - start:5.1f}s airspeed {airspeed:5.1f} m/s ({airspeed * 3.6:5.1f} km/h)  agl {fdm['position/h-agl-ft'] * FT:6.1f} m  "
            f"pitch {fdm['attitude/theta-deg']:+5.1f}  roll {fdm['attitude/phi-deg']:+5.1f}  AoA {fdm['aero/alpha-deg']:+5.1f} deg  "
            f"motors {' '.join(f'{m:.2f}' for m in motors)}  rpm {fdm['propulsion/engine[0]/propeller-rpm']:6.0f}  thrust {thrust_g:5.0f} g/motor  "
            f"profile {bench.mixer_profile + 1}  {'ARMED' if bench.fc.armed else 'disarmed'}"
        )
        if bench.elapsed - start >= DASH_MIN_S and len(speeds) > DASH_PLATEAU_S and speeds[-1] - speeds[-1 - DASH_PLATEAU_S] < DASH_PLATEAU_M_S:
            break
    top = max(speeds)
    print(f"\nTop speed at full throttle, level: {top:.1f} m/s = {top * 3.6:.0f} km/h")


def plane_report(bench: Bench, phase: str, control_count: int) -> None:
    fdm = bench.plant.fdm
    controls = " ".join(f"C{number} {fdm[f'fcs/control-{number}-pos-deg']:+5.1f}" for number in range(1, control_count + 1))
    print(
        f"{bench.elapsed:5.1f}s {phase:<10} airspeed {fdm['velocities/vt-fps'] * FT:5.1f} m/s  agl {fdm['position/h-agl-ft'] * FT:6.1f} m  "
        f"climb {-fdm['velocities/v-down-fps'] * FT:+5.1f}  roll {fdm['attitude/phi-deg']:+6.1f}  pitch {fdm['attitude/theta-deg']:+5.1f}  "
        f"heading {fdm['attitude/psi-deg']:5.1f}  AoA {fdm['aero/alpha-deg']:+5.1f}  {controls}  {'ARMED' if bench.fc.armed else 'disarmed'}"
    )


class PlaneLostHeight(Exception):
    pass


def plane_fly(bench: Bench, seconds: float, phase: str, control_count: int) -> None:
    for _ in range(int(round(seconds))):
        bench.run(1.0)
        plane_report(bench, phase, control_count)
        if bench.plant.fdm["position/h-agl-ft"] * FT < PLANE_MIN_AGL_M:
            raise PlaneLostHeight(f"below {PLANE_MIN_AGL_M:g} m AGL during '{phase}'")


def plane_test(bench: Bench, powered: bool, control_count: int) -> None:
    bench.mode("ANGLE")
    bench.arm()
    bench.sticks(throttle=PLANE_THROTTLE if powered else 0.0)
    bench.release()
    results = []
    try:
        plane_steps(bench, control_count, results)
    except PlaneLostHeight as error:
        print(f"  FAIL: {error}; the remaining steps were not flown")
    print(f"\nPlane test: {sum(results)}/{len(PLANE_STEPS)} axes respond the commanded way")


def plane_steps(bench: Bench, control_count: int, results: list[bool]) -> None:
    fdm = bench.plant.fdm
    plane_fly(bench, PLANE_LEVEL_S, "level", control_count)
    for label, stick, deflection, attitude, expected_sign in PLANE_STEPS:
        start = fdm[attitude]
        bench.sticks(**{stick: deflection})
        plane_fly(bench, PLANE_STEP_S, label, control_count)
        change = (fdm[attitude] - start + 180.0) % 360.0 - 180.0
        bench.sticks(**{stick: 0.0})
        plane_fly(bench, PLANE_RECOVER_S, "centred", control_count)
        passed = change * expected_sign >= PLANE_RESPONSE_MIN_DEG
        results.append(passed)
        print(f"  {label}: {attitude.split('/')[1]} changed {change:+.1f} deg -> {'PASS' if passed else 'FAIL'}")


def plot_flight_path(log_path: Path, png_path: Path, title: str) -> None:
    """Altitude above ground against distance flown over the ground, both axes to the same scale."""
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
    """Airspeed, altitude, pitch, pitch stick and elevator deflection against time, each on its own axis (normalised, scales shown)."""
    with open(log_path) as file:
        rows = list(csv.DictReader(file))

    def column(key: str) -> np.ndarray:
        return np.array([float(row[key]) for row in rows])

    t = column("t")
    axes_series = [  # one y-axis each: (axis label, [(trace label, values)], colour)
        ("airspeed (m/s)", [("airspeed", column("ias_kts") * KNOT_M_S)], "C0"),
        ("altitude above ground (m)", [("altitude", column("agl_m"))], "C4"),
        ("pitch (deg)", [("pitch", column("pitch"))], "C1"),
        ("pitch stick (+ = nose up)", [("pitch stick", -(column("rc2") - 1500.0) / 500.0)], "C2"),
    ]
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Fly an airframe/2 aircraft in INAV SITL on JSBSim (inav-jsbsim-bench).")
    parser.add_argument("test", choices=("hover", "cruise", "dash", "level", "plane"))
    parser.add_argument("airframe", type=Path, help="airframe/2 file")
    parser.add_argument("--sitl", type=Path, required=True, help="INAV SITL.elf")
    parser.add_argument("--inav-config", type=Path, default=None, help="real INAV CLI diff to fly (default: the export's generated hover profile)")
    parser.add_argument("--hover-s", type=float, default=15.0, help="hover test: hold time in seconds")
    parser.add_argument("--start-speed", type=float, default=None, help="level test: start airspeed in m/s (default: the operating airspeed)")
    parser.add_argument("--motor-steps", type=float, nargs="+", default=list(CRUISE_MOTOR_STEPS), help="cruise test: throttle targets, descending")
    parser.add_argument("--pack-voltage", type=float, default=None, help="battery voltage for the motors (default: nominal)")
    args = parser.parse_args()

    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    resolved, _problems = resolve_airframe(load_airframe(args.airframe, parts_data), parts_data)
    airframe = resolved.airframe
    if args.test in ("hover", "cruise", "dash") and not airframe.tailsitter or args.test == "plane" and airframe.tailsitter:
        parser.error(f"{args.airframe}: hover, cruise and dash fly tailsitters, the plane test fixed-wings and gliders")
    if args.test == "level" and not airframe.powered:
        parser.error(f"{args.airframe}: the level test needs motors")
    start_speed_m_s = airframe.operating_point.airspeed_m_s if args.start_speed is None else args.start_speed
    out_dir = (EXPORT_ROOT / args.airframe.stem).resolve()
    export = export_jsbsim(resolved, parts_data, out_dir, args.airframe.stem, args.pack_voltage)
    if args.inav_config is None:
        config_lines = [*export.inav_cli_path.read_text().splitlines(), *(GENERATED_CONFIG_LINES if airframe.tailsitter else PLANE_CONFIG_LINES)]
    else:
        config_lines, left_out = real_config_lines(args.inav_config)
        print(f"{args.inav_config}: {len(config_lines)} aircraft-defining lines kept (INAV 10 names), {left_out} left at INAV 10 defaults:")
        for line in config_lines:
            print(f"  {line}")
    profile_path = out_dir / "inav_sitl.cli"
    profile_path.write_text("\n".join([*config_lines, *SITL_SENSOR_LINES]) + "\n")
    motor_count = len(export.motor_properties)
    if not airframe.tailsitter:
        print(f"{airframe.name}: {sum(resolved.mass_kg) / 2.0 * 1000:.0f} g, {motor_count} motors, {len(resolved.controls)} control surfaces on "
              f"{len(export.servo_properties)} servo outputs, JSBSim model in {out_dir}")
        for note in export.servo_notes:
            print(f"  servo mix: {note}")
    else:
        print(f"{airframe.name}: {sum(resolved.mass_kg) / 2.0 * 1000:.0f} g, pack {export.pack_voltage_v:.2f} V, predicted hover motor command {export.hover_motor_fraction:.3f} "
              f"(stick {export.hover_stick_us} us, throttle_idle {INAV_THROTTLE_IDLE_PCT} %), JSBSim model in {out_dir}")

    if args.test == "level":
        vehicle = Vehicle(
            model=export.model_name, profile=str(profile_path), motors=export.motor_properties, servos=export.servo_properties,
            airborne=True, alt_m=LEVEL_ALT_M, speed_kts=start_speed_m_s / KNOT_M_S, init={"ic/theta-deg": 0.0}, model_dir=str(out_dir),
            tailsitter=airframe.tailsitter, hover_mixer_profile=HOVER_MIXER_PROFILE,
        )
    elif not airframe.tailsitter:
        vehicle = Vehicle(
            model=export.model_name, profile=str(profile_path), motors=export.motor_properties, servos=export.servo_properties,
            airborne=True, alt_m=PLANE_ALT_M, speed_kts=airframe.operating_point.airspeed_m_s / KNOT_M_S,
            init={"ic/theta-deg": 0.0}, model_dir=str(out_dir),
        )
    elif args.test in ("hover", "cruise"):
        vehicle = Vehicle(
            model=export.model_name, profile=str(profile_path), motors=export.motor_properties,
            airborne=True, alt_m=export.stance_height_m + STANCE_CLEARANCE_M, speed_kts=0.0,
            init={"ic/theta-deg": 90.0}, model_dir=str(out_dir), tailsitter=True, hover_mixer_profile=HOVER_MIXER_PROFILE,
        )
    else:
        vehicle = Vehicle(
            model=export.model_name, profile=str(profile_path), motors=export.motor_properties,
            airborne=True, alt_m=DASH_ALT_M, speed_kts=DASH_START_SPEED_M_S / KNOT_M_S,
            init={"ic/theta-deg": 0.0}, model_dir=str(out_dir), tailsitter=True, hover_mixer_profile=HOVER_MIXER_PROFILE,
        )
    log_path = out_dir / f"sim_inav_{args.test}.csv"
    log_path.unlink(missing_ok=True)  # a run that fails before the bench logs must not plot the previous run
    try:
        run_test(args, vehicle, log_path, resolved, export, motor_count, start_speed_m_s)
    except GroundHit as hit:
        print(f"\nHit the ground {hit}; the run ends.")
    finally:
        if log_path.exists():
            png_path = out_dir / f"sim_inav_{args.test}_path.png"
            plot_flight_path(log_path, png_path, f"{airframe.name}: {args.test} test")
            print(f"flight path: {png_path}")
            definitions = {surface_set.name: surface_set.definition.control for surface_set in resolved.surfaces}
            elevators = [Elevator(f"C{control.number} {control.surface}", int(control.servo[1:]) - 1, definitions[control.surface].deflection_min_deg,
                                  definitions[control.surface].deflection_max_deg)
                         for control in resolved.controls if "pitch" in definitions[control.surface].roles and not airframe.tailsitter]
            time_path = out_dir / f"sim_inav_{args.test}_pitch.png"
            plot_pitch_time(log_path, time_path, f"{airframe.name}: {args.test} test, pitch", elevators)
            print(f"pitch over time: {time_path}")


def run_test(args, vehicle, log_path: Path, resolved, export, motor_count: int, start_speed_m_s: float) -> None:
    airframe = resolved.airframe
    with FlightBench(vehicle, sitl=str(args.sitl), log=str(log_path)) as bench:
        if args.test == "plane":
            plane_test(bench, airframe.powered, len(resolved.controls))
        elif args.test == "hover":
            hover_test(bench, export, motor_count, args.hover_s)
        elif args.test in ("cruise", "level"):
            battery_wh = export.motor.pack_wh
            if args.test == "cruise":
                cruise_test(bench, export, motor_count, battery_wh, tuple(args.motor_steps))
            else:
                level_test(bench, export, motor_count, battery_wh, start_speed_m_s, airframe.tailsitter)
        else:
            dash_test(bench, motor_count)


if __name__ == "__main__":
    main()
