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
cruise: takeoff and cruise efficiency, flown like the VTOL plane it is: standing, arm, climb in ANGLE straight to
       TAKEOFF_ALT_M, switch to the forward profile in ACRO, the script pilot pitches over and holds altitude; then level flight at descending motor
       commands, each until the airspeed settles: speed, AoA, battery power, Wh/km, endurance and range. Battery power
       uses the JSBSim motor's own constants (V = throttle position x pack voltage, I = (V - rpm/Kv) / R).
dash:  full-throttle level dash in forward flight: airborne, level, from rest, in the forward mixer profile
       (MIXER PROFILE 2) without ANGLE (ACRO, as flown). The script is the pilot: it holds altitude, wings level and heading with the
       rate sticks, and runs until the airspeed stops rising.
"""

import argparse
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from inavbench import Bench
from inavbench.vehicles import Vehicle

from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe
from libs.jsbsim import INAV_THROTTLE_IDLE_PCT, export_jsbsim

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
CRUISE_MOTOR_STEPS = (0.70, 0.60, 0.55, 0.52, 0.50, 0.48, 0.46, 0.44, 0.42)  # throttle targets, descending (mean motor runs lower: pitch control)
CRUISE_STEP_MIN_S = 8.0
CRUISE_STEP_MAX_S = 25.0
CRUISE_STEADY_S = 3  # settled when airspeed changed less than CRUISE_STEADY_M_S over this many seconds
CRUISE_STEADY_M_S = 0.2
CRUISE_ALT_TOLERANCE_M = 5.0  # a step that cannot hold altitude within this ends the sweep
USABLE_BATTERY_FRACTION = 0.8  # of nominal LiPo energy (3.7 V/cell)
LIPO_NOMINAL_CELL_V = 3.7
ATTITUDE_SETTLE_DEG = 3.0  # arm once INAV's roll/pitch agree with truth this closely after the profile switch
ATTITUDE_SETTLE_MAX_S = 20.0
DASH_START_SPEED_M_S = 0.0  # a held start at speed feeds INAV moving GPS with an IMU at rest: its multirotor attitude estimate drifts
DASH_MAX_S = 60.0
DASH_MIN_S = 15.0
DASH_PLATEAU_S = 5  # stop once airspeed rose less than DASH_PLATEAU_M_S over this many seconds
DASH_PLATEAU_M_S = 0.3


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
    stick_limit: float = 0.5
    log: list = field(default_factory=list)

    def sticks(self, fdm) -> dict[str, float]:
        climb_m_s = -fdm["velocities/v-down-fps"] * FT
        pitch_cmd = self.alt_gain_deg_per_m * (self.target_alt_m - fdm["position/h-agl-ft"] * FT) - self.climb_gain_deg_per_m_s * climb_m_s
        pitch_cmd = min(max(pitch_cmd, self.pitch_limits_deg[0]), self.pitch_limits_deg[1])
        heading_error = (self.target_heading_deg - fdm["attitude/psi-deg"] + 180.0) % 360.0 - 180.0
        pitch = -(self.angle_gain_per_deg * (pitch_cmd - fdm["attitude/theta-deg"]) - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/q-rad_sec"]))
        roll = self.angle_gain_per_deg * -fdm["attitude/phi-deg"] - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/p-rad_sec"])
        yaw = self.angle_gain_per_deg * heading_error - self.rate_gain_per_deg_s * math.degrees(fdm["velocities/r-rad_sec"])
        return {axis: min(max(value, -self.stick_limit), self.stick_limit) for axis, value in (("pitch", pitch), ("roll", roll), ("yaw", yaw))}


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


def battery_power_w(fdm, fit, motor_count: int) -> float:
    """Battery power of all motors from the JSBSim motor's constants (lossless ESC)."""
    total = 0.0
    for index in range(motor_count):
        volts = fdm[f"fcs/throttle-pos-norm[{index}]"] * fit.full_voltage_v
        amps = max(0.0, (volts - fdm[f"propulsion/engine[{index}]/propeller-rpm"] / fit.kv_rpm_v) / fit.resistance_ohm)
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


def cruise_test(bench: Bench, export, motor_count: int, battery_wh: float) -> None:
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
    # transition: forward profile, ACRO, the pilot pitches over and holds this altitude
    pilot = DashPilot(target_alt_m=bench.truth()["agl_m"], target_heading_deg=fdm["attitude/psi-deg"])
    bench.mode("ANGLE", on=False)
    bench.mode("MIXER PROFILE 2")
    bench.sticks(throttle=(CRUISE_MOTOR_STEPS[0] - idle) / (1.0 - idle), **pilot.sticks(fdm))
    start = bench.elapsed
    lowest = bench.truth()["agl_m"]
    while 90.0 - abs(fdm["attitude/theta-deg"]) < 70.0 or bench.mixer_profile != FORWARD_MIXER_PROFILE:
        bench.sticks(**pilot.sticks(fdm))
        bench.run(0.02)
        lowest = min(lowest, bench.truth()["agl_m"])
        if bench.elapsed - start > TAKEOFF_MAX_S:
            raise RuntimeError(f"transition did not level out: pitch {fdm['attitude/theta-deg']:.1f} deg, profile {bench.mixer_profile}")
    print(f"transition: pitch under 20 deg after {bench.elapsed - start:.1f} s, lowest {lowest:.1f} m (from {pilot.target_alt_m:.1f} m)")

    points = []
    print(f"\n{'motor':>6} {'airspeed':>9} {'AoA':>6} {'pitch':>6} {'battery W':>10} {'A @pack':>8} {'Wh/km':>6} {'alt err':>8}")
    for motor_target in CRUISE_MOTOR_STEPS:
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
        power = battery_power_w(fdm, export.motor_fit, motor_count)
        point = CruisePoint(
            motor=sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count, airspeed_m_s=speeds[-1],
            aoa_deg=fdm["aero/alpha-deg"], pitch_deg=fdm["attitude/theta-deg"], power_w=power,
            altitude_error_m=bench.truth()["agl_m"] - pilot.target_alt_m,
        )
        print(f"{point.motor:6.3f} {point.airspeed_m_s:7.1f} m/s {point.aoa_deg:+5.1f} {point.pitch_deg:+5.1f} {point.power_w:9.0f}  "
              f"{point.power_w / export.motor_fit.full_voltage_v:7.1f} {point.power_w / (3.6 * point.airspeed_m_s):6.2f} {point.altitude_error_m:+7.1f} m")
        if abs(point.altitude_error_m) > CRUISE_ALT_TOLERANCE_M:
            print(f"  cannot hold altitude at motor {point.motor:.3f}: sweep ends")
            break
        points.append(point)

    usable_wh = USABLE_BATTERY_FRACTION * battery_wh
    best_range = min(points, key=lambda p: p.power_w / p.airspeed_m_s)
    best_endurance = min(points, key=lambda p: p.power_w)
    print(f"\nOn {USABLE_BATTERY_FRACTION:.0%} of {battery_wh:.1f} Wh (no battery sag modelled):")
    print(f"  best range of the steps:     {best_range.airspeed_m_s * 3.6:.0f} km/h at {best_range.power_w:.0f} W -> "
          f"{best_range.power_w / (3.6 * best_range.airspeed_m_s):.2f} Wh/km, {usable_wh / (best_range.power_w / (3.6 * best_range.airspeed_m_s)):.1f} km")
    print(f"  best endurance of the steps: {best_endurance.airspeed_m_s * 3.6:.0f} km/h at {best_endurance.power_w:.0f} W -> "
          f"{usable_wh / best_endurance.power_w * 60.0:.1f} min")


def dash_test(bench: Bench, motor_count: int) -> None:
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Fly an airframe/2 aircraft in INAV SITL on JSBSim (inav-jsbsim-bench).")
    parser.add_argument("test", choices=("hover", "cruise", "dash"))
    parser.add_argument("airframe", type=Path, help="airframe/2 file")
    parser.add_argument("--sitl", type=Path, required=True, help="INAV SITL.elf")
    parser.add_argument("--inav-config", type=Path, default=None, help="real INAV CLI diff to fly (default: the export's generated hover profile)")
    parser.add_argument("--hover-s", type=float, default=15.0, help="hover test: hold time in seconds")
    args = parser.parse_args()

    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    resolved, _problems = resolve_airframe(load_airframe(args.airframe, parts_data), parts_data)
    out_dir = (EXPORT_ROOT / args.airframe.stem).resolve()
    export = export_jsbsim(resolved, parts_data, out_dir, args.airframe.stem)
    if args.inav_config is None:
        config_lines = [*export.inav_cli_path.read_text().splitlines(), *GENERATED_CONFIG_LINES]
    else:
        config_lines, left_out = real_config_lines(args.inav_config)
        print(f"{args.inav_config}: {len(config_lines)} aircraft-defining lines kept (INAV 10 names), {left_out} left at INAV 10 defaults:")
        for line in config_lines:
            print(f"  {line}")
    profile_path = out_dir / "inav_sitl.cli"
    profile_path.write_text("\n".join([*config_lines, *SITL_SENSOR_LINES]) + "\n")
    motor_count = len(export.motor_properties)
    print(f"{resolved.airframe.name}: {sum(resolved.mass_kg) / 2.0 * 1000:.0f} g, predicted hover motor command {export.hover_motor_fraction:.3f} "
          f"(stick {export.hover_stick_us} us, throttle_idle {INAV_THROTTLE_IDLE_PCT} %), JSBSim model in {out_dir}")

    if args.test in ("hover", "cruise"):
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
    with Bench(vehicle, sitl=str(args.sitl), log=str(out_dir / f"sim_inav_{args.test}.csv")) as bench:
        if args.test == "hover":
            hover_test(bench, export, motor_count, args.hover_s)
        elif args.test == "cruise":
            airframe = resolved.airframe
            battery = parts_data["battery"][next(v for v in airframe.volumes if v.name == airframe.propulsion.battery_volume).part.partition(":")[2]]
            cruise_test(bench, export, motor_count, battery["s"] * LIPO_NOMINAL_CELL_V * battery["mah"] / 1000.0)
        else:
            dash_test(bench, motor_count)


if __name__ == "__main__":
    main()
