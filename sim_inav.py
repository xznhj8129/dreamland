"""
Usage:
python sim_inav.py aircraft/m62v1.json --sitl ../inav_development/inav/build_SITL/bin/SITL.elf
python sim_inav.py aircraft/m62v1.json --sitl <SITL.elf> --hover-s 20

Exports the airframe to JSBSim (libs/jsbsim.py) and flies it in INAV SITL on inav-jsbsim-bench (installed in the
drone env): standing on its tail, arm in ANGLE, climb, then NAV POSHOLD hover (position and altitude hold). Prints truth (JSBSim) next to what INAV
sees once a second and compares the hover motor command with the export's prediction.

Hover attitude: JSBSim pitch is +90 deg in hover (body x = nose = up), so "tilt" is 90 - pitch. INAV runs a
multirotor profile with tailsitter_orientation_offset = ON; its roll/pitch are in that hover frame.
"""

import argparse
import json
import math
from pathlib import Path

from inavbench import Bench
from inavbench.vehicles import Vehicle

from libs.airframe import PARTS_PATH, load_airframe
from libs.geometry import resolve_airframe
from libs.jsbsim import INAV_THROTTLE_IDLE_PCT, export_jsbsim

EXPORT_ROOT = Path("exports/jsbsim")
# INAV config for flying on the bench (MSP_SIMULATOR sensors and RC), as the bench's own quad profile.
SITL_PROFILE_LINES = (
    "set receiver_type = SIM (SITL)",
    "set baro_hardware = FAKE",
    "set mag_hardware = FAKE",
    "set pitot_hardware = NONE",
    "feature GPS",
    "set gps_provider = MSP",
    "set init_gyro_cal = OFF",
    "set nav_mc_althold_throttle = MID_STICK    # centre stick holds altitude",
    "# switches: AUX1 arm, AUX2 angle, AUX3 mid althold / high poshold, AUX4 mid RTH / high WP",
    "aux 0 0 0 1700 2100",
    "aux 1 1 1 1300 2100",
    "aux 2 3 2 1300 1700",
    "aux 3 11 2 1700 2100",
    "aux 4 10 3 1300 1700",
    "aux 5 28 3 1700 2100",
)
STAND_S = 2.0
CLIMB_S = 2.5
CLIMB_STICK_ABOVE_HOVER = 0.12  # throttle stick above the predicted hover stick for the climb
ALTHOLD_STICK = 0.5  # stick centre: hold altitude
STANCE_CLEARANCE_M = 0.0005


def motor_fraction(us: int) -> float:
    return (us - 1000) / 1000.0


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


def main() -> None:
    parser = argparse.ArgumentParser(description="Fly an airframe/2 aircraft in INAV SITL on JSBSim (inav-jsbsim-bench).")
    parser.add_argument("airframe", type=Path, help="airframe/2 file")
    parser.add_argument("--sitl", type=Path, required=True, help="INAV SITL.elf")
    parser.add_argument("--hover-s", type=float, default=15.0, help="NAV POSHOLD hover time in seconds")
    args = parser.parse_args()

    with open(PARTS_PATH, "r") as file:
        parts_data = json.load(file)
    resolved, _problems = resolve_airframe(load_airframe(args.airframe, parts_data), parts_data)
    out_dir = (EXPORT_ROOT / args.airframe.stem).resolve()
    export = export_jsbsim(resolved, parts_data, out_dir, args.airframe.stem)
    profile_path = out_dir / "inav_sitl.cli"
    profile_path.write_text(export.inav_cli_path.read_text() + "\n".join(SITL_PROFILE_LINES) + "\n")
    motor_count = len(export.motor_properties)
    hover_stick = (export.hover_stick_us - 1000) / 1000.0
    print(f"{resolved.airframe.name}: {sum(resolved.mass_kg) / 2.0 * 1000:.0f} g, predicted hover motor command {export.hover_motor_fraction:.3f} "
          f"(stick {export.hover_stick_us} us, throttle_idle {INAV_THROTTLE_IDLE_PCT} %), JSBSim model in {out_dir}")

    vehicle = Vehicle(
        model=export.model_name, profile=str(profile_path), motors=export.motor_properties,
        airborne=True, alt_m=export.stance_height_m + STANCE_CLEARANCE_M, speed_kts=0.0,
        init={"ic/theta-deg": 90.0}, model_dir=str(out_dir), tailsitter=True,
    )
    with Bench(vehicle, sitl=str(args.sitl), log=str(out_dir / "sim_inav.csv")) as bench:
        bench.release()  # stand on the tail contacts
        fly(bench, STAND_S, "standing", motor_count)
        bench.mode("ANGLE")
        bench.arm()
        bench.sticks(throttle=min(1.0, hover_stick + CLIMB_STICK_ABOVE_HOVER))
        fly(bench, CLIMB_S, "climb", motor_count)
        bench.sticks(throttle=ALTHOLD_STICK)
        bench.mode("NAV POSHOLD")
        hover = fly(bench, args.hover_s, "poshold", motor_count)
        settled = hover[len(hover) // 2:]
        mean = sum(settled) / len(settled)
        print(f"\nHover motor command (second half of POSHOLD): {mean:.3f}; export predicted {export.hover_motor_fraction:.3f} "
              f"({(mean / export.hover_motor_fraction - 1.0) * 100.0:+.1f} %)")


if __name__ == "__main__":
    main()
