from controllers import inav as controller

import math

from libs.fdm import pitch_deg
from libs.simrun import FT, battery_power_w
from libs.inav_flight import LevelPoint, SpeedHold, motor_fraction
from libs.jsbsim import AERO_SPEEDS_M_S

USABLE_BATTERY_FRACTION = 0.8  # of the nominal pack energy (libs/propulsion.py)
LEVEL_SPEED_STEP_M_S = 2.0
LEVEL_STEP_MIN_S = 8.0
LEVEL_STEP_MAX_S = 30.0
LEVEL_SPEED_TOLERANCE_M_S = 0.5  # a step is held once airspeed is this close to its target and the climb settled
LEVEL_SINK_LIMIT_M_S = 0.5  # ... climbing or sinking slower than this
LEVEL_STEADY_S = 3  # ... for this many seconds in a row
LEVEL_ALT_LOSS_LIMIT_M = 15.0




def run(mission):
    """A throttle loop holds each target airspeed, from sweep_from_m_s (default: the upper aero grid, allowing the
    holding tolerance) down in LEVEL_SPEED_STEP_M_S steps, until level flight cannot be held: per step speed, AoA,
    motor, battery power and current, Wh/km (means over its last LEVEL_STEADY_S seconds); then the slowest level speed, best range and best endurance on
    USABLE_BATTERY_FRACTION of the pack."""
    bench, export, motor_count = controller.engine, mission.export, mission.motor_count
    sweep_from_m_s = mission.parameters.get("sweep_from")
    controller.forward_start()
    fdm = bench.plant.fdm
    if sweep_from_m_s is None:
        sweep_from_m_s = math.floor((AERO_SPEEDS_M_S[-1] - LEVEL_SPEED_TOLERANCE_M_S) / LEVEL_SPEED_STEP_M_S) * LEVEL_SPEED_STEP_M_S
        print(f"Efficiency sweep starts at {sweep_from_m_s:g} m/s within the aero grid; use top-speed separately to test maximum speed.")
    start_speed_m_s = sweep_from_m_s
    target_alt_m = bench.truth()["agl_m"]
    speed = SpeedHold(target_m_s=start_speed_m_s)
    points = []
    print(f"\n{'target':>7} {'airspeed':>9} {'AoA':>6} {'pitch':>6} {'motor':>6} {'battery W':>10} {'A @pack':>8} {'Wh/km':>6} {'alt err':>8}")
    sweep_to_m_s = mission.parameters.get("sweep_to", float(AERO_SPEEDS_M_S[0]) + LEVEL_SPEED_TOLERANCE_M_S)
    while speed.target_m_s >= sweep_to_m_s:
        step_start, step_alt_m = bench.elapsed, bench.truth()["agl_m"]
        next_sample = step_start + 1.0
        held = []
        samples = []  # (time, motor, airspeed, AoA, pitch, battery W) every tick: a step reports the mean of its steady window
        while bench.elapsed - step_start < LEVEL_STEP_MAX_S:
            controller.forward.sticks(bench, speed.throttle(fdm["velocities/vt-fps"] * FT, 0.02))
            bench.run(0.02)
            samples.append((bench.elapsed, sum(motor_fraction(us) for us in bench.motors[:motor_count]) / motor_count, fdm["velocities/vt-fps"] * FT,
                            fdm["aero/alpha-deg"], pitch_deg(fdm), battery_power_w(fdm, export, motor_count)))
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
        mission.measure("level step", target_m_s=point.target_m_s, airspeed_m_s=point.airspeed_m_s, power_w=point.power_w,
                        climb_m_s=point.climb_m_s, aoa_deg=point.aoa_deg,
                        envelope_fallback=int(abs(point.aoa_deg) > export.forward_alpha_limit_deg))
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
    battery_wh = export.motors[0].pack_wh
    usable_wh = USABLE_BATTERY_FRACTION * battery_wh
    best_range = min(points, key=lambda p: p.power_w / p.airspeed_m_s)
    best_endurance = min(points, key=lambda p: p.power_w)
    mission.measure("performance", slowest_level_m_s=points[-1].airspeed_m_s,
                    slowest_level_aoa_deg=points[-1].aoa_deg,
                    best_range_speed_m_s=best_range.airspeed_m_s, best_wh_km=best_range.power_w / (3.6 * best_range.airspeed_m_s),
                    range_km=usable_wh * 3.6 * best_range.airspeed_m_s / best_range.power_w,
                    best_endurance_speed_m_s=best_endurance.airspeed_m_s, endurance_min=usable_wh / best_endurance.power_w * 60,
                    usable_battery_fraction=USABLE_BATTERY_FRACTION, battery_wh=battery_wh,
                    aero_speed_limit_m_s=float(AERO_SPEEDS_M_S[-1]),
                    outside_speed_grid=int(any(p.airspeed_m_s > AERO_SPEEDS_M_S[-1] or p.airspeed_m_s < AERO_SPEEDS_M_S[0] for p in points)))
    print(f"\nSlowest level flight held: {points[-1].airspeed_m_s:.1f} m/s ({points[-1].airspeed_m_s * 3.6:.0f} km/h) at AoA {points[-1].aoa_deg:+.1f} deg; "
          + (f"not held at {speed.target_m_s:.1f} m/s" if speed.target_m_s >= sweep_to_m_s else f"sweep lower bound {sweep_to_m_s:g} m/s reached"))
    print(f"On {USABLE_BATTERY_FRACTION:.0%} of {battery_wh:.1f} Wh (no battery sag modelled):")
    print(f"  best range of the steps:     {best_range.airspeed_m_s * 3.6:.0f} km/h at {best_range.power_w:.0f} W -> "
          f"{best_range.power_w / (3.6 * best_range.airspeed_m_s):.2f} Wh/km, {usable_wh / (best_range.power_w / (3.6 * best_range.airspeed_m_s)):.1f} km")
    print(f"  best endurance of the steps: {best_endurance.airspeed_m_s * 3.6:.0f} km/h at {best_endurance.power_w:.0f} W -> "
          f"{usable_wh / best_endurance.power_w * 60.0:.1f} min")
