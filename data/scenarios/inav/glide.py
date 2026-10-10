from controllers import inav as controller

import math

from libs.simrun import FT, GroundHit
GLIDE_SWEEP_FROM_FACTOR = 2.0  # the glide sweep's first target airspeed, times the operating airspeed
GLIDE_SPEED_STEP_M_S = 2.0
GLIDE_STEP_S = 8.0
GLIDE_MEASURE_S = 3.0  # each step's sink rate and glide ratio are means over its last this many seconds
GLIDE_STEADY_M_S = 0.5  # a step counts when its airspeed changed less than this over the measuring window (no zoom or dive)


START = {"start": "air", "alt": 1000}


def run(mission):
    """Motors off; the gliding pilot aims at each target airspeed with pitch, from GLIDE_SWEEP_FROM_FACTOR x the operating
    airspeed down in GLIDE_SPEED_STEP_M_S steps, until the ground: per step the sink rate and glide ratio (means over its
    last GLIDE_MEASURE_S); then the smallest sink rate and the best glide of the steady steps, whatever speed they settled at."""
    bench, flight, cruise_m_s = controller.engine, controller.forward, mission.cruise_m_s
    controller.forward_start()
    fdm = bench.plant.fdm
    pilot = flight.glide(bench, GLIDE_SWEEP_FROM_FACTOR * cruise_m_s)
    steady = []
    print(f"\n{'target':>7} {'airspeed':>9} {'sink':>9} {'glide':>6} {'AoA':>6} {'pitch':>6} {'agl':>7}")
    try:
        while pilot.target_speed_m_s > 0.0:
            step_start = bench.elapsed
            samples = []  # (time, airspeed, sink, ground speed, AoA, pitch)
            while bench.elapsed - step_start < GLIDE_STEP_S:
                controller.forward.sticks(bench, 0.0)
                bench.run(0.02)
                truth = bench.truth()
                samples.append((bench.elapsed, fdm["velocities/vt-fps"] * FT, truth["vd"], math.hypot(truth["vn"], truth["ve"]), fdm["aero/alpha-deg"], truth["pitch"]))
            window = [sample[1:] for sample in samples if sample[0] >= bench.elapsed - GLIDE_MEASURE_S]
            airspeed, sink, ground_speed, aoa, pitch = (sum(column) / len(window) for column in zip(*window))
            ratio = ground_speed / sink if sink > 0.0 else math.inf
            settled = abs(window[-1][0] - window[0][0]) < GLIDE_STEADY_M_S
            mission.measure("glide step", target_m_s=pilot.target_speed_m_s, airspeed_m_s=airspeed, sink_m_s=sink, glide_ratio=ratio if math.isfinite(ratio) else None, steady=settled)
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
