from controllers import inav as controller

from libs.simrun import FT, GroundHit
from libs.inav_flight import SpeedHold, level_line
CONTROLS_LEVEL_S = 10.0
CONTROLS_STEP_S = 3.0
CONTROLS_RECOVER_S = 5.0
CONTROLS_RESPONSE_MIN_DEG = 5.0

CONTROL_STEPS = (("roll right", "roll", 0.5, "attitude/phi-deg", 1.0), ("pull up", "pitch", -0.5, "attitude/theta-deg", 1.0),
               ("yaw right", "yaw", 0.5, "attitude/psi-deg", 1.0))


START = {"start": "air"}


def run(mission):
    """Level for CONTROLS_LEVEL_S, then a right roll, a pull up and right yaw on the sticks in turn, each for
    up to CONTROLS_STEP_S, ending on the response goal, then CONTROLS_RECOVER_S; an axis passes when the true attitude moves at least
    CONTROLS_RESPONSE_MIN_DEG the commanded way during its step, flown hands on (ForwardFlight.hands_on) through the step
    and its recovery. A powered aircraft holds speed_m_s with the throttle
    loop throughout; an unpowered one glides. Prints the control-surface deflections, so the INAV servo mix is checked
    end to end."""
    bench, flight, motor_count = controller.engine, controller.forward, mission.motor_count
    control_count = mission.control_count
    speed_m_s = mission.cruise_m_s if controller.forward.powered else None
    controller.forward_start()
    fdm = bench.plant.fdm
    speed = None if speed_m_s is None else SpeedHold(target_m_s=speed_m_s)
    results = []

    def hold(seconds: float, phase: str, override: dict[str, float] | None = None, attitude: str | None = None) -> float:
        """Fly for seconds; returns how far the attitude moved (unwrapped, deg) at its furthest from the start."""
        start, previous, moved, extreme = bench.elapsed, fdm[attitude] if attitude else 0.0, 0.0, 0.0
        next_report = start + 1.0
        while bench.elapsed - start < seconds:
            controller.forward.sticks(bench, 0.0 if speed is None else speed.throttle(fdm["velocities/vt-fps"] * FT, 0.02), override)
            bench.run(0.02)
            if attitude:
                moved += (fdm[attitude] - previous + 180.0) % 360.0 - 180.0
                previous = fdm[attitude]
                extreme = max(extreme, moved, key=abs)
                if extreme >= CONTROLS_RESPONSE_MIN_DEG:
                    break  # the commanded response is measured; recover before an unnecessary prolonged manoeuvre
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
            mission.measure(label, movement_deg=moved, passed=passed)
            print(f"  {label}: {attitude.split('/')[1]} moved {moved:+.1f} deg -> {'PASS' if passed else 'FAIL'}")
    except GroundHit as hit:
        print(f"\nHit the ground {hit}")
    print(f"\nControls: {sum(results)}/{len(CONTROL_STEPS)} axes respond the commanded way")
