"""Arm, climb to the target altitude, then hand over to INAV: NAV POSHOLD for 30 s, then NAV RTH, which returns home and lands."""


from controllers import inav as controller

from libs.simrun import GroundHit


# Parameters and telemetry: available throughout the sequence.
PARAMETERS = {
    "target_alt_m": 200,
    "climb_pitch_deg": 20,
    "climb_throttle": 0.7,
    "climb_timeout_s": 60,
    "level_pitch_deg": 0,
    "level_throttle": 0.4,
    "settle_s": 10,
    "settle_angle_tolerance_deg": 3,
    "settle_climb_limit_m_s": 0.5,
    "poshold_s": 30,
    "rth_timeout_s": 999
}
TELEMETRY = {
    "rate_hz": 20,
    "fields": [
        "agl_m",
        "airspeed_m_s",
        "aoa_deg",
        "roll",
        "pitch",
        "yaw",
        "p_deg_s",
        "q_deg_s",
        "r_deg_s"
    ]
}


# Sequence of events: runs top to bottom; an exception stops here.


def run(flight):
    flight.phase("start", "1. Enter ACRO, arm, and release the configured air/catapult/rail start.")
    if flight.start == "ground":
        raise ValueError("This template flies forward; customize the start and launch steps for ground takeoff and VTOL transition")
    controller.mode("ACRO")
    controller.takeoff(throttle=flight.parameters["climb_throttle"])

    flight.phase("launch", "2. Climb to the target altitude, settle, then check readiness.")
    flight.wait(flight.parameters["climb_timeout_s"], until=reached_altitude, control=hold_climb_attitude)
    flight.measure("climb complete")
    flight.wait(flight.parameters["settle_s"], control=hold_test_attitude)
    check_ready_for_nav(flight)

    flight.phase("poshold", "3. Hand over to INAV NAV POSHOLD and hold position.")
    controller.mode("NAV POSHOLD")
    flight.measure("nav poshold begins")
    flight.wait(flight.parameters["poshold_s"], control=hold_nav)

    flight.phase("rth", "4. Hand over to INAV NAV RTH: return home and land.")
    controller.mode("NAV RTH")
    flight.measure("nav rth begins")
    try:
        flight.wait(flight.parameters["rth_timeout_s"], control=hold_nav)
    except GroundHit as hit:
        print(f"\nLanded: {hit}")
        flight.measure("landed")
        return
    flight.measure("rth timeout, still airborne")


# Control helpers
# wait(control=...) calls the selected helper every 20 ms during the wait.


def hold_climb_attitude(flight):
    controller.attitude(pitch=flight.parameters["climb_pitch_deg"], roll=0,
                    throttle=flight.parameters["climb_throttle"])


def hold_test_attitude(flight):
    # Body pitch and roll targets; this does not hold altitude.
    controller.attitude(pitch=flight.parameters["level_pitch_deg"], roll=0,
                    throttle=flight.parameters["level_throttle"])


def hold_nav(flight):
    # Hands off: the active INAV nav mode owns attitude and throttle.
    controller.sticks(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0)


# Check helpers
# until=... is checked during a wait; check_ready_for_nav runs once afterward.


def reached_altitude(flight):
    return flight.truth["agl_m"] >= flight.parameters["target_alt_m"]


def check_ready_for_nav(flight):
    state = flight.truth
    if (abs(state["roll"]) > flight.parameters["settle_angle_tolerance_deg"]
            or abs(state["pitch"] - flight.parameters["level_pitch_deg"]) > flight.parameters["settle_angle_tolerance_deg"]
            or abs(state["vd"]) > flight.parameters["settle_climb_limit_m_s"]):
        raise RuntimeError(f"Not ready for the nav hand-over: roll {state['roll']:.1f} deg, pitch {state['pitch']:.1f} deg, climb {-state['vd']:.1f} m/s")
