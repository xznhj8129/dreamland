"""Arm, climb, hold the test attitude, check readiness, record, finish."""


from controllers import inav as controller


# Parameters and telemetry: available throughout the sequence.
PARAMETERS = {
    "target_alt_m": 200,
    "climb_pitch_deg": 20,
    "climb_throttle": 0.7,
    "climb_timeout_s": 60,
    "level_pitch_deg": 0,
    "level_throttle": 1.0,
    "settle_s": 10,
    "settle_angle_tolerance_deg": 3,
    "settle_climb_limit_m_s": 0.5,
    "test_s": 10,
    "finish_s": 2
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

    flight.phase("launch", "2. Climb to the target altitude, hold the test attitude, then check readiness.")
    flight.wait(flight.parameters["climb_timeout_s"], until=reached_altitude, control=hold_climb_attitude)
    flight.measure("climb complete")
    flight.wait(flight.parameters["settle_s"], control=hold_test_attitude)
    check_ready_for_test(flight)

    flight.phase("test", "3. Record the response while continuing to hold the test attitude.")
    flight.measure("test begins")
    flight.wait(flight.parameters["test_s"], control=hold_test_attitude)
    flight.measure("test ends")

    flight.phase("finish", "4. Hold the test attitude briefly, then return to end the simulation.")
    flight.wait(flight.parameters["finish_s"], control=hold_test_attitude)
    flight.measure("finished")


# Control helpers
# wait(control=...) calls the selected helper every 20 ms during the wait.


def hold_climb_attitude(flight):
    controller.attitude(pitch=flight.parameters["climb_pitch_deg"], roll=0,
                    throttle=flight.parameters["climb_throttle"])


def hold_test_attitude(flight):
    # Body pitch and roll targets; this does not hold altitude.
    controller.attitude(pitch=flight.parameters["level_pitch_deg"], roll=0,
                    throttle=flight.parameters["level_throttle"])


# Check helpers
# until=... is checked during a wait; check_ready_for_test runs once afterward.


def reached_altitude(flight):
    return flight.truth["agl_m"] >= flight.parameters["target_alt_m"]


def check_ready_for_test(flight):
    state = flight.truth
    if (abs(state["roll"]) > flight.parameters["settle_angle_tolerance_deg"]
            or abs(state["pitch"] - flight.parameters["level_pitch_deg"]) > flight.parameters["settle_angle_tolerance_deg"]):
        raise RuntimeError(f"Not ready for the test: roll {state['roll']:.1f} deg, pitch {state['pitch']:.1f} deg, climb {-state['vd']:.1f} m/s")
