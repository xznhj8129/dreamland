"""Unguided rocket flight: fire the igniters, leave the launcher and fly the ballistic arc with no control at all.

Starts on a rail at 85 deg (START; the SIM toolbar or CLI override it). Records rail exit, burnout, the top speed and
apogee, then flies on until it hits the ground (no recovery yet).
"""

from controllers import mcu as controller

START = {"start": "rail", "launch-pitch": 85}

BURN_LIMIT_S = 30.0  # longest wait for the motors to burn out
COAST_LIMIT_S = 300.0  # longest wait for apogee


def run(flight):
    top = {"airspeed_m_s": 0.0, "t": 0.0}

    def track(flight):
        speed = flight.state["airspeed_m_s"]
        if speed > top["airspeed_m_s"]:
            top.update(airspeed_m_s=speed, t=flight.elapsed)

    flight.phase("ignition", "1. Fire the igniters and leave the launcher.")
    controller.ignite()
    controller.release()
    flight.measure("rail exit", airspeed_m_s=flight.state["airspeed_m_s"], accel_x_g=flight.state["accel_x_g"])

    flight.phase("boost", "2. Fly until the motors burn out.")
    flight.wait(BURN_LIMIT_S, until=lambda flight: flight.state["rocket_thrust_n"] == 0.0, control=track)
    state = flight.state
    flight.measure("burnout", airspeed_m_s=state["airspeed_m_s"], agl_m=state["agl_m"], mass_kg=state["mass_kg"])

    flight.phase("coast", "3. Coast up to apogee.")
    flight.wait(COAST_LIMIT_S, until=lambda flight: flight.truth["vd"] >= 0.0, control=track)
    state = flight.state
    flight.measure("top speed", airspeed_m_s=top["airspeed_m_s"], at_s=top["t"])
    flight.measure("apogee", agl_m=state["agl_m"], airspeed_m_s=state["airspeed_m_s"], pitch_deg=state["pitch"],
                   downrange_m=(state["north_m"] ** 2 + state["east_m"] ** 2) ** 0.5)

    flight.phase("descent", "4. Fall back to the ground (no recovery yet).")
    flight.wait(flight.duration_s)
