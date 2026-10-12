"""JSBSim flight-dynamics state in domain terms: the simulator's raw property names live only here."""

import math


def roll_deg(fdm):
    """Bank angle, deg: + right wing down."""
    return fdm["attitude/phi-deg"]


def pitch_deg(fdm):
    """Pitch angle, deg: + nose up."""
    return fdm["attitude/theta-deg"]


def heading_deg(fdm):
    """Heading, deg 0..360: + right (clockwise seen from above)."""
    return fdm["attitude/psi-deg"]


def roll_rad(fdm):
    """Bank angle, rad: + right wing down."""
    return fdm["attitude/phi-rad"]


def pitch_rad(fdm):
    """Pitch angle, rad: + nose up."""
    return fdm["attitude/theta-rad"]


def heading_rad(fdm):
    """Heading, rad 0..2pi: + right (clockwise seen from above)."""
    return fdm["attitude/psi-rad"]


def roll_rate_rad_s(fdm):
    """Body roll rate, rad/s: + right wing down."""
    return fdm["velocities/p-rad_sec"]


def pitch_rate_rad_s(fdm):
    """Body pitch rate, rad/s: + nose up."""
    return fdm["velocities/q-rad_sec"]


def yaw_rate_rad_s(fdm):
    """Body yaw rate, rad/s: + nose right."""
    return fdm["velocities/r-rad_sec"]


def roll_rate_deg_s(fdm):
    """Body roll rate, deg/s: + right wing down."""
    return math.degrees(roll_rate_rad_s(fdm))


def pitch_rate_deg_s(fdm):
    """Body pitch rate, deg/s: + nose up."""
    return math.degrees(pitch_rate_rad_s(fdm))


def yaw_rate_deg_s(fdm):
    """Body yaw rate, deg/s: + nose right."""
    return math.degrees(yaw_rate_rad_s(fdm))
