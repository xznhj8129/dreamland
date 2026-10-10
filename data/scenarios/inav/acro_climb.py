"""Catapult in ACRO, hold 20° pitch / 0° roll through RC, climb to 200 m, then level off."""


from controllers import inav as controller

START = {"start": "catapult", "launch-pitch": 20, "launch-speed": 25}


def climb(flight):
    controller.attitude(pitch=20, roll=0, throttle=0.7)


def level(flight):
    controller.attitude(pitch=0, roll=0, throttle=0.4)


def reached_altitude(flight):
    return flight.truth["agl_m"] >= 200


def time_limit(flight):
    return flight.elapsed >= 90


def run(flight):
    flight.stop_when(time_limit, "90 second limit")
    controller.mode("ACRO")
    controller.takeoff(throttle=0.7)
    flight.measure("off launcher")
    flight.wait(60, until=reached_altitude, control=climb)
    flight.measure("200 m")
    flight.wait(10, control=level)
    flight.measure("finished")
