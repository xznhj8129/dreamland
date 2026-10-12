"""
Holds 100 m and wings level with simple-pid: altitude from the barometer sets a pitch target, a pitch loop on the
canards holds it (with gyro rate damping), a roll loop on the ailerons holds zero roll, and the throttle holds the
airspeed (pitot) so the pitch loop has the energy to hold altitude. The loops work in degrees of surface deflection,
turned into pulses by the aircraft's travel (TRAVEL_DEG); the canards stay within CANARD_LIMIT_DEG: at about 9 deg of
angle of attack, more stalls them.

Attitude is the gyro (Euler rates, so a banked turn's yaw rate goes where it belongs), pulled slowly towards the
accelerometer while it reads about 1 g (in a pushover or a pull-up it is not pointing down). The accelerometer is
first corrected, as INAV does, for what the aircraft's own motion adds: the change in airspeed along x (else
speeding up reads as nose up) and the centripetal acceleration of a turn (rotation x airspeed; else a steady
coordinated bank reads as level and is never corrected).

The surfaces' effect grows with dynamic pressure, so the pitch and roll commands are scaled by
(GAIN_AIRSPEED_M_S / airspeed)^2 (INAV's TPA does the same with throttle): tuned at 40 m/s, they would oscillate faster.

Wiring (from the aircraft's servo mix): S1..S4 canards, pulse up = nose up on all four; S5 ailerons (two panels on
reversers), pulse up = roll left; ESC 0 the pusher.
"""

import math

from controllers import mcu as controller

from simple_pid import PID

from libs.sensors import ACCEL_LSB_PER_G, AIR_DENSITY_KG_M3, GYRO_LSB_PER_DPS

STANDARD_GRAVITY_M_S2 = 9.80665

TARGET_ALT_M = 100.0
TARGET_AIRSPEED_M_S = 100.0  # above top speed (about 71 m/s): full throttle
THROTTLE_US = (1100, 2000)  # the airspeed loop's range
GYRO_SHARE = 0.999  # per update, how much of the attitude comes from the gyro (a 10 s pull towards the accelerometer)
ACCEL_TRUSTED_G = (0.85, 1.15)  # the accelerometer corrects the attitude only inside this
SEA_LEVEL_PA = 101325.0

ALT_TO_PITCH = (0.3, 0.02, 0.3)  # deg of pitch target per m, per m s, per m/s
PITCH_LIMIT_DEG = 10.0
GAIN_AIRSPEED_M_S = 40.0  # the pitch and roll gains are for this airspeed
GAIN_SCALE_LIMITS = (0.25, 2.0)  # (GAIN_AIRSPEED_M_S / airspeed)^2 kept inside this
AIRSPEED_SMOOTHING = 0.98  # per update, of the airspeed rate used to correct the accelerometer
TRAVEL_DEG = 45.0  # surface deflection at a 1000 / 2000 us pulse (the aircraft's control deflection_max_deg)
PITCH = (0.5, 0.25, 0.0)  # deg of canard per deg, per deg s
PITCH_RATE_DAMPING = 0.15  # deg of canard per deg/s
CANARD_LIMIT_DEG = 12.5
AIRSPEED = (30.0, 10.0, 0.0)  # us of throttle per m/s, per m
ROLL = (0.75, 0.5, 0.0)  # deg of aileron per deg, per deg s
ROLL_RATE_DAMPING = 0.075  # deg of aileron per deg/s



LOG_FIELDS = ('alt_m', 'pitot_airspeed_m_s', 'pitch_target', 'est_roll', 'est_pitch')


def baro_altitude_m(pressure_pa: float) -> float:
    """Standard atmosphere, as a BMP280 driver's readAltitude."""
    return 44330.77 * (1.0 - (pressure_pa / SEA_LEVEL_PA) ** (1.0 / 5.25588))


RATE_HZ = 100


def run(flight):
    roll = pitch = 0.0
    previous_airspeed = airspeed_rate = None
    airspeed_pid = PID(*AIRSPEED, setpoint=TARGET_AIRSPEED_M_S, sample_time=None, output_limits=(THROTTLE_US[0] - 1500, THROTTLE_US[1] - 1500))
    altitude_pid = PID(*ALT_TO_PITCH, setpoint=TARGET_ALT_M, sample_time=None, output_limits=(-PITCH_LIMIT_DEG, PITCH_LIMIT_DEG))
    pitch_pid = PID(*PITCH, setpoint=0.0, sample_time=None, output_limits=(-CANARD_LIMIT_DEG, CANARD_LIMIT_DEG))
    roll_pid = PID(*ROLL, setpoint=0.0, sample_time=None, output_limits=(-TRAVEL_DEG, TRAVEL_DEG))
    controller.arm()
    controller.release()
    while True:
        truth = flight.sensors.truth
        dt = 1.0 / RATE_HZ
        ax, ay, az = flight.sensors.imu.accel()
        gx, gy, gz = flight.sensors.imu.gyro()
        roll_rate = gx / GYRO_LSB_PER_DPS  # deg/s, + = rolling right (the IMU's y points left, z up)
        pitch_rate = -gy / GYRO_LSB_PER_DPS  # deg/s, + = nose up
        yaw_rate = -gz / GYRO_LSB_PER_DPS  # deg/s, + = nose right
        altitude = baro_altitude_m(flight.sensors.baro.pressure())
        airspeed = math.sqrt(2.0 * max(0.0, flight.sensors.pitot.pressure()) / AIR_DENSITY_KG_M3)
        if previous_airspeed is None:
            previous_airspeed, airspeed_rate = airspeed, 0.0
        airspeed_rate = AIRSPEED_SMOOTHING * airspeed_rate + (1 - AIRSPEED_SMOOTHING) * (airspeed - previous_airspeed) / dt
        previous_airspeed = airspeed
        # speeding up along x reads as nose up; flying at airspeed while rotating adds rotation x velocity on y and z
        ax -= airspeed_rate / STANDARD_GRAVITY_M_S2 * ACCEL_LSB_PER_G
        ay -= math.radians(gz / GYRO_LSB_PER_DPS) * airspeed / STANDARD_GRAVITY_M_S2 * ACCEL_LSB_PER_G
        az += math.radians(gy / GYRO_LSB_PER_DPS) * airspeed / STANDARD_GRAVITY_M_S2 * ACCEL_LSB_PER_G
        roll_rad, pitch_rad = math.radians(roll), math.radians(pitch)
        roll += (roll_rate + math.tan(pitch_rad) * (pitch_rate * math.sin(roll_rad) + yaw_rate * math.cos(roll_rad))) * dt
        pitch += (pitch_rate * math.cos(roll_rad) - yaw_rate * math.sin(roll_rad)) * dt
        if ACCEL_TRUSTED_G[0] < math.sqrt(ax * ax + ay * ay + az * az) / ACCEL_LSB_PER_G < ACCEL_TRUSTED_G[1]:
            roll = GYRO_SHARE * roll + (1 - GYRO_SHARE) * math.degrees(math.atan2(ay, az))
            pitch = GYRO_SHARE * pitch + (1 - GYRO_SHARE) * math.degrees(math.atan2(ax, math.hypot(ay, az)))

        pitch_pid.setpoint = altitude_pid(altitude, dt=dt)
        scale = max(GAIN_SCALE_LIMITS[0], min(GAIN_SCALE_LIMITS[1], (GAIN_AIRSPEED_M_S / max(airspeed, 1.0)) ** 2))
        nose_up_deg = max(-CANARD_LIMIT_DEG, min(CANARD_LIMIT_DEG, scale * (pitch_pid(pitch, dt=dt) - PITCH_RATE_DAMPING * pitch_rate)))
        roll_left_deg = -max(-TRAVEL_DEG, min(TRAVEL_DEG, scale * (roll_pid(roll, dt=dt) - ROLL_RATE_DAMPING * roll_rate)))

        for canard in range(4):
            controller.servo[canard] = round(1500 + 500 * nose_up_deg / TRAVEL_DEG)
        controller.servo[4] = round(1500 + 500 * roll_left_deg / TRAVEL_DEG)
        controller.esc[0] = round(1500 + airspeed_pid(airspeed, dt=dt))
        flight.log("alt_m", altitude)
        flight.log("pitot_airspeed_m_s", airspeed)
        flight.log("pitch_target", pitch_pid.setpoint)
        flight.log("est_roll", roll)
        flight.log("est_pitch", pitch)
        flight.wait(1 / RATE_HZ)
