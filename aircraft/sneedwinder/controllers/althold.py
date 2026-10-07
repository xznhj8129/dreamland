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

from simple_pid import PID

from libs.controller import ACCEL_LSB_PER_G, AIR_DENSITY_KG_M3, GYRO_LSB_PER_DPS, Controller

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



def baro_altitude_m(pressure_pa: float) -> float:
    """Standard atmosphere, as a BMP280 driver's readAltitude."""
    return 44330.77 * (1.0 - (pressure_pa / SEA_LEVEL_PA) ** (1.0 / 5.25588))


class AltHold(Controller):
    rate_hz = 100

    def setup(self):
        self.roll = self.pitch = 0.0
        self.airspeed = self.airspeed_rate = None
        self.airspeed_pid = PID(*AIRSPEED, setpoint=TARGET_AIRSPEED_M_S, sample_time=None, output_limits=(THROTTLE_US[0] - 1500, THROTTLE_US[1] - 1500))
        self.altitude_pid = PID(*ALT_TO_PITCH, setpoint=TARGET_ALT_M, sample_time=None, output_limits=(-PITCH_LIMIT_DEG, PITCH_LIMIT_DEG))
        self.pitch_pid = PID(*PITCH, setpoint=0.0, sample_time=None, output_limits=(-CANARD_LIMIT_DEG, CANARD_LIMIT_DEG))
        self.roll_pid = PID(*ROLL, setpoint=0.0, sample_time=None, output_limits=(-TRAVEL_DEG, TRAVEL_DEG))

    def loop(self):
        truth = self.truth
        dt = 1.0 / self.rate_hz
        ax, ay, az = self.imu.accel()
        gx, gy, gz = self.imu.gyro()
        roll_rate = gx / GYRO_LSB_PER_DPS  # deg/s, + = rolling right (the IMU's y points left, z up)
        pitch_rate = -gy / GYRO_LSB_PER_DPS  # deg/s, + = nose up
        yaw_rate = -gz / GYRO_LSB_PER_DPS  # deg/s, + = nose right
        altitude = baro_altitude_m(self.baro.pressure())
        airspeed = math.sqrt(2.0 * max(0.0, self.pitot.pressure()) / AIR_DENSITY_KG_M3)
        if self.airspeed is None:
            self.airspeed, self.airspeed_rate = airspeed, 0.0
        self.airspeed_rate = AIRSPEED_SMOOTHING * self.airspeed_rate + (1 - AIRSPEED_SMOOTHING) * (airspeed - self.airspeed) / dt
        self.airspeed = airspeed
        # speeding up along x reads as nose up; flying at airspeed while rotating adds rotation x velocity on y and z
        ax -= self.airspeed_rate / STANDARD_GRAVITY_M_S2 * ACCEL_LSB_PER_G
        ay -= math.radians(gz / GYRO_LSB_PER_DPS) * airspeed / STANDARD_GRAVITY_M_S2 * ACCEL_LSB_PER_G
        az += math.radians(gy / GYRO_LSB_PER_DPS) * airspeed / STANDARD_GRAVITY_M_S2 * ACCEL_LSB_PER_G
        roll, pitch = math.radians(self.roll), math.radians(self.pitch)
        self.roll += (roll_rate + math.tan(pitch) * (pitch_rate * math.sin(roll) + yaw_rate * math.cos(roll))) * dt
        self.pitch += (pitch_rate * math.cos(roll) - yaw_rate * math.sin(roll)) * dt
        if ACCEL_TRUSTED_G[0] < math.sqrt(ax * ax + ay * ay + az * az) / ACCEL_LSB_PER_G < ACCEL_TRUSTED_G[1]:
            self.roll = GYRO_SHARE * self.roll + (1 - GYRO_SHARE) * math.degrees(math.atan2(ay, az))
            self.pitch = GYRO_SHARE * self.pitch + (1 - GYRO_SHARE) * math.degrees(math.atan2(ax, math.hypot(ay, az)))

        self.pitch_pid.setpoint = self.altitude_pid(altitude, dt=dt)
        scale = max(GAIN_SCALE_LIMITS[0], min(GAIN_SCALE_LIMITS[1], (GAIN_AIRSPEED_M_S / max(airspeed, 1.0)) ** 2))
        nose_up_deg = max(-CANARD_LIMIT_DEG, min(CANARD_LIMIT_DEG, scale * (self.pitch_pid(self.pitch, dt=dt) - PITCH_RATE_DAMPING * pitch_rate)))
        roll_left_deg = -max(-TRAVEL_DEG, min(TRAVEL_DEG, scale * (self.roll_pid(self.roll, dt=dt) - ROLL_RATE_DAMPING * roll_rate)))

        for canard in range(4):
            self.servo[canard] = round(1500 + 500 * nose_up_deg / TRAVEL_DEG)
        self.servo[4] = round(1500 + 500 * roll_left_deg / TRAVEL_DEG)
        self.esc[0] = round(1500 + self.airspeed_pid(airspeed, dt=dt))
        self.log("alt_m", altitude)
        self.log("airspeed_m_s", airspeed)
        self.log("pitch_target", self.pitch_pid.setpoint)
        self.log("est_roll", self.roll)
        self.log("est_pitch", self.pitch)
