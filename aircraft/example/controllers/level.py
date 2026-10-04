"""
A $20 plane stabilizer, MPU-6050 style: attitude from the gyro, pulled slowly towards the accelerometer's idea of
down (one line per axis), then a PID per axis to hold wings level and a slight nose-up; the throttle is fixed. The
roll integral trims out the motor's torque.

Wiring (from the aircraft's servo mix): S1, S2 elevators (pulse up = nose down), S3 rudder, S4 left aileron
(pulse up = roll left), S5 right aileron (pulse up = roll right), ESC 0 the motor.
"""

import math

from libs.controller import GYRO_LSB_PER_DPS, Controller

THROTTLE_US = 1600
GYRO_SHARE = 0.995  # per update, how much of the attitude comes from the gyro
PITCH_TRIM_DEG = 2.0  # the nose-up attitude it holds
ROLL_P, ROLL_I, ROLL_D = 0.03, 0.02, 0.003  # stick (-1..1) per degree, per degree-second, per degree/second
PITCH_P, PITCH_D = 0.04, 0.004


class Level(Controller):
    rate_hz = 100

    def setup(self):
        self.esc[0] = THROTTLE_US
        self.roll = self.pitch = self.roll_trim = 0.0

    def loop(self):
        dt = 1.0 / self.rate_hz
        ax, ay, az = self.imu.accel()
        gx, gy, gz = self.imu.gyro()
        roll_rate = gx / GYRO_LSB_PER_DPS  # deg/s, + = rolling right (the IMU's y points left, z up)
        pitch_rate = -gy / GYRO_LSB_PER_DPS  # deg/s, + = nose up

        self.roll = GYRO_SHARE * (self.roll + roll_rate * dt) + (1 - GYRO_SHARE) * math.degrees(math.atan2(ay, az))
        self.pitch = GYRO_SHARE * (self.pitch + pitch_rate * dt) + (1 - GYRO_SHARE) * math.degrees(math.atan2(ax, az))

        self.roll_trim = max(-1.0, min(1.0, self.roll_trim - ROLL_I * self.roll * dt))
        roll_right = -ROLL_P * self.roll - ROLL_D * roll_rate + self.roll_trim
        nose_up = PITCH_P * (PITCH_TRIM_DEG - self.pitch) - PITCH_D * pitch_rate

        self.servo[0] = self.servo[1] = round(1500 - 500 * nose_up)
        self.servo[3] = round(1500 - 500 * roll_right)
        self.servo[4] = round(1500 + 500 * roll_right)
        self.log("est_roll", self.roll)
        self.log("est_pitch", self.pitch)
