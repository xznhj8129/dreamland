"""
A $20 plane stabilizer, MPU-6050 style: attitude from the gyro, pulled slowly towards the accelerometer's idea of
down (one line per axis), then a PID per axis to hold wings level and a slight nose-up; the throttle is fixed. The
roll integral trims out the motor's torque.

Wiring (from the aircraft's servo mix): S1, S2 elevators (pulse up = nose down), S3 rudder, S4 left aileron
(pulse up = roll left), S5 right aileron (pulse up = roll right), ESC 0 the motor.
"""

import math

from controllers import mcu as controller

from libs.sensors import GYRO_LSB_PER_DPS

THROTTLE_US = 1600
GYRO_SHARE = 0.995  # per update, how much of the attitude comes from the gyro
PITCH_TRIM_DEG = 2.0  # the nose-up attitude it holds
ROLL_P, ROLL_I, ROLL_D = 0.03, 0.02, 0.003  # stick (-1..1) per degree, per degree-second, per degree/second
PITCH_P, PITCH_D = 0.04, 0.004


RATE_HZ = 100
START = {"start": "air"}


LOG_FIELDS = ('est_roll', 'est_pitch')


def run(flight):
    controller.esc[0] = THROTTLE_US
    roll = pitch = roll_trim = 0.0
    controller.arm()
    controller.release()
    while True:
        dt = 1.0 / RATE_HZ
        ax, ay, az = flight.sensors.imu.accel()
        gx, gy, gz = flight.sensors.imu.gyro()
        roll_rate = gx / GYRO_LSB_PER_DPS  # deg/s, + = rolling right (the IMU's y points left, z up)
        pitch_rate = -gy / GYRO_LSB_PER_DPS  # deg/s, + = nose up

        roll = GYRO_SHARE * (roll + roll_rate * dt) + (1 - GYRO_SHARE) * math.degrees(math.atan2(ay, az))
        pitch = GYRO_SHARE * (pitch + pitch_rate * dt) + (1 - GYRO_SHARE) * math.degrees(math.atan2(ax, az))

        roll_trim = max(-1.0, min(1.0, roll_trim - ROLL_I * roll * dt))
        roll_right = -ROLL_P * roll - ROLL_D * roll_rate + roll_trim
        nose_up = PITCH_P * (PITCH_TRIM_DEG - pitch) - PITCH_D * pitch_rate

        controller.servo[0] = controller.servo[1] = round(1500 - 500 * nose_up)
        controller.servo[3] = round(1500 - 500 * roll_right)
        controller.servo[4] = round(1500 + 500 * roll_right)
        flight.log("est_roll", roll)
        flight.log("est_pitch", pitch)
        flight.wait(1 / RATE_HZ)
