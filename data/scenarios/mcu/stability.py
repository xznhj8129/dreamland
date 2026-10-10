"""PWM pitch pulse, then restore the outputs and record the free response.

Example wiring: servo outputs 0 and 1 drive the elevators; increasing PWM pushes
nose down. Set the wiring and baseline PWM values here for the aircraft under test.
There is no attitude or altitude controller.
"""

from controllers import mcu as controller

START = {"start": "air"}
MOTOR_US = 1600
SERVO_TRIM_US = 1500
PITCH_OUTPUTS = (0, 1)
PITCH_STEP_US = 40
BASELINE_S = 1.0
PULSE_S = 0.5
RESPONSE_S = 8.0


def run(flight):
    flight.phase("baseline", "1. Apply the baseline PWM and release the start.")
    controller.esc[:] = [MOTOR_US] * len(controller.esc)
    controller.servo[:] = [SERVO_TRIM_US] * len(controller.servo)
    controller.arm()
    controller.release()
    flight.wait(BASELINE_S)
    flight.measure("before pitch pulse", gyro_y_counts=flight.sensors.imu.gyro()[1])

    flight.phase("pulse", "2. Increase the elevator PWM for a short pulse.")
    for output in PITCH_OUTPUTS:
        controller.servo[output] = SERVO_TRIM_US + PITCH_STEP_US
    flight.wait(PULSE_S)
    flight.measure("pitch pulse released", gyro_y_counts=flight.sensors.imu.gyro()[1])

    flight.phase("response", "3. Restore the baseline PWM and record the response.")
    for output in PITCH_OUTPUTS:
        controller.servo[output] = SERVO_TRIM_US
    flight.wait(RESPONSE_S)
    flight.measure("open-loop response", gyro_y_counts=flight.sensors.imu.gyro()[1])
