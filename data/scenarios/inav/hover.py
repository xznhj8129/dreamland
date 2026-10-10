from controllers import inav as controller

from libs.inav_flight import STAND_S, fly
CLIMB_S = 2.5
CLIMB_STICK_ABOVE_HOVER = 0.12  # throttle stick above the predicted hover stick for the climb
ALTHOLD_STICK = 0.5  # stick centre: hold altitude


START = {"start": "ground"}


def run(mission):
    bench, export, motor_count = controller.engine, mission.export, mission.motor_count
    hold_s = mission.parameters.get("hover_s", 15.0)
    hover_stick = (export.hover_stick_us - 1000) / 1000.0
    bench.release()  # stand on the tail contacts
    fly(bench, STAND_S, "standing", motor_count)
    bench.mode("ANGLE")
    bench.arm()
    bench.sticks(throttle=min(1.0, hover_stick + CLIMB_STICK_ABOVE_HOVER))
    fly(bench, CLIMB_S, "climb", motor_count)
    bench.sticks(throttle=ALTHOLD_STICK)
    try:
        bench.mode("NAV POSHOLD")
        hold = "poshold"
    except KeyError:  # the config has no POSHOLD switch
        bench.mode("NAV ALTHOLD")
        hold = "althold"
    hover = fly(bench, hold_s, hold, motor_count)
    settled = hover[len(hover) // 2:]
    mean = sum(settled) / len(settled)
    mission.measure("hover", motor_fraction=mean, predicted_motor_fraction=export.hover_motor_fraction)
    print(f"\nHover motor command (second half of {hold}): {mean:.3f}; export predicted {export.hover_motor_fraction:.3f} "
          f"({(mean / export.hover_motor_fraction - 1.0) * 100.0:+.1f} %)")
