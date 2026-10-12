from controllers import inav as controller

from libs.inav_flight import accelerate
from libs.jsbsim import AERO_SPEEDS_M_S





def run(mission):
    bench, flight, motor_count = controller.engine, controller.forward, mission.motor_count
    controller.forward_start()
    top = accelerate(bench, flight, motor_count)
    mission.measure("top speed", airspeed_m_s=top, aero_speed_limit_m_s=float(AERO_SPEEDS_M_S[-1]),
                    outside_speed_grid=int(top > AERO_SPEEDS_M_S[-1] or top < AERO_SPEEDS_M_S[0]))
    print(f"\nTop speed at full throttle, level: {top:.1f} m/s = {top * 3.6:.0f} km/h")
