from controllers import mcu as controller

def run(flight):
    controller.esc[:] = [2000] * len(controller.esc)
    controller.arm()
    controller.release()
    while True:
        flight.wait(0.01)
