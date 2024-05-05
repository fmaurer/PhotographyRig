import time
import gpiod
from motor_driver import MotorDriver

class L298NDriver(MotorDriver):
    def __init__(self, motor_pins):
        # motor_pins should be a dictionary like {'A': [17, 18], 'B': [27, 22]}
        self.motor_pins = motor_pins
        self.chip = gpiod.Chip('/dev/gpiochip0')
        self.setup_pins()

    def setup_pins(self):
        # This function sets up motor pins
        self.lines = self.chip.get_lines(self.motor_pins)
        self.lines.request(consumer='stepper_motor', type=gpiod.LINE_REQ_DIR_OUT)

    def step_to_angle(self, degrees):
        #TODO: implement drive stepper driver to reach the full assembly's final degree of pan/tilt motors
        self.step_motor(200, 0.02)
        return 0

    def step_motor(self, steps, step_delay):
        print("Moving stepper ", str(steps), " with a ", str(step_delay), " delay")
        sequence = [
            (1, 0, 0, 1),  # This sequence should be adjusted based on your motor and wiring
            (0, 1, 0, 1),
            (0, 1, 1, 0),
            (1, 0, 1, 0)
        ]
        for i in range(steps):
            for state in sequence:
                self.lines.set_values(state)
            time.sleep(step_delay)  # small delay between steps for visibility

    def cleanup(self):
        # This function releases the GPIO lines when they are no longer needed
        self.lines.release()

