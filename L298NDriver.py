import time
import gpiod
from motor_driver import MotorDriver

class L298NDriver(MotorDriver):
    def __init__(self, motor_pins):
        # motor_pins should be a dictionary like {'A': [17, 18], 'B': [27, 22]}
        self.motor_pins = motor_pins
        self.chip = gpiod.Chip('/dev/gpiochip0')
        self.lines = self.setup_pins()

    def setup_pins(self):
        line_objects = {}
        for key, pins in self.motor_pins.items():
            line_objects[key] = [self.chip.get_line(pin) for pin in pins]
            for line in line_objects[key]:
                line.request(consumer='motor', type=gpiod.LINE_REQ_DIR_OUT, default_vals=0)
        return line_objects

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
        step_count = len(sequence)
        for i in range(steps):
            for step in range(step_count):
                self.set_pins_state(sequence[step])
                time.sleep(step_delay)

    def set_pins_state(self, state):
        for key, pins in self.lines.items():
            for pin, value in zip(pins, state):
                pin.set_value(value)

    def cleanup(self):
        for key, pins in self.lines.items():
            for pin in pins:
                pin.reconfigure(type=gpiod.LINE_REQ_DIR_OUT, default_vals=0)
        self.chip.close()

