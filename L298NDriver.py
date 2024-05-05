import time
import gpiod
from motor_driver import MotorDriver

class L298NDriver(MotorDriver):
    def __init__(self, motor_pins):
        # motor_pins should be a dictionary like {'A': [17, 18], 'B': [27, 22]}
        self.motor_pins = motor_pins
        self.chip = gpiod.Chip('gpiochip0')
        self.lines = self.setup_pins()

    def setup_pins(self):
        # Consolidate all pin numbers into a single list for batch request
        all_pins = [pin for sublist in self.motor_pins.values() for pin in sublist]
        lines = self.chip.get_lines(all_pins)  # This should be the correct way to get multiple lines
        lines.request(consumer='motor', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0]*len(all_pins))
        return lines

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
        offsets = [self.motor_pins['A'][0], self.motor_pins['A'][1], self.motor_pins['B'][0], self.motor_pins['B'][1]]
        for i, value in enumerate(state):
            self.lines.get_line(offsets[i]).set_value(value)

    def cleanup(self):
        self.lines.release()

