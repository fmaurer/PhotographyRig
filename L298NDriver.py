# L298NDriver.py
import RPi.GPIO as GPIO
import time
from motor_driver import MotorDriver

class L298NDriver(MotorDriver):
    def __init__(self, motor_pins):
        # motor_pins should be a dictionary like {'A': [17, 18], 'B': [27, 22]}
        self.motor_pins = motor_pins
        self.setup_pins()

    def setup_pins(self):
        GPIO.setmode(GPIO.BCM)
        for pins in self.motor_pins.values():
            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
    
    def step_to_angle(self, degrees):
        #TODO: implement drive stepper driver to reach the full assembly's final degree of pan/tilt motors
        self.step_motor(200, 0.02)
        return 0

    def step_motor(self, steps, step_delay):
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
        GPIO.output(self.motor_pins['A'][0], state[0])
        GPIO.output(self.motor_pins['A'][1], state[1])
        GPIO.output(self.motor_pins['B'][0], state[2])
        GPIO.output(self.motor_pins['B'][1], state[3])

    def cleanup(self):
        for pins in self.motor_pins.values():
            for pin in pins:
                GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
