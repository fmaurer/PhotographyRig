import time
from gpiozero import OutputDevice
from motor_driver import MotorDriver

class L298NDriver(MotorDriver):
    def __init__(self, motor_pins):
        # motor_pins should be a list of pin numbers
        self.motor_devices = [OutputDevice(pin) for pin in motor_pins]

    def step_to_angle(self, degrees):
        #TODO: Implement drive stepper driver to reach the full assembly's final degree of pan/tilt motors
        self.step_motor(200, 0.02)
        return 0

    def step_motor(self, steps, step_delay):
        print("Moving stepper ", str(steps), " with a ", str(step_delay), " delay")
        sequence = [
            (True, False, False, True),  # This sequence should be adjusted based on your motor and wiring
            (False, True, True, False),
            (False, True, False, True),
            (True, False, True, False)
        ]
        for _ in range(steps):
            for state in sequence:
                for device, value in zip(self.motor_devices, state):
                    device.value = value
                time.sleep(step_delay)  # Small delay between steps for visibility

    def cleanup(self):
        # This function resets all GPIO pins to low when they are no longer needed
        for device in self.motor_devices:
            device.off()
