import time
from gpiozero import OutputDevice
from motor_driver import MotorDriver

class L298NDriver(MotorDriver):
    def __init__(self, motor_pins):
        # motor_pins should be a list of pin numbers
        self.motor_devices = [OutputDevice(pin) for pin in motor_pins]

    def num_steps_for_angle(self, degrees):
        stepper_step_size = 1.8 #degrees
        driver_gear_teeth = 12
        bevel_gear_teeth = 12
        ring_gear_teeth = 48

        stage_0 = bevel_gear_teeth / driver_gear_teeth
        stage_1 = ring_gear_teeth / bevel_gear_teeth

        reduction = stage_1/stage_0

        return round(stepper_step_size / reduction)


    def step_to_angle(self, degrees):
        #self.step_motor(self.num_steps_for_angle(degrees), 0.005)
        #-------DANGER ONLY USE ABOVE WHEN SAFETIES AND SHUTOFFS IMPLEMENTED :: HAS NOT BEEN TESTED------:
        self.step_motor(50, 0.005) #200 steps for full revolution
        return 0

    def step_motor(self, steps, step_delay):
        print("Moving stepper ", str(steps), " with a ", str(step_delay), " delay")
        sequence = [
            (False, False, False, True),  # This sequence should be adjusted based on your motor and wiring
            (False, False, True, False),
            (False, True, False, False),
            (True, False, False, False)
        ]
        for i in range(steps):
            state = sequence[i % len(sequence)]  # Cycle through the sequence
            for device, value in zip(self.motor_devices, state):
                device.value = value
            time.sleep(step_delay)  # Small delay between steps for visibility

    def cleanup(self):
        # This function resets all GPIO pins to low when they are no longer needed
        for device in self.motor_devices:
            device.off()
