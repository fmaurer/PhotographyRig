import time
import math
from gpiozero import OutputDevice
from motor_driver import MotorDriver

class TMC2208Driver(MotorDriver):
    def __init__(self, step_pin, dir_pin, enable_pin, step_delay):
        # step_pin and dir_pin are the GPIO pins used for step and direction control
        self.step_device = OutputDevice(step_pin)
        self.dir_device = OutputDevice(dir_pin)
        self.enable_device = OutputDevice(enable_pin)
        self.base_delay = step_delay
        
        # Acceleration parameters
        self.min_delay = step_delay  # Fastest speed (smallest delay)
        self.max_delay = step_delay * 4  # Starting speed (largest delay)
        self.acceleration = 1.015  # Acceleration multiplier (adjust this to change acceleration rate)
        self.deceleration = 0.985  # Deceleration multiplier (adjust this to change deceleration rate)
        
        # Ramping control
        self.ramping_enabled = True  # Default to enabled
        self.ramp_settings = {
            'accel_percent': 0.3,  # Percentage of total steps for acceleration
            'max_accel_steps': 50,  # Maximum steps for acceleration/deceleration
        }

    def num_steps_for_angle(self, degrees):
        stepper_step_size = 1.8  # degrees
        driver_gear_teeth = 12
        bevel_gear_teeth = 12
        ring_gear_teeth = 48

        stage_0 = bevel_gear_teeth / driver_gear_teeth
        stage_1 = ring_gear_teeth / bevel_gear_teeth

        reduction = stage_1 / stage_0

        return round(degrees / (stepper_step_size / reduction))

    def step_to_angle(self, degrees):
        steps = self.num_steps_for_angle(degrees)
        self.step_motor(steps, 0.0002)  # 200 microseconds delay between steps .005 for tilt
        self.dir_device.value = degrees > 0
        #self.step_motor(16, 0.005)
        return 0
    
    def step_to(self, number):
        self.dir_device.value = number > 0
        self.step_motor(abs(number), self.base_delay) 

    def enable_ramping(self):
        """Enable acceleration ramping"""
        self.ramping_enabled = True
        print("Acceleration ramping enabled")

    def disable_ramping(self):
        """Disable acceleration ramping"""
        self.ramping_enabled = False
        print("Acceleration ramping disabled")

    def set_ramp_profile(self, accel_percent=None, max_accel_steps=None):
        """
        Configure ramping profile parameters
        :param accel_percent: Percentage of total steps to use for acceleration (0.0 to 1.0)
        :param max_accel_steps: Maximum number of steps to use for acceleration
        """
        if accel_percent is not None:
            if 0.0 <= accel_percent <= 1.0:
                self.ramp_settings['accel_percent'] = accel_percent
            else:
                raise ValueError("accel_percent must be between 0.0 and 1.0")
                
        if max_accel_steps is not None:
            if max_accel_steps > 0:
                self.ramp_settings['max_accel_steps'] = max_accel_steps
            else:
                raise ValueError("max_accel_steps must be greater than 0")

    def step_motor(self, steps, step_delay):
        self.enable_device.value = False
        
        if steps == 0:
            return
            
        if self.ramping_enabled:
            self._step_motor_with_ramping(steps)
        else:
            self._step_motor_constant_speed(steps)
            
        self.enable_device.value = True
        
    def _step_motor_constant_speed(self, steps):
        """Execute steps at constant speed"""
        print(f"Moving stepper {steps} steps at constant speed")
        for _ in range(steps):
            self._do_step(self.base_delay)
            
    def _step_motor_with_ramping(self, steps):
        """Execute steps with acceleration ramping"""
        print(f"Moving stepper {steps} steps with acceleration profile")
        
        # Calculate acceleration and deceleration points
        accel_steps = min(int(steps * self.ramp_settings['accel_percent']), 
                         self.ramp_settings['max_accel_steps'])
        decel_steps = accel_steps
        const_steps = steps - (accel_steps + decel_steps)
        
        current_delay = self.max_delay
        
        # Acceleration phase
        for i in range(accel_steps):
            self._do_step(current_delay)
            current_delay = max(self.min_delay, current_delay * self.acceleration)
            
        # Constant speed phase
        for i in range(const_steps):
            self._do_step(self.min_delay)
            
        # Deceleration phase
        for i in range(decel_steps):
            self._do_step(current_delay)
            current_delay = min(self.max_delay, current_delay * self.deceleration)

    def _do_step(self, delay):
        """Execute a single step with the specified delay"""
        self.step_device.value = True
        time.sleep(delay)
        self.step_device.value = False
        time.sleep(delay)

    def set_direction(self, clockwise=True):
        if clockwise:
            self.dir_device.on()  # Set the DIR pin high for clockwise
        else:
            self.dir_device.off()  # Set the DIR pin low for counterclockwise

    def cleanup(self):
        # This function resets all GPIO pins to low when they are no longer needed
        self.step_device.off()
        self.dir_device.off()
