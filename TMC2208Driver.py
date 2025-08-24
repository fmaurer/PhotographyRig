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
        
        # Movement control
        self.should_stop = False
        self.is_moving = False
        
        # Acceleration parameters
        self.min_delay = step_delay  # Fastest speed (smallest delay)
        self.max_delay = step_delay * 4  # Starting speed (largest delay)
        self.acceleration = 1.000001  # Acceleration multiplier (adjust this to change acceleration rate)
        self.deceleration = 0.999999  # Deceleration multiplier (adjust this to change deceleration rate)
        
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

    def stop_movement(self):
        """Stop any ongoing movement"""
        self.should_stop = True
        # Wait briefly for the movement to stop
        while self.is_moving:
            time.sleep(0.001)  # 1ms delay
        self.should_stop = False

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

    def set_acceleration_rate(self, acceleration_multiplier):
        """
        Set the acceleration rate multiplier
        :param acceleration_multiplier: Value > 1.0 for slower acceleration, < 1.0 for faster
        Examples:
        - 1.005: Very slow acceleration
        - 1.015: Current default (moderate)
        - 1.05: Fast acceleration
        """
        if acceleration_multiplier <= 1.0:
            raise ValueError("Acceleration multiplier must be greater than 1.0")
        self.acceleration = acceleration_multiplier
        print(f"Acceleration rate set to: {acceleration_multiplier}")

    def set_deceleration_rate(self, deceleration_multiplier):
        """
        Set the deceleration rate multiplier
        :param deceleration_multiplier: Value < 1.0 for slower deceleration, > 1.0 for faster
        Examples:
        - 0.995: Very slow deceleration
        - 0.985: Current default (moderate)
        - 0.95: Fast deceleration
        """
        if deceleration_multiplier >= 1.0:
            raise ValueError("Deceleration multiplier must be less than 1.0")
        self.deceleration = deceleration_multiplier
        print(f"Deceleration rate set to: {deceleration_multiplier}")

    def set_speed_range(self, min_delay=None, max_delay=None):
        """
        Set the speed range for acceleration/deceleration
        :param min_delay: Fastest speed (smallest delay)
        :param max_delay: Slowest speed (largest delay)
        """
        if min_delay is not None:
            self.min_delay = min_delay
        if max_delay is not None:
            self.max_delay = max_delay
        print(f"Speed range: min_delay={self.min_delay}, max_delay={self.max_delay}")

    def set_max_speed(self, max_speed_delay):
        """
        Set the maximum speed (minimum delay) for the motor
        :param max_speed_delay: Delay in seconds for maximum speed (smaller = faster)
        """
        if max_speed_delay <= 0:
            raise ValueError("Max speed delay must be greater than 0")
        self.min_delay = max_speed_delay
        print(f"Max speed set to: {max_speed_delay} seconds delay (faster = smaller delay)")

    def set_gentle_acceleration(self):
        """Set very gentle acceleration and deceleration"""
        self.acceleration = 1.005  # Very slow acceleration
        self.deceleration = 0.995  # Very slow deceleration
        self.ramp_settings['accel_percent'] = 0.4  # Use more steps for acceleration
        self.ramp_settings['max_accel_steps'] = 100  # Allow more acceleration steps
        print("Gentle acceleration profile enabled")

    def set_moderate_acceleration(self):
        """Set moderate acceleration and deceleration (current default)"""
        self.acceleration = 1.000001
        self.deceleration = 0.999999
        self.ramp_settings['accel_percent'] = 0.3
        self.ramp_settings['max_accel_steps'] = 50
        print("Moderate acceleration profile enabled")

    def set_aggressive_acceleration(self):
        """Set aggressive acceleration and deceleration"""
        self.acceleration = 1.05
        self.deceleration = 0.95
        self.ramp_settings['accel_percent'] = 0.2
        self.ramp_settings['max_accel_steps'] = 25
        print("Aggressive acceleration profile enabled")

    def step_motor(self, steps, step_delay):
        self.enable_device.value = False
        self.is_moving = True
        self.should_stop = False
        
        if steps == 0:
            self.is_moving = False
            return
            
        try:
            if self.ramping_enabled:
                self._step_motor_with_ramping(steps)
            else:
                self._step_motor_constant_speed(steps)
        finally:
            self.is_moving = False
            self.enable_device.value = True
        
    def _step_motor_constant_speed(self, steps):
        """Execute steps at constant speed"""
        print(f"Moving stepper {steps} steps at constant speed")
        for i in range(steps):
            if self.should_stop:
                print("Movement stopped by stop command")
                return
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
            if self.should_stop:
                print("Movement stopped by stop command during acceleration")
                return
            self._do_step(current_delay)
            current_delay = max(self.min_delay, current_delay * self.acceleration)
            
        # Constant speed phase
        for i in range(const_steps):
            if self.should_stop:
                print("Movement stopped by stop command during constant speed")
                return
            self._do_step(self.min_delay)
            
        # Deceleration phase
        for i in range(decel_steps):
            if self.should_stop:
                print("Movement stopped by stop command during deceleration")
                return
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
