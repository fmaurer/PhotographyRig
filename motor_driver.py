class MotorDriver:

    def step_to_angle(self, degrees):
        raise NotImplementedError
    
    def step_to(self, number):
        raise NotImplementedError

    def stop_movement(self):
        """Stop any ongoing movement - to be implemented by subclasses"""
        raise NotImplementedError
