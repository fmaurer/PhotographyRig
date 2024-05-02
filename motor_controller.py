# Assuming the MotorDriver class is defined in a file named motor_driver.py

from motor_driver import MotorDriver

class MotorController:
    def __init__(self, driver: MotorDriver):
        self.driver = driver

    def step_to_angle(self, degrees):
        self.driver.step_to_angle(degrees)
