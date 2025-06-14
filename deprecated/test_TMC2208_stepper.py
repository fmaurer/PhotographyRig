import pigpio
import time

# Pin Definitions
STEP_PIN = 17
DIR_PIN = 27
ENABLE_PIN = 22

# Initialize pigpio
pi = pigpio.pi('soft',8888)

# Set pin modes
pi.set_mode(STEP_PIN, pigpio.OUTPUT)
pi.set_mode(DIR_PIN, pigpio.OUTPUT)
pi.set_mode(ENABLE_PIN, pigpio.OUTPUT)

# Enable motor driver
pi.write(ENABLE_PIN, 0)  # 0 to enable

# Set direction
pi.write(DIR_PIN, 1)  # 1 for clockwise, 0 for counter-clockwise

# Control the motor
def step_motor(steps, delay):
    for _ in range(steps):
        pi.write(STEP_PIN, 1)
        time.sleep(delay)
        pi.write(STEP_PIN, 0)
        time.sleep(delay)

# Example: Step the motor 200 steps with a delay of 1ms between steps
step_motor(200, 0.001)

# Cleanup
pi.stop()
