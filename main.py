import L298NDriver
from motor_controller import MotorController
from astro_controller import AstroController
from websocket_server import WebSocketServer

from astropy.coordinates import EarthLocation
from astropy import units as u

def main():
    # Define pin configurations for pan and tilt
    pan_pins = [17, 18, 27, 22]
    tilt_pins = [23, 24, 25, 4]

    motor_pins_1 = {'A': [17, 18], 'B': [27, 22]}  # Define pins for two inputs per motor channel
    motor_pins_2 = {'A': [23, 24], 'B': [25, 4]}  # Define pins for two inputs per motor channel

    # Initialize the motor driver, swap out for new ones in future
    l298n_driver_1 = L298NDriver(motor_pins_1)
    l298n_driver_2 = L298NDriver(motor_pins_2)

    # Initialize the motor controller with L298N driver
    pan_motor_controller = MotorController(l298n_driver_1)
    tilt_motor_controller = MotorController(l298n_driver_2)

    # Initialize the astro controller
    location = EarthLocation(lat=37.3855*u.deg, lon=-118.5819*u.deg, height=2402*u.m) #Mammoth Lakes
    astro_controller = AstroController(pan_motor_controller, tilt_motor_controller, location)

    # Point the camera at the moon
    astro_controller.point_at_moon()

    # Initialize the WebSocket server
    #TODO: make it work with refactor where independant pan and tilt controllers
    #websocket_server = WebSocketServer(pan_motor_controller, tilt_motor_controller)

    # Start the WebSocket server
    # websocket_server.start()

    # Cleanup on program exit
    l298n_driver_1.cleanup()
    l298n_driver_2.cleanup()

if __name__ == '__main__':
    main()