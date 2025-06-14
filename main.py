#from L298NDriver import L298NDriver
from TMC2208Driver import TMC2208Driver
from motor_controller import MotorController
from astro_controller import AstroController
from websocket_server import WebSocketServer
from astropy.coordinates import EarthLocation
from astropy import units as u
import threading
import http.server
import socketserver

def start_http_server(port=8000):
    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", port), handler) as httpd:
        print(f"HTTP server started at port {port}")
        httpd.serve_forever()

def main():
    # Define pin configurations for pan and tilt
    pan_pins = [17, 18, 27, 22]
    tilt_pins = [23, 24, 25, 4]

    #motor_pins_1 = [17, 18, 27, 22]  # Define pins for two inputs per motor channel
    #motor_pins_2 = [23, 24, 25, 4] # Define pins for two inputs per motor channel
    
    # Define TMC2208 pin configuration
    STEP_PIN = 17
    DIR_PIN = 27
    ENABLE_PIN = 22
    
    STEP_PIN_TILT = 23
    DIR_PIN_TILT = 24
    ENABLE_PIN_TILT = 25

    # Initialize the motor driver, swap out for new ones in future
 #   l298n_driver_1 = L298NDriver(pan_pins)
 #   l298n_driver_2 = L298NDriver(tilt_pins)
    tmc2208_driver = TMC2208Driver(STEP_PIN, DIR_PIN, ENABLE_PIN, 0.0004)
    tmc2208_driver_2 = TMC2208Driver(STEP_PIN_TILT, DIR_PIN_TILT, ENABLE_PIN_TILT, 0.0001)

    # Initialize the motor controller with L298N driver
    pan_motor_controller = MotorController(tmc2208_driver)
    tilt_motor_controller = MotorController(tmc2208_driver_2)

    # Initialize the astro controller
    location = EarthLocation(lat=37.3855*u.deg, lon=-118.5819*u.deg, height=2402*u.m) #Mammoth Lakes
    astro_controller = AstroController(pan_motor_controller, tilt_motor_controller, location)

    # Point the camera at the moon
    #astro_controller.point_at_moon()

    # Start HTTP server in a separate thread
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()
    print("Started HTTP server thread")

    # Initialize the WebSocket server
    #TODO: figure out how to trigger photos
    websocket_server = WebSocketServer(pan_motor_controller, tilt_motor_controller)

    try:
        # Start the WebSocket server (which will start its own file server)
        websocket_server.start()
    finally:
        # Cleanup on program exit
        #l298n_driver_1.cleanup()
        #l298n_driver_2.cleanup()
        tmc2208_driver.cleanup()
        tmc2208_driver_2.cleanup()

if __name__ == '__main__':
    main()
