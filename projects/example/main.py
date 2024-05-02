from websocket_server import WebSocketServer
from motor_controller import MotorController
from astro_controller import AstroController

def main():
    # Initialize the motor controller
    motor_controller = MotorController()

    # Initialize the astro controller
    astro_controller = AstroController(motor_controller)

    # Point the camera at the moon
    astro_controller.point_at_moon()

    # Initialize the WebSocket server
    websocket_server = WebSocketServer(motor_controller)

    # Start the WebSocket server
    websocket_server.start()

if __name__ == "__main__":
    main()
