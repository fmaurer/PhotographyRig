import websockets
import asyncio

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2):
        self.motor_controller = motor_controller

    async def handler(self, websocket, path):
        async for message in websocket:
            # Parse the message and call the appropriate motor controller method
            command, degrees = message.split()
            degrees = int(degrees)

            if command == "pan":
                self.motor_controller.pan(degrees)
            elif command == "tilt":
                self.motor_controller.tilt(degrees)

    def start(self):
        start_server = websockets.serve(self.handler, "0.0.0.0", 8765)

        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
