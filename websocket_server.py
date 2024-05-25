import websockets
import asyncio
import subprocess

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2):
        self.motor_controller = motor_controller

    async def handler(self, websocket):
        async for message in websocket:
            # Parse the message and call the appropriate motor controller method
            command, argument = message.split()
            argument = int(argument)

            if command == "pan":
                self.motor_controller.pan(argument) #degrees
            elif command == "tilt":
                self.motor_controller.tilt(argument) #degrees
            elif command =="capture":
                self.trigger_camera(argument) #cam index


    def start(self):
        start_server = websockets.serve(self.handler, "0.0.0.0", 8765)

        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
    
    def trigger_camera(self, idx):
        # Assuming cam_index corresponds to /dev/v4l-subdev<index>
        # List autocompleteing '/dev/v4l-' using double tab
        device = f"/dev/v4l-subdev5" #subdev5 is zoom lens, wide angle is subdev2
        command = ["v4l2-ctl", "-d", device, "--set-fmt-video=pixelformat=JPEG,width=1280,height=720", "--stream-mmap=3", "--stream-to=output.jpg", "--stream-count=1"]
        subprocess.run(command)

