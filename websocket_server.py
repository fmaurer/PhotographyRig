import websockets
import asyncio
import subprocess
import os

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2):
        self.motor_controller = motor_controller
        # Start the video stream
        self.capture_process = self.start_stream()
        print("Video stream started")
        self.start_server()
        print("Websocket server started")

    async def handler(self, websocket):
        async for message in websocket:
            # Parse the message and call the appropriate motor controller method
            command, argument = message.split()
            argument = int(argument)

            if command == "pan":
                self.motor_controller.step_to_angle(argument) #degrees
            elif command == "tilt":
                self.motor_controller.step_to_angle(argument) #degrees
            elif command =="capture":
                self.trigger_camera(argument) #cam index


    def start_server(self):
        start_server = websockets.serve(self.handler, "0.0.0.0", 8765)

        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
    
    def trigger_camera(self, idx):
        self.stop_stream(self.capture_process)

        self.capture_process = self.capture_photo(idx)

        self.capture_process.wait()

        self.capture_process = self.start_stream()

    def start_stream(self):
        stream_command = './mediamtx'
        mediamtx_dir = os.path.expanduser('~/Downloads')
        return subprocess.Popen(stream_command, cwd=mediamtx_dir, shell=False)

    # Function to stop the video stream
    def stop_stream(self, process):
        process.terminate()
        process.wait()

    # Function to capture a photo
    def capture_photo(self,cam):
        photo_command = 'rpicam-still -o ./captures/test_'+str(cam)+'.jpg --immediate --nopreview --camera '+str(cam)
        return subprocess.Popen(photo_command, shell=True)
        #subprocess.run(photo_command, shell=True)

