import websockets
import asyncio
import subprocess
import os
import datetime

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2):
        self.motor_controller = motor_controller
        self.file_server = self.start_file_server()
        print("File server started")
        # Start the video stream
        self.videostream_process = self.start_stream()
        self.capture_process = None
        print("Video stream started")
        self.start_server()
        print("Websocket server started")
        self.SYMLINK_PATH = './captures/cam0_last.jpg'

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
        #self.stop_stream(self.capture_process)

        self.capture_process = self.capture_photo(idx)

        self.capture_process.wait()

        print("Photo saved to disk, process completed")

        #self.capture_process = self.start_stream()

    def start_stream(self):
        #stream_command = './mediamtx'
        #return subprocess.Popen(stream_command, shell=False)
        stream_command = './mediamtx'
        mediamtx_dir = os.path.expanduser('./mediamtx')
        return subprocess.Popen(stream_command, cwd=mediamtx_dir, shell=False)

    # Function to stop the video stream
    def stop_stream(self, process):
        process.terminate()
        process.wait()

    # Function to capture a photo
    def capture_photo(self,cam):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = f'./captures/cam{cam}_{timestamp}.jpg'
        photo_command = f'rpicam-still -o {file_path} --immediate --nopreview --camera {cam}'
        self.create_symlink(file_path)
        return subprocess.Popen(photo_command, shell=True)

    def create_symlink(self, file_path):
        if os.path.islink(self.SYMLINK_PATH):
            os.unlink(self.SYMLINK_PATH)
        os.symlink(file_path, self.SYMLINK_PATH)
    
    def start_file_server(self):
        server_command = 'python -m http.server --directory ./captures' #defaults to port 8000
        return subprocess.Popen(server_command, shell=True)

