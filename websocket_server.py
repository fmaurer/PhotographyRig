import websockets
import asyncio
import subprocess

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
                self.motor_controller.pan(argument) #degrees
            elif command == "tilt":
                self.motor_controller.tilt(argument) #degrees
            elif command =="capture":
                self.trigger_camera(argument) #cam index


    def start_server(self):
        start_server = websockets.serve(self.handler, "0.0.0.0", 8765)

        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
    
    def trigger_camera(self, idx):
        self.stop_stream(self.capture_process)

        self.capture_process = self.capture_photo()

        self.capture_process.wait()

        self.capture_process = self.start_stream()

    def start_stream(self):
        stream_command = (
            'rpicam-vid -t 0 --camera 1 --nopreview --codec yuv420 --width 1280 --height 720 --inline --listen -o - | '
            'ffmpeg -f rawvideo -pix_fmt yuv420p -s:v 1280x720 -i /dev/stdin -c:v libx264 -preset ultrafast -tune zerolatency -f rtsp rtsp://localhost:$RTSP_PORT/$MTX_PATH'
        ).format(rtsp_port=8889, mtx_path="cam1")
        return subprocess.Popen(stream_command, shell=True)

    # Function to stop the video stream
    def stop_stream(self, process):
        process.terminate()
        process.wait()

    # Function to capture a photo
    def capture_photo(self):
        photo_command = 'raspistill -o test_image.jpg -w 1920 -h 1080'
        return subprocess.Popen(photo_command, shell=True)
        #subprocess.run(photo_command, shell=True)

