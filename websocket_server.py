import websockets
import asyncio
import subprocess
import os
import datetime
import yaml
import re
import time
import socket
import signal
import sys

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2):
        self.SYMLINK_PATH = '/home/frodo/PhotographyRig/captures/'
        self.motor_controller = motor_controller
        self.motor_controller_tilt = motor_controller2
        self.file_server = self.start_file_server()
        print("File server started")
        
        # Initialize single video stream process
        self.videostream_process = None
        
        # Set up signal handlers
        signal.signal(signal.SIGTERM, self.handle_shutdown)
        signal.signal(signal.SIGINT, self.handle_shutdown)
        
        # Start the stream
        self.setup_stream_configs()
        self.videostream_process = self.start_stream()
        print("Video streams started")
        
        self.capture_process = None
        self.start_server()
        print("Websocket server started")
        self.last_photo_path = ''
        self.last_symlink_path = ''

    def setup_stream_configs(self):
        """Create a single config file for both cameras"""
        mediamtx_dir = os.path.expanduser('./mediamtx')
        config_path = os.path.join(mediamtx_dir, 'mediamtx.yml')
        
        # Get the RPi's local IP address
        def get_local_ip():
            try:
                # This creates a UDP socket and tries to connect to a public IP
                # It won't actually send any data but gets the local IP the system would use
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
                s.close()
                return local_ip
            except:
                return None

        local_ip = get_local_ip()
        print(f"Detected local IP: {local_ip}")
        
        # Create or update the configuration
        data = {
            'logLevel': 'info',
            'logDestinations': ['stdout'],
            'readTimeout': '30s',
            'writeTimeout': '30s',
            'rtsp': True,
            'rtspAddress': ':8554',
            'rtspTransports': ['tcp'],
            'hls': False,
            'api': False,
            'metrics': False,
            
            # WebRTC configuration
            'webrtc': True,
            'webrtcAddress': '0.0.0.0:8889',  # Listen on all interfaces
            
            # Enable both UDP and TCP for WebRTC
            'webrtcLocalUDPAddress': '0.0.0.0:8189',
            'webrtcLocalTCPAddress': '0.0.0.0:8189',
            
            # Add STUN servers
            'webrtcICEServers2': [
                {'url': 'stun:stun.l.google.com:19302'},
                {'url': 'stun:stun1.l.google.com:19302'}
            ],
            
            # Add all possible addresses that clients might use
            'webrtcAdditionalHosts': [
                'frodo.local',
                'localhost',
                '127.0.0.1',
                local_ip  # Add the RPi's local IP
            ] if local_ip else ['frodo.local', 'localhost', '127.0.0.1'],
            
            # Get IPs from network interfaces
            'webrtcIPsFromInterfaces': True,
            
            'paths': {
                'cam0': {
                    'source': 'publisher',
                    'sourceProtocol': 'tcp',
                    'runOnInit': 'bash -c \'rpicam-vid -t 0 --camera 0 --nopreview '
                               '--codec yuv420 --width 1280 --height 720 --inline '
                               '--listen --shutter 100000 --level 3.1 -o - | '
                               'ffmpeg -f rawvideo -pix_fmt yuv420p -s:v 1280x720 '
                               '-i /dev/stdin -c:v libx264 -preset ultrafast '
                               '-tune zerolatency -profile:v baseline '
                               '-b:v 1M -maxrate 1M -bufsize 500k '
                               '-g 30 -keyint_min 30 '
                               '-rtsp_transport tcp -f rtsp rtsp://localhost:8554/cam0\'',
                    'runOnInitRestart': True
                },
                'cam1': {
                    'source': 'publisher',
                    'sourceProtocol': 'tcp',
                    'runOnInit': 'bash -c \'rpicam-vid -t 0 --camera 1 --nopreview '
                               '--codec yuv420 --width 1280 --height 720 --inline '
                               '--listen --shutter 100000 --level 3.1 -o - | '
                               'ffmpeg -f rawvideo -pix_fmt yuv420p -s:v 1280x720 '
                               '-i /dev/stdin -c:v libx264 -preset ultrafast '
                               '-tune zerolatency -profile:v baseline '
                               '-b:v 1M -maxrate 1M -bufsize 500k '
                               '-g 30 -keyint_min 30 '
                               '-rtsp_transport tcp -f rtsp rtsp://localhost:8554/cam1\'',
                    'runOnInitRestart': True
                }
            }
        }
        
        # Write the configuration
        with open(config_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
            
        print("MediaMTX configuration updated with network-wide WebRTC access")

    def start_stream(self, camera_index=None):
        """Start a single MediaMTX instance for both cameras"""
        try:
            mediamtx_dir = os.path.expanduser('./mediamtx')
            config_file = 'mediamtx.yml'
            stream_command = ['./mediamtx', config_file]
            print("Starting MediaMTX server...")
            return subprocess.Popen(stream_command, cwd=mediamtx_dir, shell=False)
        except Exception as e:
            print(f"Error starting MediaMTX server: {str(e)}")
            return None

    def stop_stream(self, camera_index=None):
        """Stop the MediaMTX server"""
        if self.videostream_process:
            print("Stopping MediaMTX server...")
            try:
                # First kill any existing rpicam-vid and ffmpeg processes
                # Use SIGTERM first for graceful shutdown
                subprocess.run("pkill -TERM -f rpicam-vid", shell=True)
                subprocess.run("pkill -TERM -f ffmpeg", shell=True)
                
                # Small delay to let processes terminate gracefully
                time.sleep(1)
                
                # Now terminate MediaMTX
                self.videostream_process.terminate()
                
                try:
                    # Wait for MediaMTX to terminate
                    self.videostream_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print("MediaMTX server did not terminate gracefully, forcing stop...")
                    # If processes are still running, force kill them
                    subprocess.run("pkill -9 -f rpicam-vid", shell=True)
                    subprocess.run("pkill -9 -f ffmpeg", shell=True)
                    self.videostream_process.kill()
                    self.videostream_process.wait()
                
                # Final check to ensure all processes are cleaned up
                subprocess.run("pkill -0 -f rpicam-vid || true", shell=True)
                subprocess.run("pkill -0 -f ffmpeg || true", shell=True)
                
                print("All stream processes stopped")
                
            except Exception as e:
                print(f"Error stopping stream processes: {str(e)}")
                # Force kill as last resort
                try:
                    subprocess.run("pkill -9 -f rpicam-vid", shell=True)
                    subprocess.run("pkill -9 -f ffmpeg", shell=True)
                    self.videostream_process.kill()
                except:
                    pass
            finally:
                self.videostream_process = None
                # Small delay before allowing new streams
                time.sleep(1)

    def set_camera(self, argument):
        """Select which camera to modify settings for"""
        print(f"Selecting camera: {argument}")
        self.stop_stream()
        mediamtx_dir = os.path.expanduser('./mediamtx')
        config_path = os.path.join(mediamtx_dir, 'mediamtx.yml')
        self.update_camera_value(config_path, argument)
        self.videostream_process = self.start_stream()

    def set_shutter(self, argument):
        """Set shutter speed for both cameras"""
        print(f"Streaming camera shutter set to: {argument}")
        
        # Update both camera configs
        self.stop_stream()
        self.update_shutter_value(f'./mediamtx/mediamtx.yml', argument)
        self.videostream_process = self.start_stream()

    async def handler(self, websocket):
        async for message in websocket:
            # Parse the message and call the appropriate motor controller method
            command, argument = message.split()
            argument = int(argument)

            if command == "pan":
                self.motor_controller.step_to(argument) #degrees
            elif command == "tilt":
                self.motor_controller_tilt.step_to(argument) #degrees
            elif command == "capture":
                self.trigger_camera(argument) #cam index
            elif command == "shutter":
                # Legacy command - sets shutter for both cameras
                print("Warning: Using legacy shutter command. Consider using shutter0/shutter1 for specific cameras")
                self.set_shutter_both(argument)
            elif command == "shutter0":
                self.set_shutter_single(0, argument)
            elif command == "shutter1":
                self.set_shutter_single(1, argument)
            elif command == "exposeinside":
                # Set indoor exposure (longer shutter) for specific camera
                self.set_shutter_single(argument, 100000)  # 100ms shutter for indoor
            elif command == "exposeoutdoor":
                # Set outdoor exposure (shorter shutter) for specific camera
                self.set_shutter_single(argument, 10000)   # 10ms shutter for outdoor
            elif command == "camera":
                self.set_camera(argument) #select camera index to stream from
            elif command == "ramping":
                if argument == 1:
                    self.motor_controller.enable_ramping()
                    self.motor_controller_tilt.enable_ramping()
                else:
                    self.motor_controller.disable_ramping()
                    self.motor_controller_tilt.disable_ramping()
            elif command == "ramp_accel":
                # Argument is percentage * 100 (e.g., 30 for 0.3)
                accel_percent = argument / 100.0
                self.motor_controller.set_ramp_profile(accel_percent=accel_percent)
                self.motor_controller_tilt.set_ramp_profile(accel_percent=accel_percent)
            elif command == "ramp_steps":
                # Argument is direct number of steps
                self.motor_controller.set_ramp_profile(max_accel_steps=argument)
                self.motor_controller_tilt.set_ramp_profile(max_accel_steps=argument)

    def start_server(self):
        start_server = websockets.serve(self.handler, "0.0.0.0", 8765)

        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
    
    def trigger_camera(self, idx):
        self.capture_process = self.capture_photo(idx)
        self.capture_process.wait()
        
        # Create symlink to the new photo
        os.symlink(os.path.basename(self.last_photo_path), self.last_symlink_path)

        print("Photo saved to disk, process completed")
    
    def update_shutter_value(self, yml_file_path, new_shutter_value):
        # Load the YAML file
        with open(yml_file_path, 'r') as file:
            data = yaml.safe_load(file)

        # Update the `--shutter` value while preserving other settings
        for path_key, path_value in data.get('paths', {}).items():
            if isinstance(path_value, dict) and 'runOnInit' in path_value:
                command = path_value['runOnInit']
                updated_command = re.sub(r'--shutter\s+\d+', f'--shutter {new_shutter_value}', command)
                path_value['runOnInit'] = updated_command

        # Ensure WebRTC global settings are preserved
        data['webrtc'] = True
        data['webrtcAddress'] = ':8889'

        # Write the updated YAML back to the file
        with open(yml_file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
    
    def update_camera_value(self, yml_file_path, new_camera_value):
        # Load the YAML file
        with open(yml_file_path, 'r') as file:
            data = yaml.safe_load(file)

        # Update the `--camera` value while preserving other settings
        for path_key, path_value in data.get('paths', {}).items():
            if isinstance(path_value, dict) and 'runOnInit' in path_value:
                command = path_value['runOnInit']
                updated_command = re.sub(r'--camera\s+\d+', f'--camera {new_camera_value}', command)
                path_value['runOnInit'] = updated_command

        # Ensure WebRTC global settings are preserved
        data['webrtc'] = True
        data['webrtcAddress'] = ':8889'

        # Write the updated YAML back to the file
        with open(yml_file_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)    

    def capture_photo(self, cam):
        symlink_path = f'./captures/cam{cam}_last.jpg'

        #Remove the previous symlink if it exists
        if os.path.islink(symlink_path):
             os.remove(symlink_path)

        # Capture new photo with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f'cam{cam}_{timestamp}.jpg'
        file_path = f'./captures/{file_name}'
        photo_command = f'rpicam-still -o {file_path} --immediate --nopreview --camera {cam}'
        
        self.last_photo_path = file_path
        self.last_symlink_path = symlink_path

        # Execute the command and wait for it to complete
        return subprocess.Popen(photo_command, shell=True)

        

    def create_symlink(self, file_path):
        #if os.path.islink(self.SYMLINK_PATH):
        #    os.unlink(self.SYMLINK_PATH)
        os.symlink(self.SYMLINK_PATH+file_path, self.SYMLINK_PATH+'cam0_last.jpg')
    
    def start_file_server(self):
        """Start file server on next available port starting from 8000"""
        port = 8000
        max_attempts = 10
        
        for port_attempt in range(port, port + max_attempts):
            try:
                server_command = f'python -m http.server --directory ./captures {port_attempt}'
                process = subprocess.Popen(server_command, shell=True)
                print(f"File server started on port {port_attempt}")
                return process
            except Exception as e:
                print(f"Failed to start file server on port {port_attempt}: {e}")
                if port_attempt == port + max_attempts - 1:
                    print("Failed to find available port for file server")
                    return None
        return None

    def set_shutter_single(self, camera_index, shutter_value):
        """Set shutter speed for a specific camera"""
        print(f"Setting camera {camera_index} shutter to: {shutter_value}")
        try:
            # Stop all streams first
            self.stop_stream()
            
            # Update the configuration
            mediamtx_dir = os.path.expanduser('./mediamtx')
            config_path = os.path.join(mediamtx_dir, 'mediamtx.yml')
            
            # Load existing config
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
            
            # Update only the specific camera's shutter value
            camera_path = f'cam{camera_index}'
            if camera_path in data['paths']:
                command = data['paths'][camera_path]['runOnInit']
                updated_command = re.sub(r'--shutter\s+\d+', f'--shutter {shutter_value}', command)
                data['paths'][camera_path]['runOnInit'] = updated_command
                
                # Write the updated config
                with open(config_path, 'w') as file:
                    yaml.dump(data, file, default_flow_style=False)
            
            # Restart the stream
            print(f"Restarting MediaMTX server...")
            self.videostream_process = self.start_stream()
            print("MediaMTX server restarted")
            
        except Exception as e:
            print(f"Error setting shutter for camera {camera_index}: {str(e)}")
            # Try to restart the stream even if there was an error
            try:
                self.videostream_process = self.start_stream()
            except:
                pass

    def set_shutter_both(self, shutter_value):
        """Set shutter speed for both cameras (legacy support)"""
        print(f"Setting both cameras shutter to: {shutter_value}")
        self.set_shutter_single(0, shutter_value)
        self.set_shutter_single(1, shutter_value)

    def handle_shutdown(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print("\nShutdown signal received, cleaning up...")
        self.stop_stream()
        if self.file_server:
            self.file_server.terminate()
        sys.exit(0)

