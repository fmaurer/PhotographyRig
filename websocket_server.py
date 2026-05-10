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
import threading
import json
import math

# IMU imports
try:
    import board
    import busio
    from adafruit_bno08x import (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    IMU_AVAILABLE = True
except ImportError:
    print("IMU libraries not available. IMU functionality will be disabled.")
    IMU_AVAILABLE = False

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2):
        self.SYMLINK_PATH = '/home/frodo/PhotographyRig/captures/'
        self.motor_controller = motor_controller
        self.motor_controller_tilt = motor_controller2
        
        # Add movement tracking to prevent command stacking
        self.current_pan_movement = None
        self.current_tilt_movement = None
        self.movement_lock = threading.Lock()
        
        # IMU setup
        self.imu_data = {
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            'timestamp': 0.0
        }
        self.imu_lock = threading.Lock()
        self.imu_thread = None
        self.imu_running = False
        self.bno = None

        # IMU calibration offsets (loaded from calibration.json)
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.load_imu_calibration()
        
        # Track connected clients for broadcasting
        self.connected_clients = set()
        
        # Initialize IMU
        self.setup_imu()
        
        # Remove redundant file server - main.py already handles this
        # self.file_server = self.start_file_server()
        # print("File server started")
        
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
                               '-rtsp_transport tcp -f rtsp rtsp://localhost:8554/cam0\'',
                    'runOnInitRestart': True
                },
                'cam1': {
                    'source': 'publisher',
                    'sourceProtocol': 'tcp',
                    'runOnInit': 'bash -c \'rpicam-vid -t 0 --camera 1 --nopreview '
                               '--codec yuv420 --width 1280 --height 720 --inline '
                               '--listen --shutter 1000 --level 3.1 --hflip --vflip '
                               '--lens-position 6 --autofocus-mode manual -o - | '
                               'ffmpeg -f rawvideo -pix_fmt yuv420p -s:v 1280x720 '
                               '-i /dev/stdin -c:v libx264 -preset ultrafast '
                               '-tune zerolatency -profile:v baseline '
                               '-b:v 1M -maxrate 1M -bufsize 500k '
                               '-rtsp_transport tcp -f rtsp rtsp://localhost:8554/cam1\'',
                    'runOnInitRestart': True
                }
            }
        }
        
        # Write the configuration
        with open(config_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False)
            
        print("MediaMTX configuration updated with network-wide WebRTC access")

    def setup_imu(self):
        """Initialize the IMU sensor"""
        if not IMU_AVAILABLE:
            print("IMU libraries not available, skipping IMU initialization")
            return

        for attempt in range(3):
            try:
                i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
                self.bno = BNO08X_I2C(i2c)
                time.sleep(1)  # Give BNO08x time to boot before enabling features
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                print("IMU initialized successfully")
                # Start IMU data collection
                self.start_imu_streaming()
                return
            except Exception as e:
                print(f"IMU init attempt {attempt + 1}/3 failed: {e}")
                self.bno = None
                time.sleep(2)

        print("IMU initialization failed after 3 attempts")

    def load_imu_calibration(self):
        """Load IMU calibration offsets from calibration.json"""
        calibration_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.json')
        try:
            with open(calibration_path, 'r') as f:
                cal = json.load(f)
            self.pitch_offset = cal.get('pitch_offset', 0.0)
            self.roll_offset = cal.get('roll_offset', 0.0)
            print(f"IMU calibration loaded: pitch_offset={self.pitch_offset}, roll_offset={self.roll_offset}")
        except FileNotFoundError:
            print("No calibration.json found, using raw IMU values (no offsets applied)")
        except Exception as e:
            print(f"Error loading calibration.json: {e}, using raw IMU values")

    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to roll, pitch, yaw (radians)."""
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # clamp to 90
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_data_collection_loop(self):
        """Background thread for collecting IMU data"""
        while self.imu_running and self.bno:
            try:
                quat_i, quat_j, quat_k, quat_real = self.bno.quaternion  # (x, y, z, w)
                rx, ry, rz = self.quaternion_to_euler(quat_real, quat_i, quat_j, quat_k)
                
                # Update IMU data with thread safety, applying calibration offsets
                with self.imu_lock:
                    self.imu_data = {
                        'pitch': math.degrees(rx) - self.pitch_offset,
                        'roll': math.degrees(ry) - self.roll_offset,
                        'yaw': math.degrees(rz),
                        'timestamp': time.time()
                    }
                
                time.sleep(0.1)  # 10Hz update rate
            except Exception as e:
                print(f"Error reading IMU data: {e}")
                time.sleep(1)

    def start_imu_streaming(self):
        """Start the IMU data collection thread"""
        if self.bno:
            self.imu_running = True
            self.imu_thread = threading.Thread(
                target=self.imu_data_collection_loop,
                daemon=True,
                name="IMU-Data-Collection"
            )
            self.imu_thread.start()
            print("IMU data streaming started")

    def stop_imu_streaming(self):
        """Stop the IMU data collection thread"""
        self.imu_running = False
        if self.imu_thread:
            self.imu_thread.join(timeout=1.0)
            print("IMU data streaming stopped")

    async def send_imu_data(self, websocket):
        """Send current IMU data to a specific client"""
        try:
            with self.imu_lock:
                imu_json = json.dumps({
                    'type': 'imu_data',
                    'data': self.imu_data.copy()
                })
            await websocket.send(imu_json)
        except Exception as e:
            print(f"Error sending IMU data: {e}")

    async def broadcast_imu_data(self):
        """Broadcast IMU data to all connected clients"""
        if not self.connected_clients:
            return
            
        try:
            with self.imu_lock:
                imu_json = json.dumps({
                    'type': 'imu_data',
                    'data': self.imu_data.copy()
                })
            
            # Send to all connected clients
            disconnected_clients = set()
            for client in self.connected_clients:
                try:
                    await client.send(imu_json)
                except websockets.exceptions.ConnectionClosed:
                    disconnected_clients.add(client)
            
            # Clean up disconnected clients
            self.connected_clients -= disconnected_clients
            
        except Exception as e:
            print(f"Error broadcasting IMU data: {e}")

    def start_imu_broadcast(self):
        """Start broadcasting IMU data to all clients"""
        if not hasattr(self, 'imu_broadcast_task') or self.imu_broadcast_task.done():
            self.imu_broadcast_task = asyncio.create_task(self.imu_broadcast_loop())
            print("IMU broadcast started")

    def stop_imu_broadcast(self):
        """Stop broadcasting IMU data"""
        if hasattr(self, 'imu_broadcast_task') and not self.imu_broadcast_task.done():
            self.imu_broadcast_task.cancel()
            print("IMU broadcast stopped")

    async def imu_broadcast_loop(self):
        """Background task for broadcasting IMU data"""
        while True:
            try:
                await self.broadcast_imu_data()
                await asyncio.sleep(0.1)  # 10Hz broadcast rate
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error in IMU broadcast loop: {e}")
                await asyncio.sleep(1)

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
        """Handle incoming WebSocket messages."""
        # Add client to tracking set
        self.connected_clients.add(websocket)
        print(f"Client connected. Total clients: {len(self.connected_clients)}")
        
        try:
            async for message in websocket:
                print(f"Received message: {message}")
                parts = message.split()
                if len(parts) < 1:
                    await websocket.send("Error: Empty message")
                    continue
                    
                command = parts[0]
                
                # Handle IMU commands
                if command == "get_imu":
                    await self.send_imu_data(websocket)
                elif command == "start_imu_stream":
                    self.start_imu_broadcast()
                    await websocket.send("IMU streaming started")
                elif command == "stop_imu_stream":
                    self.stop_imu_broadcast()
                    await websocket.send("IMU streaming stopped")
                elif command == "shutdown":
                    # Handle shutdown command (no parameters needed)
                    await websocket.send("Shutting down server...")
                    print("Shutdown command received from WebSocket client")
                    await self.graceful_shutdown()
                    return  # Exit the handler
                elif command in ['accel_gentle', 'accel_moderate', 'accel_aggressive']:
                    # Handle acceleration commands (no parameters needed)
                    if command == "accel_gentle":
                        self.motor_controller.driver.set_gentle_acceleration()
                        self.motor_controller_tilt.driver.set_gentle_acceleration()
                        await websocket.send("Gentle acceleration enabled for both motors")
                    elif command == "accel_moderate":
                        self.motor_controller.driver.set_moderate_acceleration()
                        self.motor_controller_tilt.driver.set_moderate_acceleration()
                        await websocket.send("Moderate acceleration enabled for both motors")
                    elif command == "accel_aggressive":
                        self.motor_controller.driver.set_aggressive_acceleration()
                        self.motor_controller_tilt.driver.set_aggressive_acceleration()
                        await websocket.send("Aggressive acceleration enabled for both motors")
                elif command == "accel_custom":
                    # Handle custom acceleration command (requires 2 parameters)
                    if len(parts) != 3:
                        await websocket.send("Error: accel_custom requires acceleration and deceleration values")
                        continue
                    accel_rate = float(parts[1])
                    decel_rate = float(parts[2])
                    try:
                        self.motor_controller.driver.set_acceleration_rate(accel_rate)
                        self.motor_controller.driver.set_deceleration_rate(decel_rate)
                        self.motor_controller_tilt.driver.set_acceleration_rate(accel_rate)
                        self.motor_controller_tilt.driver.set_deceleration_rate(decel_rate)
                        await websocket.send(f"Custom acceleration set: accel={accel_rate}, decel={decel_rate}")
                    except ValueError as e:
                        await websocket.send(f"Error setting acceleration: {str(e)}")
                elif command == "max_speed":
                    # Handle max speed command (requires 1 parameter)
                    if len(parts) != 2:
                        await websocket.send("Error: max_speed requires one value (delay in seconds)")
                        continue
                    max_speed_delay = float(parts[1])
                    try:
                        self.motor_controller.driver.set_max_speed(max_speed_delay)
                        self.motor_controller_tilt.driver.set_max_speed(max_speed_delay)
                        await websocket.send(f"Max speed set to: {max_speed_delay} seconds delay")
                    except ValueError as e:
                        await websocket.send(f"Error setting max speed: {str(e)}")
                elif command in ['ev', 'gain', 'aperture']:
                    if len(parts) != 3:
                        await websocket.send(f"Error: {command} command requires value and camera number")
                        continue
                    value = int(parts[1])
                    camera = int(parts[2])
                    if command == 'ev':
                        self.set_exposure_value(camera, value)
                    elif command == 'gain':
                        self.set_gain_value(camera, value)
                    elif command == 'aperture':
                        self.set_aperture_value(camera, value)
                elif command == "move":
                    # Handle move command (requires 2 parameters)
                    if len(parts) != 3:
                        await websocket.send("Error: move command requires pan and tilt values")
                        continue
                    pan_arg = int(parts[1])
                    tilt_arg = int(parts[2])
                    
                    # Cancel any ongoing movements and start new ones
                    await self.cancel_and_start_pan(pan_arg)
                    await self.cancel_and_start_tilt(tilt_arg)
                else:
                    # Handle two-part commands (original format)
                    if len(parts) != 2:
                        await websocket.send("Error: Command requires exactly one argument")
                        continue
                    command, argument = parts
                    argument = int(argument)
                    
                    if command == "pan":
                        # Cancel any ongoing pan movement and start new one
                        await self.cancel_and_start_pan(argument)
                    elif command == "tilt":
                        # Cancel any ongoing tilt movement and start new one
                        await self.cancel_and_start_tilt(argument)
                    elif command == "capture":
                        self.trigger_camera(argument)
                    elif command == "shutter":
                        await self.motor_controller.set_shutter_speed(argument)
                    elif command == "exposeoutdoor":
                        await self.motor_controller.set_exposure_outdoor(argument)
                    elif command == "exposeinside":
                        await self.motor_controller.set_exposure_inside(argument)
                    elif command == "move":  # New combined command
                        if len(parts) != 3:
                            await websocket.send("Error: move command requires pan and tilt values")
                            continue
                        pan_arg = int(parts[1])
                        tilt_arg = int(parts[2])
                        
                        # Cancel any ongoing movements and start new ones
                        await self.cancel_and_start_pan(pan_arg)
                        await self.cancel_and_start_tilt(tilt_arg)
                    else:
                        await websocket.send(f"Unknown command: {command}")
        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected")
        except Exception as e:
            print(f"Error handling message: {e}")
            await websocket.send(f"Error: {str(e)}")
        finally:
            # Remove client from tracking set
            self.connected_clients.discard(websocket)
            print(f"Client disconnected. Total clients: {len(self.connected_clients)}")

    async def cancel_and_start_pan(self, steps):
        """Cancel ongoing pan movement and start new one"""
        with self.movement_lock:
            if self.current_pan_movement and self.current_pan_movement.is_alive():
                # Signal the current movement to stop
                self.motor_controller.driver.stop_movement()
                self.current_pan_movement.join(timeout=0.1)  # Wait briefly for cleanup
            
            # Start new movement
            self.current_pan_movement = threading.Thread(
                target=self.motor_controller.step_to, 
                args=(steps,)
            )
            self.current_pan_movement.start()

    async def cancel_and_start_tilt(self, steps):
        """Cancel ongoing tilt movement and start new one"""
        with self.movement_lock:
            if self.current_tilt_movement and self.current_tilt_movement.is_alive():
                # Signal the current movement to stop
                self.motor_controller_tilt.driver.stop_movement()
                self.current_tilt_movement.join(timeout=0.1)  # Wait briefly for cleanup
            
            # Start new movement
            self.current_tilt_movement = threading.Thread(
                target=self.motor_controller_tilt.step_to, 
                args=(steps,)
            )
            self.current_tilt_movement.start()

    async def graceful_shutdown(self):
        """Perform graceful shutdown of all processes"""
        print("Starting graceful shutdown...")
        
        # Stop IMU streaming
        self.stop_imu_streaming()
        
        # Stop IMU broadcasting
        self.stop_imu_broadcast()
        
        # Stop video streams
        self.stop_stream()
        
        # Stop capture process if running
        if self.capture_process:
            print("Stopping capture process...")
            self.capture_process.terminate()
            try:
                self.capture_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("Capture process did not terminate gracefully, forcing stop...")
                self.capture_process.kill()
        
        # Kill any remaining rpicam processes
        print("Cleaning up any remaining camera processes...")
        subprocess.run("pkill -TERM -f rpicam-vid", shell=True)
        subprocess.run("pkill -TERM -f rpicam-still", shell=True)
        subprocess.run("pkill -TERM -f ffmpeg", shell=True)
        subprocess.run("pkill -TERM -f mediamtx", shell=True)
        
        # Kill any Python HTTP servers (main.py HTTP server)
        print("Cleaning up HTTP servers...")
        subprocess.run("pkill -TERM -f 'python -m http.server'", shell=True)
        subprocess.run("pkill -TERM -f 'SimpleHTTPRequestHandler'", shell=True)
        subprocess.run("pkill -TERM -f 'TCPServer'", shell=True)
        
        # Kill any processes using common ports
        print("Cleaning up processes on common ports...")
        subprocess.run("fuser -k 8000/tcp 2>/dev/null || true", shell=True)
        subprocess.run("fuser -k 8001/tcp 2>/dev/null || true", shell=True)
        subprocess.run("fuser -k 8765/tcp 2>/dev/null || true", shell=True)
        subprocess.run("fuser -k 8554/tcp 2>/dev/null || true", shell=True)
        subprocess.run("fuser -k 8889/tcp 2>/dev/null || true", shell=True)
        
        # Small delay to let processes terminate
        time.sleep(2)
        
        # Force kill if still running
        subprocess.run("pkill -9 -f rpicam-vid", shell=True)
        subprocess.run("pkill -9 -f rpicam-still", shell=True)
        subprocess.run("pkill -9 -f ffmpeg", shell=True)
        subprocess.run("pkill -9 -f mediamtx", shell=True)
        subprocess.run("pkill -9 -f 'python -m http.server'", shell=True)
        subprocess.run("pkill -9 -f 'SimpleHTTPRequestHandler'", shell=True)
        subprocess.run("pkill -9 -f 'TCPServer'", shell=True)
        
        print("Graceful shutdown completed")
        print("All processes terminated. Exiting...")
        
        # Use os._exit() instead of sys.exit() to avoid asyncio issues
        os._exit(0)

    def start_server(self):
        async def start():
            async with websockets.serve(self.handler, "0.0.0.0", 8765):
                await asyncio.Future()  # run forever
        
        asyncio.run(start())
    
    def trigger_camera(self, idx):
        # Stop the video stream before taking a photo
        self.stop_stream()
        
        # Take the photo
        self.capture_process = self.capture_photo(idx)
        self.capture_process.wait()
        
        # Create symlink to the new photo
        os.symlink(os.path.basename(self.last_photo_path), self.last_symlink_path)

        print("Photo saved to disk, process completed")
        
        # Restart the video stream
        self.videostream_process = self.start_stream()
        print("Video stream restarted")
    
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
        sys.exit(0)

    def set_exposure_value(self, camera_index, ev_value):
        """Set exposure value for a specific camera"""
        print(f"Setting camera {camera_index} EV to: {ev_value}")
        try:
            self.stop_stream()
            mediamtx_dir = os.path.expanduser('./mediamtx')
            config_path = os.path.join(mediamtx_dir, 'mediamtx.yml')
            
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
            
            camera_path = f'cam{camera_index}'
            if camera_path in data['paths']:
                command = data['paths'][camera_path]['runOnInit']
                # Add EV control to the command
                updated_command = command.replace('rpicam-vid', f'rpicam-vid --ev {ev_value}')
                data['paths'][camera_path]['runOnInit'] = updated_command
                
                with open(config_path, 'w') as file:
                    yaml.dump(data, file, default_flow_style=False)
            
            self.videostream_process = self.start_stream()
            print(f"Camera {camera_index} EV updated to {ev_value}")
            
        except Exception as e:
            print(f"Error setting EV for camera {camera_index}: {str(e)}")
            try:
                self.videostream_process = self.start_stream()
            except:
                pass

    def set_gain_value(self, camera_index, gain_value):
        """Set gain value for a specific camera"""
        print(f"Setting camera {camera_index} gain to: {gain_value}")
        try:
            self.stop_stream()
            mediamtx_dir = os.path.expanduser('./mediamtx')
            config_path = os.path.join(mediamtx_dir, 'mediamtx.yml')
            
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
            
            camera_path = f'cam{camera_index}'
            if camera_path in data['paths']:
                command = data['paths'][camera_path]['runOnInit']
                # Add gain control to the command
                updated_command = command.replace('rpicam-vid', f'rpicam-vid --gain {gain_value}')
                data['paths'][camera_path]['runOnInit'] = updated_command
                
                with open(config_path, 'w') as file:
                    yaml.dump(data, file, default_flow_style=False)
            
            self.videostream_process = self.start_stream()
            print(f"Camera {camera_index} gain updated to {gain_value}")
            
        except Exception as e:
            print(f"Error setting gain for camera {camera_index}: {str(e)}")
            try:
                self.videostream_process = self.start_stream()
            except:
                pass

    def set_aperture_value(self, camera_index, aperture_value):
        """Set aperture value for a specific camera"""
        print(f"Setting camera {camera_index} aperture to: f/{aperture_value}")
        try:
            self.stop_stream()
            mediamtx_dir = os.path.expanduser('./mediamtx')
            config_path = os.path.join(mediamtx_dir, 'mediamtx.yml')
            
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)
            
            camera_path = f'cam{camera_index}'
            if camera_path in data['paths']:
                command = data['paths'][camera_path]['runOnInit']
                # Add aperture control to the command
                updated_command = command.replace('rpicam-vid', f'rpicam-vid --aperture {aperture_value}')
                data['paths'][camera_path]['runOnInit'] = updated_command
                
                with open(config_path, 'w') as file:
                    yaml.dump(data, file, default_flow_style=False)
            
            self.videostream_process = self.start_stream()
            print(f"Camera {camera_index} aperture updated to f/{aperture_value}")
            
        except Exception as e:
            print(f"Error setting aperture for camera {camera_index}: {str(e)}")
            try:
                self.videostream_process = self.start_stream()
            except:
                pass

