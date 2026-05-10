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
import signal
import sys
import os
import subprocess
import socket
import time

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Handle port info request
        if self.path == '/port-info':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            global http_server_info
            port = http_server_info.get('port', 8000)
            response = f'{{"port": {port}}}'
            self.wfile.write(response.encode())
            return
        
        # Handle captured images with proper MIME type
        if self.path.startswith('/captures/') and self.path.endswith('.jpg'):
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            
            # Serve the file
            file_path = '.' + self.path
            try:
                with open(file_path, 'rb') as f:
                    self.wfile.write(f.read())
            except FileNotFoundError:
                self.send_error(404, "File not found")
            return
        
        # Default behavior for other files
        super().do_GET()

def run_cleanup():
    """Run the force_cleanup.py script"""
    try:
        # Get the directory of the current script
        current_dir = os.path.dirname(os.path.abspath(__file__))
        cleanup_script = os.path.join(current_dir, 'force_cleanup.py')
        
        # Run the cleanup script with sudo
        subprocess.run(['sudo', 'python3', cleanup_script], check=True)
    except Exception as e:
        print(f"Error running cleanup script: {e}")

def start_http_server(port=8000):
    """Start HTTP server on next available port starting from the specified port"""
    max_attempts = 20
    
    for port_attempt in range(port, port + max_attempts):
        try:
            # First check if port is available with retries
            port_available = False
            for retry in range(3):  # Try 3 times with delays
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                result = sock.connect_ex(('localhost', port_attempt))
                sock.close()
                
                if result != 0:  # Port is available
                    port_available = True
                    break
                else:
                    print(f"Port {port_attempt} is in use, retrying in 0.5s... (attempt {retry + 1}/3)")
                    time.sleep(0.5)
            
            if not port_available:
                print(f"Port {port_attempt} is still in use after retries, trying next port...")
                continue
            
            with socketserver.TCPServer(("", port_attempt), CustomHTTPRequestHandler) as httpd:
                # Store the port information globally
                global http_server_info
                http_server_info['port'] = port_attempt
                
                print(f"Main HTTP server started at port {port_attempt}")
                httpd.serve_forever()
                return  # Exit the function if server starts successfully
                
        except OSError as e:
            if e.errno == 98:  # Address already in use
                print(f"Port {port_attempt} is already in use, trying next port...")
                continue
            else:
                print(f"Error starting HTTP server on port {port_attempt}: {e}")
                if port_attempt == port + max_attempts - 1:
                    print("Failed to find available port for HTTP server")
                    return
        except Exception as e:
            print(f"Unexpected error starting HTTP server on port {port_attempt}: {e}")
            if port_attempt == port + max_attempts - 1:
                print("Failed to find available port for HTTP server")
                return
    
    print("Failed to find available port for HTTP server after all attempts")

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
    tmc2208_driver = TMC2208Driver(STEP_PIN, DIR_PIN, ENABLE_PIN, 0.000025)
    tmc2208_driver_2 = TMC2208Driver(STEP_PIN_TILT, DIR_PIN_TILT, ENABLE_PIN_TILT, 0.0003)

    # Initialize the motor controller with L298N driver
    pan_motor_controller = MotorController(tmc2208_driver)
    tilt_motor_controller = MotorController(tmc2208_driver_2)

    # Initialize the astro controller
    location = EarthLocation(lat=37.772141367914045*u.deg, lon=-122.42168568117974*u.deg, height=42*u.m) #1699 Market St, SF, 9th floor
    astro_controller = AstroController(pan_motor_controller, tilt_motor_controller, location)

    # Point the camera at the moon
    #astro_controller.point_at_moon()

    # Global variable to store HTTP server info
    global http_server_info
    http_server_info = {'port': None, 'thread': None}

    # Start HTTP server in a separate thread with error handling
    http_thread = threading.Thread(target=start_http_server, daemon=True, name="HTTP-Server")
    http_thread.start()
    http_server_info['thread'] = http_thread
    print("Started HTTP server thread")

    # Initialize the WebSocket server
    #TODO: figure out how to trigger photos
    websocket_server = WebSocketServer(pan_motor_controller, tilt_motor_controller)

    # Set up signal handlers for graceful shutdown
    def signal_handler(signum, frame):
        print("\nReceived shutdown signal. Cleaning up...")
        # Kill any processes that might be using our ports
        subprocess.run("pkill -TERM -f 'SimpleHTTPRequestHandler'", shell=True)
        subprocess.run("pkill -TERM -f 'TCPServer'", shell=True)
        subprocess.run("fuser -k 8000/tcp 2>/dev/null || true", shell=True)
        subprocess.run("fuser -k 8001/tcp 2>/dev/null || true", shell=True)
        time.sleep(1)  # Give processes time to terminate
        run_cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Start the WebSocket server (which will start its own file server)
        websocket_server.start()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt. Shutting down...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup on program exit
        #l298n_driver_1.cleanup()
        #l298n_driver_2.cleanup()
        tmc2208_driver.cleanup()
        tmc2208_driver_2.cleanup()
        run_cleanup()

if __name__ == '__main__':
    main()
