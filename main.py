#from L298NDriver import L298NDriver
from TMC2208Driver import TMC2208Driver
from TMC2209Driver import TMC2209Driver
import tmc2209_uart
from motor_controller import MotorController
from astro_controller import AstroController
from websocket_server import WebSocketServer
from camera_manager import CameraManager
from astropy.coordinates import EarthLocation
from astropy import units as u
import threading
import faulthandler
faulthandler.register(10, all_threads=True)  # SIGUSR1: dump all thread stacks to stderr
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
        
        # samwise: user is in the dialout/gpio groups and sudo needs a password,
        # so run it directly (a sudo prompt here would hang the shutdown).
        subprocess.run([sys.executable, cleanup_script], check=True, timeout=20)
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
            
            # allow_reuse_address (SO_REUSEADDR): after a restart, the previous run's
            # closed browser connections sit in TIME_WAIT for ~60 s; without it the
            # bind fails and the UI silently moves to port 8001.
            class _ReusableTCPServer(socketserver.TCPServer):
                allow_reuse_address = True
            with _ReusableTCPServer(("", port_attempt), CustomHTTPRequestHandler) as httpd:
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
    # Half-period per step, in seconds. The driver now keeps time accurately;
    # the old one asked for 25 us / 300 us but time.sleep() overshoots ~50 us,
    # so the speeds you tuned were really ~80 us / ~350 us. These keep them.
    PAN_STEP_DELAY = 0.00008
    TILT_STEP_DELAY = 0.00035

    # Optional safety / repeatability (off = None / 0, same as before):
    #   SOFT_LIMITS: (min, max) steps from the counter zero (home_pointing).
    #   APPROACH: (direction, overshoot_steps): always finish moves going this
    #   way. If you enable it, turn off the browser backlash compensation.
    PAN_SOFT_LIMITS = None
    TILT_SOFT_LIMITS = None
    PAN_APPROACH = (0, 0)
    TILT_APPROACH = (0, 0)

    # Motor backend. samwise (current rig): TMC2209 over UART, closed loop on
    # the AS5600 output encoders. frodo (old rig): TMC2208 STEP/DIR/EN GPIO.
    # force_cleanup.py reads the same RIG_MOTORS variable.
    MOTOR_BACKEND = os.environ.get("RIG_MOTORS", "tmc2209_uart")

    if MOTOR_BACKEND == "tmc2209_uart":
        # Steps are virtual units of 1/STEPS_PER_DEG output degrees (the values
        # index.html's click-to-point assumes); the step delays above keep their
        # meaning: pan 80 us -> 2.91 deg/s, tilt 350 us -> 1.77 deg/s.
        PAN_STEPS_PER_DEG = 2146
        TILT_STEPS_PER_DEG = 805
        # -1: a positive step lowers the encoder angle. Set 2026-11-25 after the
        # UI's up/down and left/right buttons were both inverted with +1.
        PAN_STEP_SIGN = -1
        TILT_STEP_SIGN = -1
        motor_bus = tmc2209_uart.TMC2209Bus()
        motor_bus.zero_now()                     # start from a known stopped state
        # Backstop: zero both drivers on any interpreter exit (handle_shutdown
        # in websocket_server.py also stops them, before the cameras).
        import atexit
        atexit.register(motor_bus.zero_now)
        # Absolute travel limits (cable protection), from rig_limits.json.
        limits_cfg = {}
        try:
            import json
            with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "rig_limits.json")) as f:
                limits_cfg = json.load(f)
        except FileNotFoundError:
            print("WARNING: rig_limits.json not found -- NO travel limits")
        lim = limits_cfg.get("limits_deg") or {}
        encoders = tmc2209_uart.AS5600Encoders(zero_raw=limits_cfg.get("encoder_zero_raw"))
        tmc2208_driver = TMC2209Driver(motor_bus, encoders, "pan", PAN_STEP_DELAY, PAN_STEPS_PER_DEG,
                                       current_ma=1000, step_sign=PAN_STEP_SIGN,
                                       soft_limits=PAN_SOFT_LIMITS, travel_limits=lim.get("pan"))
        tmc2208_driver_2 = TMC2209Driver(motor_bus, encoders, "tilt", TILT_STEP_DELAY, TILT_STEPS_PER_DEG,
                                         current_ma=1400, step_sign=TILT_STEP_SIGN,
                                         soft_limits=TILT_SOFT_LIMITS, travel_limits=lim.get("tilt"))
        print(f"Motors: TMC2209 UART, pan {tmc2208_driver.max_dps:.2f} deg/s, "
              f"tilt {tmc2208_driver_2.max_dps:.2f} deg/s max; travel limits "
              f"pan {lim.get('pan')} tilt {lim.get('tilt')} deg")
    elif MOTOR_BACKEND == "tmc2208_gpio":
        tmc2208_driver = TMC2208Driver(STEP_PIN, DIR_PIN, ENABLE_PIN, PAN_STEP_DELAY,
                                       soft_limits=PAN_SOFT_LIMITS,
                                       approach_dir=PAN_APPROACH[0], approach_overshoot=PAN_APPROACH[1])
        tmc2208_driver_2 = TMC2208Driver(STEP_PIN_TILT, DIR_PIN_TILT, ENABLE_PIN_TILT, TILT_STEP_DELAY,
                                         soft_limits=TILT_SOFT_LIMITS,
                                         approach_dir=TILT_APPROACH[0], approach_overshoot=TILT_APPROACH[1])
    else:
        sys.exit(f"Unknown RIG_MOTORS={MOTOR_BACKEND!r} (use tmc2209_uart or tmc2208_gpio)")

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

    # Camera manager owns Picamera2 instances and pushes H.264 into MediaMTX.
    camera_manager = CameraManager()

    # Initialize the WebSocket server (it starts MediaMTX, then CameraManager).
    websocket_server = WebSocketServer(pan_motor_controller, tilt_motor_controller,
                                       camera_manager=camera_manager)

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
