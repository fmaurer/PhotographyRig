import websockets
import asyncio
import subprocess
import os
import datetime
import yaml
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
        BNO_REPORT_GAME_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    IMU_AVAILABLE = True
except ImportError:
    print("IMU libraries not available. IMU functionality will be disabled.")
    IMU_AVAILABLE = False

from pointing_calibration import PointingCalibration, earth_to_azel
from homography_calibration import HomographyCalibration
from backlash_calibration import (
    BacklashCalibrator,
    CalibrationAborted as BacklashAborted,
    CalibrationCancelled as BacklashCancelled,
)


# Camera index mapping for homography. In this rig the cam1 iframe is the
# wide context view (where the red-zoom-indicator overlay lives in index.html)
# and cam0 is the zoom / capture view. Flip these two if the UI iframes get
# rewired.
WIDE_CAM = 1
ZOOM_CAM = 0


# Commands that the UI polls at high frequency (~1 Hz) and that don't carry
# meaningful per-call information — suppress their "Received message: ..."
# log line to keep the server stdout readable. Add others here if more
# polling endpoints are introduced.
_QUIET_LOG_COMMANDS = frozenset({
    "get_pointing_state",
    "get_imu",
    "get_homography",
})

class WebSocketServer:
    def __init__(self, motor_controller, motor_controller2, camera_manager=None):
        self.SYMLINK_PATH = '/home/frodo/PhotographyRig/captures/'
        self.motor_controller = motor_controller
        self.motor_controller_tilt = motor_controller2
        self.camera_manager = camera_manager
        
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
        self.bno_lock = threading.Lock()  # serializes all I2C reads against the BNO chip
        self.imu_thread = None
        self.imu_running = False
        self.bno = None

        # IMU calibration offsets (loaded from calibration.json)
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.load_imu_calibration()
        
        # Track connected clients for broadcasting
        self.connected_clients = set()

        # Backlash calibration state (one run at a time).
        self._backlash_calibrator = None
        self._backlash_lock = threading.Lock()
        self._backlash_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "backlash_calibration.json")
        self._backlash_data = self._load_backlash_calibration()
        if self._backlash_data:
            res = self._backlash_data.get("results", {})
            try:
                yaw_p = res.get("yaw_pos", {}).get("mean")
                yaw_n = res.get("yaw_neg", {}).get("mean")
                pit_p = res.get("pitch_pos", {}).get("mean")
                pit_n = res.get("pitch_neg", {}).get("mean")
                print(f"Backlash calibration loaded: yaw=({yaw_p}/{yaw_n}), "
                      f"pitch=({pit_p}/{pit_n}) steps (pos/neg)")
            except Exception:
                pass
        
        # Initialize IMU
        self.setup_imu()

        # Pointing calibration: (lat,lon,alt) -> motor steps mapping.
        pointing_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'pointing_calibration.json')
        self.pointing = PointingCalibration(pointing_path)
        if self.pointing.observer:
            print(f"Pointing observer loaded: lat={self.pointing.observer.lat}, "
                  f"lon={self.pointing.observer.lon}, alt={self.pointing.observer.alt_m}m")
        if self.pointing.fit:
            print(f"Pointing calibration loaded: pan_k={self.pointing.fit.pan_steps_per_deg:.2f} "
                  f"steps/deg, tilt_k={self.pointing.fit.tilt_steps_per_deg:.2f} steps/deg, "
                  f"rms={self.pointing.fit.rms_residual_deg:.3f} deg")

        # Homography calibration: wide<->zoom red-box overlay.
        homography_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       'homography_calibration.json')
        self.homography = HomographyCalibration(homography_path)
        if self.homography.fit:
            f = self.homography.fit
            print(f"Homography calibration loaded: inliers={f.inliers}, rms={f.rms_px:.2f}px, "
                  f"detector={f.detector}, computed_at={f.computed_at}")
        
        # Remove redundant file server - main.py already handles this
        # self.file_server = self.start_file_server()
        # print("File server started")
        
        # Initialize single video stream process
        self.videostream_process = None
        
        # Set up signal handlers
        signal.signal(signal.SIGTERM, self.handle_shutdown)
        signal.signal(signal.SIGINT, self.handle_shutdown)
        
        # Start MediaMTX (now just a relay — no per-camera runOnInit).
        self.setup_stream_configs()
        self.videostream_process = self.start_stream()
        print("MediaMTX started")

        # CameraManager owns the Picamera2 instances and pushes H.264 into MediaMTX.
        # MediaMTX needs a moment to bind its RTSP listener before publishers connect.
        if self.camera_manager is not None:
            time.sleep(1)
            self.camera_manager.start_all()
            print("Camera streams started")
        else:
            print("WARN: WebSocketServer started without a CameraManager — "
                  "live preview will be unavailable.")

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
            
            # Paths are pure publishers. CameraManager pushes H.264 into them via
            # Picamera2 -> ffmpeg -> RTSP. No runOnInit anymore — exposure and
            # other controls are applied live via libcamera set_controls().
            'paths': {
                'cam0': {
                    'source': 'publisher',
                    'sourceProtocol': 'tcp',
                },
                'cam1': {
                    'source': 'publisher',
                    'sourceProtocol': 'tcp',
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
                # Game rotation vector = gyro+accel fusion (no magnetometer).
                # Used during IMU-assisted sweep calibration; immune to the
                # stepper magnets that would corrupt the mag-fused yaw.
                try:
                    self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
                    print("Game rotation vector enabled (for sweep calibration)")
                except Exception as e:
                    print(f"Could not enable game rotation vector: {e}")
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
                with self.bno_lock:
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

    # ---- pointing-calibration helpers --------------------------------------

    def _pan_steps_now(self):
        return self.motor_controller.driver.total_signed_steps

    def _tilt_steps_now(self):
        return self.motor_controller_tilt.driver.total_signed_steps

    def _read_game_euler_deg(self):
        """Read BNO game rotation vector (gyro+accel, no mag) -> (tilt, pan) in degrees.

        Matches the rest of this codebase's axis mapping: the IMU is mounted so
        that the rig's tilt motion shows up on the function's *roll* output (rx),
        and the rig's pan motion shows up on the function's *yaw* output (rz).
        See imu_data_collection_loop, where rx is stored as 'pitch' and the
        level calibration captures rx ≈ 90°.

        Returns (tilt_deg, pan_deg) or None if the IMU isn't available.
        """
        if not self.bno:
            return None
        with self.bno_lock:
            quat_i, quat_j, quat_k, quat_real = self.bno.game_quaternion
        roll_rad, _pitch_rad, yaw_rad = self.quaternion_to_euler(
            quat_real, quat_i, quat_j, quat_k
        )
        # In this rig: function's roll axis = camera tilt, function's yaw axis = pan.
        return math.degrees(roll_rad), math.degrees(yaw_rad)

    async def _run_imu_axis_sweep(self, websocket, axis, signed_steps,
                                  lash_pre_steps=200, settle_ms=500, sample_hz=50):
        """Move one axis a known step delta and record (steps, IMU_angle) samples,
        then fit a line. axis = 'pan' or 'tilt'. Pan uses yaw, tilt uses pitch.

        Streams the result over the websocket as JSON:
            {type: 'imu_sweep_result', axis, ok, K, intercept, rms_deg, n_samples,
             samples: [[angle_deg, steps_signed], ...], message}
        """
        if axis not in ('pan', 'tilt'):
            await websocket.send(json.dumps({
                'type': 'imu_sweep_result', 'axis': axis, 'ok': False,
                'message': "axis must be 'pan' or 'tilt'"
            }))
            return
        if not self.bno:
            await websocket.send(json.dumps({
                'type': 'imu_sweep_result', 'axis': axis, 'ok': False,
                'message': 'IMU not available'
            }))
            return
        if abs(signed_steps) < 50:
            await websocket.send(json.dumps({
                'type': 'imu_sweep_result', 'axis': axis, 'ok': False,
                'message': 'sweep too small (need at least 50 steps)'
            }))
            return

        # Pause the 10 Hz IMU collection thread so we have exclusive access at
        # the cadence we want, then sample in this thread.
        was_running = self.imu_running
        if was_running:
            self.stop_imu_streaming()

        try:
            # Lash-eating pre-move in the same direction.
            sign = 1 if signed_steps > 0 else -1
            pre = sign * abs(lash_pre_steps)
            if axis == 'pan':
                await self.cancel_and_start_pan(pre)
                mover = self.current_pan_movement
            else:
                await self.cancel_and_start_tilt(pre)
                mover = self.current_tilt_movement
            await asyncio.sleep(0.05)
            if mover:
                # Wait for pre-move to finish.
                while mover.is_alive():
                    await asyncio.sleep(0.02)
            await asyncio.sleep(0.3)  # let mechanics settle

            samples = []  # list of (angle_deg, steps_signed)

            def read_axis_angle():
                pe = self._read_game_euler_deg()
                if pe is None:
                    return None
                tilt_deg, pan_deg = pe   # mapped to the rig's axes inside the helper
                return pan_deg if axis == 'pan' else tilt_deg

            def steps_now():
                return self._pan_steps_now() if axis == 'pan' else self._tilt_steps_now()

            # Baseline sample at rest.
            t0 = time.time()
            interval = 1.0 / max(1, sample_hz)
            ang0 = read_axis_angle()
            samples.append((ang0, steps_now()))

            # Kick off the sweep move.
            if axis == 'pan':
                await self.cancel_and_start_pan(signed_steps)
                mover = self.current_pan_movement
            else:
                await self.cancel_and_start_tilt(signed_steps)
                mover = self.current_tilt_movement

            # Sample while moving.
            last = t0
            while mover and mover.is_alive():
                now = time.time()
                if now - last >= interval:
                    last = now
                    ang = read_axis_angle()
                    samples.append((ang, steps_now()))
                await asyncio.sleep(min(interval, 0.01))

            # Settling samples after motor stops.
            settle_end = time.time() + settle_ms / 1000.0
            while time.time() < settle_end:
                ang = read_axis_angle()
                samples.append((ang, steps_now()))
                await asyncio.sleep(interval)

            # Unwrap yaw for pan (game-rot-vec yaw wraps at ±180°).
            import numpy as np
            angles = [s[0] for s in samples if s[0] is not None]
            steps_list = [s[1] for s in samples if s[0] is not None]
            if len(angles) < 4:
                raise RuntimeError(f"not enough samples: {len(angles)}")
            if axis == 'pan':
                angles = np.degrees(np.unwrap(np.deg2rad(angles))).tolist()

            x = np.array(angles, dtype=float)
            y = np.array(steps_list, dtype=float)
            if np.ptp(x) < 0.5:
                raise RuntimeError(f"IMU did not register enough rotation ({np.ptp(x):.3f}°)")

            # Linear fit y = K * x + b
            A = np.column_stack([x, np.ones_like(x)])
            sol, *_ = np.linalg.lstsq(A, y, rcond=None)
            K, b = float(sol[0]), float(sol[1])
            resid_steps = y - (K * x + b)
            rms_deg = float(np.sqrt(np.mean((resid_steps / K) ** 2))) if K else float('inf')
            x_range = float(np.ptp(x))

            await websocket.send(json.dumps({
                'type': 'imu_sweep_result',
                'axis': axis,
                'ok': True,
                'K': K,                   # steps per IMU-degree
                'intercept': b,
                'rms_deg': rms_deg,
                'angle_span_deg': x_range,
                'n_samples': len(angles),
                'samples': list(zip(angles, steps_list)),
                'message': (f"{axis} sweep: K={K:.2f} steps/° over {x_range:.2f}° "
                            f"(rms residual {rms_deg:.4f}°, {len(angles)} samples)"),
            }))
        except Exception as e:
            await websocket.send(json.dumps({
                'type': 'imu_sweep_result', 'axis': axis, 'ok': False,
                'message': f'sweep failed: {e}'
            }))
        finally:
            if was_running:
                self.start_imu_streaming()

    async def _run_backlash(self, websocket, cam_idx, trials, max_steps,
                            settle_s, engage_steps, min_features,
                            debug=False, probe_steps=None,
                            pitch_probe_steps=None):
        """Drive a BacklashCalibrator from a worker thread, pumping progress
        messages back to the websocket. Broadcasts the final result so every
        connected UI sees it."""
        loop = asyncio.get_running_loop()

        async def _send_progress(msg):
            text = json.dumps(msg)
            # Send to the originating socket; tolerate it being closed.
            try:
                await websocket.send(text)
            except websockets.exceptions.ConnectionClosed:
                pass

        def _progress_cb(msg):
            # Called from the worker thread — hop back to the event loop.
            asyncio.run_coroutine_threadsafe(_send_progress(msg), loop)

        with self._backlash_lock:
            if self._backlash_calibrator is not None:
                await websocket.send(json.dumps({
                    'type': 'backlash_result', 'ok': False,
                    'error': 'calibration already running',
                }))
                return
            debug_abs, debug_url = (self._new_backlash_debug_dir()
                                    if debug else (None, None))
            calibrator = BacklashCalibrator(
                self.motor_controller, self.motor_controller_tilt,
                self.camera_manager, cam_idx,
                trials=trials, max_steps=max_steps,
                settle_s=settle_s, engage_steps=engage_steps,
                min_features=min_features,
                probe_steps=probe_steps,
                pitch_probe_steps=pitch_probe_steps,
                debug_dir=debug_abs,
                progress_callback=_progress_cb,
            )
            self._backlash_calibrator = calibrator

        try:
            payload = await asyncio.to_thread(calibrator.run)
            result = {'type': 'backlash_result', 'ok': True,
                      'path': calibrator.output_path, 'data': payload}
            # Refresh cached state so compensation picks up the new values.
            self._backlash_data = payload
        except BacklashCancelled:
            result = {'type': 'backlash_result', 'ok': False,
                      'cancelled': True, 'error': 'cancelled'}
        except BacklashAborted as e:
            result = {'type': 'backlash_result', 'ok': False,
                      'error': str(e), 'reason': e.reason}
        except Exception as e:
            result = {'type': 'backlash_result', 'ok': False,
                      'error': f'{type(e).__name__}: {e}'}
        finally:
            with self._backlash_lock:
                self._backlash_calibrator = None
        if debug_url:
            # Surface the debug URL even on cancel/abort so partial dumps
            # (e.g. the reference frame from a low_texture abort) stay
            # discoverable.
            result['debug_url'] = debug_url
        # Plot URL — prefer the per-run copy in the debug dir (tied to
        # the dataset the user just looked at). Fall back to the stable
        # project-root copy that's always overwritten on success.
        project_root = os.path.dirname(os.path.abspath(__file__))
        stable_plot = os.path.join(project_root, "backlash_plot.png")
        per_run_plot = (os.path.join(debug_abs, "backlash_plot.png")
                        if debug_abs else None)
        if per_run_plot and os.path.exists(per_run_plot) and debug_url:
            result['plot_url'] = debug_url.rstrip('/') + "/backlash_plot.png"
        elif os.path.exists(stable_plot):
            result['plot_url'] = "/backlash_plot.png"

        await self._broadcast_json(result)
        # Push the refreshed state so any open UI updates its compensation.
        if result.get('ok'):
            await self._broadcast_json(self._backlash_state_payload())

    def _load_backlash_calibration(self):
        """Best-effort load of backlash_calibration.json. Returns dict or None."""
        try:
            with open(self._backlash_path, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except Exception as e:
            print(f"backlash calibration: failed to load {self._backlash_path}: {e}")
            return None

    def _backlash_state_payload(self):
        return {
            "type": "backlash_calibration_state",
            "ok": self._backlash_data is not None,
            "data": self._backlash_data,
        }

    async def _broadcast_json(self, payload):
        """Send a JSON payload to every connected client (best-effort)."""
        if not self.connected_clients:
            return
        text = json.dumps(payload)
        disconnected = set()
        for client in self.connected_clients:
            try:
                await client.send(text)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
        self.connected_clients -= disconnected

    def _pointing_state_dict(self):
        state = self.pointing.state_dict()
        pan_now = self._pan_steps_now()
        tilt_now = self._tilt_steps_now()
        state['current_pan_steps'] = pan_now
        state['current_tilt_steps'] = tilt_now
        if self.pointing.fit:
            try:
                az, el = self.pointing.steps_to_azel(pan_now, tilt_now)
                state['current_az_deg'] = az
                state['current_el_deg'] = el
            except Exception:
                state['current_az_deg'] = None
                state['current_el_deg'] = None
        else:
            state['current_az_deg'] = None
            state['current_el_deg'] = None
        return state

    async def _send_pointing_state(self, websocket, ok=True, message=None):
        payload = {'type': 'pointing_state', 'ok': ok}
        if message is not None:
            payload['message'] = message
        payload['data'] = self._pointing_state_dict()
        await websocket.send(json.dumps(payload))

    async def _send_homography_state(self, websocket, ok=True, message=None, debug_url=None):
        payload = {'type': 'homography_state', 'ok': ok}
        if message is not None:
            payload['message'] = message
        if debug_url is not None:
            payload['debug_url'] = debug_url
        payload['data'] = self.homography.state_dict()
        await websocket.send(json.dumps(payload))

    async def _broadcast_homography_state(self, message=None, debug_url=None):
        """Push the current fit to every connected client (used after a successful calibration)."""
        if not self.connected_clients:
            return
        payload = {'type': 'homography_state', 'ok': True, 'data': self.homography.state_dict()}
        if message is not None:
            payload['message'] = message
        if debug_url is not None:
            payload['debug_url'] = debug_url
        text = json.dumps(payload)
        disconnected = set()
        for client in self.connected_clients:
            try:
                await client.send(text)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
        self.connected_clients -= disconnected

    def _auto_wide_crop_from_cache(self, margin: float = 1.0):
        """Derive a sensible wide_crop from the cached fit's bounding box.

        Expands the previous bounding box by `margin` × on each side (so margin=1.0
        means the crop is 3× the previous bbox in each dimension), then clamps to
        the wide image bounds. Returns None if no cache is available.
        """
        fit = self.homography.fit
        if fit is None or not fit.corners_wide_px or not fit.wide_size:
            return None
        xs = [c[0] for c in fit.corners_wide_px]
        ys = [c[1] for c in fit.corners_wide_px]
        x_lo, x_hi = min(xs), max(xs)
        y_lo, y_hi = min(ys), max(ys)
        w = max(1.0, x_hi - x_lo)
        h = max(1.0, y_hi - y_lo)
        mx, my = w * margin, h * margin
        cx = int(round(x_lo - mx))
        cy = int(round(y_lo - my))
        cw = int(round(w + 2 * mx))
        ch = int(round(h + 2 * my))
        full_w, full_h = int(fit.wide_size[0]), int(fit.wide_size[1])
        # Clamp.
        cx = max(0, min(full_w - 1, cx))
        cy = max(0, min(full_h - 1, cy))
        cw = max(1, min(full_w - cx, cw))
        ch = max(1, min(full_h - cy, ch))
        return (cx, cy, cw, ch)

    async def _do_compute_homography_pairs(self, wide_pts, zoom_pts):
        """Fit H from user-supplied manual point pairs.

        Captures a single zoom + wide frame just to get their actual dims (the
        UI sends coords in 1280x720 sensor space; we sanity-check that matches
        what the cameras are streaming and persist the right dims in the fit).
        """
        if self.camera_manager is None:
            raise RuntimeError("camera manager unavailable")
        with self.imu_lock:
            imu_pose = dict(self.imu_data)
        # Cheap sanity grab so we know real frame dims (and they get persisted).
        frame_wide, frame_zoom = await asyncio.gather(
            asyncio.to_thread(self.camera_manager.capture_array, WIDE_CAM, color="gray"),
            asyncio.to_thread(self.camera_manager.capture_array, ZOOM_CAM, color="gray"),
        )
        full_wide_size = [int(frame_wide.shape[1]), int(frame_wide.shape[0])]
        full_zoom_size = [int(frame_zoom.shape[1]), int(frame_zoom.shape[0])]

        debug_abs, debug_url = self._new_homography_debug_dir()
        try:
            fit = await asyncio.to_thread(
                lambda: self.homography.compute_from_pairs(
                    wide_pts, zoom_pts,
                    full_zoom_size=full_zoom_size,
                    full_wide_size=full_wide_size,
                    imu_pose=imu_pose,
                    debug_dir=debug_abs,
                )
            )
        except Exception as e:
            try:
                e.debug_url = debug_url
            except (AttributeError, TypeError):
                pass
            raise
        await asyncio.to_thread(self.homography.save)
        return fit, debug_url

    def _new_homography_debug_dir(self):
        """Generate a filesystem-safe, timestamped directory under homography_debug/.

        Returns (abs_path, url_path) — both relative to the project root, so url_path
        is ready to drop into an <a href> served by the HTTP server on port 8000.
        """
        ts = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        rel = os.path.join("homography_debug", ts)
        abs_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), rel)
        # Use forward slashes for the URL regardless of OS.
        url = "/" + rel.replace(os.sep, "/") + "/"
        return abs_path, url

    def _new_backlash_debug_dir(self):
        """Same convention as _new_homography_debug_dir but for backlash runs."""
        ts = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        rel = os.path.join("backlash_debug", ts)
        abs_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), rel)
        url = "/" + rel.replace(os.sep, "/") + "/"
        return abs_path, url

    async def _do_calibrate_homography(self, wide_crop=None, use_cache_crop=True,
                                       method="features", zoom_extra_scale=1.0):
        """Grab one frame from each camera, fit H, persist, return (fit, debug_url).

        - wide_crop: explicit (x, y, w, h) in full-wide pixels, or None.
        - use_cache_crop: if True and wide_crop is None, derive a crop from the
          cached fit's bounding box (expanded). If False, fit the full wide frame.
        - method: "features" (AKAZE/SIFT + RANSAC) or "ecc" (direct intensity
          alignment via cv2.findTransformECC).
        - zoom_extra_scale: multiplier on the auto-derived zoom downsample
          factor. <1 means more aggressive downsample (use when the real zoom
          FOV is much narrower than the wide_crop's pixel-extent suggests).

        Always writes debug artifacts to the returned debug_url — even on failure.
        """
        if self.camera_manager is None:
            raise RuntimeError("camera manager unavailable")
        if (self.current_pan_movement and self.current_pan_movement.is_alive()) or \
           (self.current_tilt_movement and self.current_tilt_movement.is_alive()):
            raise RuntimeError("cannot calibrate while motors are moving")

        if wide_crop is None and use_cache_crop:
            wide_crop = self._auto_wide_crop_from_cache()

        with self.imu_lock:
            imu_pose = dict(self.imu_data)

        # Each _CameraStream has its own lock, so these run truly in parallel.
        frame_wide, frame_zoom = await asyncio.gather(
            asyncio.to_thread(self.camera_manager.capture_array, WIDE_CAM, color="gray"),
            asyncio.to_thread(self.camera_manager.capture_array, ZOOM_CAM, color="gray"),
        )

        debug_abs, debug_url = self._new_homography_debug_dir()
        try:
            fit = await asyncio.to_thread(
                lambda: self.homography.compute(
                    frame_wide, frame_zoom, imu_pose,
                    wide_crop=wide_crop, method=method,
                    zoom_extra_scale=zoom_extra_scale,
                    debug_dir=debug_abs,
                )
            )
        except Exception as e:
            # Tag the exception so the caller can include the debug URL in its
            # error message. We don't want to mask the original error.
            try:
                e.debug_url = debug_url
            except (AttributeError, TypeError):
                pass
            raise
        # Persist off the event loop — small write but uses fsync via os.replace.
        await asyncio.to_thread(self.homography.save)
        return fit, debug_url

    # Safety caps so a sign / observer error can't spin the rig forever.
    # Adjust here if a legitimate goto exceeds these.
    MAX_PAN_DELTA_DEG = 180.0
    MAX_TILT_DELTA_DEG = 60.0

    def _preview_goto_latlon(self, lat, lon, alt_m):
        pan_target, tilt_target, az, el = self.pointing.latlon_to_steps(lat, lon, alt_m)
        pan_now = self._pan_steps_now()
        tilt_now = self._tilt_steps_now()
        pan_delta = pan_target - pan_now
        tilt_delta = tilt_target - tilt_now
        # Convert step deltas back into degrees for human-readable preview.
        f = self.pointing.fit
        pan_delta_deg = pan_delta / f.pan_steps_per_deg if f and f.pan_steps_per_deg else None
        tilt_delta_deg = tilt_delta / f.tilt_steps_per_deg if f and f.tilt_steps_per_deg else None
        return {
            'az_deg': az, 'el_deg': el,
            'pan_target_steps': pan_target, 'tilt_target_steps': tilt_target,
            'pan_delta_steps': pan_delta, 'tilt_delta_steps': tilt_delta,
            'pan_delta_deg': pan_delta_deg, 'tilt_delta_deg': tilt_delta_deg,
            'pan_now_steps': pan_now, 'tilt_now_steps': tilt_now,
        }

    async def _do_goto_latlon(self, lat, lon, alt_m, force=False):
        result = self._preview_goto_latlon(lat, lon, alt_m)
        pdd = result.get('pan_delta_deg')
        tdd = result.get('tilt_delta_deg')
        if not force:
            over_pan = pdd is not None and abs(pdd) > self.MAX_PAN_DELTA_DEG
            over_tilt = tdd is not None and abs(tdd) > self.MAX_TILT_DELTA_DEG
            if over_pan or over_tilt:
                raise RuntimeError(
                    f"goto would move pan={pdd:.1f}°, tilt={tdd:.1f}°; "
                    f"limits are pan<={self.MAX_PAN_DELTA_DEG}°, tilt<={self.MAX_TILT_DELTA_DEG}°. "
                    f"Use force_goto_latlon to override, or check that your observer location, "
                    f"target lat/lon, and IMU-derived K signs all agree."
                )
        await self.cancel_and_start_pan(result['pan_delta_steps'])
        await self.cancel_and_start_tilt(result['tilt_delta_steps'])
        return result

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
        """Stop the MediaMTX server.

        Note: ffmpeg subprocesses are owned by CameraManager via Picamera2's
        FfmpegOutput, so they're cleaned up when the manager is stopped.
        rpicam-vid is no longer spawned at all (paths are pure publishers)."""
        if self.videostream_process:
            print("Stopping MediaMTX server...")
            try:
                self.videostream_process.terminate()
                try:
                    self.videostream_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print("MediaMTX did not terminate gracefully, killing...")
                    self.videostream_process.kill()
                    self.videostream_process.wait()
                print("MediaMTX stopped")
            except Exception as e:
                print(f"Error stopping MediaMTX: {str(e)}")
                try:
                    self.videostream_process.kill()
                except Exception:
                    pass
            finally:
                self.videostream_process = None

    async def handler(self, websocket):
        """Handle incoming WebSocket messages."""
        # Add client to tracking set
        self.connected_clients.add(websocket)
        print(f"Client connected. Total clients: {len(self.connected_clients)}")

        # Push any cached homography fit so the red-box overlay appears immediately.
        if self.homography.fit is not None:
            try:
                await self._send_homography_state(websocket, message="cached fit")
            except Exception as e:
                print(f"Failed to push cached homography on connect: {e}")

        # Push cached backlash calibration so client-side comp can self-populate.
        if self._backlash_data is not None:
            try:
                await websocket.send(json.dumps(self._backlash_state_payload()))
            except Exception as e:
                print(f"Failed to push backlash state on connect: {e}")

        try:
            async for message in websocket:
                # Suppress logging for high-frequency polling commands so the
                # server stdout stays readable. These fire every ~1 s from the
                # UI's setInterval and don't carry useful information per-call.
                _command_word = message.split(None, 1)[0] if message else ""
                if _command_word not in _QUIET_LOG_COMMANDS:
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
                elif command == "set_observer_location":
                    if len(parts) != 4:
                        await websocket.send("Error: set_observer_location requires lat lon alt_m")
                        continue
                    try:
                        lat, lon, alt_m = float(parts[1]), float(parts[2]), float(parts[3])
                        self.pointing.set_observer(lat, lon, alt_m)
                        await self._send_pointing_state(websocket, message="observer set")
                    except Exception as e:
                        await websocket.send(f"Error: {e}")
                elif command == "add_pointing_reference":
                    if len(parts) != 5:
                        await websocket.send("Error: add_pointing_reference requires name lat lon alt_m (name has no spaces)")
                        continue
                    if (self.current_pan_movement and self.current_pan_movement.is_alive()) or \
                       (self.current_tilt_movement and self.current_tilt_movement.is_alive()):
                        await websocket.send("Error: cannot add reference while motors are moving")
                        continue
                    try:
                        name = parts[1]
                        lat, lon, alt_m = float(parts[2]), float(parts[3]), float(parts[4])
                        pan_now = self._pan_steps_now()
                        tilt_now = self._tilt_steps_now()
                        self.pointing.add_reference(name, lat, lon, alt_m, pan_now, tilt_now)
                        await self._send_pointing_state(websocket, message=f"reference '{name}' captured at pan={pan_now}, tilt={tilt_now}")
                    except Exception as e:
                        await websocket.send(f"Error: {e}")
                elif command == "remove_pointing_reference":
                    if len(parts) != 2:
                        await websocket.send("Error: remove_pointing_reference requires a name")
                        continue
                    removed = self.pointing.remove_reference(parts[1])
                    msg = f"removed '{parts[1]}'" if removed else f"no reference named '{parts[1]}'"
                    await self._send_pointing_state(websocket, ok=removed, message=msg)
                elif command == "list_pointing_references":
                    await self._send_pointing_state(websocket)
                elif command == "compute_pointing_calibration":
                    try:
                        fit = self.pointing.compute()
                        await self._send_pointing_state(
                            websocket,
                            message=(f"calibrated: pan={fit.pan_steps_per_deg:.2f} steps/deg, "
                                     f"tilt={fit.tilt_steps_per_deg:.2f} steps/deg, "
                                     f"rms={fit.rms_residual_deg:.3f} deg, n={fit.n_references}"),
                        )
                    except Exception as e:
                        await self._send_pointing_state(websocket, ok=False, message=f"compute failed: {e}")
                elif command in ("calibrate_homography", "calibrate_homography_ecc"):
                    # Forms accepted (same for both, only the fit method differs):
                    #   <cmd>                 -> auto crop from cache if any, else full
                    #   <cmd> full            -> force full wide image (no crop)
                    #   <cmd> cx cy w h       -> explicit crop in full-wide pixels
                    # Any of the above may also include `zoom=N.NN` to override
                    # the zoom-downsample multiplier (default 1.0).
                    method = "ecc" if command == "calibrate_homography_ecc" else "features"
                    wide_crop = None
                    use_cache_crop = True
                    zoom_extra_scale = 1.0

                    # Strip any key=value tokens out first.
                    positional = []
                    kv_error = None
                    for tok in parts[1:]:
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            if k == "zoom":
                                try:
                                    zoom_extra_scale = float(v)
                                except ValueError:
                                    kv_error = f"Error: {command} 'zoom=' value must be a float"
                                    break
                            else:
                                kv_error = f"Error: {command} unknown key '{k}' (expected 'zoom=')"
                                break
                        else:
                            positional.append(tok)
                    if kv_error:
                        await websocket.send(kv_error)
                        continue

                    if len(positional) == 1 and positional[0].lower() == "full":
                        use_cache_crop = False
                    elif len(positional) == 4:
                        try:
                            wide_crop = (int(positional[0]), int(positional[1]),
                                         int(positional[2]), int(positional[3]))
                        except ValueError:
                            await websocket.send(f"Error: {command} crop args must be ints: cx cy w h")
                            continue
                    elif len(positional) != 0:
                        await websocket.send(f"Error: {command} positional args: 0, 'full', or 'cx cy w h' (+ optional zoom=N.NN)")
                        continue
                    try:
                        fit, debug_url = await self._do_calibrate_homography(
                            wide_crop=wide_crop, use_cache_crop=use_cache_crop, method=method,
                            zoom_extra_scale=zoom_extra_scale,
                        )
                        crop_note = f", crop={fit.wide_crop}" if fit.wide_crop else ""
                        if fit.method == "ecc":
                            confidence_note = f"cc={fit.ecc_cc:.3f}" if fit.ecc_cc is not None else "cc=?"
                        else:
                            confidence_note = (f"inliers={fit.inliers}, rms={fit.rms_px:.2f}px, "
                                               f"n={fit.n_matches}")
                        msg = (f"calibrated ({fit.detector}): {confidence_note}{crop_note} "
                               f"— saved to {os.path.basename(self.homography.path)}")
                        # Broadcast so every browser tab updates its overlay.
                        await self._broadcast_homography_state(message=msg, debug_url=debug_url)
                    except Exception as e:
                        debug_url = getattr(e, "debug_url", None)
                        await self._send_homography_state(
                            websocket, ok=False,
                            message=f"calibration failed ({method}): {e}",
                            debug_url=debug_url,
                        )
                elif command == "compute_homography_pairs":
                    # Form: compute_homography_pairs N wx1 wy1 zx1 zy1 wx2 wy2 zx2 zy2 ...
                    # Coords are in full-wide / full-zoom sensor space (pixels).
                    if len(parts) < 2:
                        await websocket.send("Error: compute_homography_pairs requires N then 4N coords")
                        continue
                    try:
                        n = int(parts[1])
                    except ValueError:
                        await websocket.send("Error: compute_homography_pairs first arg must be int N")
                        continue
                    if n < 4:
                        await websocket.send(f"Error: need at least 4 pairs, got {n}")
                        continue
                    expected = 2 + 4 * n
                    if len(parts) != expected:
                        await websocket.send(
                            f"Error: compute_homography_pairs expected {expected} tokens, got {len(parts)}"
                        )
                        continue
                    try:
                        wide_pts = []
                        zoom_pts = []
                        idx = 2
                        for _ in range(n):
                            wide_pts.append([float(parts[idx]), float(parts[idx + 1])])
                            zoom_pts.append([float(parts[idx + 2]), float(parts[idx + 3])])
                            idx += 4
                    except ValueError as e:
                        await websocket.send(f"Error parsing pair coords: {e}")
                        continue
                    try:
                        fit, debug_url = await self._do_compute_homography_pairs(wide_pts, zoom_pts)
                        msg = (f"calibrated (manual_pairs): inliers={fit.inliers}/{fit.n_matches}, "
                               f"rms={fit.rms_px:.2f}px — saved to "
                               f"{os.path.basename(self.homography.path)}")
                        await self._broadcast_homography_state(message=msg, debug_url=debug_url)
                    except Exception as e:
                        debug_url = getattr(e, "debug_url", None)
                        await self._send_homography_state(
                            websocket, ok=False,
                            message=f"pairs calibration failed: {e}",
                            debug_url=debug_url,
                        )
                elif command == "get_homography":
                    await self._send_homography_state(websocket)
                elif command == "goto_latlon" or command == "force_goto_latlon":
                    if len(parts) != 4:
                        await websocket.send(f"Error: {command} requires lat lon alt_m")
                        continue
                    try:
                        lat, lon, alt_m = float(parts[1]), float(parts[2]), float(parts[3])
                        result = await self._do_goto_latlon(lat, lon, alt_m, force=(command == "force_goto_latlon"))
                        await websocket.send(json.dumps({'type': 'goto_result', 'ok': True, 'data': result}))
                    except Exception as e:
                        await websocket.send(json.dumps({'type': 'goto_result', 'ok': False, 'message': str(e)}))
                elif command == "preview_goto_latlon":
                    if len(parts) != 4:
                        await websocket.send("Error: preview_goto_latlon requires lat lon alt_m")
                        continue
                    try:
                        lat, lon, alt_m = float(parts[1]), float(parts[2]), float(parts[3])
                        result = self._preview_goto_latlon(lat, lon, alt_m)
                        await websocket.send(json.dumps({'type': 'goto_preview', 'ok': True, 'data': result}))
                    except Exception as e:
                        await websocket.send(json.dumps({'type': 'goto_preview', 'ok': False, 'message': str(e)}))
                elif command == "get_pointing_state":
                    await self._send_pointing_state(websocket)
                elif command == "home_pointing":
                    self.motor_controller.driver.reset_step_counter()
                    self.motor_controller_tilt.driver.reset_step_counter()
                    await self._send_pointing_state(websocket, message="step counters zeroed")
                elif command == "start_imu_axis_sweep":
                    if len(parts) != 3:
                        await websocket.send("Error: start_imu_axis_sweep requires <pan|tilt> <signed_steps>")
                        continue
                    try:
                        axis = parts[1]
                        signed_steps = int(parts[2])
                        # Run the sweep without blocking the message loop.
                        asyncio.create_task(self._run_imu_axis_sweep(websocket, axis, signed_steps))
                    except Exception as e:
                        await websocket.send(f"Error: {e}")
                elif command == "apply_imu_calibration":
                    if len(parts) != 3:
                        await websocket.send("Error: apply_imu_calibration requires <K_pan> <K_tilt>")
                        continue
                    try:
                        K_pan = float(parts[1])
                        K_tilt = float(parts[2])
                        fit = self.pointing.compute_with_fixed_slopes(K_pan, K_tilt)
                        await self._send_pointing_state(
                            websocket,
                            message=(f"IMU-applied calibration: pan={fit.pan_steps_per_deg:.2f} steps/deg, "
                                     f"tilt={fit.tilt_steps_per_deg:.2f} steps/deg, "
                                     f"offsets locked from {fit.n_references} reference(s), "
                                     f"rms={fit.rms_residual_deg:.3f} deg"),
                        )
                    except Exception as e:
                        await self._send_pointing_state(websocket, ok=False, message=f"apply failed: {e}")
                elif command == "calibrate_backlash":
                    # Form: calibrate_backlash <cam_idx> [trials=5] [max_steps=512]
                    #                          [settle_s=3.0] [engage_steps=150]
                    #                          [min_features=20] [debug=0]
                    #                          [probe_steps=20,40,60,80,100]
                    if len(parts) < 2:
                        await websocket.send("Error: calibrate_backlash requires cam_idx")
                        continue
                    probe_steps = None
                    pitch_probe_steps = None
                    try:
                        cam_idx = int(parts[1])
                        trials = int(parts[2]) if len(parts) > 2 else 5
                        max_steps = int(parts[3]) if len(parts) > 3 else 512
                        settle_s = float(parts[4]) if len(parts) > 4 else 3.0
                        engage_steps = int(parts[5]) if len(parts) > 5 else 150
                        min_features = int(float(parts[6])) if len(parts) > 6 else 20
                        debug = bool(int(parts[7])) if len(parts) > 7 else False
                        if len(parts) > 8 and parts[8].strip():
                            probe_steps = [int(s.strip()) for s in parts[8].split(',')
                                           if s.strip()]
                        # Pitch probes are optional. The UI sends "_" as a
                        # placeholder when the input is empty (otherwise the
                        # space-split on the command line would drop the
                        # positional slot). Treat anything without a digit
                        # as "use yaw probe_steps for pitch too".
                        if len(parts) > 9 and parts[9].strip() and any(c.isdigit() for c in parts[9]):
                            pitch_probe_steps = [int(s.strip()) for s in parts[9].split(',')
                                                 if s.strip() and s.strip().isdigit()]
                    except ValueError as e:
                        await websocket.send(f"Error: calibrate_backlash bad arg: {e}")
                        continue
                    asyncio.create_task(self._run_backlash(
                        websocket, cam_idx, trials, max_steps,
                        settle_s, engage_steps, min_features, debug,
                        probe_steps, pitch_probe_steps,
                    ))
                elif command == "get_backlash_calibration":
                    await websocket.send(json.dumps(self._backlash_state_payload()))
                elif command == "cancel_backlash_calibration":
                    with self._backlash_lock:
                        cal = self._backlash_calibrator
                    if cal is None:
                        await websocket.send(json.dumps({
                            'type': 'backlash_cancel_ack', 'ok': False,
                            'message': 'no calibration running',
                        }))
                    else:
                        cal.cancel_event.set()
                        await websocket.send(json.dumps({
                            'type': 'backlash_cancel_ack', 'ok': True,
                        }))
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
                elif command in ['ev', 'gain', 'shuttercam', 'ae']:
                    # ev <stops> <cam>       — AE compensation (enables AE)
                    # gain <val> <cam>       — manual analogue gain (disables AE)
                    # shuttercam <us> <cam>  — manual shutter for a single cam (disables AE)
                    # ae <on|off> <cam>      — toggle auto-exposure
                    if len(parts) != 3:
                        await websocket.send(f"Error: {command} command requires value and camera number")
                        continue
                    if self.camera_manager is None:
                        await websocket.send(f"Error: {command} unavailable (no CameraManager)")
                        continue
                    try:
                        camera = int(parts[2])
                        if command == 'ev':
                            self.camera_manager.set_ae_compensation(camera, float(parts[1]))
                            await websocket.send(f"ev set to {parts[1]} on cam{camera}")
                        elif command == 'gain':
                            self.camera_manager.set_exposure(camera, gain=float(parts[1]))
                            await websocket.send(f"gain set to {parts[1]} on cam{camera}")
                        elif command == 'shuttercam':
                            self.camera_manager.set_exposure(camera, shutter_us=int(parts[1]))
                            await websocket.send(f"shutter set to {parts[1]}us on cam{camera}")
                        elif command == 'ae':
                            on = parts[1].lower() in ('on', 'true', '1', 'yes')
                            self.camera_manager.set_exposure(camera, ae_enable=on)
                            await websocket.send(f"AE {'on' if on else 'off'} on cam{camera}")
                    except Exception as e:
                        await websocket.send(f"Error: {command} failed: {e!r}")
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
                elif command == "record_start":
                    if self.camera_manager is None:
                        await websocket.send("Error: record unavailable (no CameraManager)")
                    else:
                        try:
                            result = await asyncio.to_thread(self.camera_manager.start_recording)
                            await websocket.send(f"record_start {json.dumps(result)}")
                        except Exception as e:
                            await websocket.send(f"Error: record_start failed: {e!r}")
                elif command == "record_stop":
                    if self.camera_manager is None:
                        await websocket.send("Error: record unavailable (no CameraManager)")
                    else:
                        try:
                            result = await asyncio.to_thread(self.camera_manager.stop_recording)
                            await websocket.send(f"record_stop {json.dumps(result)}")
                        except Exception as e:
                            await websocket.send(f"Error: record_stop failed: {e!r}")
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
                        # shutter <us>  — applies to BOTH cameras for backwards compat
                        if self.camera_manager is None:
                            await websocket.send("Error: shutter unavailable (no CameraManager)")
                        else:
                            try:
                                for cam in (0, 1):
                                    self.camera_manager.set_exposure(cam, shutter_us=argument)
                                await websocket.send(f"shutter set to {argument}us on cam0 and cam1")
                            except Exception as e:
                                await websocket.send(f"Error: shutter failed: {e!r}")
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
        
        # Finalise any in-flight recording so MP4 files stay playable.
        if self.camera_manager is not None:
            try:
                self.camera_manager.stop_recording()
            except Exception as e:
                print(f"Error stopping recordings: {e!r}")

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
        """Capture a still while the stream keeps running (no MediaMTX restart)."""
        if self.camera_manager is None:
            print("trigger_camera: no CameraManager — capture unavailable")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f'cam{idx}_{timestamp}.jpg'
        file_path = f'./captures/{file_name}'
        symlink_path = f'./captures/cam{idx}_last.jpg'

        try:
            self.camera_manager.capture_still(idx, file_path)
        except Exception as e:
            print(f"trigger_camera({idx}): capture failed: {e!r}")
            return

        self.last_photo_path = file_path
        self.last_symlink_path = symlink_path

        if os.path.islink(symlink_path):
            os.remove(symlink_path)
        os.symlink(os.path.basename(file_path), symlink_path)
        print(f"Photo saved to {file_path}")
    

    def handle_shutdown(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print("\nShutdown signal received, cleaning up...")
        if self.camera_manager is not None:
            try:
                self.camera_manager.stop_all()
            except Exception as e:
                print(f"Error stopping CameraManager: {e!r}")
        self.stop_stream()
        sys.exit(0)

