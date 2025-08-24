#!/usr/bin/env python3
"""
WebSocket-based blob follower for RTSP streams with pan/tilt control.
Supports both blob detection and MOSSE tracking methods.

Usage:
  python websocket_blob_follower.py --rtsp-url rtsp://camera:554/stream
  python websocket_blob_follower.py --rtsp-url rtsp://camera:554/stream --detection-method mosse
"""

import cv2
import numpy as np
import threading
import time
import argparse
import json

# Try to import websocket library with fallback
try:
    import websocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    print("Warning: websocket-client library not found. Install with: pip install websocket-client")
    print("WebSocket functionality will be disabled.")
    WEBSOCKET_AVAILABLE = False

class WebSocketPanTiltController:
    """Manages WebSocket connection for pan/tilt control."""
    
    def __init__(self, websocket_url):
        if not WEBSOCKET_AVAILABLE:
            print("WebSocket functionality disabled - library not available")
            self.websocket_url = None
            self.websocket = None
            self.connected = False
            self.lock = threading.Lock()
            self.should_reconnect = False
            return
            
        self.websocket_url = websocket_url
        self.websocket = None
        self.connected = False
        self.lock = threading.Lock()
        self.should_reconnect = True
        self.reconnect_delay = 1.0  # Start with 1 second delay
        self.max_reconnect_delay = 30.0  # Max 30 second delay
        
        # Start connection in background thread
        self.connection_thread = threading.Thread(target=self._connect, daemon=True)
        self.connection_thread.start()
    
    def _connect(self):
        """Establish WebSocket connection in background thread."""
        if not WEBSOCKET_AVAILABLE:
            return
            
        while self.should_reconnect:
            try:
                print(f"Attempting to connect to WebSocket at {self.websocket_url}")
                
                # Try to resolve the hostname first
                try:
                    import socket
                    if self.websocket_url.startswith("ws://"):
                        host = self.websocket_url[5:].split(":")[0]
                        port = int(self.websocket_url[5:].split(":")[1].split("/")[0])
                        socket.gethostbyname(host)
                        print(f"✓ Hostname resolved: {host}:{port}")
                    else:
                        print("ℹ Using direct connection")
                except Exception as e:
                    print(f"⚠ Warning: Could not resolve hostname: {e}")
                
                self.websocket = websocket.create_connection(self.websocket_url, timeout=5)
                
                with self.lock:
                    self.connected = True
                    self.reconnect_delay = 1.0  # Reset delay on successful connection
                
                print(f"Connected to WebSocket at {self.websocket_url}")
                
                # Keep connection alive and monitor health
                while self.connected and self.should_reconnect:
                    try:
                        # Send ping to keep connection alive
                        try:
                            self.websocket.ping()
                        except (BrokenPipeError, ConnectionResetError, OSError) as e:
                            print(f"Connection error during ping: {e}")
                            break
                        
                        # Check if connection is still responsive
                        if hasattr(self.websocket, 'sock') and self.websocket.sock:
                            # Try to get socket info to check if it's still alive
                            try:
                                self.websocket.sock.getpeername()
                            except (OSError, ConnectionError, BrokenPipeError, ConnectionResetError):
                                print("WebSocket connection lost, will reconnect...")
                                break
                        
                        time.sleep(1)
                        
                    except (websocket.WebSocketConnectionClosedException, 
                           websocket.WebSocketBadStatusException,
                           OSError, ConnectionError, BrokenPipeError, ConnectionResetError) as e:
                        print(f"WebSocket connection error: {e}")
                        break
                        
            except websocket.WebSocketConnectionClosedException as e:
                print(f"WebSocket connection closed: {e}")
                with self.lock:
                    self.connected = False
            except websocket.WebSocketBadStatusException as e:
                print(f"WebSocket bad status: {e}")
                with self.lock:
                    self.connected = False
            except websocket.WebSocketTimeoutException as e:
                print(f"WebSocket connection timeout: {e}")
                with self.lock:
                    self.connected = False
            except OSError as e:
                if "Connection refused" in str(e):
                    print(f"Connection refused - server may not be running at {self.websocket_url}")
                elif "No route to host" in str(e):
                    print(f"No route to host - check network connectivity to {self.websocket_url}")
                else:
                    print(f"OS Error during connection: {e}")
                with self.lock:
                    self.connected = False
            except Exception as e:
                print(f"Failed to connect to WebSocket: {e}")
                with self.lock:
                    self.connected = False
            
            # If we get here, connection was lost or failed
            with self.lock:
                self.connected = False
            
            if self.websocket:
                try:
                    self.websocket.close()
                except:
                    pass
                self.websocket = None
            
            # Wait before reconnecting, with exponential backoff
            if self.should_reconnect:
                print(f"Reconnecting in {self.reconnect_delay:.1f} seconds...")
                time.sleep(self.reconnect_delay)
                self.reconnect_delay = min(self.reconnect_delay * 2, self.max_reconnect_delay)
    
    def send_command(self, command):
        """Send pan/tilt command over WebSocket."""
        if not WEBSOCKET_AVAILABLE:
            print(f"WebSocket disabled - would send: {command}")
            return False
            
        if not self.connected or self.websocket is None:
            return False
        
        try:
            with self.lock:
                if self.websocket and self.connected:
                    # Double-check connection is still alive
                    try:
                        if hasattr(self.websocket, 'sock') and self.websocket.sock:
                            self.websocket.sock.getpeername()
                    except (OSError, ConnectionError):
                        print("Connection lost during send, will reconnect...")
                        self.connected = False
                        return False
                    
                    try:
                        self.websocket.send(command)
                        return True
                    except BrokenPipeError:
                        print("Broken pipe error - connection lost, will reconnect...")
                        self.connected = False
                        return False
                    except ConnectionResetError:
                        print("Connection reset error - connection lost, will reconnect...")
                        self.connected = False
                        return False
                else:
                    return False
                    
        except (websocket.WebSocketConnectionClosedException,
               websocket.WebSocketBadStatusException,
               OSError, ConnectionError, BrokenPipeError, ConnectionResetError) as e:
            print(f"Failed to send command '{command}': {e}")
            with self.lock:
                self.connected = False
            return False
        except Exception as e:
            print(f"Unexpected error sending command '{command}': {e}")
            return False
    
    def is_connected(self):
        """Check if WebSocket is currently connected."""
        if not WEBSOCKET_AVAILABLE:
            return False
        with self.lock:
            return self.connected and self.websocket is not None
    
    def close(self):
        """Close WebSocket connection."""
        self.should_reconnect = False
        with self.lock:
            self.connected = False
        
        if self.websocket:
            try:
                self.websocket.close()
            except:
                pass
            self.websocket = None

class RTSPStreamProcessor:
    """Handles RTSP stream connection and frame processing."""
    
    def __init__(self, rtsp_url, frame_skip=2, resize_factor=0.5, buffer_size=3):
        self.rtsp_url = rtsp_url
        self.frame_skip = frame_skip
        self.resize_factor = resize_factor
        self.buffer_size = buffer_size
        self.cap = None
        self.frame_count = 0
        
    def connect(self):
        """Connect to RTSP stream."""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                raise Exception(f"Could not open RTSP stream: {self.rtsp_url}")
            
            # Set buffer size to reduce latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
            print(f"Connected to RTSP stream: {self.rtsp_url}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to RTSP stream: {e}")
            return False
    
    def read_frame(self):
        """Read and process a frame from the stream."""
        if self.cap is None or not self.cap.isOpened():
            return None
        
        # Skip frames for performance
        for _ in range(self.frame_skip):
            self.cap.grab()
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        self.frame_count += 1
        
        # Resize frame if needed
        if self.resize_factor != 1.0:
            frame = cv2.resize(frame, None, 
                             fx=self.resize_factor, fy=self.resize_factor, 
                             interpolation=cv2.INTER_AREA)
        
        return frame
    
    def close(self):
        """Close RTSP connection."""
        if self.cap:
            self.cap.release()

class MoonBlobDetector:
    """Detects moon-like objects using blob detection, contours, and Hough circles."""
    
    def __init__(self, min_blob_area=64000, max_blob_area=80000, 
                 min_circularity=0.2, min_convexity=0.8, 
                 min_inertia_ratio=0.4, brightness_threshold=100):
        self.min_blob_area = min_blob_area
        self.max_blob_area = max_blob_area
        self.min_circularity = min_circularity
        self.min_convexity = min_convexity
        self.min_inertia_ratio = min_inertia_ratio
        self.brightness_threshold = brightness_threshold
        
        # Create blob detector
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = self.min_blob_area
        params.maxArea = self.max_blob_area
        params.minCircularity = self.min_circularity
        params.minConvexity = self.min_convexity
        params.minInertiaRatio = self.min_inertia_ratio
        params.filterByColor = True
        params.blobColor = 255  # White blobs
        
        self.blob_detector = cv2.SimpleBlobDetector_create(params)
    
    def detect(self, frame):
        """Detect moon-like objects using multiple methods."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Method 1: Blob detection
        keypoints = self.blob_detector.detect(gray)
        if keypoints:
            # Get the largest blob
            largest_kp = max(keypoints, key=lambda x: x.size)
            if largest_kp.size >= self.min_blob_area:
                return (int(largest_kp.pt[0]), int(largest_kp.pt[1]), 
                       int(largest_kp.size), int(largest_kp.size))
        
        # Method 2: Contour detection with brightness threshold
        _, thresh = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_blob_area <= area <= self.max_blob_area:
                # Check circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity >= self.min_circularity:
                        if area > best_area:
                            best_area = area
                            best_contour = contour
        
        if best_contour is not None:
            x, y, w, h = cv2.boundingRect(best_contour)
            return (x + w//2, y + h//2, w, h)
        
        # Method 3: Hough circles
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                 param1=50, param2=30, minRadius=100, maxRadius=200)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle[0], circle[1], circle[2]
                area = np.pi * r * r
                if self.min_blob_area <= area <= self.max_blob_area:
                    return (x, y, r*2, r*2)
        
        return None

class MOSSETracker:
    """MOSSE tracker for continuous object tracking."""
    
    def __init__(self):
        self.tracker = None
        self.bbox = None
        self.detection_interval = 5  # Re-detect every N frames
        self.frame_count = 0
        
    def create_tracker(self):
        """Create MOSSE tracker instance."""
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMOSSE_create"):
            return cv2.legacy.TrackerMOSSE_create()
        elif hasattr(cv2, "TrackerMOSSE_create"):
            return cv2.TrackerMOSSE_create()
        else:
            raise RuntimeError("MOSSE tracker not available. Install opencv-contrib-python")
    
    def detect_plane_bbox(self, frame):
        """Detect plane-like objects using the method from test_MOSSE_tracker.py."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Plane tends to be bright and low-saturation (nearly white)
        whiteish = (s < 60) & (v > 170)

        # Kill smooth sky; keep plane edges
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hp = cv2.subtract(gray, cv2.GaussianBlur(gray, (0, 0), 7))
        edges = hp > 2

        mask = (whiteish & edges).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            area = w * h
            if area < 120:
                continue
            ar = max(w, h) / max(1, min(w, h))  # elongation
            if 1.2 <= ar <= 10 and area > best_area:
                best_area = area
                best = (x, y, w, h)
        return best
    
    def detect(self, frame):
        """Detect and track objects using MOSSE tracker."""
        self.frame_count += 1
        
        # Re-detect periodically or if we lost the tracker
        need_detect = (self.frame_count % self.detection_interval == 0) or (self.tracker is None)
        
        if need_detect:
            # Use plane detection method
            bbox = self.detect_plane_bbox(frame)
            if bbox is not None:
                self.tracker = self.create_tracker()
                self.tracker.init(frame, tuple(map(float, bbox)))  # MOSSE expects floats
                self.bbox = bbox
                return bbox
        else:
            # Update tracker
            if self.tracker is not None:
                ok_trk, tb = self.tracker.update(frame)
                if ok_trk:
                    self.bbox = tuple(map(int, tb))
                    return self.bbox
                else:
                    self.tracker = None  # force re-detect next loop
        
        return self.bbox

class WebSocketBlobFollower:
    """Main class for following objects with pan/tilt control."""
    
    def __init__(self, rtsp_url, websocket_url="ws://frodo.local:8765", 
                 frame_skip=2, resize_factor=0.5, buffer_size=3,
                 enable_pan_tilt=True, detection_method="blob"):
        self.rtsp_processor = RTSPStreamProcessor(rtsp_url, frame_skip, resize_factor, buffer_size)
        
        # Initialize detection method
        if detection_method == "mosse":
            self.detector = MOSSETracker()
        else:
            self.detector = MoonBlobDetector()
        
        self.pan_tilt_controller = None
        if enable_pan_tilt:
            self.pan_tilt_controller = WebSocketPanTiltController(websocket_url)
        
        # Pan/tilt control parameters
        self.center_x = None
        self.center_y = None
        self.pan_threshold = 50
        self.tilt_threshold = 50
        self.pan_speed = 1
        self.tilt_speed = 1
        
        # Smoothing for object center
        self.smooth_center = None
        self.smoothing_factor = 0.7
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
    
    def update_fps(self):
        """Update FPS calculation."""
        self.fps_counter += 1
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def smooth_center_point(self, center):
        """Apply exponential smoothing to object center."""
        if self.smooth_center is None:
            self.smooth_center = center
        else:
            self.smooth_center = (
                int(self.smoothing_factor * center[0] + (1 - self.smoothing_factor) * self.smooth_center[0]),
                int(self.smoothing_factor * center[1] + (1 - self.smoothing_factor) * self.smooth_center[1])
            )
        return self.smooth_center
    
    def calculate_pan_tilt(self, object_center, frame_shape):
        """Calculate pan/tilt adjustments based on object position."""
        if self.center_x is None:
            self.center_x = frame_shape[1] // 2
            self.center_y = frame_shape[0] // 2
        
        obj_x, obj_y = object_center
        
        # Calculate offsets from center
        pan_offset = obj_x - self.center_x
        tilt_offset = obj_y - self.center_y
        
        # Only send commands if WebSocket is connected
        if not self.pan_tilt_controller or not self.pan_tilt_controller.is_connected():
            return
        
        # Apply thresholds and send commands
        if abs(pan_offset) > self.pan_threshold:
            # Pan: positive offset means object is right of center, so pan right (positive)
            # Negative offset means object is left of center, so pan left (negative)
            pan_amount = min(abs(pan_offset) // 10, self.pan_speed)
            if pan_offset > 0:  # Object is right of center, pan right
                command = f"pan {pan_amount}"
            else:  # Object is left of center, pan left
                command = f"pan -{pan_amount}"
            
            if not self.pan_tilt_controller.send_command(command):
                print(f"Failed to send pan command: {command}")
        
        if abs(tilt_offset) > self.tilt_threshold:
            # Tilt: positive offset means object is below center, so tilt down (positive)
            # Negative offset means object is above center, so tilt up (negative)
            tilt_amount = min(abs(tilt_offset) // 10, self.tilt_speed)
            if tilt_offset > 0:  # Object is below center, tilt down
                command = f"tilt {tilt_amount}"
            else:  # Object is above center, tilt up
                command = f"tilt -{tilt_amount}"
            
            if not self.pan_tilt_controller.send_command(command):
                print(f"Failed to send tilt command: {command}")
    
    def run(self):
        """Main processing loop."""
        if not self.rtsp_processor.connect():
            print("Failed to connect to RTSP stream")
            return
        
        print("Starting object tracking...")
        print("Press 'q' to quit, 'p' to pause")
        
        paused = False
        
        try:
            while True:
                if not paused:
                    frame = self.rtsp_processor.read_frame()
                    if frame is None:
                        print("Failed to read frame, retrying...")
                        time.sleep(0.1)
                        continue
                    
                    # Detect object
                    detection_result = self.detector.detect(frame)
                    
                    if detection_result is not None:
                        if len(detection_result) == 4:  # x, y, w, h
                            x, y, w, h = detection_result
                            center = (x + w//2, y + h//2)
                        else:  # x, y
                            center = detection_result
                        
                        # Smooth the center point
                        smooth_center = self.smooth_center_point(center)
                        
                        # Draw detection overlay
                        if len(detection_result) == 4:
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(frame, smooth_center, 5, (0, 0, 255), -1)
                        
                        # Calculate and send pan/tilt commands
                        self.calculate_pan_tilt(smooth_center, frame.shape)
                    
                    # Update FPS
                    self.update_fps()
                    
                    # Draw FPS overlay
                    cv2.putText(frame, f"FPS: {self.current_fps}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Draw connection status
                    if self.pan_tilt_controller:
                        status = "Connected" if self.pan_tilt_controller.is_connected() else "Disconnected"
                        color = (0, 255, 0) if self.pan_tilt_controller.is_connected() else (0, 0, 255)
                        cv2.putText(frame, f"WebSocket: {status}", (10, 70),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    
                    # Show frame
                    cv2.imshow("Object Tracking", frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('p'):
                    paused = not paused
                    print("Paused" if paused else "Resumed")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        if self.pan_tilt_controller:
            self.pan_tilt_controller.close()
        self.rtsp_processor.close()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="WebSocket-based object follower for RTSP streams")
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam0',
                       help='RTSP stream URL (default: rtsp://localhost:8554/cam0)')
    parser.add_argument('--websocket-url', default="ws://frodo.local:8765",
                       help='WebSocket server URL (default: ws://frodo.local:8765)')
    parser.add_argument('--frame-skip', type=int, default=2,
                       help='Number of frames to skip for performance (default: 2)')
    parser.add_argument('--resize-factor', type=float, default=0.5,
                       help='Frame resize factor for performance (default: 0.5)')
    parser.add_argument('--buffer-size', type=int, default=3,
                       help='Camera buffer size (default: 3)')
    parser.add_argument('--detection-method', choices=['blob', 'mosse'], default='blob',
                       help='Detection method: blob (default) or mosse tracker')
    parser.add_argument('--disable-pan-tilt', action='store_true',
                       help='Disable automatic pan/tilt control')
    
    args = parser.parse_args()
    
    # Check WebSocket availability
    if not WEBSOCKET_AVAILABLE and not args.disable_pan_tilt:
        print("Warning: WebSocket library not available. Pan/tilt control will be disabled.")
        args.disable_pan_tilt = True
    
    follower = WebSocketBlobFollower(
        rtsp_url=args.rtsp_url,
        websocket_url=args.websocket_url,
        frame_skip=args.frame_skip,
        resize_factor=args.resize_factor,
        buffer_size=args.buffer_size,
        enable_pan_tilt=not args.disable_pan_tilt,
        detection_method=args.detection_method
    )
    
    try:
        follower.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        follower.cleanup()

if __name__ == "__main__":
    main() 