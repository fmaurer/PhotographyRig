#!/usr/bin/env python3
"""
Optimized Blob Detection for Moon Tracking
Uses multiple detection methods for precise center coordinates
"""

import cv2
import numpy as np
import time
import argparse
import threading
import json
from collections import deque

# Try to import websocket, but don't fail if not available
try:
    import websocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    print("Warning: websocket-client not installed. Pan/tilt control will be disabled.")
    print("Install with: pip install websocket-client")
    WEBSOCKET_AVAILABLE = False

class OptimizedBlobDetector:
    def __init__(self, rtsp_url, frame_skip=2, resize_factor=0.5, buffer_size=3, enable_pan_tilt=False):
        self.rtsp_url = rtsp_url
        self.frame_skip = frame_skip
        self.resize_factor = resize_factor
        self.buffer_size = buffer_size
        self.enable_pan_tilt = enable_pan_tilt
        
        # Blob detection parameters - optimized for moon
        self.min_blob_area = 64000  # Minimum area for moon detection
        self.max_blob_area = 80000  # Maximum area for moon detection
        self.min_circularity = 0.2  # Moon should be roughly circular
        self.min_convexity = 0.8    # Moon should be convex
        self.min_inertia_ratio = 0.4  # Moon should be roughly round
        
        # Brightness threshold for moon detection
        self.brightness_threshold = 100  # Moon is typically bright
        
        # Hough Circle parameters
        self.min_radius = 100
        self.max_radius = 1000
        self.hough_param1 = 40
        self.hough_param2 = 30
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        self.frame_count = 0
        
        # Temporal smoothing for stable tracking
        self.prev_center = None
        self.smoothing_factor = 0.7  # Higher = more smoothing (0.0-1.0)
        
        # WebSocket pan/tilt control
        self.ws = None
        self.ws_connected = False
        self.last_pan_command = 0
        self.last_tilt_command = 0
        self.last_command_time = 0
        self.command_rate_limit = 0.5  # Minimum seconds between commands
        
        # Pan/tilt conversion factors (will be adjusted based on resize factor)
        self.pan_steps_per_pixel = 15
        self.tilt_steps_per_pixel = 5
        self.min_pan_steps = 7
        self.min_tilt_steps = 5
        self.max_pan_steps = 300
        self.max_tilt_steps = 150
        
        # Threading for display
        self.display_queue = deque(maxlen=2)
        self.processed_queue = deque(maxlen=2)
        self.running = False
        
        # Setup blob detector
        self.setup_blob_detector()
        
        # Setup WebSocket connection if pan/tilt is enabled
        if self.enable_pan_tilt and WEBSOCKET_AVAILABLE:
            self.setup_websocket()
            # Adjust conversion factors for resize factor
            self.pan_steps_per_pixel = int(15 * self.resize_factor)
            self.tilt_steps_per_pixel = int(5 * self.resize_factor)
            print(f"Adjusted conversion factors: {self.pan_steps_per_pixel} pan steps/pixel, {self.tilt_steps_per_pixel} tilt steps/pixel")
        elif self.enable_pan_tilt and not WEBSOCKET_AVAILABLE:
            print("Warning: Pan/tilt control requested but websocket-client not available")
            self.enable_pan_tilt = False
        
    def setup_blob_detector(self):
        """Setup blob detector with moon-optimized parameters"""
        params = cv2.SimpleBlobDetector_Params()
        
        # Filter by area
        params.filterByArea = True
        params.minArea = self.min_blob_area
        params.maxArea = self.max_blob_area
        
        # Filter by circularity
        params.filterByCircularity = True
        params.minCircularity = self.min_circularity
        params.maxCircularity = 1.0
        
        # Filter by convexity
        params.filterByConvexity = True
        params.minConvexity = self.min_convexity
        params.maxConvexity = 1.0
        
        # Filter by inertia ratio (roundness)
        params.filterByInertia = True
        params.minInertiaRatio = self.min_inertia_ratio
        params.maxInertiaRatio = 1.0
        
        # Filter by color (brightness)
        params.filterByColor = True
        params.blobColor = 255  # White blobs (bright moon)
        
        self.blob_detector = cv2.SimpleBlobDetector_create(params)
        
    def setup_camera(self):
        """Setup camera with optimized parameters"""
        cap = cv2.VideoCapture(self.rtsp_url)
        if not cap.isOpened():
            print("Failed to open RTSP stream")
            return None
            
        # Set buffer size to prevent accumulation
        cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
        
        # Set lower resolution if possible
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        return cap
    
    def setup_websocket(self):
        """Setup WebSocket connection for pan/tilt control"""
        try:
            self.ws = websocket.WebSocketApp(
                "ws://frodo.local:8765",
                on_open=self.on_ws_open,
                on_message=self.on_ws_message,
                on_error=self.on_ws_error,
                on_close=self.on_ws_close
            )
            
            # Start WebSocket connection in a separate thread
            ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            ws_thread.start()
            print("WebSocket connection started")
            
        except Exception as e:
            print(f"Failed to setup WebSocket: {e}")
    
    def on_ws_open(self, ws):
        """WebSocket connection opened"""
        self.ws_connected = True
        print("WebSocket connected")
    
    def on_ws_message(self, ws, message):
        """WebSocket message received"""
        print(f"WebSocket received: {message}")
    
    def on_ws_error(self, ws, error):
        """WebSocket error"""
        print(f"WebSocket error: {error}")
        self.ws_connected = False
    
    def on_ws_close(self, ws, close_status_code, close_msg):
        """WebSocket connection closed"""
        print(f"WebSocket closed: {close_msg}")
        self.ws_connected = False
    
    def send_pan_tilt_command(self, pan_steps, tilt_steps):
        """Send pan/tilt commands via WebSocket with rate limiting"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < self.command_rate_limit:
            return
        
        # Only send if connected and commands are significant
        if not self.ws_connected:
            return
        
        # Check minimum and maximum step requirements
        pan_send = pan_steps if abs(pan_steps) >= self.min_pan_steps else 0
        tilt_send = tilt_steps if abs(tilt_steps) >= self.min_tilt_steps else 0
        
        # Apply maximum limits
        if abs(pan_send) > self.max_pan_steps:
            pan_send = self.max_pan_steps if pan_send > 0 else -self.max_pan_steps
        if abs(tilt_send) > self.max_tilt_steps:
            tilt_send = self.max_tilt_steps if tilt_send > 0 else -self.max_tilt_steps
        
        # Send commands
        if pan_send != 0:
            self.ws.send(f"pan {pan_send}")
            print(f"Sent pan command: {pan_send}")
        
        if tilt_send != 0:
            self.ws.send(f"tilt {tilt_send}")
            print(f"Sent tilt command: {tilt_send}")
        
        self.last_command_time = current_time
    
    def calculate_pan_tilt_adjustment(self, offset_x, offset_y):
        """Calculate pan/tilt adjustments based on offset from center"""
        # Convert pixel offset to steps
        pan_steps = int(-offset_x * self.pan_steps_per_pixel)  # Negative for correct direction
        tilt_steps = int(offset_y * self.tilt_steps_per_pixel)  # Negative for correct direction
        
        # Debug output
        #print(f"Offset: ({offset_x}, {offset_y}) -> Pan: {pan_steps}, Tilt: {tilt_steps}")
        
        return pan_steps, tilt_steps
    
    def detect_moon_blobs(self, gray):
        """Detect moon using blob detection"""
        # Apply brightness threshold to isolate bright objects
        _, thresh = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
        
        # Morphological operations to clean up the image
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Detect blobs
        keypoints = self.blob_detector.detect(thresh)
        
        return keypoints, thresh
    
    def detect_moon_contours(self, gray):
        """Detect moon using contour analysis"""
        # Apply brightness threshold
        _, thresh = cv2.threshold(gray, self.brightness_threshold, 255, cv2.THRESH_BINARY)
        
        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area and circularity
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_blob_area <= area <= self.max_blob_area:
                # Calculate circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity >= self.min_circularity:
                        valid_contours.append(contour)
        
        return valid_contours, thresh
    
    def detect_moon_hough(self, gray):
        """Detect moon using Hough Circle detection"""
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # Detect circles
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        return circles
    
    def smooth_center(self, new_center):
        """Apply temporal smoothing to center coordinates"""
        if new_center is None:
            return None
        
        if self.prev_center is None:
            self.prev_center = new_center
            return new_center
        
        # Simple exponential smoothing
        x = int(self.smoothing_factor * self.prev_center[0] + (1 - self.smoothing_factor) * new_center[0])
        y = int(self.smoothing_factor * self.prev_center[1] + (1 - self.smoothing_factor) * new_center[1])
        
        self.prev_center = (x, y)
        return (x, y)
    
    def process_frame(self, frame):
        """Process frame for moon detection using multiple methods"""
        # Resize frame for faster processing
        if self.resize_factor != 1.0:
            height, width = frame.shape[:2]
            new_width = int(width * self.resize_factor)
            new_height = int(height * self.resize_factor)
            frame = cv2.resize(frame, (new_width, new_height))
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply slight blur to reduce noise
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # Method 1: Blob detection
        blob_keypoints, blob_thresh = self.detect_moon_blobs(gray)
        
        # Method 2: Contour analysis
        contours, contour_thresh = self.detect_moon_contours(gray)
        
        # Method 3: Hough Circle detection
        circles = self.detect_moon_hough(gray)
        
        # Combine results and find best center
        best_center = None
        best_method = None
        best_confidence = 0
        best_area = 0  # Track the area of the detected blob
        
        # Process blob detection results
        if blob_keypoints:
            # Sort by size (largest first)
            blob_keypoints = sorted(blob_keypoints, key=lambda x: x.size, reverse=True)
            best_blob = blob_keypoints[0]
            best_center = (int(best_blob.pt[0]), int(best_blob.pt[1]))
            best_method = "blob"
            best_confidence = best_blob.size / 100  # Normalize confidence
            best_area = int(best_blob.size * best_blob.size * np.pi / 4)  # Approximate area from size
        
        # Process contour results
        if contours:
            # Find the most circular contour
            best_contour = None
            best_circularity = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > best_circularity:
                        best_circularity = circularity
                        best_contour = contour
            
            if best_contour is not None:
                M = cv2.moments(best_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    contour_center = (cx, cy)
                    
                    # Use contour if it's more circular than blob
                    if best_circularity > best_confidence:
                        best_center = contour_center
                        best_method = "contour"
                        best_confidence = best_circularity
                        best_area = int(cv2.contourArea(best_contour))  # Actual contour area
        
        # Process Hough Circle results
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # Use the largest circle
            largest_circle = circles[0, 0]
            hough_center = (largest_circle[0], largest_circle[1])
            hough_radius = largest_circle[2]
            
            # Use Hough if no other method found or if it's more confident
            if best_center is None or hough_radius > best_confidence * 50:
                best_center = hough_center
                best_method = "hough"
                best_confidence = hough_radius / 100
                best_area = int(np.pi * hough_radius * hough_radius)  # Circle area
        
        # Create visualization
        processed_frame = frame.copy()
        processed_gray = gray.copy()
        
        # Apply temporal smoothing to the detected center
        if best_center is not None:
            best_center = self.smooth_center(best_center)
        
        # Draw detection results
        if best_center is not None:
            x, y = best_center
            
            # Ensure coordinates are integers
            x, y = int(x), int(y)
            
            # Draw center point
            cv2.circle(processed_frame, (x, y), 5, (0, 0, 255), -1)
            cv2.circle(processed_gray, (x, y), 5, 255, -1)
            
            # Draw method indicator or offset
            method_colors = {"blob": (0, 255, 0), "contour": (255, 0, 0), "hough": (0, 255, 255)}
            color = method_colors.get(best_method, (255, 255, 255))
            
            # Calculate offset from center (in resized frame coordinates)
            center_x = processed_frame.shape[1] // 2
            center_y = processed_frame.shape[0] // 2
            offset_x = int(x - center_x)
            offset_y = int(y - center_y)
            
            if best_method == "hough":
                # Show offset instead of "HOUGH"
                cv2.putText(processed_frame, f"({offset_x}, {offset_y})", (x+10, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                # Show method name for other methods
                cv2.putText(processed_frame, f"{best_method.upper()}", (x+10, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw line to image center
            cv2.line(processed_frame, (center_x, center_y), (x, y), (255, 0, 0), 2)
            
            # Show offset in status area
            cv2.putText(processed_frame, f"Offset: ({offset_x}, {offset_y})", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add debug info
            cv2.putText(processed_frame, f"Coords: ({x}, {y})", (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(processed_frame, f"Center: ({center_x}, {center_y})", (10, 130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw all detection methods
        # Draw blob keypoints
        for kp in blob_keypoints[:3]:  # Show top 3
            cv2.circle(processed_frame, (int(kp.pt[0]), int(kp.pt[1])), int(kp.size/2), (0, 255, 0), 2)
        
        # Draw contours
        cv2.drawContours(processed_frame, contours, -1, (255, 0, 0), 2)
        
        # Draw Hough circles
        if circles is not None:
            for circle in circles[0, :3]:  # Show top 3
                cv2.circle(processed_frame, (circle[0], circle[1]), circle[2], (0, 255, 255), 2)
        
        return best_center, best_method, best_confidence, best_area, processed_frame, processed_gray, blob_thresh
    
    def display_thread(self):
        """Separate thread for display to prevent blocking"""
        while self.running:
            if self.display_queue and self.processed_queue:
                frame = self.display_queue.popleft()
                processed_frame = self.processed_queue.popleft()
                
                cv2.imshow('Moon Blob Detection', frame)
                cv2.imshow('Processed Frame (Grayscale)', processed_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break
            else:
                time.sleep(0.01)
    
    def run(self):
        """Main processing loop with optimizations"""
        print(f"Connecting to RTSP stream: {self.rtsp_url}")
        print(f"Optimizations: frame_skip={self.frame_skip}, resize={self.resize_factor}, buffer_size={self.buffer_size}")
        print(f"Blob params: min_area={self.min_blob_area}, max_area={self.max_blob_area}")
        print(f"Circularity: {self.min_circularity}, Brightness threshold: {self.brightness_threshold}")
        
        cap = self.setup_camera()
        if cap is None:
            return
        
        self.running = True
        
        # Start display thread
        display_thread = threading.Thread(target=self.display_thread)
        display_thread.start()
        
        print("Press 'q' to quit")
        
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read frame")
                    time.sleep(0.1)
                    continue
                
                self.frame_count += 1
                
                # Skip frames to reduce processing load
                if self.frame_count % self.frame_skip != 0:
                    continue
                
                # Update FPS
                self.fps_counter += 1
                if time.time() - self.fps_start_time >= 1.0:
                    self.fps = self.fps_counter
                    self.fps_counter = 0
                    self.fps_start_time = time.time()
                
                # Process frame
                moon_center, detection_method, confidence, blob_area, processed_frame, processed_gray, threshold_img = self.process_frame(frame)
                
                # Draw center crosshair
                center_x = processed_frame.shape[1] // 2
                center_y = processed_frame.shape[0] // 2
                cv2.line(processed_frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
                cv2.line(processed_frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
                
                # Draw FPS and frame info
                cv2.putText(processed_frame, f"FPS: {self.fps}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(processed_frame, f"Frame: {self.frame_count}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if moon_center:
                    x, y = moon_center
                    # Scale back to original coordinates for output
                    scale = 1.0 / self.resize_factor
                    x_scaled = int(x * scale)
                    y_scaled = int(y * scale)
                    
                    #print(f"Moon detected at ({x_scaled}, {y_scaled}) using {detection_method} method, confidence: {confidence:.2f}, area: {blob_area}")
                    
                    # Calculate offset from center (in resized frame coordinates)
                    center_x = processed_frame.shape[1] // 2
                    center_y = processed_frame.shape[0] // 2
                    offset_x = int(x - center_x)
                    offset_y = int(y - center_y)
                    
                    # Send pan/tilt commands to center the moon if enabled
                    if self.enable_pan_tilt:
                        # Use the same offset values shown on screen for pan/tilt calculation
                        pan_steps, tilt_steps = self.calculate_pan_tilt_adjustment(offset_x, offset_y)
                        self.send_pan_tilt_command(pan_steps, tilt_steps)
                    else:
                        pan_steps, tilt_steps = 0, 0
                    
                    # Draw confidence
                    cv2.putText(processed_frame, f"Confidence: {confidence:.2f}", (10, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # Draw pan/tilt info
                    cv2.putText(processed_frame, f"Pan: {pan_steps}, Tilt: {tilt_steps}", (10, 140), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Add to display queue (replace if full)
                if len(self.display_queue) >= self.display_queue.maxlen:
                    self.display_queue.clear()
                if len(self.processed_queue) >= self.processed_queue.maxlen:
                    self.processed_queue.clear()
                self.display_queue.append(processed_frame)
                self.processed_queue.append(processed_gray)
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.running = False
            cap.release()
            display_thread.join()
            cv2.destroyAllWindows()
            print("Moon blob detection completed")

def main():
    parser = argparse.ArgumentParser(description='Optimized blob detection for moon tracking')
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam0',
                       help='RTSP stream URL')
    parser.add_argument('--frame-skip', type=int, default=2,
                       help='Process every Nth frame (default: 2)')
    parser.add_argument('--resize-factor', type=float, default=0.5,
                       help='Resize factor for processing (default: 0.5)')
    parser.add_argument('--buffer-size', type=int, default=3,
                       help='Camera buffer size (default: 3)')
    parser.add_argument('--min-area', type=int, default=1000,
                       help='Minimum blob area (default: 1000)')
    parser.add_argument('--max-area', type=int, default=50000,
                       help='Maximum blob area (default: 50000)')
    parser.add_argument('--circularity', type=float, default=0.6,
                       help='Minimum circularity (default: 0.6)')
    parser.add_argument('--brightness', type=int, default=100,
                       help='Brightness threshold (default: 100)')
    parser.add_argument('--enable-pan-tilt', action='store_true',
                       help='Enable automatic pan/tilt control to center the moon')
    
    args = parser.parse_args()
    
    detector = OptimizedBlobDetector(
        rtsp_url=args.rtsp_url,
        frame_skip=args.frame_skip,
        resize_factor=args.resize_factor,
        buffer_size=args.buffer_size,
        enable_pan_tilt=args.enable_pan_tilt
    )
    
    # Override default parameters with command line arguments
    detector.min_blob_area = args.min_area
    detector.max_blob_area = args.max_area
    detector.min_circularity = args.circularity
    detector.brightness_threshold = args.brightness
    
    try:
        detector.run()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 