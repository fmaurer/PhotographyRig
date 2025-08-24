#!/usr/bin/env python3
"""
Fast Object Tracking Test Script
Uses RTSP stream from cam0 to test motor responsiveness for object tracking

This script will:
1. Connect to the RTSP stream
2. Detect and track objects (faces, motion, or colored objects)
3. Calculate pan/tilt commands to keep object centered
4. Send commands via WebSocket to test motor responsiveness
"""

import cv2
import numpy as np
import time
import websocket
import json
import threading
from collections import deque
import argparse

class ObjectTracker:
    def __init__(self, rtsp_url, websocket_url="ws://localhost:8765", 
                 tracking_mode="motion", debug=True):
        self.rtsp_url = rtsp_url
        self.websocket_url = websocket_url
        self.tracking_mode = tracking_mode
        self.debug = debug
        
        # Video capture
        self.cap = None
        
        # WebSocket connection
        self.ws = None
        self.ws_connected = False
        
        # Tracking parameters
        self.frame_width = 1280
        self.frame_height = 720
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Motion detection
        self.prev_frame = None
        self.motion_threshold = 25
        self.min_motion_area = 1000
        
        # Face detection
        self.face_cascade = None
        if tracking_mode == "face":
            self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Color tracking
        self.color_lower = np.array([0, 100, 100])  # Red color range
        self.color_upper = np.array([10, 255, 255])
        
        # PID control for smooth tracking
        self.pan_pid = PIDController(kp=0.1, ki=0.01, kd=0.05)
        self.tilt_pid = PIDController(kp=0.1, ki=0.01, kd=0.05)
        
        # Command history for rate limiting
        self.last_command_time = 0
        self.command_interval = 0.1  # Minimum time between commands (seconds)
        
        # Statistics
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        
        # Motor step conversion (adjust based on your setup)
        self.pan_steps_per_pixel = 2.0
        self.tilt_steps_per_pixel = 2.0
        
    def connect_websocket(self):
        """Connect to WebSocket server"""
        try:
            self.ws = websocket.create_connection(self.websocket_url, timeout=5)
            self.ws_connected = True
            print(f"Connected to WebSocket server at {self.websocket_url}")
            return True
        except Exception as e:
            print(f"Failed to connect to WebSocket: {e}")
            return False
    
    def send_motor_command(self, pan_steps, tilt_steps):
        """Send motor command via WebSocket"""
        if not self.ws_connected or self.ws is None:
            return False
            
        current_time = time.time()
        if current_time - self.last_command_time < self.command_interval:
            return False  # Rate limiting
            
        try:
            # Send pan and tilt commands
            if abs(pan_steps) > 5:  # Only send if movement is significant
                self.ws.send(f"pan {int(pan_steps)}")
                if self.debug:
                    print(f"Sent pan command: {int(pan_steps)}")
                    
            if abs(tilt_steps) > 5:  # Only send if movement is significant
                self.ws.send(f"tilt {int(tilt_steps)}")
                if self.debug:
                    print(f"Sent tilt command: {int(tilt_steps)}")
                    
            self.last_command_time = current_time
            return True
        except Exception as e:
            print(f"Error sending motor command: {e}")
            self.ws_connected = False
            return False
    
    def detect_motion(self, frame):
        """Detect motion in the frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        if self.prev_frame is None:
            self.prev_frame = gray
            return None
        
        # Calculate difference between current and previous frame
        frame_delta = cv2.absdiff(self.prev_frame, gray)
        thresh = cv2.threshold(frame_delta, self.motion_threshold, 255, cv2.THRESH_BINARY)[1]
        
        # Dilate to fill in holes
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour (most motion)
        largest_contour = None
        max_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_motion_area and area > max_area:
                max_area = area
                largest_contour = contour
        
        self.prev_frame = gray
        
        if largest_contour is not None:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2
            return (center_x, center_y, w, h)
        
        return None
    
    def detect_faces(self, frame):
        """Detect faces in the frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        
        if len(faces) > 0:
            # Use the largest face
            largest_face = max(faces, key=lambda x: x[2] * x[3])
            x, y, w, h = largest_face
            center_x = x + w // 2
            center_y = y + h // 2
            return (center_x, center_y, w, h)
        
        return None
    
    def detect_color(self, frame):
        """Detect colored objects in the frame"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                return (center_x, center_y, w, h)
        
        return None
    
    def calculate_motor_commands(self, object_center):
        """Calculate pan/tilt motor commands to center the object"""
        if object_center is None:
            return 0, 0
            
        center_x, center_y = object_center[0], object_center[1]
        
        # Calculate error from center
        pan_error = center_x - self.center_x
        tilt_error = center_y - self.center_y
        
        # Use PID controllers for smooth tracking
        pan_output = self.pan_pid.update(pan_error)
        tilt_output = self.tilt_pid.update(tilt_error)
        
        # Convert to motor steps
        pan_steps = int(pan_output * self.pan_steps_per_pixel)
        tilt_steps = int(tilt_output * self.tilt_steps_per_pixel)
        
        return pan_steps, tilt_steps
    
    def draw_debug_info(self, frame, object_center, pan_steps, tilt_steps):
        """Draw debug information on frame"""
        # Draw center crosshair
        cv2.line(frame, (self.center_x - 20, self.center_y), (self.center_x + 20, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), (self.center_x, self.center_y + 20), (0, 255, 0), 2)
        
        # Draw object center if detected
        if object_center is not None:
            center_x, center_y = object_center[0], object_center[1]
            cv2.circle(frame, (center_x, center_y), 10, (0, 0, 255), -1)
            cv2.line(frame, (self.center_x, self.center_y), (center_x, center_y), (255, 0, 0), 2)
        
        # Draw text information
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Mode: {self.tracking_mode}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Pan: {pan_steps}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Tilt: {tilt_steps}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        if self.ws_connected:
            cv2.putText(frame, "WebSocket: Connected", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "WebSocket: Disconnected", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    def run(self):
        """Main tracking loop"""
        print(f"Starting object tracking with mode: {self.tracking_mode}")
        print(f"Connecting to RTSP stream: {self.rtsp_url}")
        
        # Connect to RTSP stream
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            print("Failed to open RTSP stream")
            return
        
        # Connect to WebSocket
        if not self.connect_websocket():
            print("Warning: WebSocket connection failed. Running in simulation mode.")
        
        print("Press 'q' to quit, 'r' to reconnect WebSocket")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to read frame from RTSP stream")
                time.sleep(1)
                continue
            
            # Update FPS counter
            self.fps_counter += 1
            if time.time() - self.fps_start_time >= 1.0:
                self.fps = self.fps_counter
                self.fps_counter = 0
                self.fps_start_time = time.time()
            
            # Detect object based on tracking mode
            object_center = None
            if self.tracking_mode == "motion":
                object_center = self.detect_motion(frame)
            elif self.tracking_mode == "face":
                object_center = self.detect_faces(frame)
            elif self.tracking_mode == "color":
                object_center = self.detect_color(frame)
            
            # Calculate motor commands
            pan_steps, tilt_steps = self.calculate_motor_commands(object_center)
            
            # Send motor commands
            if self.ws_connected:
                self.send_motor_command(pan_steps, tilt_steps)
            
            # Draw debug information
            if self.debug:
                self.draw_debug_info(frame, object_center, pan_steps, tilt_steps)
                cv2.imshow('Object Tracking Test', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                print("Reconnecting to WebSocket...")
                self.connect_websocket()
        
        # Cleanup
        if self.cap:
            self.cap.release()
        if self.ws:
            self.ws.close()
        cv2.destroyAllWindows()
        print("Object tracking test completed")


class PIDController:
    """Simple PID controller for smooth tracking"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error):
        self.integral += error
        derivative = error - self.prev_error
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        return output


def main():
    parser = argparse.ArgumentParser(description='Test object tracking with motor control')
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam0',
                       help='RTSP stream URL')
    parser.add_argument('--websocket-url', default='ws://localhost:8765',
                       help='WebSocket server URL')
    parser.add_argument('--mode', choices=['motion', 'face', 'color'], default='motion',
                       help='Tracking mode')
    parser.add_argument('--no-debug', action='store_true',
                       help='Disable debug display')
    
    args = parser.parse_args()
    
    tracker = ObjectTracker(
        rtsp_url=args.rtsp_url,
        websocket_url=args.websocket_url,
        tracking_mode=args.mode,
        debug=not args.no_debug
    )
    
    try:
        tracker.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main() 