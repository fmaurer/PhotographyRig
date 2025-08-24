#!/usr/bin/env python3
"""
Optimized Motion Detection Test for Raspberry Pi
Addresses performance issues that cause video delay
"""

import cv2
import numpy as np
import time
import argparse
import threading
from collections import deque

class OptimizedMotionDetector:
    def __init__(self, rtsp_url, frame_skip=2, resize_factor=0.5, buffer_size=3):
        self.rtsp_url = rtsp_url
        self.frame_skip = frame_skip  # Process every Nth frame
        self.resize_factor = resize_factor  # Resize frames for faster processing
        self.buffer_size = buffer_size  # Limit buffer size
        
        # Motion detection parameters - optimized for small objects
        self.prev_frame = None
        self.motion_threshold = 15  # Lowered from 25 for more sensitive detection
        self.min_motion_area = 50   # Reduced from 500 for small objects
        self.max_motion_area = 5000 # Added to filter out very large areas
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        self.frame_count = 0
        
        # Threading for display
        self.display_queue = deque(maxlen=2)  # Only keep latest frame for display
        self.processed_queue = deque(maxlen=2)  # Queue for processed grayscale frames
        self.threshold_queue = deque(maxlen=2)  # Queue for threshold images
        self.running = False
        
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
    
    def process_frame(self, frame):
        """Process frame for motion detection"""
        # Resize frame for faster processing
        if self.resize_factor != 1.0:
            height, width = frame.shape[:2]
            new_width = int(width * self.resize_factor)
            new_height = int(height * self.resize_factor)
            frame = cv2.resize(frame, (new_width, new_height))
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Use smaller kernel for better small object detection
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        
        motion_detected = False
        motion_center = None
        motion_area = 0
        processed_gray = gray.copy()  # Keep a copy for display
        threshold_img = None  # Will store the threshold image
        
        if self.prev_frame is not None:
            # Calculate difference
            frame_delta = cv2.absdiff(self.prev_frame, gray)
            thresh = cv2.threshold(frame_delta, self.motion_threshold, 255, cv2.THRESH_BINARY)[1]
            threshold_img = thresh.copy()  # Store for display
            thresh = cv2.dilate(thresh, None, iterations=2)  # Increased for better connectivity
            
            # Find contours
            contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Track multiple motion areas
            motion_objects = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if self.min_motion_area <= area <= self.max_motion_area:
                    x, y, w, h = cv2.boundingRect(contour)
                    center = (x + w // 2, y + h // 2)
                    motion_objects.append({
                        'contour': contour,
                        'area': area,
                        'center': center,
                        'bbox': (x, y, w, h)
                    })
            
            # Sort by area (largest first) and take top 3
            motion_objects.sort(key=lambda obj: obj['area'], reverse=True)
            motion_objects = motion_objects[:3]
            
            if motion_objects:
                motion_detected = True
                # Use the largest object as primary
                primary_obj = motion_objects[0]
                motion_center = primary_obj['center']
                motion_area = primary_obj['area']
                
                # Draw all detected motion objects
                for i, obj in enumerate(motion_objects):
                    x, y, w, h = obj['bbox']
                    center = obj['center']
                    
                    # Different colors for different objects
                    color = (255, 255, 255) if i == 0 else (200, 200, 200) if i == 1 else (150, 150, 150)
                    
                    cv2.rectangle(processed_gray, (x, y), (x + w, y + h), color, 2)
                    cv2.circle(processed_gray, center, 3, color, -1)
                    
                    # Add area label
                    cv2.putText(processed_gray, f"{obj['area']}", (x, y-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        self.prev_frame = gray
        return motion_detected, motion_center, motion_area, frame, processed_gray, threshold_img
    
    def display_thread(self):
        """Separate thread for display to prevent blocking"""
        while self.running:
            if self.display_queue and self.processed_queue and self.threshold_queue:
                frame = self.display_queue.popleft()
                processed_frame = self.processed_queue.popleft()
                threshold_frame = self.threshold_queue.popleft()
                
                cv2.imshow('Motion Detection Test', frame)
                cv2.imshow('Processed Frame (Grayscale)', processed_frame)
                cv2.imshow('Motion Threshold', threshold_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break
            else:
                time.sleep(0.01)  # Small delay to prevent busy waiting
    
    def run(self):
        """Main processing loop with optimizations"""
        print(f"Connecting to RTSP stream: {self.rtsp_url}")
        print(f"Optimizations: frame_skip={self.frame_skip}, resize={self.resize_factor}, buffer_size={self.buffer_size}")
        
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
                motion_detected, motion_center, motion_area, processed_frame, processed_gray, threshold_img = self.process_frame(frame)
                
                # Draw motion detection
                if motion_detected and motion_center:
                    x, y = motion_center
                    # Scale back to original coordinates for display
                    scale = 1.0 / self.resize_factor
                    x_scaled = int(x * scale)
                    y_scaled = int(y * scale)
                    
                    # Draw bounding box (approximate)
                    box_size = 50
                    cv2.rectangle(processed_frame, 
                                (x - box_size//2, y - box_size//2), 
                                (x + box_size//2, y + box_size//2), 
                                (0, 255, 0), 2)
                    cv2.circle(processed_frame, (x, y), 5, (0, 0, 255), -1)
                    
                    # Draw line to center
                    center_x = processed_frame.shape[1] // 2
                    center_y = processed_frame.shape[0] // 2
                    cv2.line(processed_frame, (center_x, center_y), (x, y), (255, 0, 0), 2)
                    
                    print(f"Motion detected at ({x_scaled}, {y_scaled}), area: {motion_area}")
                
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
                
                # Add to display queue (replace if full)
                if len(self.display_queue) >= self.display_queue.maxlen:
                    self.display_queue.clear()
                if len(self.processed_queue) >= self.processed_queue.maxlen:
                    self.processed_queue.clear()
                if len(self.threshold_queue) >= self.threshold_queue.maxlen:
                    self.threshold_queue.clear()
                self.display_queue.append(processed_frame)
                self.processed_queue.append(processed_gray)
                self.threshold_queue.append(threshold_img if threshold_img is not None else np.zeros_like(processed_gray))
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.running = False
            cap.release()
            display_thread.join()
            cv2.destroyAllWindows()
            print("Motion detection test completed")

def main():
    parser = argparse.ArgumentParser(description='Optimized motion detection test for Raspberry Pi')
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam0',
                       help='RTSP stream URL')
    parser.add_argument('--frame-skip', type=int, default=2,
                       help='Process every Nth frame (default: 2)')
    parser.add_argument('--resize-factor', type=float, default=0.5,
                       help='Resize factor for processing (default: 0.5)')
    parser.add_argument('--buffer-size', type=int, default=3,
                       help='Camera buffer size (default: 3)')
    parser.add_argument('--motion-threshold', type=int, default=15,
                       help='Motion detection threshold (default: 15)')
    parser.add_argument('--min-area', type=int, default=50,
                       help='Minimum motion area (default: 50)')
    parser.add_argument('--max-area', type=int, default=5000,
                       help='Maximum motion area (default: 5000)')
    
    args = parser.parse_args()
    
    detector = OptimizedMotionDetector(
        rtsp_url=args.rtsp_url,
        frame_skip=args.frame_skip,
        resize_factor=args.resize_factor,
        buffer_size=args.buffer_size
    )
    
    # Override default parameters with command line arguments
    detector.motion_threshold = args.motion_threshold
    detector.min_motion_area = args.min_area
    detector.max_motion_area = args.max_area
    
    try:
        detector.run()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 