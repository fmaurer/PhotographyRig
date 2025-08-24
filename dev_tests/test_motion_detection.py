#!/usr/bin/env python3
"""
Simple Motion Detection Test
Tests motion detection on RTSP stream without motor control
"""

import cv2
import numpy as np
import time
import argparse

def test_motion_detection(rtsp_url):
    """Test motion detection on RTSP stream"""
    print(f"Connecting to RTSP stream: {rtsp_url}")
    
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("Failed to open RTSP stream")
        return
    
    # Motion detection parameters
    prev_frame = None
    motion_threshold = 25
    min_motion_area = 1000
    
    # FPS tracking
    fps_counter = 0
    fps_start_time = time.time()
    fps = 0
    
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            time.sleep(1)
            continue
        
        # Update FPS
        fps_counter += 1
        if time.time() - fps_start_time >= 1.0:
            fps = fps_counter
            fps_counter = 0
            fps_start_time = time.time()
        
        # Motion detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        if prev_frame is not None:
            # Calculate difference
            frame_delta = cv2.absdiff(prev_frame, gray)
            thresh = cv2.threshold(frame_delta, motion_threshold, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)
            
            # Find contours
            contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Find largest motion area
            largest_contour = None
            max_area = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_motion_area and area > max_area:
                    max_area = area
                    largest_contour = contour
            
            # Draw motion detection
            if largest_contour is not None:
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Draw line to center
                center_x_frame = frame.shape[1] // 2
                center_y_frame = frame.shape[0] // 2
                cv2.line(frame, (center_x_frame, center_y_frame), (center_x, center_y), (255, 0, 0), 2)
                
                print(f"Motion detected at ({center_x}, {center_y}), area: {max_area}")
        
        prev_frame = gray
        
        # Draw center crosshair
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
        
        # Draw FPS
        cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Motion Detection Test', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("Motion detection test completed")

def main():
    parser = argparse.ArgumentParser(description='Test motion detection on RTSP stream')
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam1',
                       help='RTSP stream URL')
    
    args = parser.parse_args()
    
    try:
        test_motion_detection(args.rtsp_url)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 