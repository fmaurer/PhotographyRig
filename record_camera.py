#!/usr/bin/env python3
"""
RTSP Stream Recorder
Records RTSP stream to video files with timestamp-based naming
"""

import cv2
import time
import argparse
import os
from datetime import datetime

def record_rtsp_stream(rtsp_url, output_dir="recordings", duration=None, fps=30, show_display=False):
    """Record RTSP stream to video file"""
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Connecting to RTSP stream: {rtsp_url}")
    cap = cv2.VideoCapture(rtsp_url)
    
    if not cap.isOpened():
        print("Failed to open RTSP stream")
        return
    
    # Get stream properties
    stream_fps = cap.get(cv2.CAP_PROP_FPS)
    if stream_fps <= 0:
        stream_fps = fps  # Use default if can't detect
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"Stream properties: {width}x{height} @ {stream_fps} FPS")
    
    # Generate output filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = os.path.join(output_dir, f"recording_{timestamp}.mp4")
    
    # Initialize video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_filename, fourcc, stream_fps, (width, height))
    
    if not out.isOpened():
        print("Failed to create video writer")
        cap.release()
        return
    
    print(f"Recording to: {output_filename}")
    if show_display:
        print("Press 'q' to stop recording")
    else:
        print("Press Ctrl+C to stop recording")
    
    frame_count = 0
    start_time = time.time()
    last_status_time = start_time
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame")
                time.sleep(0.1)
                continue
            
            # Write frame to video file
            out.write(frame)
            frame_count += 1
            
            # Display recording info
            elapsed_time = time.time() - start_time
            current_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
            
            # Show status every 5 seconds when running headless
            if not show_display and time.time() - last_status_time >= 5.0:
                print(f"Recording... Time: {elapsed_time:.1f}s, Frames: {frame_count}, FPS: {current_fps:.1f}")
                last_status_time = time.time()
            
            # Create display frame with info overlay (only if show_display is True)
            if show_display:
                display_frame = frame.copy()
                cv2.putText(display_frame, f"Recording: {output_filename}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Time: {elapsed_time:.1f}s", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Frames: {frame_count}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, f"FPS: {current_fps:.1f}", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('RTSP Recorder', display_frame)
                
                # Check for quit key (only when display is shown)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            if duration and elapsed_time >= duration:
                print(f"Recording duration ({duration}s) reached")
                break
                
    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
    
    finally:
        # Cleanup
        cap.release()
        out.release()
        if show_display:
            cv2.destroyAllWindows()
        
        # Print recording summary
        total_time = time.time() - start_time
        print(f"\nRecording completed:")
        print(f"File: {output_filename}")
        print(f"Duration: {total_time:.2f} seconds")
        print(f"Frames: {frame_count}")
        print(f"Average FPS: {frame_count/total_time:.2f}")

def main():
    parser = argparse.ArgumentParser(description='Record RTSP stream to video file')
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam1',
                       help='RTSP stream URL')
    parser.add_argument('--output-dir', default='recordings',
                       help='Output directory for recordings')
    parser.add_argument('--duration', type=float,
                       help='Recording duration in seconds (optional)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Target FPS for recording')
    parser.add_argument('--show-display', action='store_true',
                       help='Show debug window with live preview')
    
    args = parser.parse_args()
    
    try:
        record_rtsp_stream(args.rtsp_url, args.output_dir, args.duration, args.fps, args.show_display)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main() 