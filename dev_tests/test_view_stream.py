#!/usr/bin/env python3
"""
Simple RTSP Stream Viewer

A basic example script that demonstrates the RTSPStreamProcessor class
by displaying a live stream with FPS counter and basic controls.

Usage:
    python test_view_stream.py --rtsp-url rtsp://localhost:8554/cam1
    python test_view_stream.py --rtsp-url rtsp://localhost:8554/cam1 --frame-skip 2 --resize 0.5
"""

import cv2
import time
import argparse
from rtsp_stream_processor import RTSPStreamProcessor


class StreamViewer:
    """Simple stream viewer with FPS display and basic controls."""
    
    def __init__(self, rtsp_url, frame_skip=1, resize_factor=1.0, crop_center=False):
        """
        Initialize the stream viewer.
        
        Args:
            rtsp_url (str): RTSP stream URL
            frame_skip (int): Number of frames to skip for performance
            resize_factor (float): Frame resize factor (1.0 = original size)
            crop_center (bool): Whether to center crop to 1280x720
        """
        self.rtsp_processor = RTSPStreamProcessor(
            rtsp_url=rtsp_url,
            frame_skip=frame_skip,
            resize_factor=resize_factor,
            crop_center=crop_center
        )
        
        # FPS tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        # Display settings
        self.window_name = "RTSP Stream Viewer"
        self.show_info = True
        
    def update_fps(self):
        """Update FPS calculation."""
        self.fps_counter += 1
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def draw_overlay(self, frame):
        """Draw FPS and stream information overlay on frame."""
        # Draw FPS counter
        cv2.putText(frame, f"FPS: {self.current_fps}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw stream info
        if self.show_info:
            info = self.rtsp_processor.get_stream_info()
            cv2.putText(frame, f"Resolution: {info['width']}x{info['height']}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Frames: {info['frame_count']}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Connected: {info['is_connected']}", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Draw processing info
            cv2.putText(frame, f"Frame Skip: {self.rtsp_processor.frame_skip}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Resize: {self.rtsp_processor.resize_factor}", (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Crop: {'ON' if self.rtsp_processor.crop_center else 'OFF'}", (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Draw controls help
        cv2.putText(frame, "Press 'q' to quit, 'i' to toggle info, 's' to save frame", (10, frame.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def save_frame(self, frame):
        """Save current frame to file."""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"captured_frame_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Frame saved as: {filename}")
    
    def run(self):
        """Main viewing loop."""
        if not self.rtsp_processor.connect():
            print("Failed to connect to RTSP stream")
            return
        
        print("Starting stream viewer...")
        print("Press 'q' to quit, 'i' to toggle info, 's' to save frame")
        print("Press '1-5' to change frame skip, 'r' to reset frame skip")
        
        # Create window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        try:
            while True:
                # Read frame
                frame = self.rtsp_processor.read_frame()
                if frame is None:
                    print("Failed to read frame, retrying...")
                    time.sleep(0.1)
                    continue
                
                # Update FPS
                self.update_fps()
                
                # Draw overlay
                frame = self.draw_overlay(frame)
                
                # Display frame
                cv2.imshow(self.window_name, frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('i'):
                    self.show_info = not self.show_info
                    print(f"Info display: {'ON' if self.show_info else 'OFF'}")
                elif key == ord('s'):
                    self.save_frame(frame)
                elif key == ord('1'):
                    self.rtsp_processor.frame_skip = 1
                    print(f"Frame skip set to: {self.rtsp_processor.frame_skip}")
                elif key == ord('2'):
                    self.rtsp_processor.frame_skip = 2
                    print(f"Frame skip set to: {self.rtsp_processor.frame_skip}")
                elif key == ord('3'):
                    self.rtsp_processor.frame_skip = 3
                    print(f"Frame skip set to: {self.rtsp_processor.frame_skip}")
                elif key == ord('4'):
                    self.rtsp_processor.frame_skip = 4
                    print(f"Frame skip set to: {self.rtsp_processor.frame_skip}")
                elif key == ord('5'):
                    self.rtsp_processor.frame_skip = 5
                    print(f"Frame skip set to: {self.rtsp_processor.frame_skip}")
                elif key == ord('r'):
                    self.rtsp_processor.frame_skip = 1
                    print(f"Frame skip reset to: {self.rtsp_processor.frame_skip}")
                elif key == ord('c'):
                    # Manually clear frame buffer
                    self.rtsp_processor._clear_frame_buffer()
                    print("Manually cleared frame buffer")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        self.rtsp_processor.close()
        cv2.destroyAllWindows()
        print("Stream viewer closed")


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(description="Simple RTSP Stream Viewer")
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam1',
                       help='RTSP stream URL (default: rtsp://localhost:8554/cam1)')
    parser.add_argument('--frame-skip', type=int, default=1,
                       help='Number of frames to skip for performance (default: 1)')
    parser.add_argument('--resize', type=float, default=1.0,
                       help='Frame resize factor (default: 1.0)')
    parser.add_argument('--crop-center', action='store_true',
                       help='Center crop stream to 1280x720')
    
    args = parser.parse_args()
    
    # Create and run viewer
    viewer = StreamViewer(
        rtsp_url=args.rtsp_url,
        frame_skip=args.frame_skip,
        resize_factor=args.resize,
        crop_center=args.crop_center
    )
    
    try:
        viewer.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        viewer.cleanup()


if __name__ == "__main__":
    main()
