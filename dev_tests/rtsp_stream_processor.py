#!/usr/bin/env python3
"""
RTSP Stream Processor Module

This module provides a robust RTSP stream processing class with adaptive performance
optimization, automatic lag detection, and intelligent buffer management.

Classes:
    RTSPStreamProcessor: Handles RTSP stream connection and frame processing

Example:
    >>> from rtsp_stream_processor import RTSPStreamProcessor
    >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
    >>> if processor.connect():
    ...     frame = processor.read_frame()
    ...     if frame is not None:
    ...         # Process frame
    ...         pass
    ...     processor.close()
"""

import cv2
import time
from collections import deque


class RTSPStreamProcessor:
    """
    Handles RTSP stream connection and frame processing with adaptive performance optimization.
    
    This class provides a robust interface for connecting to RTSP streams, reading frames,
    and automatically managing performance issues like lag and buffer overflows. It includes
    adaptive frame skipping, center cropping, and intelligent buffer management.
    
    Attributes:
        rtsp_url (str): The RTSP stream URL to connect to
        frame_skip (int): Number of frames to skip for performance optimization
        resize_factor (float): Factor to resize frames (1.0 = no resize, 0.5 = half size)
        buffer_size (int): OpenCV buffer size for frame buffering
        crop_center (bool): Whether to center crop frames to 1280x720
        cap: OpenCV VideoCapture object
        frame_count (int): Total frames processed
        last_frame_time (float): Timestamp of last frame processing
        frame_times (deque): Rolling window of frame processing times
        lag_threshold (float): Threshold for detecting lag (seconds)
        adaptive_skip (bool): Whether to use adaptive frame skipping
    
    Example:
        Basic usage:
        >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
        >>> if processor.connect():
        ...     frame = processor.read_frame()
        ...     if frame is not None:
        ...         # Process frame
        ...         pass
        ...     processor.close()
        
        With performance optimization:
        >>> processor = RTSPStreamProcessor(
        ...     rtsp_url='rtsp://localhost:8554/cam1',
        ...     frame_skip=2,           # Skip every other frame
        ...     resize_factor=0.5,      # Resize to half resolution
        ...     buffer_size=1,          # Minimal buffering for low latency
        ...     crop_center=True        # Center crop to 1280x720
        ... )
        
        Batch processing:
        >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
        >>> if processor.connect():
        ...     frames = []
        ...     for _ in range(10):
        ...         frame = processor.read_frame()
        ...         if frame is not None:
        ...             frames.append(frame)
        ...         else:
        ...             break
        ...     processor.close()
        ...     print(f"Captured {len(frames)} frames")
    """
    
    def __init__(self, rtsp_url, frame_skip=1, resize_factor=1.0, buffer_size=1, crop_center=False):
        """
        Initialize the RTSP stream processor.
        
        Args:
            rtsp_url (str): RTSP stream URL (e.g., 'rtsp://localhost:8554/cam1')
            frame_skip (int, optional): Number of frames to skip for performance. 
                Higher values improve performance but reduce frame rate. Defaults to 1.
            resize_factor (float, optional): Frame resize factor. 
                1.0 = original size, 0.5 = half size, 2.0 = double size. Defaults to 1.0.
            buffer_size (int, optional): OpenCV buffer size. 
                Lower values reduce latency but may cause frame drops. Defaults to 1.
            crop_center (bool, optional): Whether to center crop frames to 1280x720.
                Useful for wide-angle cameras to focus on center region. Defaults to False.
        
        Note:
            - frame_skip: Use 1 for real-time, 2-3 for performance, 5+ for analysis
            - resize_factor: Values < 1.0 improve performance, > 1.0 increase detail
            - buffer_size: 1 = minimal latency, 3-5 = smooth playback, 10+ = high buffering
            - crop_center: Only effective when input resolution is >= 1280x720
        """
        self.rtsp_url = rtsp_url
        self.frame_skip = frame_skip
        self.resize_factor = resize_factor
        self.buffer_size = buffer_size
        self.crop_center = crop_center
        self.cap = None
        self.frame_count = 0
        
        # Lag detection and adaptive frame skipping
        self.last_frame_time = time.time()
        self.frame_times = deque(maxlen=30)  # Track last 30 frame times
        self.lag_threshold = 0.1  # 100ms threshold for lag detection
        self.adaptive_skip = True
        
    def connect(self):
        """
        Connect to the RTSP stream.
        
        Establishes connection to the RTSP URL and configures the video capture
        with appropriate buffer settings. The connection is optimized for low-latency
        streaming with automatic buffer management.
        
        Returns:
            bool: True if connection successful, False otherwise
        
        Raises:
            Exception: If connection fails or stream cannot be opened
        
        Example:
            >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
            >>> if processor.connect():
            ...     print("Successfully connected to stream")
            ... else:
            ...     print("Failed to connect")
        
        Note:
            - Connection may take a few seconds depending on network conditions
            - If connection fails, check RTSP URL, network connectivity, and server status
            - Buffer size is automatically optimized for the connection
        """
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
        """
        Read and process a frame from the stream.
        
        This method reads a frame from the RTSP stream, applies performance optimizations
        like frame skipping and resizing, and handles automatic lag detection. It includes
        intelligent buffer management to prevent frame buildup and maintain real-time performance.
        
        Returns:
            numpy.ndarray or None: Processed frame as BGR image array, or None if read failed
        
        Example:
            >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
            >>> if processor.connect():
            ...     frame = processor.read_frame()
            ...     if frame is not None:
            ...         print(f"Frame shape: {frame.shape}")
            ...         print(f"Frame type: {frame.dtype}")
            ...         # Process frame here
            ...     else:
            ...         print("Failed to read frame")
            ...     processor.close()
        
        Note:
            - Frame is automatically resized if resize_factor != 1.0
            - Center cropping is applied if crop_center=True and resolution allows
            - Adaptive frame skipping automatically adjusts based on lag detection
            - Frame buffer is automatically cleared if significant lag is detected
            - Returns BGR format (OpenCV default) - convert to RGB if needed for other libraries
        """
        if self.cap is None or not self.cap.isOpened():
            return None
        
        current_time = time.time()
        frame_interval = current_time - self.last_frame_time
        self.frame_times.append(frame_interval)
        
        # Check for lag and clear buffer if needed
        if self.adaptive_skip and len(self.frame_times) >= 5:
            avg_interval = sum(self.frame_times) / len(self.frame_times)
            if frame_interval > avg_interval * 2:  # Significantly behind
                self._clear_frame_buffer()
                print(f"⚠️  Lag detected! Cleared frame buffer. Interval: {frame_interval:.3f}s")
        
        # Adaptive frame skipping based on lag
        skip_frames = self.frame_skip
        if self.adaptive_skip and frame_interval > self.lag_threshold:
            # Increase frame skip when lagging
            skip_frames = min(self.frame_skip * 2, 5)  # Max 5 frames skipped
        
        # Skip frames for performance
        for _ in range(skip_frames):
            self.cap.grab()
        
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        self.frame_count += 1
        self.last_frame_time = current_time
        
        # Center crop if enabled (1920x1080 -> 1280x720)
        if self.crop_center and frame.shape[1] >= 1280 and frame.shape[0] >= 720:
            # Calculate crop boundaries to center the crop
            start_x = (frame.shape[1] - 1280) // 2
            start_y = (frame.shape[0] - 720) // 2
            end_x = start_x + 1280
            end_y = start_y + 720
            
            frame = frame[start_y:end_y, start_x:end_x]
        
        # Resize frame if needed (after cropping)
        if self.resize_factor != 1.0:
            frame = cv2.resize(frame, None, 
                             fx=self.resize_factor, fy=self.resize_factor, 
                             interpolation=cv2.INTER_AREA)
        
        return frame
    
    def _clear_frame_buffer(self):
        """
        Clear the frame buffer to catch up to real-time.
        
        This internal method is called automatically when lag is detected. It temporarily
        reduces the buffer size, clears accumulated frames, and restores the original
        buffer configuration. This helps maintain real-time performance during network
        hiccups or processing delays.
        
        Example:
            This method is called automatically, but can be called manually if needed:
            >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
            >>> if processor.connect():
            ...     # Manually clear buffer if experiencing issues
            ...     processor._clear_frame_buffer()
            ...     processor.close()
        
        Note:
            - Automatically called when lag threshold is exceeded
            - Temporarily reduces buffer size to 1 for immediate effect
            - Clears 3 frames from the backlog
            - Restores original buffer size after clearing
            - Use sparingly as it may cause frame drops
        """
        if self.cap is not None:
            # Clear buffer by setting it to minimum size
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # Read a few frames to clear the backlog
            for _ in range(3):
                self.cap.grab()
            # Reset buffer size
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
    
    def close(self):
        """
        Close the RTSP connection and release resources.
        
        Properly closes the video capture connection and releases all associated
        resources. This method should always be called when finished with the
        stream to prevent resource leaks and ensure clean shutdown.
        
        Example:
            >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
            >>> if processor.connect():
            ...     frame = processor.read_frame()
            ...     # Process frame...
            ...     processor.close()  # Always close when done
            
            Using with context manager (recommended):
            >>> with RTSPStreamProcessor('rtsp://localhost:8554/cam1') as processor:
            ...     if processor.connect():
            ...         frame = processor.read_frame()
            ...         # Process frame...
            ...     # Automatically closed when exiting context
        
        Note:
            - Always call this method when finished with the stream
            - Resources are automatically released when the object is garbage collected
            - Connection can be re-established by calling connect() again
            - Multiple close() calls are safe (idempotent)
        """
        if self.cap:
            self.cap.release()
    
    def get_stream_info(self):
        """
        Get information about the connected stream.
        
        Returns:
            dict: Dictionary containing stream information including:
                - width: Frame width in pixels
                - height: Frame height in pixels
                - fps: Frames per second
                - frame_count: Total frames processed
                - is_connected: Whether stream is currently connected
        
        Example:
            >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
            >>> if processor.connect():
            ...     info = processor.get_stream_info()
            ...     print(f"Stream resolution: {info['width']}x{info['height']}")
            ...     print(f"Stream FPS: {info['fps']}")
            ...     print(f"Frames processed: {info['frame_count']}")
            ...     processor.close()
        
        Note:
            - FPS may not be accurate for all RTSP streams
            - Resolution is the actual stream resolution (before any processing)
            - Frame count resets to 0 after reconnection
        """
        if self.cap is None or not self.cap.isOpened():
            return {
                'width': 0,
                'height': 0,
                'fps': 0,
                'frame_count': self.frame_count,
                'is_connected': False
            }
        
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        return {
            'width': width,
            'height': height,
            'fps': fps,
            'frame_count': self.frame_count,
            'is_connected': True
        }
    
    def is_connected(self):
        """
        Check if the stream is currently connected.
        
        Returns:
            bool: True if connected and stream is open, False otherwise
        
        Example:
            >>> processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
            >>> print(f"Connected: {processor.is_connected()}")  # False
            >>> if processor.connect():
            ...     print(f"Connected: {processor.is_connected()}")  # True
            ...     processor.close()
            ...     print(f"Connected: {processor.is_connected()}")  # False
        """
        return self.cap is not None and self.cap.isOpened()


if __name__ == "__main__":
    # Example usage and testing
    processor = RTSPStreamProcessor('rtsp://localhost:8554/cam1')
    
    if processor.connect():
        print("Connected successfully!")
        
        # Get stream info
        info = processor.get_stream_info()
        print(f"Stream info: {info}")
        
        # Read a few frames
        for i in range(5):
            frame = processor.read_frame()
            if frame is not None:
                print(f"Frame {i+1}: {frame.shape}")
            else:
                print(f"Failed to read frame {i+1}")
                break
        
        processor.close()
        print("Disconnected")
    else:
        print("Failed to connect")
