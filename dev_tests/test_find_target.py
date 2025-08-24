#!/usr/bin/env python3
"""
Tiny Object Detector for Wide-Angle Cam1 Stream
Detects very small changes and progressively filters potential targets by tracing their paths.

Usage:
  python test_find_target.py --rtsp-url rtsp://localhost:8554/cam1
  python test_find_target.py --rtsp-url rtsp://localhost:8554/cam1 --sensitivity high
"""

import cv2
import numpy as np
import threading
import time
import argparse
import json
from collections import deque, defaultdict
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import math

@dataclass
class Target:
    """Represents a detected target with tracking information."""
    id: int
    center: Tuple[int, int]
    size: Tuple[int, int]
    confidence: float
    first_seen: float
    last_seen: float
    path: deque
    velocity: Tuple[float, float]
    classification: str
    frame_count: int
    predicted_path: List[Tuple[int, int]] = None

class TinyObjectDetector:
    """Detects tiny objects using frame differencing and motion analysis."""
    
    def __init__(self, sensitivity='medium', min_area=1, max_area=5000):
        self.sensitivity = sensitivity
        self.min_area = min_area
        self.max_area = max_area
        
        # Frame differencing parameters
        self.prev_frame = None
        self.background_model = None
        self.learning_rate = 0.005  # Reduced from 0.01 - slower background adaptation
        
        # Motion detection parameters
        self.motion_threshold = self._get_sensitivity_threshold()
        self.noise_reduction = 1  # Reduced to preserve tiny blobs
        
        # Enhanced motion filtering for slow-growing objects
        self.min_motion_intensity = 0.05  # Reduced from 0.15 - minimum motion intensity (0.0-1.0)
        self.motion_growth_threshold = 0.03  # Reduced from 0.1 - threshold for detecting slow growth
        
        # Cloud filtering parameters
        self.min_velocity_threshold = 2.0  # Minimum pixels per frame to be considered moving
        self.cloud_filter_enabled = True
        
        # Target tracking
        self.next_target_id = 1
        self.targets: Dict[int, Target] = {}
        self.target_history: Dict[int, deque] = defaultdict(lambda: deque(maxlen=100))
        
        # Classification parameters
        self.classification_thresholds = {
            'high': {'confidence': 0.8, 'path_length': 10, 'velocity': 2.0},
            'medium': {'confidence': 0.6, 'path_length': 7, 'velocity': 1.5},
            'low': {'confidence': 0.4, 'path_length': 5, 'velocity': 1.0}
        }
    
    def _get_sensitivity_threshold(self):
        """Get motion threshold based on sensitivity level."""
        thresholds = {
            'low': 8,         # Reduced from 20
            'medium': 5,      # Reduced from 12
            'medium-high': 3, # Reduced from 8
            'high': 2         # Reduced from 5
        }
        return thresholds.get(self.sensitivity, 5)
    
    def detect_motion(self, frame):
        """Detect motion using frame differencing and background subtraction."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)  # Reduced blur to preserve tiny details
        
        # Initialize background model
        if self.background_model is None:
            self.background_model = gray.astype(np.float32)
        
        # Update background model
        cv2.accumulateWeighted(gray, self.background_model, self.learning_rate)
        
        # Calculate difference from background
        diff = cv2.absdiff(gray, self.background_model.astype(np.uint8))
        
        # Apply threshold
        _, thresh = cv2.threshold(diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
        
        # Filter out slow-growing objects by checking motion intensity
        thresh = self._filter_slow_growth(thresh, diff)
        
        # Minimal noise reduction to preserve tiny blobs
        if self.noise_reduction > 1:
            kernel = np.ones((self.noise_reduction, self.noise_reduction), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        return thresh
    
    def _filter_slow_growth(self, thresh, diff):
        """Filter out slow-growing objects by analyzing motion intensity patterns."""
        # Create a copy of the threshold image
        filtered_thresh = thresh.copy()
        
        # Find contours in the threshold image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate motion intensity in this region
            roi = diff[y:y+h, x:x+w]
            if roi.size == 0:
                continue
            
            # Calculate average motion intensity (0.0 to 1.0)
            motion_intensity = np.mean(roi) / 255.0
            
            # Calculate contour area
            area = cv2.contourArea(contour)
            
            # Filter out objects with low motion intensity (slow-growing)
            if motion_intensity < self.min_motion_intensity:
                # Fill this contour with black (remove it)
                cv2.fillPoly(filtered_thresh, [contour], 0)
                continue
            
            # Additional check: filter out large objects with low motion intensity
            # (these are likely slow-growing clouds or shadows)
            if area > 100 and motion_intensity < self.motion_growth_threshold:
                cv2.fillPoly(filtered_thresh, [contour], 0)
                continue
        
        return filtered_thresh
    
    def find_contours(self, motion_mask):
        """Find contours in motion mask and filter by size."""
        contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area <= area <= self.max_area:
                # Additional filtering for tiny objects
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = max(w, h) / max(1, min(w, h))
                
                # Filter out very elongated objects (likely noise)
                if aspect_ratio < 5.0:
                    # Additional cloud filtering (if enabled)
                    if not self.cloud_filter_enabled or (not self._is_likely_cloud(contour, area, w, h) and not self._is_slow_moving(contour, motion_mask)):
                        valid_contours.append(contour)
        
        return valid_contours
    
    def _is_likely_cloud(self, contour, area, width, height):
        """Filter out slow-moving cloud-like objects."""
        # Large, slow-moving objects are likely clouds
        if area > 1000:  # Large area threshold
            return True
        
        # Very wide or tall objects (cloud edges)
        if width > 200 or height > 200:
            return True
        
        # Check if contour is very irregular (cloud-like)
        perimeter = cv2.arcLength(contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            # Very low circularity indicates irregular cloud shapes
            if circularity < 0.1:
                return True
        
        return False
    
    def _has_consistent_velocity(self, contour, motion_mask):
        """Check if object has consistent velocity pattern (not erratic)."""
        # This will be called from the tracker after we have velocity data
        return True  # Placeholder - actual logic in tracker
    
    def _is_slow_moving(self, contour, motion_mask):
        """Check if object is moving slowly (likely a cloud)."""
        x, y, w, h = cv2.boundingRect(contour)
        
        # Get motion intensity in the contour region
        mask_roi = motion_mask[y:y+h, x:x+w]
        if mask_roi.size == 0:
            return True
        
        # Calculate motion intensity
        motion_intensity = np.mean(mask_roi) / 255.0
        
        # Low motion intensity indicates slow movement
        if motion_intensity < 0.3:  # Adjustable threshold
            return True
        
        return False
    
    def calculate_confidence(self, contour, motion_mask):
        """Calculate confidence score for a detected contour."""
        # Area-based confidence (boosted for tiny objects)
        area = cv2.contourArea(contour)
        if area <= 10:  # Boost confidence for very small objects
            area_confidence = 0.8  # High confidence for tiny blobs
        else:
            area_confidence = min(area / self.max_area, 1.0)
        
        # Shape confidence (prefer more compact shapes)
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = max(w, h) / max(1, min(w, h))
        shape_confidence = max(0, 1.0 - (aspect_ratio - 1.0) / 4.0)
        
        # Motion intensity confidence
        mask_roi = motion_mask[y:y+h, x:x+w]
        motion_intensity = np.mean(mask_roi) / 255.0
        motion_confidence = motion_intensity
        
        # Combined confidence (boosted for tiny objects)
        if area <= 10:
            confidence = (area_confidence * 0.4 + 
                         shape_confidence * 0.2 + 
                         motion_confidence * 0.4)  # More weight on area and motion
        else:
            confidence = (area_confidence * 0.3 + 
                         shape_confidence * 0.3 + 
                         motion_confidence * 0.4)
        
        return confidence

class TargetTracker:
    """Tracks detected targets across frames."""
    
    def __init__(self, max_distance=100, max_disappeared=30):
        self.max_distance = max_distance
        self.max_disappeared = max_disappeared
        self.targets: Dict[int, Target] = {}
        self.disappeared: Dict[int, int] = {}
        self.next_id = 1
        
        # Velocity and area consistency filtering
        self.min_velocity_threshold = 3.0   # Changed from 10.0 to 3.0 - minimum pixels per frame
        self.max_velocity_threshold = 25.0  # Changed from 50.0 to 25.0 - maximum pixels per frame
        self.area_change_threshold = 0.6    # Back to 0.6 - more permissive area change limit
        self.velocity_consistency_threshold = 0.4  # Back to 0.4 - more permissive velocity consistency
    
    def update(self, detections: List[Tuple[Tuple[int, int], Tuple[int, int], float]]):
        """Update target tracking with new detections."""
        current_time = time.time()
        
        # If no targets exist, create new ones
        if len(self.targets) == 0:
            for center, size, confidence in detections:
                self._create_target(center, size, confidence, current_time)
            return
        
        # Match detections to existing targets
        matched_targets = set()
        matched_detections = set()
        
        for target_id, target in self.targets.items():
            if target_id in matched_targets:
                continue
                
            best_match = None
            best_distance = float('inf')
            
            for i, (center, size, confidence) in enumerate(detections):
                if i in matched_detections:
                    continue
                
                distance = self._calculate_distance(target.center, center)
                if distance <= self.max_distance and distance < best_distance:
                    best_distance = distance
                    best_match = (i, center, size, confidence)
            
            if best_match is not None:
                idx, center, size, confidence = best_match
                self._update_target(target_id, center, size, confidence, current_time)
                matched_targets.add(target_id)
                matched_detections.add(idx)
        
        # Create new targets for unmatched detections
        for i, (center, size, confidence) in enumerate(detections):
            if i not in matched_detections:
                self._create_target(center, size, confidence, current_time)
        
        # Update disappeared count for unmatched targets
        for target_id in self.targets:
            if target_id not in matched_targets:
                self.disappeared[target_id] = self.disappeared.get(target_id, 0) + 1
        
        # Remove targets that have disappeared for too long
        self._cleanup_disappeared_targets()
    
    def _create_target(self, center, size, confidence, current_time):
        """Create a new target."""
        # Check if new target meets basic criteria
        if not self._passes_new_target_filters(center, size):
            return
        
        target = Target(
            id=self.next_id,
            center=center,
            size=size,
            confidence=confidence,
            first_seen=current_time,
            last_seen=current_time,
            path=deque([center], maxlen=100),
            velocity=(0.0, 0.0),
            classification="unknown",
            frame_count=1,
            predicted_path=None
        )
        
        self.targets[self.next_id] = target
        self.disappeared[self.next_id] = 0
        self.next_id += 1
    
    def _update_target(self, target_id, center, size, confidence, current_time):
        """Update an existing target."""
        target = self.targets[target_id]
        
        # Calculate velocity
        if len(target.path) > 0:
            prev_center = target.path[-1]
            dx = center[0] - prev_center[0]
            dy = center[1] - prev_center[1]
            velocity = (dx, dy)
        else:
            velocity = (0.0, 0.0)
        
        # Check velocity and area consistency
        if not self._passes_consistency_filters(target, center, size, velocity):
            # Mark target for removal
            self.disappeared[target_id] = self.max_disappeared
            return
        
        # Update target
        target.center = center
        target.size = size
        target.confidence = confidence
        target.last_seen = current_time
        target.path.append(center)
        target.velocity = velocity
        target.frame_count += 1
        
        # Reset disappeared count
        self.disappeared[target_id] = 0
    
    def _calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    def _cleanup_disappeared_targets(self):
        """Remove targets that have disappeared for too long."""
        to_remove = []
        for target_id, count in self.disappeared.items():
            if count >= self.max_disappeared:
                to_remove.append(target_id)
        
        for target_id in to_remove:
            del self.targets[target_id]
            del self.disappeared[target_id]
    
    def _passes_consistency_filters(self, target, new_center, new_size, new_velocity):
        """Check if target update passes velocity and area consistency filters."""
        # Calculate current speed
        speed = math.sqrt(new_velocity[0]**2 + new_velocity[1]**2)
        
        # Check velocity thresholds
        if speed < self.min_velocity_threshold or speed > self.max_velocity_threshold:
            return False
        
        # Check area consistency (if we have previous size data)
        if target.size != (0, 0):
            old_area = target.size[0] * target.size[1]
            new_area = new_size[0] * new_size[1]
            
            if old_area > 0:
                area_change_ratio = abs(new_area - old_area) / old_area
                if area_change_ratio > self.area_change_threshold:
                    return False
        
        # Check velocity consistency over time
        if len(target.path) >= 3:
            # Calculate velocity variance over last few frames
            recent_velocities = []
            for i in range(1, min(4, len(target.path))):
                if i < len(target.path):
                    prev = target.path[-(i+1)]
                    curr = target.path[-i]
                    dx = curr[0] - prev[0]
                    dy = curr[1] - prev[1]
                    recent_velocities.append(math.sqrt(dx*dx + dy*dy))
            
            if len(recent_velocities) >= 2:
                mean_velocity = sum(recent_velocities) / len(recent_velocities)
                variance = sum((v - mean_velocity) ** 2 for v in recent_velocities) / len(recent_velocities)
                std_dev = math.sqrt(variance)
                
                # Check if velocity is consistent (low variance)
                if mean_velocity > 0:
                    velocity_cv = std_dev / mean_velocity  # Coefficient of variation
                    if velocity_cv > self.velocity_consistency_threshold:
                        return False
        
        return True
    
    def _passes_new_target_filters(self, center, size):
        """Check if new target meets basic creation criteria."""
        # Check area size (avoid very small or very large objects)
        area = size[0] * size[1]
        if area < 10 or area > 2000:  # Adjust these thresholds as needed
            return False
        
        # Check aspect ratio (avoid very elongated objects)
        aspect_ratio = max(size[0], size[1]) / max(1, min(size[0], size[1]))
        if aspect_ratio > 4.0:  # Max 4:1 aspect ratio
            return False
        
        return True

class TargetClassifier:
    """Classifies targets based on their behavior patterns."""
    
    def __init__(self):
        self.classification_rules = {
            'linear_mover': {
                'min_path_length': 10,
                'max_velocity': 20.0,
                'min_confidence': 0.5,
                'pattern': 'linear',
                'max_direction_changes': 0.2  # Max 20% direction changes
            },
            'curved_mover': {
                'min_path_length': 12,
                'max_velocity': 15.0,
                'min_confidence': 0.5,
                'pattern': 'curved',
                'max_direction_changes': 0.4  # Max 40% direction changes
            }
        }
    
    def calculate_predictive_path(self, target: Target, prediction_frames: int = 30) -> List[Tuple[int, int]]:
        """Calculate predictive path for linear/curved movers."""
        if len(target.path) < 3:
            return []
        
        # Get recent velocity trend
        recent_points = list(target.path)[-5:]  # Last 5 points
        if len(recent_points) < 3:
            return []
        
        # Calculate average velocity from recent movement
        velocities = []
        for i in range(1, len(recent_points)):
            dx = recent_points[i][0] - recent_points[i-1][0]
            dy = recent_points[i][1] - recent_points[i-1][1]
            velocities.append((dx, dy))
        
        if not velocities:
            return []
        
        # Average velocity
        avg_dx = sum(v[0] for v in velocities) / len(velocities)
        avg_dy = sum(v[1] for v in velocities) / len(velocities)
        
        # Current position
        current_x, current_y = target.center
        
        # Generate predicted path
        predicted_path = []
        for i in range(1, prediction_frames + 1):
            pred_x = int(current_x + avg_dx * i)
            pred_y = int(current_y + avg_dy * i)
            predicted_path.append((pred_x, pred_y))
        
        return predicted_path
    
    def classify_target(self, target: Target) -> str:
        """Classify a target based on its characteristics."""
        if len(target.path) < 5:
            return "unknown"
        
        # Calculate path characteristics
        path_length = len(target.path)
        avg_velocity = self._calculate_average_velocity(target)
        path_pattern = self._analyze_path_pattern(target.path)
        direction_change_ratio = self._calculate_direction_change_ratio(target.path)
        
        # Filter out hovering/erratic objects
        if path_pattern in ['hovering', 'erratic'] or direction_change_ratio > 0.6:
            return "filtered_out"
        
        # Additional check for erratic movement patterns
        if self._is_velocity_inconsistent(target) or self._has_rapid_direction_changes(target.path):
            return "filtered_out"
        
        # Apply classification rules
        for class_name, rules in self.classification_rules.items():
            if (path_length >= rules['min_path_length'] and
                avg_velocity <= rules['max_velocity'] and
                target.confidence >= rules['min_confidence'] and
                path_pattern == rules['pattern'] and
                direction_change_ratio <= rules['max_direction_changes']):
                return class_name
        
        return "unknown"
    
    def _calculate_average_velocity(self, target: Target) -> float:
        """Calculate average velocity over the target's path."""
        if len(target.path) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(target.path)):
            prev = target.path[i-1]
            curr = target.path[i]
            distance = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
            total_distance += distance
        
        return total_distance / (len(target.path) - 1)
    
    def _analyze_path_pattern(self, path: deque) -> str:
        """Analyze the pattern of the target's path."""
        if len(path) < 5:
            return "unknown"
        
        # Calculate direction changes using the same threshold as direction_change_ratio
        direction_changes = 0
        for i in range(2, len(path)):
            prev = path[i-2]
            curr = path[i-1]
            next_point = path[i]
            
            # Calculate angles
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_point[1] - curr[1], next_point[0] - curr[0])
            
            # Normalize angles
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff > math.pi / 6:  # 30 degrees (same as direction_change_ratio)
                direction_changes += 1
        
        # Classify pattern with stricter thresholds
        change_ratio = direction_changes / max(1, len(path) - 2)
        
        if change_ratio < 0.15:  # Stricter threshold for linear
            return "linear"
        elif change_ratio < 0.35:  # Stricter threshold for curved
            return "curved"
        elif change_ratio < 0.5:  # Lower threshold for hovering
            return "hovering"
        else:
            return "erratic"
    
    def _calculate_direction_change_ratio(self, path: deque) -> float:
        """Calculate the ratio of direction changes in the path."""
        if len(path) < 3:
            return 0.0
        
        direction_changes = 0
        for i in range(2, len(path)):
            prev = path[i-2]
            curr = path[i-1]
            next_point = path[i]
            
            # Calculate angles
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_point[1] - curr[1], next_point[0] - curr[0])
            
            # Normalize angles
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff > math.pi / 6:  # 30 degrees (more sensitive)
                direction_changes += 1
        
        return direction_changes / max(1, len(path) - 2)
    
    def _is_velocity_inconsistent(self, target: Target) -> bool:
        """Check if target has inconsistent velocity patterns."""
        if len(target.path) < 6:
            return False
        
        # Calculate velocity for each segment
        velocities = []
        for i in range(1, len(target.path)):
            prev = target.path[i-1]
            curr = target.path[i]
            velocity = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
            velocities.append(velocity)
        
        if len(velocities) < 3:
            return False
        
        # Check for high variance in velocity (erratic movement)
        mean_velocity = sum(velocities) / len(velocities)
        variance = sum((v - mean_velocity) ** 2 for v in velocities) / len(velocities)
        std_dev = math.sqrt(variance)
        
        # If standard deviation is more than 50% of mean, consider it inconsistent
        return std_dev > (mean_velocity * 0.5) if mean_velocity > 0 else False
    
    def _has_rapid_direction_changes(self, path: deque) -> bool:
        """Check for rapid, consecutive direction changes."""
        if len(path) < 4:
            return False
        
        rapid_changes = 0
        for i in range(2, len(path) - 1):
            # Check three consecutive points for rapid changes
            prev = path[i-1]
            curr = path[i]
            next_point = path[i+1]
            
            # Calculate angles
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_point[1] - curr[1], next_point[0] - curr[0])
            
            # Normalize angles
            angle_diff = abs(angle2 - angle1)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Count rapid changes (more than 60 degrees)
            if angle_diff > math.pi / 3:  # 60 degrees
                rapid_changes += 1
                # If we have 2 or more rapid changes in a short path, it's erratic
                if rapid_changes >= 2:
                    return True
        
        return False

from rtsp_stream_processor import RTSPStreamProcessor

class TinyObjectFinder:
    """Main class for finding and tracking tiny objects."""
    
    def __init__(self, rtsp_url, sensitivity='medium', frame_skip=1, resize_factor=1.0, disable_cloud_filter=True, reset_interval=5, full_reset=False, crop_center=False):
        self.rtsp_processor = RTSPStreamProcessor(rtsp_url, frame_skip, resize_factor, crop_center=crop_center)
        self.detector = TinyObjectDetector(sensitivity)
        self.detector.cloud_filter_enabled = not disable_cloud_filter
        self.tracker = TargetTracker()
        self.classifier = TargetClassifier()
        
        # Display settings
        self.show_paths = True
        self.show_labels = True
        self.show_confidence = True
        self.show_debug = False  # Debug mode to show all contours
        
        # Reset settings
        self.reset_interval = reset_interval  # Reset every X seconds
        self.full_reset = full_reset  # Whether to do full or smart reset
        self.last_reset_time = time.time()
        self.reset_count = 0
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        # Statistics
        self.total_targets_detected = 0
        self.current_targets = 0
    
    def update_fps(self):
        """Update FPS calculation."""
        self.fps_counter += 1
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def process_frame(self, frame):
        """Process a single frame to detect and track targets."""
        # Detect motion
        motion_mask = self.detector.detect_motion(frame)
        
        # Find contours
        contours = self.detector.find_contours(motion_mask)
        
        # Extract detection information
        detections = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w//2, y + h//2)
            size = (w, h)
            confidence = self.detector.calculate_confidence(contour, motion_mask)
            
            detections.append((center, size, confidence))
        
        # Update tracking
        self.tracker.update(detections)
        
        # Classify targets and calculate predicted paths
        for target in self.tracker.targets.values():
            if target.classification == "unknown":
                target.classification = self.classifier.classify_target(target)
            
            # Calculate predicted path for valid movers
            if target.classification in ['linear_mover', 'curved_mover']:
                target.predicted_path = self.classifier.calculate_predictive_path(target)
        
        # Update statistics
        self.current_targets = len(self.tracker.targets)
        self.total_targets_detected = max(self.total_targets_detected, self.current_targets)
        
        return motion_mask, detections
    
    def check_and_reset(self):
        """Check if it's time to reset detection and tracking."""
        current_time = time.time()
        if current_time - self.last_reset_time >= self.reset_interval:
            self._reset_detection()
            self.last_reset_time = current_time
            self.reset_count += 1
    
    def _reset_detection(self):
        """Reset detection with configurable behavior."""
        if self.full_reset:
            print(f"🔄 Full reset #{self.reset_count} - clearing all targets")
            
            # Reset detector background model
            self.detector.background_model = None
            
            # Clear all tracked targets
            self.tracker.targets.clear()
            self.tracker.disappeared.clear()
            self.tracker.next_id = 1
            
            # Reset statistics
            self.current_targets = 0
            
            print("✅ Full reset complete - fresh start for all targets")
        else:
            print(f"🔄 Smart reset #{self.reset_count} - preserving good targets")
            
            # Reset detector background model
            self.detector.background_model = None
            
            # Smart target filtering: keep only legitimate linear/curved movers
            targets_to_keep = {}
            targets_removed = 0
            
            for target_id, target in self.tracker.targets.items():
                # Re-evaluate classification to catch misclassified erratic movers
                if target.classification in ['linear_mover', 'curved_mover']:
                    # Force re-classification to catch any that became erratic
                    target.classification = self.classifier.classify_target(target)
                
                # Keep targets that are still classified as legitimate movers
                if target.classification in ['linear_mover', 'curved_mover']:
                    # Additional checks: ensure they're still moving recently and consistently
                    if (time.time() - target.last_seen < 2.0 and  # Seen in last 2 seconds
                        self._is_movement_consistent(target) and  # Check for erratic jumping
                        self._has_reasonable_acceleration(target)):  # Check acceleration
                        targets_to_keep[target_id] = target
                    else:
                        targets_removed += 1
                        print(f"🚫 Removed erratic target ID:{target.id} - inconsistent movement or acceleration")
                else:
                    targets_removed += 1
            
            # Update tracker with preserved targets
            self.tracker.targets = targets_to_keep
            self.tracker.disappeared = {tid: 0 for tid in targets_to_keep.keys()}
            
            # Update statistics
            self.current_targets = len(targets_to_keep)
            
            print(f"✅ Smart reset complete - kept {len(targets_to_keep)} good targets, removed {targets_removed} false positives")
    
    def _is_movement_consistent(self, target: Target) -> bool:
        """Check if target movement is consistent (not jumping randomly)."""
        if len(target.path) < 4:
            return True  # Not enough data to judge
        
        # Calculate distances between consecutive points
        distances = []
        for i in range(1, len(target.path)):
            prev = target.path[i-1]
            curr = target.path[i]
            distance = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
            distances.append(distance)
        
        if len(distances) < 3:
            return True
        
        # Check for sudden large jumps (erratic movement)
        mean_distance = sum(distances) / len(distances)
        for distance in distances:
            # If any single movement is more than 3x the average, it's erratic
            if distance > mean_distance * 3.0:
                return False
        
        return True
    
    def _has_reasonable_acceleration(self, target: Target) -> bool:
        """Check if target has reasonable acceleration (not teleporting)."""
        if len(target.path) < 3:
            return True  # Not enough data to judge
        
        # Calculate velocities between consecutive points
        velocities = []
        for i in range(1, len(target.path)):
            prev = target.path[i-1]
            curr = target.path[i]
            velocity = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
            velocities.append(velocity)
        
        if len(velocities) < 2:
            return True
        
        # Calculate acceleration (change in velocity)
        accelerations = []
        for i in range(1, len(velocities)):
            accel = abs(velocities[i] - velocities[i-1])
            accelerations.append(accel)
        
        if not accelerations:
            return True
        
        # Check for unreasonable acceleration spikes
        mean_accel = sum(accelerations) / len(accelerations)
        for accel in accelerations:
            # If acceleration is more than 5x the mean, it's unreasonable
            if accel > mean_accel * 5.0:
                return False
        
        return True
    
    def draw_overlay(self, frame, motion_mask, detections):
        """Draw detection and tracking overlay on frame."""
        # Draw motion mask (semi-transparent)
        motion_colored = cv2.cvtColor(motion_mask, cv2.COLOR_GRAY2BGR)
        motion_colored[motion_mask > 0] = [0, 255, 255]  # Yellow for motion
        
        # Blend motion mask with frame
        alpha = 0.3
        frame = cv2.addWeighted(frame, 1-alpha, motion_colored, alpha, 0)
        
        # Get top 5 highest confidence targets (excluding filtered out ones)
        valid_targets = [t for t in self.tracker.targets.values() 
                        if t.classification not in ['filtered_out', 'unknown']]
        sorted_targets = sorted(valid_targets, 
                               key=lambda t: t.confidence, reverse=True)[:5]
        
        # NEW: Show fastest moving contours with size info
        if len(valid_targets) > 0:
            # Sort by velocity (speed)
            fastest_targets = sorted(valid_targets, 
                                    key=lambda t: math.sqrt(t.velocity[0]**2 + t.velocity[1]**2), 
                                    reverse=True)[:3]
            
            # Draw contours for fastest objects with size labels
            for i, target in enumerate(fastest_targets):
                # Get the actual contour for this target
                x, y = target.center
                w, h = target.size
                area = w * h
                speed = math.sqrt(target.velocity[0]**2 + target.velocity[1]**2)
                
                # Draw contour outline in bright cyan
                x1, y1 = x - w//2, y - h//2
                x2, y2 = x + w//2, y + h//2
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 3)  # Bright cyan
                
                # Draw size and speed info
                size_text = f"Fast{i+1}: {w}x{h}={area}px, {speed:.1f}px/f"
                cv2.putText(frame, size_text, (x1, y1-25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Draw center point
                cv2.circle(frame, target.center, 5, (255, 255, 0), -1)
        
        # Draw all detected contours for debugging (small red dots)
        if hasattr(self, 'show_debug') and self.show_debug:
            contours = self.detector.find_contours(motion_mask)
            for contour in contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m10"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 1, (0, 0, 255), -1)  # Red dot for all contours
        
        # NEW: Show raw motion mask contours (before filtering) for debugging
        if hasattr(self, 'show_debug') and self.show_debug:
            # Get raw contours from motion mask before filtering
            raw_contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in raw_contours:
                area = cv2.contourArea(contour)
                if area > 5:  # Only show contours above noise threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    # Draw small blue rectangles for raw motion
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 1)  # Blue for raw motion
                    
                    # Show area and motion intensity info
                    roi = motion_mask[y:y+h, x:x+w]
                    if roi.size > 0:
                        motion_intensity = np.mean(roi) / 255.0
                        cv2.putText(frame, f"{area:.0f}px", (x, y-5), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
        
        # Draw targets and paths (only top 5 valid ones)
        for target in sorted_targets:
            # Draw bounding box
            x, y = target.center
            w, h = target.size
            x1, y1 = x - w//2, y - h//2
            x2, y2 = x + w//2, y + h//2
            
            # Color based on classification
            color = self._get_target_color(target.classification)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            cv2.circle(frame, target.center, 3, color, -1)
            
            # Draw path
            if self.show_paths and len(target.path) > 1:
                path_points = list(target.path)
                for i in range(1, len(path_points)):
                    cv2.line(frame, path_points[i-1], path_points[i], color, 1)
            
            # Draw predicted path
            if target.predicted_path and len(target.predicted_path) > 1:
                # Draw predicted path in dashed style
                for i in range(1, len(target.predicted_path)):
                    if i % 3 == 0:  # Skip every 3rd point for dashed effect
                        continue
                    cv2.line(frame, target.predicted_path[i-1], target.predicted_path[i], 
                            (255, 255, 255), 1, cv2.LINE_AA)  # White dashed line
            
            # Draw label
            if self.show_labels:
                label = f"ID:{target.id} {target.classification}"
                cv2.putText(frame, label, (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Draw confidence
            if self.show_confidence:
                conf_text = f"{target.confidence:.2f}"
                cv2.putText(frame, conf_text, (x1, y2+15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Draw velocity and area info for debugging
            speed = math.sqrt(target.velocity[0]**2 + target.velocity[1]**2)
            area = target.size[0] * target.size[1]
            vel_text = f"V:{speed:.1f} A:{area}"
            cv2.putText(frame, vel_text, (x1, y2+35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw statistics
        cv2.putText(frame, f"FPS: {self.current_fps}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Targets: {self.current_targets}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Total: {self.total_targets_detected}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"Showing: Top 5", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Show reset countdown
        time_until_reset = self.reset_interval - (time.time() - self.last_reset_time)
        cv2.putText(frame, f"Reset in: {time_until_reset:.1f}s", (10, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"Resets: {self.reset_count}", (10, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Show crop status
        if self.rtsp_processor.crop_center:
            cv2.putText(frame, "CROP: 1920x1080 -> 1280x720", (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Add contour size statistics to the info display
        if len(valid_targets) > 0:
            areas = [t.size[0] * t.size[1] for t in valid_targets]
            min_area = min(areas)
            max_area = max(areas)
            avg_area = sum(areas) / len(areas)
            
            cv2.putText(frame, f"Area Range: {min_area}-{max_area}px", (10, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Avg Area: {avg_area:.0f}px", (10, 270),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Show filtering criteria
        cv2.putText(frame, f"Filters: V:{self.tracker.min_velocity_threshold}-{self.tracker.max_velocity_threshold}px/f", (10, 300),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Area: {self.tracker.area_change_threshold*100:.0f}% max change", (10, 320),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Velocity CV: {self.tracker.velocity_consistency_threshold*100:.0f}% max", (10, 340),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Show motion filtering parameters
        cv2.putText(frame, f"Motion: {self.detector.min_motion_intensity*100:.0f}% min intensity", (10, 360),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Growth: {self.detector.motion_growth_threshold*100:.0f}% threshold", (10, 380),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, f"Threshold: {self.detector.motion_threshold} (sensitivity: {self.detector.sensitivity})", (10, 400),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Show lag status and frame timing
        if len(self.rtsp_processor.frame_times) >= 3:
            avg_interval = sum(self.rtsp_processor.frame_times) / len(self.rtsp_processor.frame_times)
            current_interval = self.rtsp_processor.frame_times[-1] if self.rtsp_processor.frame_times else 0
            
            # Color based on lag status
            lag_color = (0, 255, 0) if current_interval <= avg_interval * 1.5 else (0, 255, 255) if current_interval <= avg_interval * 2 else (0, 0, 255)
            
            cv2.putText(frame, f"Frame Interval: {current_interval:.3f}s", (10, 420),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, lag_color, 1)
            cv2.putText(frame, f"Avg Interval: {avg_interval:.3f}s", (10, 440),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, lag_color, 1)
            
            # Show lag warning if needed
            if current_interval > avg_interval * 2:
                cv2.putText(frame, "LAG DETECTED!", (10, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return frame
    
    def _get_target_color(self, classification):
        """Get color for target classification."""
        colors = {
            'linear_mover': (0, 255, 0),      # Green
            'curved_mover': (255, 255, 0),    # Cyan
            'unknown': (128, 128, 128),       # Gray
            'filtered_out': (64, 64, 64)      # Dark gray
        }
        return colors.get(classification, (128, 128, 128))
    
    def run(self):
        """Main processing loop."""
        if not self.rtsp_processor.connect():
            print("Failed to connect to RTSP stream")
            return
        
        print("Starting tiny object detection...")
        print("Press 'q' to quit, 'p' to pause")
        print("Press '1-4' to toggle display options")
        print("Press '5' to toggle debug mode")
        print("Press '6' to manually clear frame buffer")
        print("Press '7' to toggle adaptive frame skipping")
        print("Press '8' to toggle all filtering (emergency override)")
        
        paused = False
        
        try:
            while True:
                if not paused:
                    frame = self.rtsp_processor.read_frame()
                    if frame is None:
                        print("Failed to read frame, retrying...")
                        time.sleep(0.1)
                        continue
                    
                    # Process frame
                    motion_mask, detections = self.process_frame(frame)
                    
                    # Check if it's time to reset
                    self.check_and_reset()
                    
                    # Draw overlay
                    frame = self.draw_overlay(frame, motion_mask, detections)
                    
                    # Update FPS
                    self.update_fps()
                    
                    # Show frame
                    cv2.imshow("Tiny Object Detection", frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('p'):
                    paused = not paused
                    print("Paused" if paused else "Resumed")
                elif key == ord('1'):
                    self.show_paths = not self.show_paths
                    print(f"Paths: {'ON' if self.show_paths else 'OFF'}")
                elif key == ord('2'):
                    self.show_labels = not self.show_labels
                    print(f"Labels: {'ON' if self.show_labels else 'OFF'}")
                elif key == ord('3'):
                    self.show_confidence = not self.show_confidence
                    print(f"Confidence: {'ON' if self.show_confidence else 'OFF'}")
                elif key == ord('4'):
                    # Print current targets info
                    print(f"\nCurrent Targets ({len(self.tracker.targets)}):")
                    for target in self.tracker.targets.values():
                        print(f"  ID:{target.id} - {target.classification} - "
                              f"Conf:{target.confidence:.2f} - "
                              f"Path:{len(target.path)} - "
                              f"Age:{time.time() - target.first_seen:.1f}s")
                elif key == ord('5'):
                    self.show_debug = not self.show_debug
                    print(f"Debug mode: {'ON' if self.show_debug else 'OFF'}")
                elif key == ord('6'):
                    # Manually clear frame buffer
                    self.rtsp_processor._clear_frame_buffer()
                    print("🔄 Manually cleared frame buffer")
                elif key == ord('7'):
                    # Toggle adaptive frame skipping
                    self.rtsp_processor.adaptive_skip = not self.rtsp_processor.adaptive_skip
                    print(f"Adaptive frame skipping: {'ON' if self.rtsp_processor.adaptive_skip else 'OFF'}")
                elif key == ord('8'):
                    # Toggle all filtering (emergency override)
                    self.detector.min_motion_intensity = 0.0 if self.detector.min_motion_intensity > 0 else 0.15
                    self.detector.motion_growth_threshold = 0.0 if self.detector.motion_growth_threshold > 0 else 0.1
                    self.tracker.min_velocity_threshold = 0.0 if self.tracker.min_velocity_threshold > 0 else 1.0
                    print(f"All filtering: {'OFF' if self.detector.min_motion_intensity == 0 else 'ON'}")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        self.rtsp_processor.close()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="Tiny Object Detector for Wide-Angle Cam1 Stream")
    parser.add_argument('--rtsp-url', default='rtsp://localhost:8554/cam1',
                       help='RTSP stream URL (default: rtsp://localhost:8554/cam1)')
    parser.add_argument('--sensitivity', choices=['low', 'medium', 'medium-high', 'high'], default='medium',
                       help='Detection sensitivity (default: medium)')
    parser.add_argument('--frame-skip', type=int, default=1,
                       help='Number of frames to skip for performance (default: 1)')
    parser.add_argument('--resize-factor', type=float, default=1.0,
                       help='Frame resize factor for performance (default: 1.0)')
    parser.add_argument('--disable-cloud-filter', action='store_true',
                       help='Disable cloud filtering (may show more false positives)')
    parser.add_argument('--reset-interval', type=int, default=5,
                       help='Reset detection every X seconds (default: 5)')
    parser.add_argument('--full-reset', action='store_true',
                       help='Do full reset (clear all targets) instead of smart reset')
    parser.add_argument('--crop-center', action='store_true',
                       help='Center crop 1920x1080 stream to 1280x720 for better pixel density')
    
    args = parser.parse_args()
    
    finder = TinyObjectFinder(
        rtsp_url=args.rtsp_url,
        sensitivity=args.sensitivity,
        frame_skip=args.frame_skip,
        resize_factor=args.resize_factor,
        disable_cloud_filter=args.disable_cloud_filter,
        reset_interval=args.reset_interval,
        full_reset=args.full_reset,
        crop_center=args.crop_center
    )
    
    try:
        finder.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        finder.cleanup()

if __name__ == "__main__":
    main()
