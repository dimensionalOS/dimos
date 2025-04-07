import cv2
from reactivex import Observable
from reactivex import operators as ops
import numpy as np
from dimos.perception.common.ibvs import ObjectDistanceEstimator

class ObjectTrackingStream:
    def __init__(self, camera_intrinsics=None, camera_pitch=0.0, camera_height=1.0):
        """
        Initialize an object tracking stream using OpenCV's CSRT tracker.
        
        Args:
            camera_intrinsics: List in format [fx, fy, cx, cy] where:
                - fx: Focal length in x direction (pixels)
                - fy: Focal length in y direction (pixels)
                - cx: Principal point x-coordinate (pixels)
                - cy: Principal point y-coordinate (pixels)
            camera_pitch: Camera pitch angle in radians (positive is up)
            camera_height: Height of the camera from the ground in meters
        """
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        
        # Initialize distance estimator if camera parameters are provided
        self.distance_estimator = None
        if camera_intrinsics is not None:
            # Convert [fx, fy, cx, cy] to 3x3 camera matrix
            fx, fy, cx, cy = camera_intrinsics
            K = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float32)
                
            self.distance_estimator = ObjectDistanceEstimator(
                K=K,
                camera_pitch=camera_pitch,
                camera_height=camera_height
            )
        
    def track(self, bbox, distance=None, size=None):
        """
        Update the tracker with a new bounding box.
        This should be called whenever a new detection is available.
        
        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]
            distance: Optional - Known distance to object (meters)
            size: Optional - Known size of object (meters)
            
        Returns:
            bool: True if tracking was initialized successfully
        """
        # Convert from [x1, y1, x2, y2] to [x, y, width, height] format for OpenCV
        x1, y1, x2, y2 = bbox
        self.tracking_bbox = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
        
        # Create a new tracker (OpenCV trackers can't be reused after failure)
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracking_initialized = False  # Will be initialized on next frame
        
        # Update distance estimator if we have one
        if self.distance_estimator is not None:
            # If we have a known size, set it directly
            if size is not None:
                self.distance_estimator.set_estimated_object_size(size)
            # If we have a known distance, use it to estimate the object size
            elif distance is not None:
                self.distance_estimator.estimate_object_size(bbox, distance)
        
        return True
    
    def stop_track(self):
        """
        Stop tracking the current object.
        This resets the tracker and all tracking state.
        
        Returns:
            bool: True if tracking was successfully stopped
        """
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        
        return True
    
    def create_stream(self, video_stream: Observable) -> Observable:
        """
        Create an Observable stream of object tracking results from a video stream.
        
        Args:
            video_stream: Observable that emits video frames
            
        Returns:
            Observable that emits dictionaries containing tracking results and visualizations
        """
        def process_frame(frame):
            # Create a copy for visualization
            viz_frame = frame.copy()
            
            # Initialize or update tracker
            if self.tracker is not None:
                if not self.tracking_initialized:
                    # Initialize tracker with the first frame and given bbox
                    success = self.tracker.init(frame, self.tracking_bbox)
                    self.tracking_initialized = success
                else:
                    # Update tracker with new frame
                    success, bbox = self.tracker.update(frame)
                    
                    if success:
                        # Convert from [x, y, width, height] to [x1, y1, x2, y2]
                        x, y, w, h = [int(v) for v in bbox]
                        bbox = [x, y, x + w, y + h]
                        self.tracking_bbox = (x, y, w, h)  # Save for next frame
                        
                        # Draw bounding box and info
                        x1, y1, x2, y2 = bbox
                        cv2.rectangle(viz_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Create target data dictionary
                        target_data = {
                            "target_id": 0,  # As requested, set to 0
                            "bbox": bbox,
                            "confidence": 1.0,  # As requested, set to 1.0
                        }
                        
                        # Add distance and angle if estimator is available
                        if self.distance_estimator is not None and self.distance_estimator.estimated_object_size is not None:
                            distance, angle = self.distance_estimator.estimate_distance_angle(bbox)
                            if distance is not None:
                                target_data["distance"] = distance
                                target_data["angle"] = angle
                                
                                # Add distance information to visualization
                                dist_text = f"Object: {distance:.2f}m, {np.rad2deg(angle):.1f} deg"
                            else:
                                dist_text = "Object Tracking"
                        else:
                            dist_text = "Object Tracking"
                        
                        # Add black background for better visibility
                        text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                        # Position at top-right corner
                        cv2.rectangle(
                            viz_frame,
                            (x2 - text_size[0], y1 - text_size[1] - 5),
                            (x2, y1),
                            (0, 0, 0), -1
                        )
                        
                        # Draw text in white at top-right
                        cv2.putText(
                            viz_frame, dist_text,
                            (x2 - text_size[0], y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
                        )
                        
                        # Create the result dictionary
                        result = {
                            "frame": frame,
                            "viz_frame": viz_frame,
                            "targets": [target_data]  # List with single target
                        }
                        return result
            
            # If tracking failed or not initialized, return frame without targets
            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "targets": []
            }
        
        return video_stream.pipe(
            ops.map(process_frame)
        )
    
    def cleanup(self):
        """Clean up resources."""
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False