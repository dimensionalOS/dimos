import cv2
from reactivex import Observable
from reactivex import operators as ops
import numpy as np
from dimos.perception.common.ibvs import ObjectDistanceEstimator
from dimos.models.depth.metric3d import Metric3D
from dimos.perception.detection2d.utils import calculate_depth_from_bbox
class ObjectTrackingStream:
    def __init__(self, camera_intrinsics=None, camera_pitch=0.0, camera_height=1.0, 
                 reid_threshold=5, reid_fail_tolerance=10, gt_depth_scale=1000.0):
        """
        Initialize an object tracking stream using OpenCV's CSRT tracker with ORB re-ID.
        
        Args:
            camera_intrinsics: List in format [fx, fy, cx, cy] where:
                - fx: Focal length in x direction (pixels)
                - fy: Focal length in y direction (pixels)
                - cx: Principal point x-coordinate (pixels)
                - cy: Principal point y-coordinate (pixels)
            camera_pitch: Camera pitch angle in radians (positive is up)
            camera_height: Height of the camera from the ground in meters
            reid_threshold: Minimum good feature matches needed to confirm re-ID.
            reid_fail_tolerance: Number of consecutive frames Re-ID can fail before 
                                 tracking is stopped.
            gt_depth_scale: Ground truth depth scale factor for Metric3D model
        """
        self.tracker = None
        self.tracking_bbox = None # Stores (x, y, w, h) for tracker initialization
        self.tracking_initialized = False
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.original_des = None # Store original ORB descriptors
        self.reid_threshold = reid_threshold
        self.reid_fail_tolerance = reid_fail_tolerance
        self.reid_fail_count = 0 # Counter for consecutive re-id failures
        
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
            
        # Initialize depth model
        self.depth_model = Metric3D(gt_depth_scale)
        if camera_intrinsics is not None:
            self.depth_model.update_intrinsic(camera_intrinsics)
        
    def track(self, bbox, frame=None, distance=None, size=None):
        """
        Set the initial bounding box for tracking. Features are extracted later.
        
        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]
            frame: Optional - Current frame for depth estimation and feature extraction
            distance: Optional - Known distance to object (meters)
            size: Optional - Known size of object (meters)
            
        Returns:
            bool: True if intention to track is set (bbox is valid)
        """
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            print(f"Warning: Invalid initial bbox provided: {bbox}. Tracking not started.")
            self.stop_track() # Ensure clean state
            return False 
            
        self.tracking_bbox = (x1, y1, w, h) # Store in (x, y, w, h) format
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracking_initialized = False # Reset flag
        self.original_des = None # Clear previous descriptors
        self.reid_fail_count = 0 # Reset counter on new track
        print(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Calculate depth only if distance and size not provided
        if frame is not None and distance is None and size is None:
            depth_estimate = calculate_depth_from_bbox(self.depth_model, frame, bbox)
            if depth_estimate is not None:
                print(f"Estimated depth for object: {depth_estimate:.2f}m")

        # Update distance estimator if needed
        if self.distance_estimator is not None:
            if size is not None: self.distance_estimator.set_estimated_object_size(size)
            elif distance is not None: self.distance_estimator.estimate_object_size(bbox, distance)
            elif depth_estimate is not None: self.distance_estimator.estimate_object_size(bbox, depth_estimate)
            else: print("No distance or size provided. Cannot estimate object size.")

        return True # Indicate intention to track is set
        
    def calculate_depth_from_bbox(self, frame, bbox):
        """
        Calculate the average depth of an object within a bounding box.
        Uses the 25th to 75th percentile range to filter outliers.
        
        Args:
            frame: The image frame
            bbox: Bounding box in format [x1, y1, x2, y2]
            
        Returns:
            float: Average depth in meters, or None if depth estimation fails
        """
        try:
            # Get depth map for the entire frame
            depth_map = self.depth_model.infer_depth(frame)
            depth_map = np.array(depth_map)
            
            # Extract region of interest from the depth map
            x1, y1, x2, y2 = map(int, bbox)
            roi_depth = depth_map[y1:y2, x1:x2]
            
            if roi_depth.size == 0:
                return None
                
            # Calculate 25th and 75th percentile to filter outliers
            p25 = np.percentile(roi_depth, 25)
            p75 = np.percentile(roi_depth, 75)
            
            # Filter depth values within this range
            filtered_depth = roi_depth[(roi_depth >= p25) & (roi_depth <= p75)]
            
            # Calculate average depth (convert to meters)
            if filtered_depth.size > 0:
                return np.mean(filtered_depth) / 1000.0  # Convert mm to meters
                
            return None
        except Exception as e:
            print(f"Error calculating depth from bbox: {e}")
            return None
    
    def reid(self, frame, current_bbox) -> bool:
        """Check if features in current_bbox match stored original features."""
        if self.original_des is None: return True # Cannot re-id if no original features
        x1, y1, x2, y2 = map(int, current_bbox)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return False # Empty ROI cannot match

        _, des_current = self.orb.detectAndCompute(roi, None)
        if des_current is None or len(des_current) < 2: return False # Need at least 2 descriptors for knnMatch

        # Handle case where original_des has only 1 descriptor (cannot use knnMatch with k=2)
        if len(self.original_des) < 2:
            matches = self.bf.match(self.original_des, des_current)
            good_matches = len(matches)
        else:
            matches = self.bf.knnMatch(self.original_des, des_current, k=2)
            # Apply Lowe's ratio test robustly
            good_matches = 0
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches += 1
        
        # print(f"ReID: Good Matches={good_matches}, Threshold={self.reid_threshold}") # Debug
        return good_matches >= self.reid_threshold

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
        self.original_des = None
        self.reid_fail_count = 0 # Reset counter
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
            viz_frame = frame.copy()
            tracker_succeeded = False # Success from tracker.update()
            reid_confirmed_this_frame = False # Result of reid() call for this frame
            final_success = False # Overall success considering re-id tolerance
            target_data = None
            current_bbox_x1y1x2y2 = None # Store current bbox if tracking succeeds

            if self.tracker is not None and self.tracking_bbox is not None:
                if not self.tracking_initialized:
                    # Extract initial features and initialize tracker on first frame
                    x_init, y_init, w_init, h_init = self.tracking_bbox
                    roi = frame[y_init:y_init+h_init, x_init:x_init+w_init]
                    
                    if roi.size > 0:
                        _, self.original_des = self.orb.detectAndCompute(roi, None)
                        if self.original_des is None:
                            print("Warning: No ORB features found in initial ROI during stream processing.")
                        else:
                            print(f"Initial ORB features extracted: {len(self.original_des)}")
                            
                        # Initialize the tracker
                        init_success = self.tracker.init(frame, self.tracking_bbox)
                        if init_success:
                            self.tracking_initialized = True
                            tracker_succeeded = True
                            reid_confirmed_this_frame = True # Assume re-id true on init
                            current_bbox_x1y1x2y2 = [x_init, y_init, x_init + w_init, y_init + h_init]
                            print("Tracker initialized successfully.")
                        else:
                            print("Error: Tracker initialization failed in stream.")
                            self.stop_track() # Reset if init fails
                    else:
                        print("Error: Empty ROI during tracker initialization in stream.")
                        self.stop_track() # Reset if ROI is bad
                        
                else: # Tracker already initialized, perform update and re-id
                    tracker_succeeded, bbox_cv = self.tracker.update(frame)
                    if tracker_succeeded:
                        x, y, w, h = map(int, bbox_cv)
                        current_bbox_x1y1x2y2 = [x, y, x + w, y + h]
                        # Perform re-ID check
                        reid_confirmed_this_frame = self.reid(frame, current_bbox_x1y1x2y2)

                        if reid_confirmed_this_frame:
                            self.reid_fail_count = 0 # Reset counter on success
                        else:
                            self.reid_fail_count += 1 # Increment counter on failure
                            print(f"Re-ID failed ({self.reid_fail_count}/{self.reid_fail_tolerance}). Continuing track...")
            
            # --- Determine final success and stop tracking if needed --- 
            if tracker_succeeded:
                if self.reid_fail_count >= self.reid_fail_tolerance:
                    print(f"Re-ID failed consecutively {self.reid_fail_count} times. Target lost.")
                    final_success = False # Stop tracking
                else:
                    final_success = True # Tracker ok, Re-ID ok or within tolerance
            else: # Tracker update failed
                final_success = False
                if self.tracking_initialized:
                     print("Tracker update failed. Stopping track.")
            
            # --- Post-processing based on final_success --- 
            if final_success and current_bbox_x1y1x2y2 is not None:
                # Tracking is considered successful (tracker ok, re-id ok or within tolerance)
                x1, y1, x2, y2 = current_bbox_x1y1x2y2
                # Visualize based on *this frame's* re-id result
                viz_color = (0, 255, 0) if reid_confirmed_this_frame else (0, 165, 255) # Green if confirmed, Orange if failed but tolerated
                cv2.rectangle(viz_frame, (x1, y1), (x2, y2), viz_color, 2)
                
                target_data = {
                    "target_id": 0, "bbox": current_bbox_x1y1x2y2, "confidence": 1.0,
                    "reid_confirmed": reid_confirmed_this_frame # Report actual re-id status
                }
                
                dist_text = "Object Tracking"
                if not reid_confirmed_this_frame: dist_text += " (Re-ID Failed - Tolerated)"
                
                if self.distance_estimator is not None and self.distance_estimator.estimated_object_size is not None:
                    distance, angle = self.distance_estimator.estimate_distance_angle(current_bbox_x1y1x2y2)
                    if distance is not None:
                        target_data["distance"] = distance
                        target_data["angle"] = angle
                        dist_text = f"Object: {distance:.2f}m, {np.rad2deg(angle):.1f} deg"
                        if not reid_confirmed_this_frame: dist_text += " (Re-ID Failed - Tolerated)"
                        
                text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                label_bg_y = max(y1 - text_size[1] - 5, 0)
                cv2.rectangle(viz_frame, (x1, label_bg_y), (x1 + text_size[0], y1), (0,0,0), -1)
                cv2.putText(viz_frame, dist_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            elif self.tracking_initialized: # Tracking stopped this frame (either tracker fail or re-id tolerance exceeded)
                self.stop_track() # Reset tracker state and counter
            
            # else: # Not tracking or initialization failed, do nothing, return empty result
            #     pass 

            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "targets": [target_data] if target_data else []
            }
        
        return video_stream.pipe(
            ops.map(process_frame)
        )
    
    def cleanup(self):
        """Clean up resources."""
        self.stop_track()