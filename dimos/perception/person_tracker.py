from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.perception.detection2d.utils import filter_detections
from dimos.perception.common.ibvs import PersonDistanceEstimator
from reactivex import Observable
from reactivex import operators as ops
import numpy as np
import cv2


class PersonTrackingStream:
    def __init__(
        self,
        model_path="yolo11n.pt",
        device="cuda",
        camera_intrinsics=None,
        camera_pitch=0.0,
        camera_height=1.0
    ):
        """
        Initialize a person tracking stream using Yolo2DDetector and PersonDistanceEstimator.
        
        Args:
            model_path: Path to the YOLO model file
            device: Computation device ("cuda" or "cpu")
            camera_intrinsics: List in format [fx, fy, cx, cy] where:
                - fx: Focal length in x direction (pixels)
                - fy: Focal length in y direction (pixels)
                - cx: Principal point x-coordinate (pixels)
                - cy: Principal point y-coordinate (pixels)
            camera_pitch: Camera pitch angle in radians (positive is up)
            camera_height: Height of the camera from the ground in meters
        """
        self.detector = Yolo2DDetector(
            model_path=model_path,
            device=device
        )
        
        # Initialize distance estimator
        if camera_intrinsics is None:
            raise ValueError("Camera intrinsics are required for distance estimation")
            
        # Validate camera intrinsics format [fx, fy, cx, cy]
        if not isinstance(camera_intrinsics, (list, tuple, np.ndarray)) or len(camera_intrinsics) != 4:
            raise ValueError("Camera intrinsics must be provided as [fx, fy, cx, cy]")
        
        # Convert [fx, fy, cx, cy] to 3x3 camera matrix
        fx, fy, cx, cy = camera_intrinsics
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
            
        self.distance_estimator = PersonDistanceEstimator(
            K=K,
            camera_pitch=camera_pitch,
            camera_height=camera_height
        )
    
    def create_stream(self, video_stream: Observable) -> Observable:
        """
        Create an Observable stream of person tracking results from a video stream.
        
        Args:
            video_stream: Observable that emits video frames
            
        Returns:
            Observable that emits dictionaries containing tracking results and visualizations
        """
        def process_frame(frame):
            # Detect people in the frame
            bboxes, track_ids, class_ids, confidences, names = self.detector.process_image(frame)
            
            # Filter to keep only person detections using filter_detections
            filtered_bboxes, filtered_track_ids, filtered_class_ids, filtered_confidences, filtered_names = (
                filter_detections(
                    bboxes, track_ids, class_ids, confidences, names,
                    class_filter=[0],  # 0 is the class_id for person
                    name_filter=['person']
                )
            )
            
            # Create visualization
            viz_frame = self.detector.visualize_results(
                frame,
                filtered_bboxes,
                filtered_track_ids,
                filtered_class_ids,
                filtered_confidences,
                filtered_names
            )
            
            # Calculate distance and angle for each person
            targets = []
            for i, bbox in enumerate(filtered_bboxes):
                target_data = {
                    "target_id": filtered_track_ids[i] if i < len(filtered_track_ids) else -1,
                    "bbox": bbox,
                    "confidence": filtered_confidences[i] if i < len(filtered_confidences) else None,
                }
                
                distance, angle = self.distance_estimator.estimate_distance_angle(bbox)
                target_data["distance"] = distance
                target_data["angle"] = angle
                
                # Add text to visualization
                x1, y1, x2, y2 = map(int, bbox)
                dist_text = f"{distance:.2f}m, {np.rad2deg(angle):.1f} deg"
                
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
                
                targets.append(target_data)
            
            # Create the result dictionary
            result = {
                "frame": frame,
                "viz_frame": viz_frame,
                "targets": targets
            }
            
            return result
        
        return video_stream.pipe(
            ops.map(process_frame)
        )
    
    def cleanup(self):
        """Clean up resources."""
        pass  # No specific cleanup needed for now
