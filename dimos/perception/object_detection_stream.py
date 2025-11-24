import cv2
import numpy as np
from reactivex import Observable
from reactivex import operators as ops

from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.models.depth.metric3d import Metric3D
from dimos.perception.detection2d.utils import (
    calculate_depth_from_bbox,
    calculate_object_size_from_bbox,
    calculate_position_rotation_from_bbox
)
from dimos.types.vector import Vector
from typing import Optional, Union

class ObjectDetectionStream:
    """
    A stream processor that:
    1. Detects objects using a Detector (Detic or Yolo)
    2. Estimates depth using Metric3D
    3. Calculates 3D position and dimensions using camera intrinsics
    4. Transforms coordinates to map frame
    
    Provides a stream of structured object data with position and rotation information.
    """
    
    def __init__(
        self,
        camera_intrinsics=None,  # [fx, fy, cx, cy]
        device="cuda",
        gt_depth_scale=1000.0,
        min_confidence=0.5,
        class_filter=None,  # Optional list of class names to filter (e.g., ["person", "car"])
        transform_to_map=None,  # Optional function to transform coordinates to map frame
        detector: Optional[Union[Detic2DDetector, Yolo2DDetector]] = None,
        video_stream: Observable = None
    ):
        """
        Initialize the ObjectDetectionStream.
        
        Args:
            camera_intrinsics: List [fx, fy, cx, cy] with camera parameters
            device: Device to run inference on ("cuda" or "cpu")
            gt_depth_scale: Ground truth depth scale for Metric3D
            min_confidence: Minimum confidence for detections
            class_filter: Optional list of class names to filter
            transform_to_map: Optional function to transform pose to map coordinates
            detector: Optional detector instance (Detic or Yolo)
            video_stream: Observable of video frames to process (if provided, returns a stream immediately)
        """
        self.min_confidence = min_confidence
        self.class_filter = class_filter
        self.transform_to_map = transform_to_map
        # Initialize object detector
        self.detector = detector or Detic2DDetector(vocabulary=None, threshold=min_confidence)
        
        # Initialize depth estimation model
        self.depth_model = Metric3D(gt_depth_scale)
        
        # Set up camera intrinsics
        self.camera_intrinsics = camera_intrinsics
        if camera_intrinsics is not None:
            self.depth_model.update_intrinsic(camera_intrinsics)
            
            # Create 3x3 camera matrix for calculations
            fx, fy, cx, cy = camera_intrinsics
            self.camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float32)
        else:
            raise ValueError("camera_intrinsics must be provided")
            
        # If video_stream is provided, create and store the stream immediately
        self.stream = None
        if video_stream is not None:
            self.stream = self.create_stream(video_stream)
    
    def create_stream(self, video_stream: Observable) -> Observable:
        """
        Create an Observable stream of object data from a video stream.
        
        Args:
            video_stream: Observable that emits video frames
            
        Returns:
            Observable that emits dictionaries containing object data 
            with position and rotation information
        """
        def process_frame(frame):
            # Detect objects
            bboxes, track_ids, class_ids, confidences, names = self.detector.process_image(frame)
            
            # Create visualization
            viz_frame = frame.copy()
            
            # Process detections
            objects = []
            
            for i, bbox in enumerate(bboxes):
                # Skip if confidence is too low
                if i < len(confidences) and confidences[i] < self.min_confidence:
                    continue
                    
                # Skip if class filter is active and class not in filter
                class_name = names[i] if i < len(names) else None
                if self.class_filter and class_name not in self.class_filter:
                    continue
                
                # Get depth for this object
                depth = calculate_depth_from_bbox(self.depth_model, frame, bbox)
                if depth is None:
                    # Skip objects with invalid depth
                    continue
                
                # Calculate object position and rotation
                position, rotation = calculate_position_rotation_from_bbox(bbox, depth, self.camera_intrinsics)
                
                # Get object dimensions
                width, height = calculate_object_size_from_bbox(bbox, depth, self.camera_intrinsics)
                
                # Transform to map frame if a transform function is provided
                try:
                    if self.transform_to_map:
                        position = Vector([position['x'], position['y'], position['z']])
                        rotation = Vector([rotation['roll'], rotation['pitch'], rotation['yaw']])
                        position, rotation = self.transform_to_map(position, rotation, source_frame="base_link")
                        position = dict(x=position.x, y=position.y, z=position.z)
                        rotation = dict(roll=rotation.x, pitch=rotation.y, yaw=rotation.z)
                except Exception as e:
                    print(f"Error transforming to map frame: {e}")
                    position, rotation = position, rotation
                
                # Create object data dictionary
                object_data = {
                    "object_id": track_ids[i] if i < len(track_ids) else -1,
                    "bbox": bbox,
                    "depth": depth,
                    "confidence": confidences[i] if i < len(confidences) else None,
                    "class_id": class_ids[i] if i < len(class_ids) else None,
                    "label": class_name,
                    "position": position,
                    "rotation": rotation,
                    "size": {
                        "width": width,
                        "height": height
                    }
                }
                
                objects.append(object_data)
                
                # Add visualization
                x1, y1, x2, y2 = map(int, bbox)
                color = (0, 255, 0)  # Green for detected objects
                
                # Draw bounding box
                cv2.rectangle(viz_frame, (x1, y1), (x2, y2), color, 2)
                
                # Add text for class and position
                text = f"{class_name}: {depth:.2f}m"
                pos_text = f"Pos: ({position['x']:.2f}, {position['y']:.2f})"
                
                # Draw text background
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(viz_frame, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), (0, 0, 0), -1)
                
                # Draw text
                cv2.putText(viz_frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # Position text below
                cv2.putText(viz_frame, pos_text, (x1, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            return {
                "frame": frame,
                "viz_frame": viz_frame,
                "objects": objects
            }
        self.stream = video_stream.pipe(
            ops.map(process_frame)
        )

        return self.stream
    
    def get_stream(self):
        """
        Returns the current detection stream if available.
        Creates a new one with the provided video_stream if not already created.
        
        Returns:
            Observable: The reactive stream of detection results
        """
        if self.stream is None:
            raise ValueError("Stream not initialized. Either provide a video_stream during initialization or call create_stream first.")
        return self.stream
        
    def get_formatted_stream(self):
        """
        Returns a formatted stream of object detection data for better readability.
        This is especially useful for LLMs like Claude that need structured text input.
        
        Returns:
            Observable: A stream of formatted string representations of object data
        """
        if self.stream is None:
            raise ValueError("Stream not initialized. Either provide a video_stream during initialization or call create_stream first.")
            
        def format_detection_data(result):
            # Extract objects from result
            objects = result.get("objects", [])

            if not objects:
                return "No objects detected."
                
            formatted_data = "[DETECTED OBJECTS]\n"
            
            for i, obj in enumerate(objects):
                pos = obj["position"]
                rot = obj["rotation"]
                size = obj["size"]
                bbox = obj["bbox"]
                
                # Format each object with a multiline f-string for better readability
                bbox_str = f"[{int(bbox[0])}, {int(bbox[1])}, {int(bbox[2])}, {int(bbox[3])}]"
                formatted_data += f"Object {i+1}: {obj['label']}\n"\
                f"  ID: {obj['object_id']}\n"\
                f"  Confidence: {obj['confidence']:.2f}\n"\
                f"  Position: x={pos['x']:.2f}m, y={pos['y']:.2f}m, z={pos['z']:.2f}m\n"\
                f"  Rotation: yaw={rot['yaw']:.2f} rad\n"\
                f"  Size: width={size['width']:.2f}m, height={size['height']:.2f}m\n"\
                f"  Depth: {obj['depth']:.2f}m\n"\
                f"  Bounding box: {bbox_str}\n"\
                "----------------------------------\n"
            
            return formatted_data
            
        # Return a new stream with the formatter applied
        return self.stream.pipe(
            ops.map(format_detection_data)
        )

    def cleanup(self):
        """Clean up resources."""
        pass
