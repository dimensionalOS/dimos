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
    calculate_position_rotation_from_bbox,
)
from dimos.types.vector import Vector
from typing import Optional, Union
from dimos.types.manipulation import ObjectData

from dimos.utils.logging_config import setup_logger

# Initialize logger for the ObjectDetectionStream
logger = setup_logger("dimos.perception.object_detection_stream")


class ObjectDetectionStream:
    """
    A stream processor that:
    1. Detects objects using a Detector (Detic or Yolo)
    2. Estimates depth using Metric3D
    3. Calculates 3D position and dimensions using camera intrinsics
    4. Transforms coordinates to map frame
    5. Draws bounding boxes and segmentation masks on the frame

    Provides a stream of structured object data with position and rotation information.
    """

    def __init__(
        self,
        camera_intrinsics=None,  # [fx, fy, cx, cy]
        device="cuda",
        gt_depth_scale=1000.0,
        min_confidence=0.7,
        class_filter=None,  # Optional list of class names to filter (e.g., ["person", "car"])
        transform_to_map=None,  # Optional function to transform coordinates to map frame
        detector: Optional[Union[Detic2DDetector, Yolo2DDetector]] = None,
        video_stream: Observable = None,
        disable_depth: bool = False,  # Flag to disable monocular Metric3D depth estimation
        draw_masks: bool = False,  # Flag to enable drawing segmentation masks
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
            disable_depth: Flag to disable monocular Metric3D depth estimation
            draw_masks: Flag to enable drawing segmentation masks
        """
        self.min_confidence = min_confidence
        self.class_filter = class_filter
        self.transform_to_map = transform_to_map
        self.disable_depth = disable_depth
        self.draw_masks = draw_masks
        # Initialize object detector
        self.detector = detector or Detic2DDetector(vocabulary=None, threshold=min_confidence)
        # Set up camera intrinsics
        self.camera_intrinsics = camera_intrinsics

        # Initialize depth estimation model
        self.depth_model = None
        if not disable_depth:
            self.depth_model = Metric3D(gt_depth_scale)

            if camera_intrinsics is not None:
                self.depth_model.update_intrinsic(camera_intrinsics)

                # Create 3x3 camera matrix for calculations
                fx, fy, cx, cy = camera_intrinsics
                self.camera_matrix = np.array(
                    [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
                )
            else:
                raise ValueError("camera_intrinsics must be provided")
        else:
            logger.info("Depth estimation disabled")

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
            bboxes, track_ids, class_ids, confidences, names, masks = self.detector.process_image(
                frame
            )

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

                if not self.disable_depth:
                    # Get depth for this object
                    depth = calculate_depth_from_bbox(self.depth_model, frame, bbox)
                    if depth is None:
                        # Skip objects with invalid depth
                        continue
                    # Calculate object position and rotation
                    position, rotation = calculate_position_rotation_from_bbox(
                        bbox, depth, self.camera_intrinsics
                    )
                    # Get object dimensions
                    width, height = calculate_object_size_from_bbox(
                        bbox, depth, self.camera_intrinsics
                    )

                    # Transform to map frame if a transform function is provided
                    try:
                        if self.transform_to_map:
                            position = Vector([position["x"], position["y"], position["z"]])
                            rotation = Vector(
                                [rotation["roll"], rotation["pitch"], rotation["yaw"]]
                            )
                            position, rotation = self.transform_to_map(
                                position, rotation, source_frame="base_link"
                            )
                    except Exception as e:
                        logger.error(f"Error transforming to map frame: {e}")
                        position, rotation = position, rotation

                else:
                    depth = -1
                    position = Vector(0, 0, 0)
                    rotation = Vector(0, 0, 0)
                    width = -1
                    height = -1

                # Create a properly typed ObjectData instance
                object_data: ObjectData = {
                    "object_id": track_ids[i] if i < len(track_ids) else -1,
                    "bbox": bbox,
                    "depth": depth,
                    "confidence": confidences[i] if i < len(confidences) else None,
                    "class_id": class_ids[i] if i < len(class_ids) else None,
                    "label": class_name,
                    "position": position,
                    "rotation": rotation,
                    "size": {"width": width, "height": height},
                    "segmentation_mask": masks[i],
                }

                objects.append(object_data)

                # Add visualization
                x1, y1, x2, y2 = map(int, bbox)
                color = (0, 255, 0)  # Green for detected objects
                mask_color = (0, 200, 200)  # Yellow-green for masks

                # Draw segmentation mask if available and valid
                try:
                    if self.draw_masks and object_data["segmentation_mask"] is not None:
                        # Create a colored mask overlay
                        mask = object_data["segmentation_mask"].astype(np.uint8)
                        colored_mask = np.zeros_like(viz_frame)
                        colored_mask[mask > 0] = mask_color

                        # Apply the mask with transparency
                        alpha = 0.5  # transparency factor
                        mask_area = mask > 0
                        viz_frame[mask_area] = cv2.addWeighted(
                            viz_frame[mask_area], 1 - alpha, colored_mask[mask_area], alpha, 0
                        )

                        # Draw mask contour
                        contours, _ = cv2.findContours(
                            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                        )
                        cv2.drawContours(viz_frame, contours, -1, mask_color, 2)
                except Exception as e:
                    logger.warning(f"Error drawing segmentation mask: {e}")

                # Draw bounding box with metadata
                try:
                    cv2.rectangle(viz_frame, (x1, y1), (x2, y2), color, 1)

                    # Add text for class only (removed position data)
                    # Handle possible None values for class_name or track_ids[i]
                    class_text = class_name if class_name is not None else "Unknown"
                    id_text = (
                        track_ids[i] if i < len(track_ids) and track_ids[i] is not None else "?"
                    )
                    text = f"{class_text}, ID: {id_text}"

                    # Draw text background with smaller font
                    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
                    cv2.rectangle(
                        viz_frame,
                        (x1, y1 - text_size[1] - 5),
                        (x1 + text_size[0], y1),
                        (0, 0, 0),
                        -1,
                    )

                    # Draw text with smaller font
                    cv2.putText(
                        viz_frame,
                        text,
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                except Exception as e:
                    logger.warning(f"Error drawing bounding box or text: {e}")

            return {"frame": frame, "viz_frame": viz_frame, "objects": objects}

        self.stream = video_stream.pipe(ops.map(process_frame))

        return self.stream

    def get_stream(self):
        """
        Returns the current detection stream if available.
        Creates a new one with the provided video_stream if not already created.

        Returns:
            Observable: The reactive stream of detection results
        """
        if self.stream is None:
            raise ValueError(
                "Stream not initialized. Either provide a video_stream during initialization or call create_stream first."
            )
        return self.stream

    def get_formatted_stream(self):
        """
        Returns a formatted stream of object detection data for better readability.
        This is especially useful for LLMs like Claude that need structured text input.

        Returns:
            Observable: A stream of formatted string representations of object data
        """
        if self.stream is None:
            raise ValueError(
                "Stream not initialized. Either provide a video_stream during initialization or call create_stream first."
            )

        def format_detection_data(result):
            # Extract objects from result
            objects = result.get("objects", [])

            if not objects:
                return "No objects detected."

            formatted_data = "[DETECTED OBJECTS]\n"
            try:
                for i, obj in enumerate(objects):
                    pos = obj["position"]
                    rot = obj["rotation"]
                    size = obj["size"]
                    bbox = obj["bbox"]

                    # Format each object with a multiline f-string for better readability
                    bbox_str = f"[{bbox[0]}, {bbox[1]}, {bbox[2]}, {bbox[3]}]"
                    formatted_data += (
                        f"Object {i + 1}: {obj['label']}\n"
                        f"  ID: {obj['object_id']}\n"
                        f"  Confidence: {obj['confidence']:.2f}\n"
                        f"  Position: x={pos.x:.2f}m, y={pos.y:.2f}m, z={pos.z:.2f}m\n"
                        f"  Rotation: yaw={rot.z:.2f} rad\n"
                        f"  Size: width={size['width']:.2f}m, height={size['height']:.2f}m\n"
                        f"  Depth: {obj['depth']:.2f}m\n"
                        f"  Bounding box: {bbox_str}\n"
                        "----------------------------------\n"
                    )
            except Exception as e:
                logger.warning(f"Error formatting object {i}: {e}")
                formatted_data += f"Object {i + 1}: [Error formatting data]"
                formatted_data += "\n----------------------------------\n"

            return formatted_data

        # Return a new stream with the formatter applied
        return self.stream.pipe(ops.map(format_detection_data))

    def cleanup(self):
        """Clean up resources."""
        pass
