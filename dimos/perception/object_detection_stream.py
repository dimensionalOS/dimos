# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import numpy as np
from reactivex import Observable, operators as ops

from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector  # type: ignore[import-untyped]

try:
    from dimos.perception.detection2d.detic_2d_det import (  # type: ignore[import-untyped]
        Detic2DDetector,
    )

    DETIC_AVAILABLE = True
except (ModuleNotFoundError, ImportError):
    DETIC_AVAILABLE = False
    Detic2DDetector = None
from collections.abc import Callable
from typing import TYPE_CHECKING

from dimos.models.depth.metric3d import Metric3D
from dimos.perception.common.utils import draw_object_detection_visualization
from dimos.perception.detection2d.utils import (  # type: ignore[attr-defined]
    calculate_depth_from_bbox,
    calculate_object_size_from_bbox,
    calculate_position_rotation_from_bbox,
)
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import transform_robot_to_map  # type: ignore[attr-defined]

if TYPE_CHECKING:
    from dimos.types.manipulation import ObjectData

# Initialize logger for the ObjectDetectionStream
logger = setup_logger()


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

    def __init__(  # type: ignore[no-untyped-def]
        self,
        camera_intrinsics=None,  # [fx, fy, cx, cy]
        device: str = "cuda",
        gt_depth_scale: float = 1000.0,
        min_confidence: float = 0.7,
        class_filter=None,  # Optional list of class names to filter (e.g., ["person", "car"])
        get_pose: Callable | None = None,  # type: ignore[type-arg]  # Optional function to transform coordinates to map frame
        detector: Detic2DDetector | Yolo2DDetector | None = None,
        video_stream: Observable = None,  # type: ignore[assignment, type-arg]
        disable_depth: bool = False,  # Flag to disable monocular Metric3D depth estimation
        draw_masks: bool = False,  # Flag to enable drawing segmentation masks
    ) -> None:
        """
        Initialize the ObjectDetectionStream.

        Args:
            camera_intrinsics: List [fx, fy, cx, cy] with camera parameters
            device: Device to run inference on ("cuda" or "cpu")
            gt_depth_scale: Ground truth depth scale for Metric3D
            min_confidence: Minimum confidence for detections
            class_filter: Optional list of class names to filter
            get_pose: Optional function to transform pose to map coordinates
            detector: Optional detector instance (Detic or Yolo)
            video_stream: Observable of video frames to process (if provided, returns a stream immediately)
            disable_depth: Flag to disable monocular Metric3D depth estimation
            draw_masks: Flag to enable drawing segmentation masks
        """
        self.min_confidence = min_confidence
        self.class_filter = class_filter
        self.get_pose = get_pose
        self.disable_depth = disable_depth
        self.draw_masks = draw_masks
        # Initialize object detector
        if detector is not None:
            self.detector = detector
        else:
            if DETIC_AVAILABLE:
                try:
                    self.detector = Detic2DDetector(vocabulary=None, threshold=min_confidence)
                    logger.info("Using Detic2DDetector")
                except Exception as e:
                    logger.warning(
                        f"Failed to initialize Detic2DDetector: {e}. Falling back to Yolo2DDetector."
                    )
                    self.detector = Yolo2DDetector()
            else:
                logger.info("Detic not available. Using Yolo2DDetector.")
                self.detector = Yolo2DDetector()
        # Set up camera intrinsics
        self.camera_intrinsics = camera_intrinsics

        # Initialize depth estimation model
        self.depth_model = None
        if not disable_depth:
            try:
                self.depth_model = Metric3D(gt_depth_scale=gt_depth_scale)

                if camera_intrinsics is not None:
                    self.depth_model.update_intrinsic(camera_intrinsics)  # type: ignore[no-untyped-call]

                    # Create 3x3 camera matrix for calculations
                    fx, fy, cx, cy = camera_intrinsics
                    self.camera_matrix = np.array(
                        [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
                    )
                else:
                    raise ValueError("camera_intrinsics must be provided")

                logger.info("Depth estimation enabled with Metric3D")
            except Exception as e:
                logger.warning(f"Failed to initialize Metric3D depth model: {e}")
                logger.warning("Falling back to disable_depth=True mode")
                self.disable_depth = True
                self.depth_model = None
        else:
            logger.info("Depth estimation disabled")

        # If video_stream is provided, create and store the stream immediately
        self.stream = None
        if video_stream is not None:
            self.stream = self.create_stream(video_stream)

    def create_stream(self, video_stream: Observable) -> Observable:  # type: ignore[type-arg]
        """
        Create an Observable stream of object data from a video stream.

        Args:
            video_stream: Observable that emits video frames

        Returns:
            Observable that emits dictionaries containing object data
            with position and rotation information
        """

        def process_frame(frame):  # type: ignore[no-untyped-def]
            # TODO: More modular detector output interface
            bboxes, track_ids, class_ids, confidences, names, *mask_data = (  # type: ignore[misc]
                *self.detector.process_image(frame),
                [],
            )

            masks = (
                mask_data[0]  # type: ignore[has-type]
                if mask_data and len(mask_data[0]) == len(bboxes)  # type: ignore[has-type]
                else [None] * len(bboxes)  # type: ignore[has-type]
            )

            # Create visualization
            viz_frame = frame.copy()

            # Process detections
            objects = []
            if not self.disable_depth:
                depth_map = self.depth_model.infer_depth(frame)  # type: ignore[union-attr]
                depth_map = np.array(depth_map)
            else:
                depth_map = None

            for i, bbox in enumerate(bboxes):  # type: ignore[has-type]
                # Skip if confidence is too low
                if i < len(confidences) and confidences[i] < self.min_confidence:  # type: ignore[has-type]
                    continue

                # Skip if class filter is active and class not in filter
                class_name = names[i] if i < len(names) else None  # type: ignore[has-type]
                if self.class_filter and class_name not in self.class_filter:
                    continue

                if not self.disable_depth and depth_map is not None:
                    # Get depth for this object
                    depth = calculate_depth_from_bbox(depth_map, bbox)  # type: ignore[no-untyped-call]
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
                        if self.get_pose:
                            # position and rotation are already Vector objects, no need to convert
                            robot_pose = self.get_pose()
                            position, rotation = transform_robot_to_map(
                                robot_pose["position"], robot_pose["rotation"], position, rotation
                            )
                    except Exception as e:
                        logger.error(f"Error transforming to map frame: {e}")
                        position, rotation = position, rotation

                else:
                    depth = -1
                    position = Vector(0, 0, 0)  # type: ignore[arg-type]
                    rotation = Vector(0, 0, 0)  # type: ignore[arg-type]
                    width = -1
                    height = -1

                # Create a properly typed ObjectData instance
                object_data: ObjectData = {
                    "object_id": track_ids[i] if i < len(track_ids) else -1,  # type: ignore[has-type]
                    "bbox": bbox,
                    "depth": depth,
                    "confidence": confidences[i] if i < len(confidences) else None,  # type: ignore[has-type, typeddict-item]
                    "class_id": class_ids[i] if i < len(class_ids) else None,  # type: ignore[has-type, typeddict-item]
                    "label": class_name,  # type: ignore[typeddict-item]
                    "position": position,
                    "rotation": rotation,
                    "size": {"width": width, "height": height},
                    "segmentation_mask": masks[i],
                }

                objects.append(object_data)

            # Create visualization using common function
            viz_frame = draw_object_detection_visualization(
                viz_frame, objects, draw_masks=self.draw_masks, font_scale=1.5
            )

            return {"frame": frame, "viz_frame": viz_frame, "objects": objects}

        self.stream = video_stream.pipe(ops.map(process_frame))

        return self.stream

    def get_stream(self):  # type: ignore[no-untyped-def]
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

    def get_formatted_stream(self):  # type: ignore[no-untyped-def]
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

        def format_detection_data(result):  # type: ignore[no-untyped-def]
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

    def cleanup(self) -> None:
        """Clean up resources."""
        pass
