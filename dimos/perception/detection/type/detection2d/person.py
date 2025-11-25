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

from dataclasses import dataclass

# Import for type checking only to avoid circular imports
from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2  # type: ignore[import-untyped]
import numpy as np

from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.types.timestamped import to_ros_stamp
from dimos.utils.decorators.decorators import simple_mcache

if TYPE_CHECKING:
    from ultralytics.engine.results import Results


@dataclass
class Detection2DPerson(Detection2DBBox):
    """Represents a detected person with pose keypoints."""

    # Pose keypoints - additional fields beyond Detection2DBBox
    keypoints: np.ndarray  # type: ignore[type-arg]  # [17, 2] - x,y coordinates
    keypoint_scores: np.ndarray  # type: ignore[type-arg]  # [17] - confidence scores

    # Optional normalized coordinates
    bbox_normalized: np.ndarray | None = None  # type: ignore[type-arg]  # [x1, y1, x2, y2] in 0-1 range
    keypoints_normalized: np.ndarray | None = None  # type: ignore[type-arg]  # [17, 2] in 0-1 range

    # Image dimensions for context
    image_width: int | None = None
    image_height: int | None = None

    # Keypoint names (class attribute)
    KEYPOINT_NAMES = [
        "nose",
        "left_eye",
        "right_eye",
        "left_ear",
        "right_ear",
        "left_shoulder",
        "right_shoulder",
        "left_elbow",
        "right_elbow",
        "left_wrist",
        "right_wrist",
        "left_hip",
        "right_hip",
        "left_knee",
        "right_knee",
        "left_ankle",
        "right_ankle",
    ]

    @classmethod
    def from_ultralytics_result(
        cls, result: "Results", idx: int, image: Image
    ) -> "Detection2DPerson":
        """Create Detection2DPerson from ultralytics Results object with pose keypoints.

        Args:
            result: Ultralytics Results object containing detection and keypoint data
            idx: Index of the detection in the results
            image: Source image

        Returns:
            Detection2DPerson instance

        Raises:
            ValueError: If the result doesn't contain keypoints or is not a person detection
        """
        # Validate that this is a pose detection result
        if not hasattr(result, "keypoints") or result.keypoints is None:
            raise ValueError(
                "Cannot create Detection2DPerson from result without keypoints. "
                "This appears to be a regular detection result, not a pose detection. "
                "Use Detection2DBBox.from_ultralytics_result() instead."
            )

        if not hasattr(result, "boxes") or result.boxes is None:
            raise ValueError("Cannot create Detection2DPerson from result without bounding boxes")

        # Check if this is actually a person detection (class 0 in COCO)
        class_id = int(result.boxes.cls[idx].cpu())
        if class_id != 0:  # Person is class 0 in COCO
            class_name = (
                result.names.get(class_id, f"class_{class_id}")
                if hasattr(result, "names")
                else f"class_{class_id}"
            )
            raise ValueError(
                f"Cannot create Detection2DPerson from non-person detection. "
                f"Got class {class_id} ({class_name}), expected class 0 (person)."
            )

        # Extract bounding box as tuple for Detection2DBBox
        bbox_array = result.boxes.xyxy[idx].cpu().numpy()

        bbox: Bbox = (
            float(bbox_array[0]),
            float(bbox_array[1]),
            float(bbox_array[2]),
            float(bbox_array[3]),
        )

        bbox_norm = (
            result.boxes.xyxyn[idx].cpu().numpy() if hasattr(result.boxes, "xyxyn") else None
        )

        confidence = float(result.boxes.conf[idx].cpu())
        class_id = int(result.boxes.cls[idx].cpu())

        # Extract keypoints
        if result.keypoints.xy is None or result.keypoints.conf is None:
            raise ValueError("Keypoints xy or conf data is missing from the result")

        keypoints = result.keypoints.xy[idx].cpu().numpy()
        keypoint_scores = result.keypoints.conf[idx].cpu().numpy()
        keypoints_norm = (
            result.keypoints.xyn[idx].cpu().numpy()
            if hasattr(result.keypoints, "xyn") and result.keypoints.xyn is not None
            else None
        )

        # Get image dimensions
        height, width = result.orig_shape

        # Extract track ID if available
        track_id = idx  # Use index as default
        if hasattr(result.boxes, "id") and result.boxes.id is not None:
            track_id = int(result.boxes.id[idx].cpu())

        # Get class name
        name = result.names.get(class_id, "person") if hasattr(result, "names") else "person"

        return cls(
            # Detection2DBBox fields
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=image.ts,
            image=image,
            # Person specific fields
            keypoints=keypoints,
            keypoint_scores=keypoint_scores,
            bbox_normalized=bbox_norm,
            keypoints_normalized=keypoints_norm,
            image_width=width,
            image_height=height,
        )

    @classmethod
    def from_yolo(cls, result: "Results", idx: int, image: Image) -> "Detection2DPerson":
        """Alias for from_ultralytics_result for backward compatibility."""
        return cls.from_ultralytics_result(result, idx, image)

    @classmethod
    def from_ros_detection2d(cls, *args, **kwargs) -> "Detection2DPerson":  # type: ignore[no-untyped-def]
        """Conversion from ROS Detection2D is not supported for Detection2DPerson.

        The ROS Detection2D message format does not include keypoint data,
        which is required for Detection2DPerson. Use Detection2DBBox for
        round-trip ROS conversions, or store keypoints separately.

        Raises:
            NotImplementedError: Always raised as this conversion is impossible
        """
        raise NotImplementedError(
            "Cannot convert from ROS Detection2D to Detection2DPerson. "
            "The ROS Detection2D message format does not contain keypoint data "
            "(keypoints and keypoint_scores) which are required fields for Detection2DPerson. "
            "Consider using Detection2DBBox for ROS conversions, or implement a custom "
            "message format that includes pose keypoints."
        )

    def get_keypoint(self, name: str) -> tuple[np.ndarray, float]:  # type: ignore[type-arg]
        """Get specific keypoint by name.
        Returns:
            Tuple of (xy_coordinates, confidence_score)
        """
        if name not in self.KEYPOINT_NAMES:
            raise ValueError(f"Invalid keypoint name: {name}. Must be one of {self.KEYPOINT_NAMES}")

        idx = self.KEYPOINT_NAMES.index(name)
        return self.keypoints[idx], self.keypoint_scores[idx]

    def get_visible_keypoints(self, threshold: float = 0.5) -> list[tuple[str, np.ndarray, float]]:  # type: ignore[type-arg]
        """Get all keypoints above confidence threshold.
        Returns:
            List of tuples: (keypoint_name, xy_coordinates, confidence)
        """
        visible = []
        for i, (name, score) in enumerate(
            zip(self.KEYPOINT_NAMES, self.keypoint_scores, strict=False)
        ):
            if score > threshold:
                visible.append((name, self.keypoints[i], score))
        return visible

    @simple_mcache
    def is_valid(self) -> bool:
        valid_keypoints = sum(1 for score in self.keypoint_scores if score > 0.8)
        return valid_keypoints >= 5

    @property
    def width(self) -> float:
        """Get width of bounding box."""
        x1, _, x2, _ = self.bbox
        return x2 - x1

    @property
    def height(self) -> float:
        """Get height of bounding box."""
        _, y1, _, y2 = self.bbox
        return y2 - y1

    @property
    def center(self) -> tuple[float, float]:
        """Get center point of bounding box."""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def to_points_annotation(self) -> list[PointsAnnotation]:
        """Override to include keypoint visualizations along with bounding box."""
        annotations = []

        # First add the bounding box from parent class
        annotations.extend(super().to_points_annotation())

        # Add keypoints as circles
        visible_keypoints = self.get_visible_keypoints(threshold=0.3)

        # Create points for visible keypoints
        if visible_keypoints:
            keypoint_points = []
            for _name, xy, _conf in visible_keypoints:
                keypoint_points.append(Point2(float(xy[0]), float(xy[1])))

            # Add keypoints as circles
            annotations.append(
                PointsAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    outline_color=Color(r=0.0, g=1.0, b=0.0, a=1.0),  # Green outline
                    fill_color=Color(r=0.0, g=1.0, b=0.0, a=0.5),  # Semi-transparent green
                    thickness=2.0,
                    points_length=len(keypoint_points),
                    points=keypoint_points,
                    type=PointsAnnotation.POINTS,  # Draw as individual points/circles
                )
            )

        # Add skeleton connections (COCO skeleton)
        skeleton_connections = [
            # Face
            (0, 1),
            (0, 2),
            (1, 3),
            (2, 4),  # nose to eyes, eyes to ears
            # Arms
            (5, 6),  # shoulders
            (5, 7),
            (7, 9),  # left arm
            (6, 8),
            (8, 10),  # right arm
            # Torso
            (5, 11),
            (6, 12),
            (11, 12),  # shoulders to hips, hip to hip
            # Legs
            (11, 13),
            (13, 15),  # left leg
            (12, 14),
            (14, 16),  # right leg
        ]

        # Draw skeleton lines between connected keypoints
        for start_idx, end_idx in skeleton_connections:
            if (
                start_idx < len(self.keypoint_scores)
                and end_idx < len(self.keypoint_scores)
                and self.keypoint_scores[start_idx] > 0.3
                and self.keypoint_scores[end_idx] > 0.3
            ):
                start_point = Point2(
                    float(self.keypoints[start_idx][0]), float(self.keypoints[start_idx][1])
                )
                end_point = Point2(
                    float(self.keypoints[end_idx][0]), float(self.keypoints[end_idx][1])
                )

                annotations.append(
                    PointsAnnotation(
                        timestamp=to_ros_stamp(self.ts),
                        outline_color=Color(r=0.0, g=0.8, b=1.0, a=0.8),  # Cyan
                        thickness=1.5,
                        points_length=2,
                        points=[start_point, end_point],
                        type=PointsAnnotation.LINE_LIST,
                    )
                )

        return annotations

    def to_text_annotation(self) -> list[TextAnnotation]:
        """Override to include pose information in text annotations."""
        # Get base annotations from parent
        annotations = super().to_text_annotation()

        # Add pose-specific info
        visible_count = len(self.get_visible_keypoints(threshold=0.5))
        x1, _y1, _x2, y2 = self.bbox

        annotations.append(
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=x1, y=y2 + 40),  # Below confidence text
                text=f"keypoints: {visible_count}/17",
                font_size=18,
                text_color=Color(r=0.0, g=1.0, b=0.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=0.7),
            )
        )

        return annotations
