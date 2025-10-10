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
from typing import TYPE_CHECKING, List, Optional, Tuple

import numpy as np
from dimos_lcm.foxglove_msgs.ImageAnnotations import PointsAnnotation, TextAnnotation
from dimos_lcm.foxglove_msgs.Point2 import Point2

from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.type.detection2d import Bbox, Detection2DBBox
from dimos.types.timestamped import to_ros_stamp

if TYPE_CHECKING:
    from ultralytics.engine.results import Results


@dataclass
class Person(Detection2DBBox):
    """Represents a detected person with pose keypoints."""

    # Pose keypoints - additional fields beyond Detection2DBBox
    keypoints: np.ndarray  # [17, 2] - x,y coordinates
    keypoint_scores: np.ndarray  # [17] - confidence scores

    # Optional normalized coordinates
    bbox_normalized: Optional[np.ndarray] = None  # [x1, y1, x2, y2] in 0-1 range
    keypoints_normalized: Optional[np.ndarray] = None  # [17, 2] in 0-1 range

    # Image dimensions for context
    image_width: Optional[int] = None
    image_height: Optional[int] = None

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
    def from_yolo(cls, result: "Results", person_idx: int, image: Image) -> "Person":
        """Create Person instance from YOLO results.

        Args:
            result: Single Results object from YOLO
            person_idx: Index of the person in the detection results
            image: Original image for the detection
        """
        # Extract bounding box as tuple for Detection2DBBox
        bbox_array = result.boxes.xyxy[person_idx].cpu().numpy()

        bbox: Bbox = (
            float(bbox_array[0]),
            float(bbox_array[1]),
            float(bbox_array[2]),
            float(bbox_array[3]),
        )

        bbox_norm = (
            result.boxes.xyxyn[person_idx].cpu().numpy() if hasattr(result.boxes, "xyxyn") else None
        )

        confidence = float(result.boxes.conf[person_idx].cpu())
        class_id = int(result.boxes.cls[person_idx].cpu())

        # Extract keypoints
        keypoints = result.keypoints.xy[person_idx].cpu().numpy()
        keypoint_scores = result.keypoints.conf[person_idx].cpu().numpy()
        keypoints_norm = (
            result.keypoints.xyn[person_idx].cpu().numpy()
            if hasattr(result.keypoints, "xyn")
            else None
        )

        # Get image dimensions
        height, width = result.orig_shape

        return cls(
            # Detection2DBBox fields
            bbox=bbox,
            track_id=person_idx,  # Use person index as track_id for now
            class_id=class_id,
            confidence=confidence,
            name="person",
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

    def get_keypoint(self, name: str) -> Tuple[np.ndarray, float]:
        """Get specific keypoint by name.
        Returns:
            Tuple of (xy_coordinates, confidence_score)
        """
        if name not in self.KEYPOINT_NAMES:
            raise ValueError(f"Invalid keypoint name: {name}. Must be one of {self.KEYPOINT_NAMES}")

        idx = self.KEYPOINT_NAMES.index(name)
        return self.keypoints[idx], self.keypoint_scores[idx]

    def get_visible_keypoints(self, threshold: float = 0.5) -> List[Tuple[str, np.ndarray, float]]:
        """Get all keypoints above confidence threshold.
        Returns:
            List of tuples: (keypoint_name, xy_coordinates, confidence)
        """
        visible = []
        for i, (name, score) in enumerate(zip(self.KEYPOINT_NAMES, self.keypoint_scores)):
            if score > threshold:
                visible.append((name, self.keypoints[i], score))
        return visible

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
    def center(self) -> Tuple[float, float]:
        """Get center point of bounding box."""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def to_points_annotation(self) -> List[PointsAnnotation]:
        """Override to include keypoint visualizations along with bounding box."""
        annotations = []

        # First add the bounding box from parent class
        annotations.extend(super().to_points_annotation())

        # Add keypoints as circles
        visible_keypoints = self.get_visible_keypoints(threshold=0.3)

        # Create points for visible keypoints
        if visible_keypoints:
            keypoint_points = []
            for name, xy, conf in visible_keypoints:
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

    def to_text_annotation(self) -> List[TextAnnotation]:
        """Override to include pose information in text annotations."""
        # Get base annotations from parent
        annotations = super().to_text_annotation()

        # Add pose-specific info
        visible_count = len(self.get_visible_keypoints(threshold=0.5))
        x1, y1, x2, y2 = self.bbox

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
