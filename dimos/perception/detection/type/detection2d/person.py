# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.utils.decorators.decorators import simple_mcache

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

    from dimos.msgs.sensor_msgs.Image import Image


@dataclass
class Detection2DPerson(Detection2DBBox):
    """Represents a detected person with pose keypoints."""

    # Pose keypoints - additional fields beyond Detection2DBBox
    keypoints: np.ndarray  # [17, 2] - x,y coordinates
    keypoint_scores: np.ndarray  # [17] - confidence scores

    # Optional normalized coordinates
    bbox_normalized: np.ndarray | None = None  # [x1, y1, x2, y2] in 0-1 range
    keypoints_normalized: np.ndarray | None = None  # [17, 2] in 0-1 range

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
    def from_ultralytics_result(cls, result: Results, idx: int, image: Image) -> Detection2DPerson:
        # The adapter imports the detection types, so import it here to avoid a cycle.
        from dimos.perception.detection.detectors.ultralytics_utils import person_from_result

        return person_from_result(result, idx, image, detection_type=cls)

    @classmethod
    def from_yolo(cls, result: Results, idx: int, image: Image) -> Detection2DPerson:
        return cls.from_ultralytics_result(result, idx, image)

    @classmethod
    def from_ros_detection2d(cls, *args, **kwargs) -> Detection2DPerson:  # type: ignore[no-untyped-def]
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

    def get_keypoint(self, name: str) -> tuple[np.ndarray, float]:
        """Get specific keypoint by name.
        Returns:
            Tuple of (xy_coordinates, confidence_score)
        """
        if name not in self.KEYPOINT_NAMES:
            raise ValueError(f"Invalid keypoint name: {name}. Must be one of {self.KEYPOINT_NAMES}")

        idx = self.KEYPOINT_NAMES.index(name)
        return self.keypoints[idx], self.keypoint_scores[idx]

    def get_visible_keypoints(self, threshold: float = 0.5) -> list[tuple[str, np.ndarray, float]]:
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
