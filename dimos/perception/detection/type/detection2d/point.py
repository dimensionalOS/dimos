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

from dimos_generated.vision_msgs.msg import (
    BoundingBox2D,
    Detection2D as ROSDetection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

from dimos.msgs.image import image_from_array, image_view
from dimos.perception.detection.type.detection2d.base import Detection2D

if TYPE_CHECKING:
    from dimos_generated.sensor_msgs.msg import Image


@dataclass
class Detection2DPoint(Detection2D):
    """A 2D point detection, visualized as a circle."""

    x: float
    y: float
    name: str
    ts: float
    image: Image
    track_id: int = -1
    class_id: int = -1
    confidence: float = 1.0

    def to_repr_dict(self) -> dict[str, str]:
        """Return a dictionary representation for display purposes."""
        return {
            "name": self.name,
            "track": str(self.track_id),
            "conf": f"{self.confidence:.2f}",
            "point": f"({self.x:.0f},{self.y:.0f})",
        }

    def cropped_image(self, padding: int = 20) -> Image:
        """Return a cropped version of the image focused on the point.

        Args:
            padding: Pixels to add around the point (default: 20)

        Returns:
            Cropped Image containing the area around the point
        """
        x, y = int(self.x), int(self.y)
        pixels = image_view(self.image)
        cropped = pixels[
            max(0, y - padding) : min(self.image.height, y + padding),
            max(0, x - padding) : min(self.image.width, x + padding),
        ]
        return image_from_array(cropped, encoding=self.image.encoding, header=self.image.header)

    def to_ros_detection2d(self) -> ROSDetection2D:
        """Convert point to ROS Detection2D message (as zero-size bbox at point)."""
        return ROSDetection2D(
            header=self.image.header,
            bbox=BoundingBox2D(
                center=Pose2D(
                    position=Point2D(x=self.x, y=self.y),
                    theta=0.0,
                ),
                size_x=0.0,
                size_y=0.0,
            ),
            results=[
                ObjectHypothesisWithPose(
                    hypothesis=ObjectHypothesis(
                        class_id=str(self.class_id),
                        score=self.confidence,
                    )
                )
            ],
            id=str(self.track_id),
        )

    def is_valid(self) -> bool:
        """Check if the point is within image bounds."""
        if self.image.height and self.image.width:
            h, w = self.image.height, self.image.width
            return bool(0 <= self.x <= w and 0 <= self.y <= h)
        return True
