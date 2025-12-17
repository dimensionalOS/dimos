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

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
    CircleAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2  # type: ignore[import-untyped]
from dimos_lcm.vision_msgs import (  # type: ignore[import-untyped]
    BoundingBox2D,
    Detection2D as ROSDetection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.std_msgs import Header
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.types.timestamped import to_ros_stamp

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import Image


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
        return self.image.crop(
            x - padding,
            y - padding,
            2 * padding,
            2 * padding,
        )

    @property
    def diameter(self) -> float:
        return self.image.width / 40

    def to_circle_annotation(self) -> list[CircleAnnotation]:
        """Return circle annotations for visualization."""
        return [
            CircleAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=self.x, y=self.y),
                diameter=self.diameter,
                thickness=1.0,
                fill_color=Color.from_string(self.name, alpha=0.3),
                outline_color=Color.from_string(self.name, alpha=1.0, brightness=1.25),
            )
        ]

    def to_text_annotation(self) -> list[TextAnnotation]:
        """Return text annotations for visualization."""
        font_size = self.image.width / 80

        # Build label text
        if self.class_id == -1:
            if self.track_id == -1:
                label_text = self.name
            else:
                label_text = f"{self.name}_{self.track_id}"
        else:
            label_text = f"{self.name}_{self.class_id}_{self.track_id}"

        annotations = [
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=self.x + self.diameter / 2, y=self.y + self.diameter / 2),
                text=label_text,
                font_size=font_size,
                text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=1),
            ),
        ]

        # Only show confidence if it's not 1.0
        if self.confidence != 1.0:
            annotations.append(
                TextAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    position=Point2(x=self.x + self.diameter / 2 + 2, y=self.y + font_size + 2),
                    text=f"{self.confidence:.2f}",
                    font_size=font_size,
                    text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                    background_color=Color(r=0, g=0, b=0, a=1),
                )
            )

        return annotations

    def to_image_annotations(self) -> ImageAnnotations:
        """Convert detection to Foxglove ImageAnnotations for visualization."""
        texts = self.to_text_annotation()
        circles = self.to_circle_annotation()

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=[],
            points_length=0,
            circles=circles,
            circles_length=len(circles),
        )

    def to_ros_detection2d(self) -> ROSDetection2D:
        """Convert point to ROS Detection2D message (as zero-size bbox at point)."""
        return ROSDetection2D(
            header=Header(self.ts, "camera_link"),
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
                    ObjectHypothesis(
                        class_id=self.class_id,
                        score=self.confidence,
                    )
                )
            ],
            id=str(self.track_id),
        )

    def is_valid(self) -> bool:
        """Check if the point is within image bounds."""
        if self.image.shape:
            h, w = self.image.shape[:2]
            return bool(0 <= self.x <= w and 0 <= self.y <= h)
        return True

    def lcm_encode(self):  # type: ignore[no-untyped-def]
        return self.to_image_annotations().lcm_encode()
