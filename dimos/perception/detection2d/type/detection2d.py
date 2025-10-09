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

import hashlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, List, Tuple

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    PointsAnnotation,
    TextAnnotation,
)
from dimos_lcm.foxglove_msgs.Point2 import Point2
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
from dimos_lcm.vision_msgs import (
    Detection2D as ROSDetection2D,
)
from rich.console import Console
from rich.text import Text

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Header
from dimos.perception.detection2d.type.imageDetections import ImageDetections
from dimos.types.timestamped import Timestamped, to_ros_stamp, to_timestamp

if TYPE_CHECKING:
    from dimos.perception.detection2d.type.person import Person

Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]

# yolo and detic have bad output formats
InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[str]]

Detection = Tuple[Bbox, int, int, float, str]
Detections = List[Detection]


def _hash_to_color(name: str) -> str:
    """Generate a consistent color for a given name using hash."""
    # List of rich colors to choose from
    colors = [
        "cyan",
        "magenta",
        "yellow",
        "blue",
        "green",
        "red",
        "bright_cyan",
        "bright_magenta",
        "bright_yellow",
        "bright_blue",
        "bright_green",
        "bright_red",
        "purple",
        "white",
        "pink",
    ]

    # Hash the name and pick a color
    hash_value = hashlib.md5(name.encode()).digest()[0]
    return colors[hash_value % len(colors)]


# yolo and detic have bad formats this translates into list of detections
def better_detection_format(inconvinient_detections: InconvinientDetectionFormat) -> Detections:
    bboxes, track_ids, class_ids, confidences, names = inconvinient_detections
    return [
        (bbox, track_id, class_id, confidence, name if name else "")
        for bbox, track_id, class_id, confidence, name in zip(
            bboxes, track_ids, class_ids, confidences, names
        )
    ]


class Detection2D(Timestamped):
    @abstractmethod
    def cropped_image(self, padding: int = 20) -> Image: ...

    @abstractmethod
    def to_image_annotations(self) -> ImageAnnotations: ...


@dataclass
class Detection2DBBox(Detection2D):
    bbox: Bbox
    track_id: int
    class_id: int
    confidence: float
    name: str
    ts: float
    image: Image

    def to_repr_dict(self) -> Dict[str, Any]:
        """Return a dictionary representation of the detection for display purposes."""
        x1, y1, x2, y2 = self.bbox
        return {
            "name": self.name,
            "class": str(self.class_id),
            "track": str(self.track_id),
            "conf": f"{self.confidence:.2f}",
            "bbox": f"[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]",
        }

    # return focused image, only on the bbox
    def cropped_image(self, padding: int = 20) -> Image:
        """Return a cropped version of the image focused on the bounding box.

        Args:
            padding: Pixels to add around the bounding box (default: 20)

        Returns:
            Cropped Image containing only the detection area plus padding
        """
        x1, y1, x2, y2 = map(int, self.bbox)
        return self.image.crop(
            x1 - padding, y1 - padding, x2 - x1 + 2 * padding, y2 - y1 + 2 * padding
        )

    def __str__(self):
        console = Console(force_terminal=True, legacy_windows=False)
        d = self.to_repr_dict()

        # Build the string representation
        parts = [
            Text(f"{self.__class__.__name__}("),
        ]

        # Add any extra fields (e.g., points for Detection3D)
        extra_keys = [k for k in d.keys() if k not in ["class"]]
        for key in extra_keys:
            if d[key] == "None":
                parts.append(Text(f"{key}={d[key]}", style="dim"))
            else:
                parts.append(Text(f"{key}={d[key]}", style=_hash_to_color(key)))

        parts.append(Text(")"))

        # Render to string
        with console.capture() as capture:
            console.print(*parts, end="")
        return capture.get().strip()

    def bbox_2d_volume(self) -> float:
        x1, y1, x2, y2 = self.bbox
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        return width * height

    @classmethod
    def from_detector(
        cls, raw_detections: InconvinientDetectionFormat, **kwargs
    ) -> List["Detection2D"]:
        return [
            cls.from_detection(raw, **kwargs) for raw in better_detection_format(raw_detections)
        ]

    @classmethod
    def from_detection(cls, raw_detection: Detection, **kwargs) -> "Detection2D":
        bbox, track_id, class_id, confidence, name = raw_detection

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            **kwargs,
        )

    def get_bbox_center(self) -> CenteredBbox:
        x1, y1, x2, y2 = self.bbox
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        width = float(x2 - x1)
        height = float(y2 - y1)
        return (center_x, center_y, width, height)

    def to_ros_bbox(self) -> BoundingBox2D:
        center_x, center_y, width, height = self.get_bbox_center()
        return BoundingBox2D(
            center=Pose2D(
                position=Point2D(x=center_x, y=center_y),
                theta=0.0,
            ),
            size_x=width,
            size_y=height,
        )

    def lcm_encode(self):
        return self.to_image_annotations().lcm_encode()

    def to_text_annotation(self) -> List[TextAnnotation]:
        x1, y1, x2, y2 = self.bbox

        font_size = 20

        return [
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=x1, y=y2 + font_size),
                text=f"confidence: {self.confidence:.3f}",
                font_size=font_size,
                text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=1),
            ),
            TextAnnotation(
                timestamp=to_ros_stamp(self.ts),
                position=Point2(x=x1, y=y1),
                text=f"{self.name}_{self.class_id}_{self.track_id}",
                font_size=font_size,
                text_color=Color(r=1.0, g=1.0, b=1.0, a=1),
                background_color=Color(r=0, g=0, b=0, a=1),
            ),
        ]

    def to_points_annotation(self) -> List[PointsAnnotation]:
        x1, y1, x2, y2 = self.bbox

        thickness = 1

        return [
            PointsAnnotation(
                timestamp=to_ros_stamp(self.ts),
                outline_color=Color(r=0.0, g=0.0, b=0.0, a=1.0),
                fill_color=Color.from_string(self.name, alpha=0.15),
                thickness=thickness,
                points_length=4,
                points=[
                    Point2(x1, y1),
                    Point2(x1, y2),
                    Point2(x2, y2),
                    Point2(x2, y1),
                ],
                type=PointsAnnotation.LINE_LOOP,
            )
        ]

    # this is almost never called directly since this is a single detection
    # and ImageAnnotations message normally contains multiple detections annotations
    # so ImageDetections2D and ImageDetections3D normally implements this for whole image
    def to_image_annotations(self) -> ImageAnnotations:
        points = self.to_points_annotation()
        texts = self.to_text_annotation()

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
        )

    @classmethod
    def from_ros_detection2d(cls, ros_det: ROSDetection2D, **kwargs) -> "Detection2D":
        """Convert from ROS Detection2D message to Detection2D object."""
        # Extract bbox from ROS format
        center_x = ros_det.bbox.center.position.x
        center_y = ros_det.bbox.center.position.y
        width = ros_det.bbox.size_x
        height = ros_det.bbox.size_y

        # Convert centered bbox to corner format
        x1 = center_x - width / 2.0
        y1 = center_y - height / 2.0
        x2 = center_x + width / 2.0
        y2 = center_y + height / 2.0
        bbox = (x1, y1, x2, y2)

        # Extract hypothesis info
        class_id = 0
        confidence = 0.0
        if ros_det.results:
            hypothesis = ros_det.results[0].hypothesis
            class_id = hypothesis.class_id
            confidence = hypothesis.score

        # Extract track_id
        track_id = int(ros_det.id) if ros_det.id.isdigit() else 0

        # Extract timestamp
        ts = to_timestamp(ros_det.header.stamp)

        # Name is not stored in ROS Detection2D, so we'll use a placeholder
        # Remove 'name' from kwargs if present to avoid duplicate
        name = kwargs.pop("name", f"class_{class_id}")

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=ts,
            **kwargs,
        )

    def to_ros_detection2d(self) -> ROSDetection2D:
        return ROSDetection2D(
            header=Header(self.ts, "camera_link"),
            bbox=self.to_ros_bbox(),
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


class ImageDetections2D(ImageDetections[Detection2D]):
    @classmethod
    def from_bbox_detector(
        cls, image: Image, raw_detections: InconvinientDetectionFormat, **kwargs
    ) -> "ImageDetections2D":
        return cls(
            image=image,
            detections=Detection2DBBox.from_detector(raw_detections, image=image, ts=image.ts),
        )

    @classmethod
    def from_pose_detector(
        cls, image: Image, people: List["Person"], **kwargs
    ) -> "ImageDetections2D":
        """Create ImageDetections2D from a list of Person detections.
        Args:
            image: Source image
            people: List of Person objects with pose keypoints
        Returns:
            ImageDetections2D containing the pose detections
        """
        return cls(
            image=image,
            detections=people,  # Person objects are already Detection2D subclasses
        )
