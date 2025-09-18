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

import functools
import hashlib
from dataclasses import dataclass
from typing import Any, Dict, Generic, List, Optional, Tuple, TypeVar

import numpy as np
from dimos_lcm.foxglove_msgs.Color import Color
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
from rich.table import Table
from rich.text import Text

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.types.timestamped import Timestamped, to_ros_stamp, to_timestamp

Bbox = Tuple[float, float, float, float]
CenteredBbox = Tuple[float, float, float, float]
# yolo and detic have bad output formats
InconvinientDetectionFormat = Tuple[List[Bbox], List[int], List[int], List[float], List[str]]

Detection = Tuple[Bbox, int, int, float, str]
Detections = List[Detection]


# yolo and detic have bad formats this translates into list of detections
def better_detection_format(inconvinient_detections: InconvinientDetectionFormat) -> Detections:
    bboxes, track_ids, class_ids, confidences, names = inconvinient_detections
    return [
        (bbox, track_id, class_id, confidence, name if name else "")
        for bbox, track_id, class_id, confidence, name in zip(
            bboxes, track_ids, class_ids, confidences, names
        )
    ]


@dataclass
class Detection2D(Timestamped):
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
            "conf": self.confidence,
            "bbox": f"[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]",
        }

    def to_image(self) -> Image:
        return self.image

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

        # Create confidence text with color based on value
        conf_color = "green" if d["conf"] > 0.8 else "yellow" if d["conf"] > 0.5 else "red"
        conf_text = Text(f"{d['conf']:.1%}", style=conf_color)

        # Build the string representation
        parts = [
            Text(f"{self.__class__.__name__}("),
            Text(d["name"], style="bold cyan"),
            Text(f" cls={d['class']} trk={d['track']} "),
            conf_text,
            Text(f" {d['bbox']}"),
        ]

        # Add any extra fields (e.g., points for Detection3D)
        extra_keys = [k for k in d.keys() if k not in ["name", "class", "track", "conf", "bbox"]]
        for key in extra_keys:
            if d[key] == "None":
                parts.append(Text(f" {key}={d[key]}", style="dim"))
            else:
                parts.append(Text(f" {key}={d[key]}", style="blue"))

        parts.append(Text(")"))

        # Render to string
        with console.capture() as capture:
            console.print(*parts, end="")
        return capture.get().strip()

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
        return self.to_imageannotations().lcm_encode()

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
                fill_color=Color(r=1.0, g=0.0, b=0.0, a=0.15),
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
    def to_annotations(self) -> ImageAnnotations:
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

    def to_3d(self, **kwargs) -> "Detection3D":
        return Detection3D(
            image=self.image,
            bbox=self.bbox,
            track_id=self.track_id,
            class_id=self.class_id,
            confidence=self.confidence,
            name=self.name,
            ts=self.ts,
            **kwargs,
        )


@dataclass
class Detection3D(Detection2D):
    pointcloud: PointCloud2
    transform: Transform

    def localize(self, pointcloud: PointCloud2) -> Detection3D:
        self.pointcloud = pointcloud
        return self

    @functools.cached_property
    def center(self) -> Vector3:
        """Calculate the center of the pointcloud in world frame."""
        points = np.asarray(self.pointcloud.pointcloud.points)
        center = points.mean(axis=0)
        return Vector3(*center)

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using pointcloud center.

        Returns pose in world frame with identity rotation.
        The pointcloud is already in world frame.
        """
        return PoseStamped(
            ts=self.ts,
            frame_id="world",
            position=self.center,
            orientation=(0.0, 0.0, 0.0, 1.0),  # Identity quaternion
        )

    def to_repr_dict(self) -> Dict[str, Any]:
        d = super().to_repr_dict()

        # Add pointcloud info
        d["points"] = str(len(self.pointcloud))

        # Calculate distance from camera
        # The pointcloud is in world frame, and transform gives camera position in world
        center_world = self.center
        # Camera position in world frame is the translation part of the transform
        camera_pos = self.transform.translation
        # Use Vector3 subtraction and magnitude
        distance = (center_world - camera_pos).magnitude()
        d["dist"] = f"{distance:.2f}m"

        return d


T = TypeVar("T", bound="Detection2D")


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


class ImageDetections(Generic[T]):
    image: Image
    detections: List[T]

    def __init__(self, image: Image, detections: List[T]):
        self.image = image
        self.detections = detections
        for det in self.detections:
            if not det.ts:
                det.ts = image.ts

    def __str__(self):
        console = Console(force_terminal=True, legacy_windows=False)

        # Dynamically build columns based on the first detection's dict keys
        if not self.detections:
            return "Empty ImageDetections"

        # Create a table for detections
        table = Table(
            title=f"{self.__class__.__name__} [{len(self.detections)} detections @ {to_timestamp(self.image.ts):.3f}]",
            show_header=True,
            show_edge=True,
        )

        # Cache all repr_dicts to avoid double computation
        detection_dicts = [det.to_repr_dict() for det in self.detections]

        first_dict = detection_dicts[0]
        table.add_column("#", style="dim")
        for col in first_dict.keys():
            color = _hash_to_color(col)
            table.add_column(col.title(), style=color)

        # Add each detection to the table
        for i, d in enumerate(detection_dicts):
            row = [str(i)]

            for key in first_dict.keys():
                if key == "conf":
                    # Color-code confidence
                    conf_color = "green" if d[key] > 0.8 else "yellow" if d[key] > 0.5 else "red"
                    row.append(Text(f"{d[key]:.1%}", style=conf_color))
                elif key == "points" and d.get(key) == "None":
                    row.append(Text(d.get(key, ""), style="dim"))
                else:
                    row.append(str(d.get(key, "")))
            table.add_row(*row)

        with console.capture() as capture:
            console.print(table)
        return capture.get().strip()

    def __len__(self):
        return len(self.detections)

    def __iter__(self):
        return iter(self.detections)

    def __getitem__(self, index):
        return self.detections[index]

    def to_ros_detection2d_array(self) -> Detection2DArray:
        return Detection2DArray(
            detections_length=len(self.detections),
            header=Header(self.image.ts, "camera_optical"),
            detections=[det.to_ros_detection2d() for det in self.detections],
        )

    def to_image_annotations(self) -> ImageAnnotations:
        def flatten(xss):
            return [x for xs in xss for x in xs]

        texts = flatten(det.to_text_annotation() for det in self.detections)
        points = flatten(det.to_points_annotation() for det in self.detections)

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
        )


class ImageDetections2D(ImageDetections[Detection2D]):
    @classmethod
    def from_detector(
        cls, image: Image, raw_detections: InconvinientDetectionFormat, **kwargs
    ) -> "ImageDetections2D":
        return cls(
            image=image,
            detections=Detection2D.from_detector(raw_detections, image=image, ts=image.ts),
        )


class ImageDetections3D(ImageDetections[Detection3D]):
    """Specialized class for 3D detections in an image."""

    ...
