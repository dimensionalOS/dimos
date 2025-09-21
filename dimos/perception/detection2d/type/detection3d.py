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
from typing import Any, Dict, Generic, List, TypeVar

import numpy as np
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.type.detection2d import Detection2D
from dimos.types.timestamped import to_timestamp


@dataclass
class Detection3D(Detection2D):
    pointcloud: PointCloud2
    transform: Transform

    def from_2d(self, det: Detection2D, **kwargs) -> "Detection3D":
        return Detection3D(
            image=det.image,
            bbox=det.bbox,
            track_id=det.track_id,
            class_id=det.class_id,
            confidence=det.confidence,
            name=det.name,
            ts=det.ts,
            **kwargs,
        )

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


class ImageDetections3D(ImageDetections[Detection3D]):
    """Specialized class for 3D detections in an image."""

    ...
