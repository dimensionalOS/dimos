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
from dimos_lcm.geometry_msgs import Point
from dimos_lcm.std_msgs import ColorRGBA
from dimos_lcm.visualization_msgs import Marker, MarkerArray
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection2d.type.detection2d import Detection2D
from dimos.perception.detection2d.type.imageDetections import ImageDetections
from dimos.types.timestamped import to_timestamp


@dataclass
class Detection3D(Detection2D):
    pointcloud: PointCloud2
    transform: Transform

    @classmethod
    def from_2d(cls, det: Detection2D, **kwargs) -> "Detection3D":
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

    def get_bounding_box(self):
        """Get axis-aligned bounding box of the detection's pointcloud."""
        return self.pointcloud.get_axis_aligned_bounding_box()

    def get_oriented_bounding_box(self):
        """Get oriented bounding box of the detection's pointcloud."""
        return self.pointcloud.get_oriented_bounding_box()

    def get_bounding_box_dimensions(self) -> tuple[float, float, float]:
        """Get dimensions (width, height, depth) of the detection's bounding box."""
        return self.pointcloud.get_bounding_box_dimensions()

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


class ImageDetections3D(ImageDetections[Detection3D]):
    """Specialized class for 3D detections in an image."""

    def to_ros_markers(self, ns: str = "detections_3d") -> MarkerArray:
        """Convert all detections to a ROS MarkerArray.

        Args:
            ns: Namespace for the markers

        Returns:
            MarkerArray containing oriented bounding box markers for all detections
        """
        marker_array = MarkerArray()
        marker_array.markers = []

        for i, detection in enumerate(self.detections):
            marker = detection.to_ros_marker(marker_id=i, ns=ns)
            marker_array.markers.append(marker)

        marker_array.markers_length = len(marker_array.markers)
        return marker_array
