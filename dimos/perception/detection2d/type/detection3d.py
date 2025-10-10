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
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, TypeVar

import numpy as np
from dimos_lcm.sensor_msgs import CameraInfo
from lcm_msgs.builtin_interfaces import Duration
from lcm_msgs.foxglove_msgs import CubePrimitive, SceneEntity, SceneUpdate, TextPrimitive
from lcm_msgs.geometry_msgs import Point, Pose, Quaternion
from lcm_msgs.geometry_msgs import Vector3 as LCMVector3

from dimos.msgs.foxglove_msgs.Color import Color
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.perception.detection2d.type.detection2d import Detection2D, Detection2DBBox
from dimos.perception.detection2d.type.imageDetections import ImageDetections
from dimos.types.timestamped import to_ros_stamp


@dataclass
class Detection3D(Detection2DBBox):
    transform: Transform
    frame_id: str

    @classmethod
    def from_2d(
        cls,
        det: Detection2D,
        distance: float,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
    ) -> Optional["Detection3D"]:
        raise NotImplementedError()

    @functools.cached_property
    def center(self) -> Vector3:
        return Vector3(*self.pointcloud.center)

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using pointcloud center.

        Returns pose in world frame with identity rotation.
        The pointcloud is already in world frame.
        """
        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
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

    def bounding_box_intersects(self, other: "Detection3D") -> bool:
        """Check if this detection's bounding box intersects with another's."""
        return self.pointcloud.bounding_box_intersects(other.pointcloud)

    def to_repr_dict(self) -> Dict[str, Any]:
        # Calculate distance from camera
        # The pointcloud is in world frame, and transform gives camera position in world
        center_world = self.center
        # Camera position in world frame is the translation part of the transform
        camera_pos = self.transform.translation
        # Use Vector3 subtraction and magnitude
        distance = (center_world - camera_pos).magnitude()

        parent_dict = super().to_repr_dict()
        # Remove bbox key if present
        parent_dict.pop("bbox", None)

        return {
            **parent_dict,
            "dist": f"{distance:.2f}m",
            "points": str(len(self.pointcloud)),
        }

    def to_foxglove_scene_entity(self, entity_id: str = None) -> "SceneEntity":
        """Convert detection to a Foxglove SceneEntity with cube primitive and text label.

        Args:
            entity_id: Optional custom entity ID. If None, generates one from name and hash.

        Returns:
            SceneEntity with cube bounding box and text label
        """

        # Create a cube primitive for the bounding box
        cube = CubePrimitive()

        # Get the axis-aligned bounding box
        aabb = self.get_bounding_box()

        # Set pose from axis-aligned bounding box
        cube.pose = Pose()
        cube.pose.position = Point()
        # Get center of the axis-aligned bounding box
        aabb_center = aabb.get_center()
        cube.pose.position.x = aabb_center[0]
        cube.pose.position.y = aabb_center[1]
        cube.pose.position.z = aabb_center[2]

        # For axis-aligned box, use identity quaternion (no rotation)
        cube.pose.orientation = Quaternion()
        cube.pose.orientation.x = 0
        cube.pose.orientation.y = 0
        cube.pose.orientation.z = 0
        cube.pose.orientation.w = 1

        # Set size from axis-aligned bounding box
        cube.size = LCMVector3()
        aabb_extent = aabb.get_extent()
        cube.size.x = aabb_extent[0]  # width
        cube.size.y = aabb_extent[1]  # height
        cube.size.z = aabb_extent[2]  # depth

        # Set color based on name hash
        cube.color = Color.from_string(self.name, alpha=0.2)

        # Create text label
        text = TextPrimitive()
        text.pose = Pose()
        text.pose.position = Point()
        text.pose.position.x = aabb_center[0]
        text.pose.position.y = aabb_center[1]
        text.pose.position.z = aabb_center[2] + aabb_extent[2] / 2 + 0.1  # Above the box
        text.pose.orientation = Quaternion()
        text.pose.orientation.x = 0
        text.pose.orientation.y = 0
        text.pose.orientation.z = 0
        text.pose.orientation.w = 1
        text.billboard = True
        text.font_size = 20.0
        text.scale_invariant = True
        text.color = Color()
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = self.scene_entity_label()

        # Create scene entity
        entity = SceneEntity()
        entity.timestamp = to_ros_stamp(self.ts)
        entity.frame_id = self.frame_id
        entity.id = str(self.track_id)
        entity.lifetime = Duration()
        entity.lifetime.sec = 0  # Persistent
        entity.lifetime.nanosec = 0
        entity.frame_locked = False

        # Initialize all primitive arrays
        entity.metadata_length = 0
        entity.metadata = []
        entity.arrows_length = 0
        entity.arrows = []
        entity.cubes_length = 1
        entity.cubes = [cube]
        entity.spheres_length = 0
        entity.spheres = []
        entity.cylinders_length = 0
        entity.cylinders = []
        entity.lines_length = 0
        entity.lines = []
        entity.triangles_length = 0
        entity.triangles = []
        entity.texts_length = 1
        entity.texts = [text]
        entity.models_length = 0
        entity.models = []

        return entity

    def scene_entity_label(self) -> str:
        return f"{self.track_id}/{self.name} ({self.confidence:.0%})"


T = TypeVar("T", bound="Detection2D")


class ImageDetections3D(ImageDetections[Detection3D]):
    """Specialized class for 3D detections in an image."""

    def to_foxglove_scene_update(self) -> "SceneUpdate":
        """Convert all detections to a Foxglove SceneUpdate message.

        Returns:
            SceneUpdate containing SceneEntity objects for all detections
        """

        # Create SceneUpdate message with all detections
        scene_update = SceneUpdate()
        scene_update.deletions_length = 0
        scene_update.deletions = []
        scene_update.entities = []

        # Process each detection
        for i, detection in enumerate(self.detections):
            entity = detection.to_foxglove_scene_entity(entity_id=f"detection_{detection.name}_{i}")
            scene_update.entities.append(entity)

        scene_update.entities_length = len(scene_update.entities)
        return scene_update
