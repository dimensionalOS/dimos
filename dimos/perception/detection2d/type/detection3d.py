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
from typing import Any, Dict, Optional, TypeVar

import numpy as np
from dimos_lcm.sensor_msgs import CameraInfo
from lcm_msgs.builtin_interfaces import Duration
from lcm_msgs.foxglove_msgs import Color, CubePrimitive, SceneEntity, SceneUpdate, TextPrimitive
from lcm_msgs.geometry_msgs import Point, Pose, Quaternion
from lcm_msgs.geometry_msgs import Vector3 as LCMVector3

from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.perception.detection2d.type.detection2d import Detection2D
from dimos.perception.detection2d.type.imageDetections import ImageDetections
from dimos.types.timestamped import to_ros_stamp


@dataclass
class Detection3D(Detection2D):
    pointcloud: PointCloud2
    transform: Transform

    @classmethod
    def from_2d(
        cls,
        det: Detection2D,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_camera_transform: Transform,
        height_filter: Optional[float] = None,
    ) -> Optional["Detection3D"]:
        """Create a Detection3D from a 2D detection by projecting world pointcloud.

        This method handles:
        1. Projecting world pointcloud to camera frame
        2. Filtering points within the 2D detection bounding box
        3. Cleaning up the pointcloud (height filter, outlier removal)
        4. Hidden point removal from camera perspective

        Args:
            det: The 2D detection
            world_pointcloud: Full pointcloud in world frame
            camera_info: Camera calibration info
            world_to_camera_transform: Transform from world to camera frame
            height_filter: Optional minimum height filter (in world frame)

        Returns:
            Detection3D with filtered pointcloud, or None if no valid points
        """
        # Extract camera parameters
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        image_width = camera_info.width
        image_height = camera_info.height

        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        # Convert pointcloud to numpy array
        world_points = world_pointcloud.as_numpy()

        # Project points to camera frame
        points_homogeneous = np.hstack([world_points, np.ones((world_points.shape[0], 1))])
        extrinsics_matrix = world_to_camera_transform.to_matrix()
        points_camera = (extrinsics_matrix @ points_homogeneous.T).T

        # Filter out points behind the camera
        valid_mask = points_camera[:, 2] > 0
        points_camera = points_camera[valid_mask]
        world_points = world_points[valid_mask]

        if len(world_points) == 0:
            return None

        # Project to 2D
        points_2d_homogeneous = (camera_matrix @ points_camera[:, :3].T).T
        points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

        # Filter points within image bounds
        in_image_mask = (
            (points_2d[:, 0] >= 0)
            & (points_2d[:, 0] < image_width)
            & (points_2d[:, 1] >= 0)
            & (points_2d[:, 1] < image_height)
        )
        points_2d = points_2d[in_image_mask]
        world_points = world_points[in_image_mask]

        if len(world_points) == 0:
            return None

        # Extract bbox from Detection2D
        x_min, y_min, x_max, y_max = det.bbox

        # Find points within this detection box (with small margin)
        margin = 5  # pixels
        in_box_mask = (
            (points_2d[:, 0] >= x_min - margin)
            & (points_2d[:, 0] <= x_max + margin)
            & (points_2d[:, 1] >= y_min - margin)
            & (points_2d[:, 1] <= y_max + margin)
        )

        detection_points = world_points[in_box_mask]

        if detection_points.shape[0] == 0:
            return None

        # Create initial pointcloud for this detection
        detection_pc = PointCloud2.from_numpy(
            detection_points,
            frame_id=world_pointcloud.frame_id,
            timestamp=world_pointcloud.ts,
        )

        # Apply height filter if specified
        if height_filter is not None:
            detection_pc = detection_pc.filter_by_height(height_filter)
            if len(detection_pc.pointcloud.points) == 0:
                return None

        # Remove statistical outliers
        try:
            pcd = detection_pc.pointcloud
            statistical, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            detection_pc = PointCloud2(statistical, detection_pc.frame_id, detection_pc.ts)
        except Exception:
            # If outlier removal fails, continue with original
            pass

        # Hidden point removal from camera perspective
        camera_position = world_to_camera_transform.inverse().translation
        camera_pos_np = camera_position.to_numpy()

        try:
            pcd = detection_pc.pointcloud
            _, visible_indices = pcd.hidden_point_removal(camera_pos_np, radius=100.0)
            visible_pcd = pcd.select_by_index(visible_indices)
            detection_pc = PointCloud2(
                visible_pcd, frame_id=detection_pc.frame_id, ts=detection_pc.ts
            )
        except Exception:
            # If hidden point removal fails, continue with current pointcloud
            pass

        # Final check for empty pointcloud
        if len(detection_pc.pointcloud.points) == 0:
            return None

        # Create Detection3D with filtered pointcloud
        return Detection3D(
            image=det.image,
            bbox=det.bbox,
            track_id=det.track_id,
            class_id=det.class_id,
            confidence=det.confidence,
            name=det.name,
            ts=det.ts,
            pointcloud=detection_pc,
            transform=world_to_camera_transform,
        )

    @functools.cached_property
    def center(self) -> Vector3:
        return self.pointcloud.center

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

    def bounding_box_intersects(self, other: "Detection3D") -> bool:
        """Check if this detection's bounding box intersects with another's."""
        return self.pointcloud.bounding_box_intersects(other.pointcloud)

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

        # Set color (red with transparency)
        cube.color = Color()
        cube.color.r = 1.0
        cube.color.g = 0.0
        cube.color.b = 0.0
        cube.color.a = 0.2

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
        text.font_size = 25.0
        text.scale_invariant = True
        text.color = Color()
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"{self.track_id}/{self.name} ({self.confidence:.0%})"

        # Create scene entity
        entity = SceneEntity()
        entity.timestamp = to_ros_stamp(self.ts)
        entity.frame_id = "world"
        entity.id = self.track_id
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

    def to_foxglove_scene_update(self) -> "SceneUpdate":
        """Convert all detections to a Foxglove SceneUpdate message.

        Returns:
            SceneUpdate containing SceneEntity objects for all detections
        """
        from lcm_msgs.foxglove_msgs import SceneUpdate

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
