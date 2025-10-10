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
from dimos.perception.detection2d.type.detection2d import Detection2D
from dimos.perception.detection2d.type.detection3d import Detection3D
from dimos.perception.detection2d.type.imageDetections import ImageDetections
from dimos.types.timestamped import to_ros_stamp

Detection3DPCFilter = Callable[
    [Detection2D, PointCloud2, CameraInfo, Transform], Optional["Detection3DPC"]
]


def height_filter(height=0.1) -> Detection3DPCFilter:
    return lambda det, pc, ci, tf: pc.filter_by_height(height)


def statistical(nb_neighbors=40, std_ratio=0.5) -> Detection3DPCFilter:
    def filter_func(
        det: Detection2D, pc: PointCloud2, ci: CameraInfo, tf: Transform
    ) -> Optional[PointCloud2]:
        try:
            statistical, removed = pc.pointcloud.remove_statistical_outlier(
                nb_neighbors=nb_neighbors, std_ratio=std_ratio
            )
            return PointCloud2(statistical, pc.frame_id, pc.ts)
        except Exception as e:
            # print("statistical filter failed:", e)
            return None

    return filter_func


def raycast() -> Detection3DPCFilter:
    def filter_func(
        det: Detection2D, pc: PointCloud2, ci: CameraInfo, tf: Transform
    ) -> Optional[PointCloud2]:
        try:
            camera_pos = tf.inverse().translation
            camera_pos_np = camera_pos.to_numpy()
            _, visible_indices = pc.pointcloud.hidden_point_removal(camera_pos_np, radius=100.0)
            visible_pcd = pc.pointcloud.select_by_index(visible_indices)
            return PointCloud2(visible_pcd, pc.frame_id, pc.ts)
        except Exception as e:
            # print("raycast filter failed:", e)
            return None

    return filter_func


def radius_outlier(min_neighbors: int = 20, radius: float = 0.3) -> Detection3DPCFilter:
    """
    Remove isolated points: keep only points that have at least `min_neighbors`
    neighbors within `radius` meters (same units as your point cloud).
    """

    def filter_func(
        det: Detection2D, pc: PointCloud2, ci: CameraInfo, tf: Transform
    ) -> Optional[PointCloud2]:
        filtered_pcd, removed = pc.pointcloud.remove_radius_outlier(
            nb_points=min_neighbors, radius=radius
        )
        return PointCloud2(filtered_pcd, pc.frame_id, pc.ts)

    return filter_func


@dataclass
class Detection3DPC(Detection3D):
    pointcloud: PointCloud2

    @classmethod
    def from_2d(
        cls,
        det: Detection2D,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        # filters are to be adjusted based on the sensor noise characteristics if feeding
        # sensor data directly
        filters: list[Callable[[PointCloud2], PointCloud2]] = [
            # height_filter(0.1),
            raycast(),
            radius_outlier(),
            statistical(),
        ],
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
            world_to_camerlka_transform: Transform from world to camera frame
            filters: List of functions to apply to the pointcloud for filtering
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
        extrinsics_matrix = world_to_optical_transform.to_matrix()
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
            # print(f"No points found in detection bbox after projection. {det.name}")
            return None

        # Create initial pointcloud for this detection
        initial_pc = PointCloud2.from_numpy(
            detection_points,
            frame_id=world_pointcloud.frame_id,
            timestamp=world_pointcloud.ts,
        )

        # Apply filters - each filter needs all 4 arguments
        detection_pc = initial_pc
        for filter_func in filters:
            result = filter_func(det, detection_pc, camera_info, world_to_optical_transform)
            if result is None:
                return None
            detection_pc = result

        # Final check for empty pointcloud
        if len(detection_pc.pointcloud.points) == 0:
            return None

        # Create Detection3D with filtered pointcloud
        return cls(
            image=det.image,
            bbox=det.bbox,
            track_id=det.track_id,
            class_id=det.class_id,
            confidence=det.confidence,
            name=det.name,
            ts=det.ts,
            pointcloud=detection_pc,
            transform=world_to_optical_transform,
            frame_id=world_pointcloud.frame_id,
        )


class ImageDetections3DPC(ImageDetections[Detection3DPC]):
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
