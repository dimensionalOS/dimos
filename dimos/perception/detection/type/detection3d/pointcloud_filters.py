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

from collections.abc import Callable
from typing import TYPE_CHECKING

from dimos_generated.geometry_msgs.msg import TransformStamped
from dimos_generated.sensor_msgs.msg import CameraInfo, PointCloud2
import numpy as np
import open3d as o3d

from dimos.msgs.geometry import inverse_transform
from dimos.msgs.pointcloud import pointcloud_xyz, select_points

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox

PointCloudFilter = Callable[
    ["Detection2DBBox", PointCloud2, CameraInfo, TransformStamped], PointCloud2 | None
]


def _open3d_cloud(pc: PointCloud2) -> o3d.geometry.PointCloud:
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pointcloud_xyz(pc)))


def _selected(pc: PointCloud2, indices: list[int]) -> PointCloud2:
    keep = np.zeros(pc.width * pc.height, dtype=bool)
    keep[indices] = True
    return select_points(pc, keep)


def height_filter(height: float = 0.1) -> PointCloudFilter:
    return lambda det, pc, ci, tf: select_points(pc, pointcloud_xyz(pc)[:, 2] >= height)


def statistical(nb_neighbors: int = 40, std_ratio: float = 0.5) -> PointCloudFilter:
    def filter_func(
        det: Detection2DBBox, pc: PointCloud2, ci: CameraInfo, tf: TransformStamped
    ) -> PointCloud2 | None:
        try:
            _, indices = _open3d_cloud(pc).remove_statistical_outlier(
                nb_neighbors=nb_neighbors, std_ratio=std_ratio
            )
            return _selected(pc, indices)
        except RuntimeError:
            return None

    return filter_func


def raycast() -> PointCloudFilter:
    def filter_func(
        det: Detection2DBBox, pc: PointCloud2, ci: CameraInfo, tf: TransformStamped
    ) -> PointCloud2 | None:
        try:
            camera_pos = inverse_transform(tf).transform.translation
            _, indices = _open3d_cloud(pc).hidden_point_removal(
                np.array([camera_pos.x, camera_pos.y, camera_pos.z]), radius=100.0
            )
            return _selected(pc, indices)
        except RuntimeError:
            return None

    return filter_func


def radius_outlier(min_neighbors: int = 20, radius: float = 0.3) -> PointCloudFilter:
    """Keep points with at least ``min_neighbors`` within ``radius`` meters."""

    def filter_func(
        det: Detection2DBBox, pc: PointCloud2, ci: CameraInfo, tf: TransformStamped
    ) -> PointCloud2 | None:
        _, indices = _open3d_cloud(pc).remove_radius_outlier(nb_points=min_neighbors, radius=radius)
        return _selected(pc, indices)

    return filter_func
