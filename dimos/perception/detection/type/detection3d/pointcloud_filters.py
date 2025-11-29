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

from collections.abc import Callable

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.perception.detection.type.detection2d import Detection2DBBox

# Filters take Detection2DBBox, PointCloud2, CameraInfo, Transform and return filtered PointCloud2 or None
PointCloudFilter = Callable[
    [Detection2DBBox, PointCloud2, CameraInfo, Transform], PointCloud2 | None
]


def height_filter(height: float = 0.1) -> PointCloudFilter:
    return lambda det, pc, ci, tf: pc.filter_by_height(height)


def statistical(nb_neighbors: int = 40, std_ratio: float = 0.5) -> PointCloudFilter:
    def filter_func(
        det: Detection2DBBox, pc: PointCloud2, ci: CameraInfo, tf: Transform
    ) -> PointCloud2 | None:
        try:
            statistical, _removed = pc.pointcloud.remove_statistical_outlier(
                nb_neighbors=nb_neighbors, std_ratio=std_ratio
            )
            return PointCloud2(statistical, pc.frame_id, pc.ts)
        except Exception:
            # print("statistical filter failed:", e)
            return None

    return filter_func


def raycast() -> PointCloudFilter:
    def filter_func(
        det: Detection2DBBox, pc: PointCloud2, ci: CameraInfo, tf: Transform
    ) -> PointCloud2 | None:
        try:
            camera_pos = tf.inverse().translation
            camera_pos_np = camera_pos.to_numpy()
            _, visible_indices = pc.pointcloud.hidden_point_removal(camera_pos_np, radius=100.0)
            visible_pcd = pc.pointcloud.select_by_index(visible_indices)
            return PointCloud2(visible_pcd, pc.frame_id, pc.ts)
        except Exception:
            # print("raycast filter failed:", e)
            return None

    return filter_func


def radius_outlier(min_neighbors: int = 20, radius: float = 0.3) -> PointCloudFilter:
    """
    Remove isolated points: keep only points that have at least `min_neighbors`
    neighbors within `radius` meters (same units as your point cloud).
    """

    def filter_func(
        det: Detection2DBBox, pc: PointCloud2, ci: CameraInfo, tf: Transform
    ) -> PointCloud2 | None:
        filtered_pcd, _removed = pc.pointcloud.remove_radius_outlier(
            nb_points=min_neighbors, radius=radius
        )
        return PointCloud2(filtered_pcd, pc.frame_id, pc.ts)

    return filter_func
