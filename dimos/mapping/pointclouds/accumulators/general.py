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

import numpy as np
from open3d.geometry import PointCloud  # type: ignore[import-untyped]
from open3d.io import read_point_cloud  # type: ignore[import-untyped]

from dimos.core.global_config import GlobalConfig  # type: ignore[import-untyped]


class GeneralPointCloudAccumulator:
    _point_cloud: PointCloud
    _voxel_size: float

    def __init__(self, voxel_size: float, global_config: GlobalConfig) -> None:
        self._point_cloud = PointCloud()
        self._voxel_size = voxel_size

        if global_config.mujoco_global_map_from_pointcloud:
            path = global_config.mujoco_global_map_from_pointcloud
            self._point_cloud = read_point_cloud(path)

    def get_point_cloud(self) -> PointCloud:
        return self._point_cloud

    def add(self, point_cloud: PointCloud) -> None:
        """Voxelise *frame* and splice it into the running map."""
        new_pct = point_cloud.voxel_down_sample(voxel_size=self._voxel_size)

        # Skip for empty pointclouds.
        if len(new_pct.points) == 0:
            return

        self._point_cloud = _splice_cylinder(self._point_cloud, new_pct, shrink=0.5)


def _splice_cylinder(
    map_pcd: PointCloud,
    patch_pcd: PointCloud,
    axis: int = 2,
    shrink: float = 0.95,
) -> PointCloud:
    center = patch_pcd.get_center()
    patch_pts = np.asarray(patch_pcd.points)

    # Axes perpendicular to cylinder
    axes = [0, 1, 2]
    axes.remove(axis)

    planar_dists = np.linalg.norm(patch_pts[:, axes] - center[axes], axis=1)
    radius = planar_dists.max() * shrink

    axis_min = (patch_pts[:, axis].min() - center[axis]) * shrink + center[axis]
    axis_max = (patch_pts[:, axis].max() - center[axis]) * shrink + center[axis]

    map_pts = np.asarray(map_pcd.points)
    planar_dists_map = np.linalg.norm(map_pts[:, axes] - center[axes], axis=1)

    victims = np.nonzero(
        (planar_dists_map < radius)
        & (map_pts[:, axis] >= axis_min)
        & (map_pts[:, axis] <= axis_max)
    )[0]

    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd
