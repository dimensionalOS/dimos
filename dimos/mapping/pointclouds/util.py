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

from collections.abc import Iterable
import colorsys
from pathlib import Path

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from open3d.geometry import PointCloud  # type: ignore[import-untyped]


def read_pointcloud(path: Path) -> PointCloud:
    return o3d.io.read_point_cloud(path)


def sum_pointclouds(pointclouds: Iterable[PointCloud]) -> PointCloud:
    it = iter(pointclouds)
    ret = next(it)
    for x in it:
        ret += x
    return ret.remove_duplicated_points()


def height_colorize(pointcloud: PointCloud) -> None:
    points = np.asarray(pointcloud.points)
    z_values = points[:, 2]
    z_min = z_values.min()
    z_max = z_values.max()

    z_normalized = (z_values - z_min) / (z_max - z_min)

    # Create rainbow color map.
    colors = np.array([colorsys.hsv_to_rgb(0.7 * (1 - h), 1.0, 1.0) for h in z_normalized])

    pointcloud.colors = o3d.utility.Vector3dVector(colors)


def visualize(pointcloud: PointCloud) -> None:
    voxel_size = 0.05  # 0.05m voxels
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pointcloud, voxel_size=voxel_size)
    o3d.visualization.draw_geometries(
        [voxel_grid],
        window_name="Combined Point Clouds (Voxelized)",
        width=1024,
        height=768,
    )
