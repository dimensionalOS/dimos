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

from dataclasses import dataclass
from typing import Optional, Tuple
import time
import numpy as np
import open3d as o3d
import reactivex.operators as ops
from reactivex import interval
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.types.costmap import Costmap, pointcloud_to_costmap


class Map(Module):
    lidar: In[LidarMessage] = None
    global_map: Out[LidarMessage] = None
    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()

    def __init__(
        self,
        voxel_size: float = 0.05,
        cost_resolution: float = 0.05,
        global_publish_interval: Optional[float] = None,
        **kwargs,
    ):
        self.voxel_size = voxel_size
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        super().__init__(**kwargs)

    @rpc
    def start(self):
        self.lidar.subscribe(self.add_frame)

        if self.global_publish_interval is not None:
            interval(self.global_publish_interval).subscribe(
                lambda _: self.global_map.publish(self.to_lidar_message())
            )

    def to_lidar_message(self) -> LidarMessage:
        return LidarMessage(
            pointcloud=self.pointcloud,
            origin=[0.0, 0.0, 0.0],
            resolution=self.voxel_size,
            ts=time.time(),
        )

    @rpc
    def add_frame(self, frame: LidarMessage) -> "Map":
        """Voxelise *frame* and splice it into the running map."""
        new_pct = frame.pointcloud.voxel_down_sample(voxel_size=self.voxel_size)
        self.pointcloud = splice_cylinder(self.pointcloud, new_pct, shrink=0.5)

    def consume(self, observable: Observable[LidarMessage]) -> Observable["Map"]:
        """Reactive operator that folds a stream of `LidarMessage` into the map."""
        return observable.pipe(ops.map(self.add_frame))

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self.pointcloud

    @rpc
    def costmap(self) -> Costmap:
        """Return a fully inflated cost-map in a `Costmap` wrapper."""
        inflate_radius_m = 0.5 * self.voxel_size if self.voxel_size > self.cost_resolution else 0.0
        grid, origin_xy = pointcloud_to_costmap(
            self.pointcloud,
            resolution=self.cost_resolution,
            inflate_radius_m=inflate_radius_m,
        )

        return Costmap(grid=grid, origin=[*origin_xy, 0.0], resolution=self.cost_resolution)


def splice_sphere(
    map_pcd: o3d.geometry.PointCloud,
    patch_pcd: o3d.geometry.PointCloud,
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
    center = patch_pcd.get_center()
    radius = np.linalg.norm(np.asarray(patch_pcd.points) - center, axis=1).max() * shrink
    dists = np.linalg.norm(np.asarray(map_pcd.points) - center, axis=1)
    victims = np.nonzero(dists < radius)[0]
    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd


def splice_cylinder(
    map_pcd: o3d.geometry.PointCloud,
    patch_pcd: o3d.geometry.PointCloud,
    axis: int = 2,
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
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


def _inflate_lethal(costmap: np.ndarray, radius: int, lethal_val: int = 100) -> np.ndarray:
    """Return *costmap* with lethal cells dilated by *radius* grid steps (circular)."""
    if radius <= 0 or not np.any(costmap == lethal_val):
        return costmap

    mask = costmap == lethal_val
    dilated = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius or (dx == 0 and dy == 0):
                continue
            dilated |= np.roll(mask, shift=(dy, dx), axis=(0, 1))

    out = costmap.copy()
    out[dilated] = lethal_val
    return out
