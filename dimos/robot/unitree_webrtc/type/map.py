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

import time

import numpy as np
import open3d as o3d
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core import DimosCluster, In, LCMTransport, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree.connection.go2 import Go2ConnectionProtocol
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage


class Map(Module):
    lidar: In[LidarMessage] = None
    global_map: Out[LidarMessage] = None
    global_costmap: Out[OccupancyGrid] = None
    local_costmap: Out[OccupancyGrid] = None

    pointcloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud()

    def __init__(
        self,
        voxel_size: float = 0.05,
        cost_resolution: float = 0.05,
        global_publish_interval: float | None = None,
        min_height: float = 0.15,
        max_height: float = 0.6,
        global_config: GlobalConfig | None = None,
        **kwargs,
    ) -> None:
        self.voxel_size = voxel_size
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        self.min_height = min_height
        self.max_height = max_height

        if global_config:
            if global_config.use_simulation:
                self.min_height = 0.3

        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        unsub = self.lidar.subscribe(self.add_frame)
        self._disposables.add(Disposable(unsub))

        def publish(_) -> None:
            self.global_map.publish(self.to_lidar_message())

            # temporary, not sure if it belogs in mapper
            # used only for visualizations, not for any algo
            occupancygrid = OccupancyGrid.from_pointcloud(
                self.to_lidar_message(),
                resolution=self.cost_resolution,
                min_height=self.min_height,
                max_height=self.max_height,
            )

            self.global_costmap.publish(occupancygrid)

        if self.global_publish_interval is not None:
            unsub = interval(self.global_publish_interval).subscribe(publish)
            self._disposables.add(unsub)

    @rpc
    def stop(self) -> None:
        super().stop()

    def to_PointCloud2(self) -> PointCloud2:
        return PointCloud2(
            pointcloud=self.pointcloud,
            ts=time.time(),
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

        # Skip for empty pointclouds.
        if len(new_pct.points) == 0:
            return self

        self.pointcloud = splice_cylinder(self.pointcloud, new_pct, shrink=0.5)
        local_costmap = OccupancyGrid.from_pointcloud(
            frame,
            resolution=self.cost_resolution,
            min_height=0.15,
            max_height=0.6,
        ).gradient(max_distance=0.25)
        self.local_costmap.publish(local_costmap)

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self.pointcloud


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


mapper = Map.blueprint


def deploy(dimos: DimosCluster, connection: Go2ConnectionProtocol):
    mapper = dimos.deploy(Map, global_publish_interval=1.0)
    mapper.global_map.transport = LCMTransport("/global_map", LidarMessage)
    mapper.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    mapper.local_costmap.transport = LCMTransport("/local_costmap", OccupancyGrid)
    mapper.lidar.connect(connection.pointcloud)
    mapper.start()
    return mapper


__all__ = ["Map", "mapper"]
