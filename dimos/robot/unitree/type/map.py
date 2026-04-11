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

from pathlib import Path
import time
from typing import Any

import open3d as o3d  # type: ignore[import-untyped]
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.mapping.pointclouds.accumulators.general import GeneralPointCloudAccumulator
from dimos.mapping.pointclouds.accumulators.protocol import PointCloudAccumulator
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.connection import Go2ConnectionProtocol


class MapConfig(ModuleConfig):
    voxel_size: float = 0.05
    cost_resolution: float = 0.05
    global_publish_interval: float | None = None
    min_height: float = 0.10
    max_height: float = 0.5


class Map(Module):
    config: MapConfig
    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    _point_cloud_accumulator: PointCloudAccumulator
    _preloaded_occupancy: OccupancyGrid | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.voxel_size = self.config.voxel_size
        self.cost_resolution = self.config.cost_resolution
        self.global_publish_interval = self.config.global_publish_interval
        self.min_height = self.config.min_height
        self.max_height = self.config.max_height
        self._point_cloud_accumulator = GeneralPointCloudAccumulator(self.voxel_size, self.config.g)

        if self.config.g.simulation:
            self.min_height = 0.3

    @rpc
    def start(self) -> None:
        super().start()

        self.register_disposable(Disposable(self.lidar.subscribe(self.add_frame)))

        if self.global_publish_interval is not None:
            unsub = interval(self.global_publish_interval).subscribe(self._publish)
            self.register_disposable(unsub)

    @rpc
    def stop(self) -> None:
        super().stop()

    def to_PointCloud2(self) -> PointCloud2:
        return PointCloud2(
            pointcloud=self._point_cloud_accumulator.get_point_cloud(),
            ts=time.time(),
        )

    # TODO: Why is this RPC?
    @rpc
    def add_frame(self, frame: PointCloud2) -> None:
        self._point_cloud_accumulator.add(frame.pointcloud)

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self._point_cloud_accumulator.get_point_cloud()

    def _publish(self, _: Any) -> None:
        self.global_map.publish(self.to_PointCloud2())

        occupancygrid = general_occupancy(
            self.to_PointCloud2(),
            resolution=self.cost_resolution,
            min_height=self.min_height,
            max_height=self.max_height,
        )

        # When debugging occupancy navigation, load a predefined occupancy grid.
        if self.config.g.mujoco_global_costmap_from_occupancy:
            if self._preloaded_occupancy is None:
                path = Path(self.config.g.mujoco_global_costmap_from_occupancy)
                self._preloaded_occupancy = OccupancyGrid.from_path(path)
            occupancygrid = self._preloaded_occupancy

        self.global_costmap.publish(occupancygrid)


def deploy(dimos: ModuleCoordinator, connection: Go2ConnectionProtocol):  # type: ignore[no-untyped-def]
    mapper = dimos.deploy(Map, global_publish_interval=1.0)
    mapper.global_map.transport = LCMTransport("/global_map", PointCloud2)
    mapper.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    mapper.lidar.connect(connection.pointcloud)  # type: ignore[attr-defined]
    mapper.start()
    return mapper
