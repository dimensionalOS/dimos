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

from pathlib import Path
import time
from typing import Any

import open3d as o3d  # type: ignore[import-untyped]
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core import DimosCluster, In, LCMTransport, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.mapping.pointclouds.accumulators.general import GeneralPointCloudAccumulator
from dimos.mapping.pointclouds.accumulators.protocol import PointCloudAccumulator
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree.connection.go2 import Go2ConnectionProtocol
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage


class Map(Module):
    lidar: In[LidarMessage]
    global_map: Out[LidarMessage]
    global_costmap: Out[OccupancyGrid]

    _point_cloud_accumulator: PointCloudAccumulator
    _global_config: GlobalConfig
    _preloaded_occupancy: OccupancyGrid | None = None

    def __init__(  # type: ignore[no-untyped-def]
        self,
        voxel_size: float = 0.05,
        cost_resolution: float = 0.05,
        global_publish_interval: float | None = None,
        min_height: float = 0.10,
        max_height: float = 0.5,
        global_config: GlobalConfig | None = None,
        **kwargs,
    ) -> None:
        self.voxel_size = voxel_size
        self.cost_resolution = cost_resolution
        self.global_publish_interval = global_publish_interval
        self.min_height = min_height
        self.max_height = max_height
        self._global_config = global_config or GlobalConfig()
        self._point_cloud_accumulator = GeneralPointCloudAccumulator(
            self.voxel_size, self._global_config
        )

        if self._global_config.simulation:
            self.min_height = 0.3

        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(Disposable(self.lidar.subscribe(self.add_frame)))

        if self.global_publish_interval is not None:
            unsub = interval(self.global_publish_interval).subscribe(self._publish)
            self._disposables.add(unsub)

    @rpc
    def stop(self) -> None:
        super().stop()

    def to_PointCloud2(self) -> PointCloud2:
        return PointCloud2(
            pointcloud=self._point_cloud_accumulator.get_point_cloud(),
            ts=time.time(),
        )

    def to_lidar_message(self) -> LidarMessage:
        return LidarMessage(
            pointcloud=self._point_cloud_accumulator.get_point_cloud(),
            origin=[0.0, 0.0, 0.0],
            resolution=self.voxel_size,
            ts=time.time(),
        )

    # TODO: Why is this RPC?
    @rpc
    def add_frame(self, frame: LidarMessage) -> None:
        self._point_cloud_accumulator.add(frame.pointcloud)

    @property
    def o3d_geometry(self) -> o3d.geometry.PointCloud:
        return self._point_cloud_accumulator.get_point_cloud()

    def _publish(self, _: Any) -> None:
        self.global_map.publish(self.to_lidar_message())

        occupancygrid = general_occupancy(
            self.to_lidar_message(),
            resolution=self.cost_resolution,
            min_height=self.min_height,
            max_height=self.max_height,
        )

        # When debugging occupancy navigation, load a predefined occupancy grid.
        if self._global_config.mujoco_global_costmap_from_occupancy:
            if self._preloaded_occupancy is None:
                path = Path(self._global_config.mujoco_global_costmap_from_occupancy)
                self._preloaded_occupancy = OccupancyGrid.from_path(path)
            occupancygrid = self._preloaded_occupancy

        self.global_costmap.publish(occupancygrid)


mapper = Map.blueprint


def deploy(dimos: DimosCluster, connection: Go2ConnectionProtocol):  # type: ignore[no-untyped-def]
    mapper = dimos.deploy(Map, global_publish_interval=1.0)  # type: ignore[attr-defined]
    mapper.global_map.transport = LCMTransport("/global_map", LidarMessage)
    mapper.global_costmap.transport = LCMTransport("/global_costmap", OccupancyGrid)
    mapper.lidar.connect(connection.pointcloud)  # type: ignore[attr-defined]
    mapper.start()
    return mapper


__all__ = ["Map", "mapper"]
