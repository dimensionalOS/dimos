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

from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.accumulators.general import GeneralPointCloudAccumulator
from dimos.mapping.pointclouds.accumulators.protocol import PointCloudAccumulator
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.message_utils import occupancy, pointcloud, time as message_time
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]


class MapConfig(ModuleConfig):
    global_costmap_from_occupancy: Path | None = None
    global_map_from_pointcloud: Path | None = None
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
        self._point_cloud_accumulator = GeneralPointCloudAccumulator(
            self.voxel_size, self.config.global_map_from_pointcloud
        )

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
        cloud = self._point_cloud_accumulator.get_point_cloud()
        header = Header(stamp=message_time.time_from_seconds(time.time()), frame_id="")
        points = np.asarray(cloud.points)
        if cloud.has_colors():
            colors = np.rint(np.asarray(cloud.colors) * 255).astype(np.uint8)
            return pointcloud.from_xyz_rgb(points, colors, header=header)
        return pointcloud.from_xyz(points, header=header)

    # TODO: Why is this RPC?
    @rpc
    def add_frame(self, frame: PointCloud2) -> None:
        self._point_cloud_accumulator.add(pointcloud.to_open3d(frame))

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
        if self.config.global_costmap_from_occupancy:
            if self._preloaded_occupancy is None:
                path = Path(self.config.global_costmap_from_occupancy)
                self._preloaded_occupancy = occupancy.from_file(
                    path,
                    header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
                )
            occupancygrid = self._preloaded_occupancy

        self.global_costmap.publish(occupancygrid)
