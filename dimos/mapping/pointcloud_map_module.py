# Copyright 2026 Dimensional Inc.
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

"""Generic pointcloud map module backed by DimOS's existing accumulator."""

from __future__ import annotations

import time
from typing import Any

from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.accumulators.general import GeneralPointCloudAccumulator
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PointCloudMapConfig(ModuleConfig):
    voxel_size: float = 0.05
    publish_interval: float = 0.5


class PointCloudMapModule(Module):
    """Accumulate lidar pointclouds and publish a global map."""

    config: PointCloudMapConfig

    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._accumulator = GeneralPointCloudAccumulator(
            voxel_size=self.config.voxel_size,
            global_config=self.config.g,
        )

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.lidar.subscribe(self._add_frame)))
        self.register_disposable(
            interval(self.config.publish_interval).subscribe(lambda _: self._publish())
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _add_frame(self, msg: PointCloud2) -> None:
        self._accumulator.add(msg.pointcloud)

    def _publish(self) -> None:
        self.global_map.publish(
            PointCloud2(
                pointcloud=self._accumulator.get_point_cloud(),
                ts=time.time(),
                frame_id="world",
            )
        )


__all__ = ["PointCloudMapConfig", "PointCloudMapModule"]
