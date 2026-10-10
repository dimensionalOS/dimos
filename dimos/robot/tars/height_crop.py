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

"""Crops the global map to the height band that matters for 2D navigation."""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class HeightCropConfig(ModuleConfig):
    # Points above this (world z) are dropped. A tilted lidar sees the ceiling where it
    # never saw the floor; height_cost_occupancy reads such a cell as a wall as tall as the
    # room, so keep only what the robot could hit.
    max_z: float = 1.0


class HeightCrop(Module):
    """global_map -> nav_map: the map below `max_z`, for the costmap."""

    config: HeightCropConfig
    global_map: In[PointCloud2]
    nav_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.global_map.subscribe(self._on_map)))

    def _on_map(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        self.nav_map.publish(
            PointCloud2.from_numpy(
                points[points[:, 2] < self.config.max_z], frame_id=msg.frame_id, timestamp=msg.ts
            )
        )
