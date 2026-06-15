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

from dataclasses import asdict

from pydantic import Field

from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.memory2.puremodule import PureModule, PureModuleConfig, latest, tick
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Config(PureModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)


class CostMapper(PureModule):
    config: Config
    global_map: In[PointCloud2] = tick()
    relocalized_map: In[PointCloud2] = latest()
    global_costmap: Out[OccupancyGrid]

    def step(self, global_map: PointCloud2, relocalized_map: PointCloud2 | None) -> OccupancyGrid:
        msg = relocalized_map if relocalized_map is not None else global_map
        return self._calculate_costmap(msg)

    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        occupancy_function = OCCUPANCY_ALGOS[self.config.algo]
        return occupancy_function(msg, **asdict(self.config.config))
