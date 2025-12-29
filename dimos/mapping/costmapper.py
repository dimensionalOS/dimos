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

from dataclasses import asdict, dataclass, field

from reactivex import operators as ops
import rerun as rr

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.dashboard import rerun_init
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.metrics import timed


@dataclass
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = field(default_factory=HeightCostConfig)


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()

        def _publish_costmap(grid: OccupancyGrid) -> None:
            self.global_costmap.publish(grid)
            rr.log("world/nav/costmap", grid.to_rerun())

        self._disposables.add(
            self.global_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.map(self._calculate_costmap))
            .subscribe(_publish_costmap)
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config))


cost_mapper = CostMapper.blueprint
