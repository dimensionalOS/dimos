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

from dataclasses import asdict, dataclass, field
import time

from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import ModuleConfig
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = field(default_factory=HeightCostConfig)


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    def __init__(self, cfg: GlobalConfig = global_config, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._global_config = cfg

    @rpc
    def start(self) -> None:
        super().start()

        def _publish_costmap(grid: OccupancyGrid, calc_time_ms: float, rx_monotonic: float) -> None:
            self.global_costmap.publish(grid)

        def _calculate_and_time(
            msg: PointCloud2,
        ) -> tuple[OccupancyGrid, float, float]:
            rx_monotonic = time.monotonic()  # Capture receipt time
            start = time.perf_counter()
            grid = self._calculate_costmap(msg)
            elapsed_ms = (time.perf_counter() - start) * 1000
            return grid, elapsed_ms, rx_monotonic

        self._disposables.add(
            self.global_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.map(_calculate_and_time))
            .subscribe(lambda result: _publish_costmap(result[0], result[1], result[2]))
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config))


cost_mapper = CostMapper.blueprint
