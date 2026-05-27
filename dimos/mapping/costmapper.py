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
import time

from pydantic import Field
from reactivex import combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
    height_cost_and_terrain_class_occupancy,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)


class CostMapper(Module):
    config: Config
    global_map: In[PointCloud2]
    merged_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]
    terrain_classmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()

        def _select_map(
            pair: tuple[PointCloud2, PointCloud2 | None],
        ) -> PointCloud2:
            gmap, merged = pair
            return merged if merged is not None else gmap

        def _publish_costmap(
            maps: tuple[OccupancyGrid, OccupancyGrid | None],
            calc_time_ms: float,
            rx_monotonic: float,
        ) -> None:
            costmap, terrain_classmap = maps
            self.global_costmap.publish(costmap)
            if terrain_classmap is not None:
                self.terrain_classmap.publish(terrain_classmap)

        def _calculate_and_time(
            msg: PointCloud2,
        ) -> tuple[tuple[OccupancyGrid, OccupancyGrid | None], float, float]:
            rx_monotonic = time.monotonic()  # Capture receipt time
            start = time.perf_counter()
            maps = self._calculate_maps(msg)
            elapsed_ms = (time.perf_counter() - start) * 1000
            return maps, elapsed_ms, rx_monotonic

        self.register_disposable(
            combine_latest(
                self.global_map.observable(),  # type: ignore[no-untyped-call]
                self.merged_map.observable().pipe(ops.start_with(None)),  # type: ignore[no-untyped-call,arg-type]
            )
            .pipe(ops.map(_select_map))
            .pipe(ops.map(_calculate_and_time))
            .subscribe(lambda result: _publish_costmap(result[0], result[1], result[2]))
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        return self._calculate_maps(msg)[0]

    def _calculate_maps(self, msg: PointCloud2) -> tuple[OccupancyGrid, OccupancyGrid | None]:
        if self.config.algo == "height_cost":
            return height_cost_and_terrain_class_occupancy(
                msg, **asdict(self.config.config)
            )

        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config)), None
