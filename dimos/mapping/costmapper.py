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
from typing import Any

import numpy as np
from pydantic import Field
from reactivex import combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_COLOR_UNKNOWN = (0, 0, 0, 0)
_COLOR_FREE = (72, 73, 129, 255)
_COLOR_OCCUPIED = (255, 140, 0, 255)
_COLOR_LETHAL = (220, 30, 30, 255)

# Indexed by grid value + 1: 0 = unknown, 1 = free, 2..101 = cost 1..100.
_COSTMAP_COLOR_LOOKUP_TABLE = np.empty((102, 4), dtype=np.uint8)
_COSTMAP_COLOR_LOOKUP_TABLE[0] = _COLOR_UNKNOWN
_COSTMAP_COLOR_LOOKUP_TABLE[1] = _COLOR_FREE
_COSTMAP_COLOR_LOOKUP_TABLE[2:101] = _COLOR_OCCUPIED
_COSTMAP_COLOR_LOOKUP_TABLE[101] = _COLOR_LETHAL

_COSTMAP_Z_OFFSET = 0.02


def costmap_to_rerun(grid: OccupancyGrid) -> Any:
    return grid.to_rerun(
        color_lookup_table=_COSTMAP_COLOR_LOOKUP_TABLE,
        z_offset=_COSTMAP_Z_OFFSET,
    )


class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)
    # for robots that cant see directly below themself
    initial_safe_radius_meters: float = 0.0


class CostMapper(Module):
    config: Config
    global_map: In[PointCloud2]
    merged_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()

        def _select_map(
            pair: tuple[PointCloud2, PointCloud2 | None],
        ) -> PointCloud2:
            gmap, merged = pair
            return merged if merged is not None else gmap

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
        occupancy_function = OCCUPANCY_ALGOS[self.config.algo]
        grid = occupancy_function(msg, **asdict(self.config.config))
        self._apply_initial_safe_radius(grid)
        return grid

    def _apply_initial_safe_radius(self, grid: OccupancyGrid) -> None:
        radius_meters = self.config.initial_safe_radius_meters
        if radius_meters <= 0 or grid.grid.size == 0:
            return

        resolution = grid.resolution
        origin_x = grid.origin.position.x
        origin_y = grid.origin.position.y

        rows, columns = np.ogrid[: grid.grid.shape[0], : grid.grid.shape[1]]
        cell_world_x = columns * resolution + origin_x
        cell_world_y = rows * resolution + origin_y
        distance_squared_meters = cell_world_x**2 + cell_world_y**2

        # Half-cell tolerance: a cell counts as inside if any part of it overlaps
        # the disc. Avoids floating-point boundary flakiness from radius/resolution.
        effective_radius_meters = radius_meters + resolution * 0.5
        safe_mask = distance_squared_meters <= effective_radius_meters**2
        grid.grid[safe_mask] = 0
