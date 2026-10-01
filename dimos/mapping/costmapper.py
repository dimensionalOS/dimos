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

from dimos_generated.nav_msgs.msg import OccupancyGrid
from dimos_generated.sensor_msgs.msg import PointCloud2
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
from dimos.msgs.geometry import pose_matrix
from dimos.msgs.occupancy import occupancy_view
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
        if radius_meters <= 0:
            return
        cells = occupancy_view(grid).copy()
        if cells.size == 0:
            return

        resolution = grid.info.resolution
        matrix = pose_matrix(grid.info.origin)
        rows, columns = np.ogrid[: cells.shape[0], : cells.shape[1]]
        local_x = columns * resolution
        local_y = rows * resolution
        cell_world_x = matrix[0, 0] * local_x + matrix[0, 1] * local_y + matrix[0, 3]
        cell_world_y = matrix[1, 0] * local_x + matrix[1, 1] * local_y + matrix[1, 3]
        distance_squared_meters = cell_world_x**2 + cell_world_y**2

        # Half-cell tolerance: a cell counts as inside if any part of it overlaps
        # the disc. Avoids floating-point boundary flakiness from radius/resolution.
        effective_radius_meters = radius_meters + resolution * 0.5
        safe_mask = distance_squared_meters <= effective_radius_meters**2
        cells[safe_mask] = 0
        grid.data = cells.ravel()
