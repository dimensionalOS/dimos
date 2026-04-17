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

import time
from threading import RLock

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.operations import (
    fuse_planning_map,
    update_confirmation_counts,
    update_structural_map,
)
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


class NavigationMap:
    _global_config: GlobalConfig
    _structural_map: OccupancyGrid | None = None
    _live_map: OccupancyGrid | None = None
    _planning_map: OccupancyGrid | None = None
    _confirmation_counts: np.ndarray | None = None
    _live_map_received_at: float = 0.0
    _lock: RLock

    def __init__(self, global_config: GlobalConfig) -> None:
        self._global_config = global_config
        self._lock = RLock()

    def update(self, occupancy_grid: OccupancyGrid) -> None:
        received_at = time.time()
        with self._lock:
            if self._structural_map is None:
                self._initialize_maps(occupancy_grid)
            else:
                self._update_structural_map(occupancy_grid)

            self._live_map = occupancy_grid.copy()
            self._live_map_received_at = received_at
            self._refresh_planning_map(received_at, occupancy_grid.ts)

    @property
    def structural_map(self) -> OccupancyGrid:
        with self._lock:
            if self._structural_map is None:
                raise ValueError("No structural map available")
            return self._structural_map

    @property
    def live_map(self) -> OccupancyGrid:
        with self._lock:
            live_map = self._get_live_map(time.time())
            if live_map is None:
                raise ValueError("No current live map available")
            return live_map

    @property
    def planning_map(self) -> OccupancyGrid:
        with self._lock:
            now = time.time()
            self._refresh_planning_map(now, now)
            if self._planning_map is None:
                raise ValueError("No current planning map available")
            return self._planning_map

    @property
    def confirmation_counts(self) -> np.ndarray:
        with self._lock:
            if self._confirmation_counts is None:
                raise ValueError("No confirmation counts available")
            return self._confirmation_counts.copy()

    @property
    def binary_costmap(self) -> OccupancyGrid:
        """
        Compatibility alias for planning map.
        """
        return self.planning_map

    @property
    def gradient_costmap(self) -> OccupancyGrid:
        return self.make_gradient_costmap()

    def make_gradient_costmap(self, robot_increase: float = 1.0) -> OccupancyGrid:
        """
        Get the latest navigation map created from inflating and applying a
        gradient to the binary costmap.
        """

        planning_map = self.planning_map

        return make_navigation_map(
            planning_map,
            self._global_config.robot_width * robot_increase,
            strategy=self._global_config.planner_strategy,
        )

    def _initialize_maps(self, occupancy_grid: OccupancyGrid) -> None:
        structural_grid = occupancy_grid.copy()
        counts = np.zeros_like(occupancy_grid.grid, dtype=np.int16)
        counts[occupancy_grid.grid >= CostValues.OCCUPIED] = (
            self._global_config.structural_write_threshold
        )
        counts[occupancy_grid.grid == CostValues.FREE] = (
            self._global_config.structural_clear_threshold
        )

        self._structural_map = structural_grid
        self._confirmation_counts = counts

    def _update_structural_map(self, occupancy_grid: OccupancyGrid) -> None:
        if self._structural_map is None or self._confirmation_counts is None:
            raise ValueError("Structural map is not initialized")

        if not self._global_config.enable_structural_update:
            return

        self._confirmation_counts = update_confirmation_counts(
            self._confirmation_counts, occupancy_grid
        )
        self._structural_map = update_structural_map(
            self._structural_map,
            self._confirmation_counts,
            self._global_config.structural_write_threshold,
            self._global_config.structural_clear_threshold,
            occupancy_grid.ts,
        )

    def _get_live_map(self, now: float) -> OccupancyGrid | None:
        if self._live_map is None:
            return None

        age = now - self._live_map_received_at
        if age > self._global_config.live_map_retention_sec:
            return None

        return self._live_map

    def _refresh_planning_map(self, now: float, output_ts: float) -> None:
        if self._structural_map is None:
            self._planning_map = None
            return

        live_map = self._get_live_map(now)
        self._planning_map = fuse_planning_map(self._structural_map, live_map, output_ts)
