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

"""Occupancy costmap from a world-frame map cloud — a stateless pure module.

Pure port of :class:`dimos.mapping.costmapper.CostMapper`. Each map cloud is
turned into an :class:`OccupancyGrid` by the configured occupancy algorithm
(``dimos.mapping.pointclouds.occupancy``), with an optional safe disc cleared
around the origin for robots that cannot see directly beneath themselves. No
accumulation: the costmap is a pure function of one cloud, so the shape is a
stateless ``step``, not a fold or Mealy.

Two inputs mirror the legacy module's ``combine_latest``: ``global_map`` is the
tick (the reliable high-rate driver — legacy ``combine_latest`` cannot emit
until it has fired at least once), and ``merged_map`` is an optional
``latest()`` preferred when present (the legacy ``start_with(None)`` fallback).
The one behavioral difference from ``combine_latest``: a ``merged_map`` update
between two ``global_map`` ticks does not force an extra immediate recompute —
it is picked up on the next ``global_map`` tick. At the global-map rate
(~7.6 Hz on the go2 stack) that window is sub-frame.

The grid's own ``ts`` is the source cloud's ``ts`` (set by the occupancy
algorithm), so egress is replay-deterministic — no wall clock in the data path.
"""

from __future__ import annotations

from dataclasses import asdict

import numpy as np

from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure import pm


class PureCostMapper(pm.PureModule):
    """Turn the current best world-frame map cloud into an occupancy costmap."""

    algo: str = "height_cost"  # key into OCCUPANCY_ALGOS
    occupancy: OccupancyConfig = HeightCostConfig()  # algo-matched config (kwargs)
    initial_safe_radius_meters: float = 0.0  # clear a disc at the origin; 0 disables

    class In(pm.In):
        global_map: PointCloud2 = pm.tick(expect_hz=8)  # reliable driver
        merged_map: PointCloud2 | None = pm.latest(default=None)  # preferred when present

    class Out(pm.Out):
        global_costmap: OccupancyGrid = pm.contract(min_hz=0.25)

    def step(self, i: In) -> Out:
        """Costmap the merged cloud if present, else the global cloud."""
        cloud = i.merged_map if i.merged_map is not None else i.global_map
        return PureCostMapper.Out(global_costmap=self._calculate_costmap(cloud))

    def _calculate_costmap(self, cloud: PointCloud2) -> OccupancyGrid:
        """Run the configured occupancy algorithm, then apply the safe radius."""
        occupancy_function = OCCUPANCY_ALGOS[self.algo]
        grid = occupancy_function(cloud, **asdict(self.occupancy))
        self._apply_initial_safe_radius(grid)
        return grid

    def _apply_initial_safe_radius(self, grid: OccupancyGrid) -> None:
        """Zero every cell whose disc overlaps the origin-centred safe radius."""
        radius_meters = self.initial_safe_radius_meters
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
