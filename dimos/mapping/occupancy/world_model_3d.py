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

import math
import time

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.operations_3d import (
    apply_time_decay,
    compute_persistence,
    integrate_hit_miss,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


class WorldModel3D:
    _global_config: GlobalConfig
    _log_odds: np.ndarray | None
    _last_seen_ts: np.ndarray | None
    _persistence_score: np.ndarray | None
    _resolution: float
    _origin: Pose
    _frame_id: str
    _last_update_ts: float

    def __init__(self, global_config: GlobalConfig) -> None:
        self._global_config = global_config
        self._log_odds = None
        self._last_seen_ts = None
        self._persistence_score = None
        self._resolution = 0.05
        self._origin = Pose()
        self._frame_id = "world"
        self._last_update_ts = 0.0

    def update_from_observation(self, occupancy_grid: OccupancyGrid) -> None:
        now = occupancy_grid.ts if occupancy_grid.ts > 0 else time.time()
        self._ensure_initialized(occupancy_grid)
        self.decay(now)
        self.raycast_update(occupancy_grid, now)
        self._last_update_ts = now

    def raycast_update(self, occupancy_grid: OccupancyGrid, now: float) -> None:
        if self._log_odds is None or self._last_seen_ts is None or self._persistence_score is None:
            raise ValueError("WorldModel3D not initialized")

        if occupancy_grid.grid.shape != self._log_odds.shape[1:]:
            raise ValueError(
                f"Observation shape mismatch: {occupancy_grid.grid.shape} vs {self._log_odds.shape[1:]}"
            )

        occupied_mask_2d = occupancy_grid.grid >= CostValues.OCCUPIED
        free_mask_2d = occupancy_grid.grid == CostValues.FREE

        occupied_mask = np.broadcast_to(occupied_mask_2d, self._log_odds.shape)
        free_mask = np.broadcast_to(free_mask_2d, self._log_odds.shape)

        self._log_odds = integrate_hit_miss(
            self._log_odds,
            occupied_mask,
            free_mask,
            self._global_config.world3d_hit_inc,
            self._global_config.world3d_miss_dec,
            self._global_config.world3d_max_abs_log_odds,
        )
        self._persistence_score = compute_persistence(
            self._persistence_score,
            occupied_mask,
            free_mask,
            max_persistence=self._global_config.world3d_max_persistence,
        )

        observed_mask = occupied_mask | free_mask
        self._last_seen_ts[observed_mask] = now

    def decay(self, now: float) -> None:
        if self._log_odds is None or self._last_seen_ts is None:
            return
        self._log_odds = apply_time_decay(
            self._log_odds,
            self._last_seen_ts,
            now,
            self._global_config.world3d_decay_sec,
        )

    def project_2d(self, z_min: float | None = None, z_max: float | None = None) -> OccupancyGrid:
        if self._log_odds is None or self._last_seen_ts is None:
            raise ValueError("No world model observation available")

        z_start, z_end = self._z_index_range(z_min, z_max)
        log_view = self._log_odds[z_start:z_end]
        seen_view = self._last_seen_ts[z_start:z_end] > 0

        occupied = np.any(log_view >= self._global_config.world3d_occupied_log_odds, axis=0)
        known = np.any(seen_view, axis=0)
        free = np.any(log_view <= self._global_config.world3d_free_log_odds, axis=0)

        grid = np.full(log_view.shape[1:], CostValues.UNKNOWN, dtype=np.int8)
        grid[known | free] = CostValues.FREE
        grid[occupied] = CostValues.OCCUPIED

        ts = self._last_update_ts if self._last_update_ts > 0 else time.time()
        return OccupancyGrid(
            grid=grid,
            resolution=self._resolution,
            origin=self._origin,
            frame_id=self._frame_id,
            ts=ts,
        )

    def query_local_obstacles(
        self, mask: np.ndarray, z_min: float | None = None, z_max: float | None = None
    ) -> bool:
        projected = self.project_2d(z_min=z_min, z_max=z_max)
        if projected.grid.shape != mask.shape:
            raise ValueError(f"Mask shape mismatch: {mask.shape} vs {projected.grid.shape}")
        return bool(np.any(projected.grid[mask] >= CostValues.OCCUPIED))

    @property
    def persistence_score(self) -> np.ndarray:
        if self._persistence_score is None:
            raise ValueError("No world model observation available")
        return self._persistence_score.copy()

    def _ensure_initialized(self, occupancy_grid: OccupancyGrid) -> None:
        if self._log_odds is not None:
            return

        n_layers = max(
            1,
            int(
                math.ceil(
                    (self._global_config.world3d_project_z_max - self._global_config.world3d_project_z_min)
                    / self._global_config.world3d_voxel_size
                )
            ),
        )
        height, width = occupancy_grid.grid.shape
        shape = (n_layers, height, width)

        self._log_odds = np.zeros(shape, dtype=np.float32)
        self._last_seen_ts = np.zeros(shape, dtype=np.float64)
        self._persistence_score = np.zeros(shape, dtype=np.int16)
        self._resolution = occupancy_grid.resolution
        self._origin = occupancy_grid.origin
        self._frame_id = occupancy_grid.frame_id

    def _z_index_range(self, z_min: float | None, z_max: float | None) -> tuple[int, int]:
        if self._log_odds is None:
            raise ValueError("No world model observation available")

        n_layers = self._log_odds.shape[0]
        model_min = self._global_config.world3d_project_z_min
        voxel = self._global_config.world3d_voxel_size

        effective_min = model_min if z_min is None else z_min
        effective_max = self._global_config.world3d_project_z_max if z_max is None else z_max
        if effective_max <= effective_min:
            raise ValueError(f"Invalid z range: z_min={effective_min}, z_max={effective_max}")

        start = int(math.floor((effective_min - model_min) / voxel))
        end = int(math.ceil((effective_max - model_min) / voxel))
        start = max(0, min(start, n_layers - 1))
        end = max(start + 1, min(end, n_layers))
        return start, end
