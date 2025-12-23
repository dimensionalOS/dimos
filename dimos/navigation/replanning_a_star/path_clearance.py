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

from threading import RLock

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.path_mask import make_path_mask
from dimos.msgs.nav_msgs import Path
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


class PathClearance:
    _costmap: OccupancyGrid | None = None
    _last_costmap: OccupancyGrid | None = None
    _path_lookup_distance: float = 3.0
    _max_distance_cache: float = 1.0
    _last_used_shape: tuple[int, ...] | None = None
    _last_mask: NDArray[np.bool_] | None = None
    _last_used_pose: int | None = None
    _global_config: GlobalConfig
    _lock: RLock
    _path: Path
    _pose_index: int

    def __init__(self, global_config: GlobalConfig, path: Path) -> None:
        self._global_config = global_config
        self._path = path
        self._pose_index = 0
        self._lock = RLock()

    def update_costmap(self, costmap: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = costmap

    def update_pose_index(self, index: int) -> None:
        with self._lock:
            self._pose_index = index

    @property
    def mask(self) -> NDArray[np.bool_]:
        with self._lock:
            costmap = self._costmap
            pose_index = self._pose_index

        assert costmap is not None

        if (
            self._last_mask is not None
            and self._last_used_pose is not None
            and costmap.grid.shape == self._last_used_shape
            and self._pose_distance(self._last_used_pose, pose_index) < self._max_distance_cache
        ):
            return self._last_mask

        self._last_mask = make_path_mask(
            occupancy_grid=costmap,
            path=self._path,
            robot_width=self._global_config.robot_width,
            pose_index=pose_index,
            max_length=self._path_lookup_distance,
        )

        self._last_used_shape = costmap.grid.shape
        self._last_used_pose = pose_index

        return self._last_mask

    def is_obstacle_ahead(self) -> bool:
        with self._lock:
            costmap = self._costmap

        if costmap is None:
            return True

        return bool(np.any(costmap.grid[self.mask] == CostValues.OCCUPIED))

    def _pose_distance(self, index1: int, index2: int) -> float:
        p1 = self._path.poses[index1].position
        p2 = self._path.poses[index2].position
        return p1.distance(p2)
