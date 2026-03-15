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

from abc import ABC, abstractmethod
from threading import RLock
import time

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.patrolling.routers.visitation_history import VisitationHistory


class BasePatrolRouter(ABC):
    _occupancy_grid_min_update_interval_s = 60.0
    _occupancy_grid: OccupancyGrid | None
    _occupancy_grid_updated_at: float
    _pose: PoseStamped | None
    _lock: RLock
    _clearance_radius_m: float

    def __init__(self, clearance_radius_m: float) -> None:
        self._occupancy_grid = None
        self._occupancy_grid_updated_at = 0.0
        self._visitation = VisitationHistory(clearance_radius_m)
        self._pose = None
        self._lock = RLock()
        self._clearance_radius_m = clearance_radius_m

    @property
    def _visited(self) -> NDArray[np.bool_] | None:
        return self._visitation.visited

    def handle_occupancy_grid(self, msg: OccupancyGrid) -> None:
        with self._lock:
            now = time.monotonic()
            if (
                self._occupancy_grid is not None
                and now - self._occupancy_grid_updated_at
                < self._occupancy_grid_min_update_interval_s
            ):
                return
            self._occupancy_grid = msg
            self._occupancy_grid_updated_at = now
            self._visitation.update_grid(msg)

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._pose = msg
            if self._occupancy_grid is None:
                return
            self._visitation.handle_odom(msg.position.x, msg.position.y)

    def get_saturation(self) -> float:
        with self._lock:
            return self._visitation.get_saturation()

    def reset(self) -> None:
        with self._lock:
            self._occupancy_grid = None
            self._occupancy_grid_updated_at = 0.0
            self._visitation.reset()

    @abstractmethod
    def next_goal(self) -> PoseStamped | None: ...
