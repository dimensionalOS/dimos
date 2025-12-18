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

from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


class NavigationMap:
    _global_config: GlobalConfig
    _binary: OccupancyGrid | None = None
    _lock: RLock

    def __init__(self, global_config: GlobalConfig) -> None:
        self._global_config = global_config
        self._lock = RLock()

    def update(self, occupancy_grid: OccupancyGrid) -> None:
        with self._lock:
            self._binary = occupancy_grid

    @property
    def binary_costmap(self) -> OccupancyGrid:
        """
        Get the latest binary costmap received from the global costmap source.
        """

        with self._lock:
            if self._binary is None:
                raise ValueError("No current global costmap available")

            return self._binary

    @property
    def gradient_costmap(self) -> OccupancyGrid:
        return self.make_gradient_costmap()

    def make_gradient_costmap(self, robot_increase: float = 1.0) -> OccupancyGrid:
        """
        Get the latest navigation map created from inflating and applying a
        gradient to the binary costmap.
        """

        with self._lock:
            binary = self._binary
            if binary is None:
                raise ValueError("No current global costmap available")

        return make_navigation_map(
            binary,
            self._global_config.robot_width * robot_increase,
            strategy=self._global_config.planner_strategy,
        )
