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

from typing import Literal, TypeAlias

from dimos.mapping.occupancy.gradient import voronoi_gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.occupancy.operations import overlay_occupied, smooth_occupied
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

NavigationStrategy: TypeAlias = Literal["simple", "mixed"]


def make_navigation_map(
    occupancy_grid: OccupancyGrid, robot_width: float, strategy: NavigationStrategy
) -> OccupancyGrid:
    half_width = robot_width / 2
    gradient_distance = 1.5

    if strategy == "simple":
        costmap = simple_inflate(occupancy_grid, half_width)
    elif strategy == "mixed":
        costmap = smooth_occupied(occupancy_grid)
        costmap = simple_inflate(costmap, half_width)
        costmap = overlay_occupied(costmap, occupancy_grid)
    else:
        raise ValueError(f"Unknown strategy: {strategy}")

    return voronoi_gradient(costmap, max_distance=gradient_distance)
