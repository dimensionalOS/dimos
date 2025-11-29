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

from dimos.msgs.geometry_msgs import VectorLike
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.global_planner.general_astar import general_astar
from dimos.navigation.global_planner.min_cost_astar import min_cost_astar
from dimos.utils.logging_config import setup_logger

Algorithm: TypeAlias = Literal["general", "min_cost"]

logger = setup_logger(__file__)


def astar(
    algorithm: Algorithm,
    costmap: OccupancyGrid,
    goal: VectorLike,
    start: VectorLike,
) -> Path | None:
    """
    A* path planning algorithm from start to goal position.

    Args:
        algorithm: The A* algorithm variant to use ("general" or "min_cost")
        costmap: Costmap object containing the environment
        goal: Goal position as any vector-like object
        start: Start position as any vector-like object (default: origin [0,0])

    Returns:
        Path object containing waypoints, or None if no path found
    """

    match algorithm:
        case "general":
            return general_astar(costmap, goal, start)
        case "min_cost":
            return min_cost_astar(costmap, goal, start)
        case _:
            raise NotImplementedError()
