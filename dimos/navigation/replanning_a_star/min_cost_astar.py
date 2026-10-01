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

import heapq
import math

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import OccupancyGrid, Path

from dimos.msgs.occupancy import grid_to_world, occupancy_view, world_to_grid
from dimos.utils.logging_config import setup_logger

# Try to import C++ extension for faster pathfinding
try:
    from dimos.navigation.replanning_a_star.min_cost_astar_ext import (
        min_cost_astar_cpp as _astar_cpp,
    )

    _USE_CPP = True
    _CPP_IMPORT_ERROR: ImportError | None = None
except ImportError as e:
    _USE_CPP = False
    _CPP_IMPORT_ERROR = e

logger = setup_logger()

# Define possible movements (8-connected grid with diagonal movements)
_directions = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
]

# Cost for each movement (straight vs diagonal)
_sc = 1.0  # Straight cost
_dc = 1.42  # Diagonal cost (approximately sqrt(2))
_movement_costs = [_sc, _sc, _sc, _sc, _dc, _dc, _dc, _dc]


# Heuristic function (Octile distance for 8-connected grid)
def _heuristic(x1: int, y1: int, x2: int, y2: int) -> float:
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    # Octile distance: optimal for 8-connected grids with diagonal movement
    return (dx + dy) + (_dc - 2 * _sc) * min(dx, dy)


def _reconstruct_path(
    parents: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
    costmap: OccupancyGrid,
) -> Path:
    coordinates = [current]
    while current in parents:
        current = parents[current]
        coordinates.append(current)
    coordinates.reverse()
    return _reconstruct_path_from_coords(coordinates, costmap)


def _reconstruct_path_from_coords(
    path_coords: list[tuple[int, int]],
    costmap: OccupancyGrid,
) -> Path:
    return Path(
        header=costmap.header,
        poses=[
            PoseStamped(
                header=costmap.header,
                pose=Pose(position=grid_to_world(costmap, coordinate), orientation=Quaternion(w=1)),
            )
            for coordinate in path_coords
        ],
    )


def min_cost_astar(
    costmap: OccupancyGrid,
    goal: Point,
    start: Point | None = None,
    cost_threshold: int = 100,
    unknown_penalty: float = 0.8,
    use_cpp: bool = True,
) -> Path | None:
    cells = occupancy_view(costmap)
    start_vector = world_to_grid(costmap, start if start is not None else Point())
    goal_vector = world_to_grid(costmap, goal)
    # Ignore sub-nanocell rotation roundoff before assigning a point to its cell.
    start_tuple = (math.floor(round(start_vector[0], 9)), math.floor(round(start_vector[1], 9)))
    goal_tuple = (math.floor(round(goal_vector[0], 9)), math.floor(round(goal_vector[1], 9)))

    if not (0 <= goal_tuple[0] < costmap.info.width and 0 <= goal_tuple[1] < costmap.info.height):
        return None
    if not (0 <= start_tuple[0] < costmap.info.width and 0 <= start_tuple[1] < costmap.info.height):
        return None

    if use_cpp:
        if _USE_CPP:
            path_coords = _astar_cpp(
                cells,
                start_tuple[0],
                start_tuple[1],
                goal_tuple[0],
                goal_tuple[1],
                cost_threshold,
                unknown_penalty,
            )
            if not path_coords:
                return None
            return _reconstruct_path_from_coords(path_coords, costmap)
        else:
            logger.warning(
                "C++ A* module could not be imported (%s). Using Python.",
                _CPP_IMPORT_ERROR,
            )

    open_set: list[tuple[float, float, tuple[int, int]]] = []  # Priority queue for nodes to explore
    closed_set: set[tuple[int, int]] = set()  # Set of explored nodes

    # Dictionary to store cost and distance from start, and parents for each node
    # Track cumulative cell cost and path length separately
    cost_score: dict[tuple[int, int], float] = {start_tuple: 0.0}  # Cumulative cell cost
    dist_score: dict[tuple[int, int], float] = {start_tuple: 0.0}  # Cumulative path length
    parents: dict[tuple[int, int], tuple[int, int]] = {}

    # Start with the starting node
    # Priority: (total_cost + heuristic_cost, total_distance + heuristic_distance, node)
    h_dist = _heuristic(start_tuple[0], start_tuple[1], goal_tuple[0], goal_tuple[1])
    heapq.heappush(open_set, (0.0, h_dist, start_tuple))

    while open_set:
        _, _, current = heapq.heappop(open_set)
        current_x, current_y = current

        if current in closed_set:
            continue

        if current == goal_tuple:
            return _reconstruct_path(parents, current, costmap)

        closed_set.add(current)

        for i, (dx, dy) in enumerate(_directions):
            neighbor_x, neighbor_y = current_x + dx, current_y + dy
            neighbor = (neighbor_x, neighbor_y)

            if not (0 <= neighbor_x < costmap.info.width and 0 <= neighbor_y < costmap.info.height):
                continue

            if neighbor in closed_set:
                continue

            neighbor_val = cells[neighbor_y, neighbor_x]

            if neighbor_val >= cost_threshold:
                continue

            if neighbor_val == -1:
                cell_cost = cost_threshold * unknown_penalty
                if cell_cost >= cost_threshold:
                    continue
            elif neighbor_val == 0:
                cell_cost = 0.0
            else:
                cell_cost = neighbor_val

            tentative_cost = cost_score[current] + cell_cost
            tentative_dist = dist_score[current] + _movement_costs[i]

            # Get the current scores for the neighbor or set to infinity if not yet explored
            neighbor_cost = cost_score.get(neighbor, float("inf"))
            neighbor_dist = dist_score.get(neighbor, float("inf"))

            # If this path to the neighbor is better (prioritize cost, then distance)
            if (tentative_cost, tentative_dist) < (neighbor_cost, neighbor_dist):
                # Update the neighbor's scores and parent
                parents[neighbor] = current
                cost_score[neighbor] = tentative_cost
                dist_score[neighbor] = tentative_dist

                # Calculate priority: cost first, then distance (both with heuristic)
                h_dist = _heuristic(neighbor_x, neighbor_y, goal_tuple[0], goal_tuple[1])
                priority_cost = tentative_cost
                priority_dist = tentative_dist + h_dist

                # Add the neighbor to the open set with its priority
                heapq.heappush(open_set, (priority_cost, priority_dist, neighbor))

    return None
