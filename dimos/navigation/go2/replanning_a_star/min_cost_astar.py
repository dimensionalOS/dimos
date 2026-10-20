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

from copy import deepcopy
import heapq
from typing import overload

import numpy as np
from numpy.typing import NDArray

from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped as CdrPoseStamped,
    Quaternion as CdrQuaternion,
)
from dimos.messages.nav_msgs.msg import OccupancyGrid as CdrOccupancyGrid, Path as CdrPath
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.occupancy import grid_to_world, occupancy_view, world_to_grid
from dimos.utils.logging_config import setup_logger

# Try to import C++ extension for faster pathfinding
try:
    from dimos.navigation.go2.replanning_a_star.min_cost_astar_ext import (
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
    start_tuple: tuple[int, int],
) -> list[tuple[int, int]]:
    coordinates = []
    while current in parents:
        coordinates.append(current)
        current = parents[current]
    coordinates.append(start_tuple)
    coordinates.reverse()
    return coordinates


def _reconstruct_path_from_coords(
    path_coords: list[tuple[int, int]],
    costmap: OccupancyGrid | CdrOccupancyGrid,
) -> Path | CdrPath:
    if isinstance(costmap, CdrOccupancyGrid):
        return CdrPath(
            header=deepcopy(costmap.header),
            poses=[
                CdrPoseStamped(
                    header=deepcopy(costmap.header),
                    pose=Pose(
                        position=grid_to_world(costmap, point),
                        orientation=CdrQuaternion(x=0, y=0, z=0, w=1),
                    ),
                )
                for point in path_coords
            ],
        )
    waypoints = []
    for coordinate in path_coords:
        point = costmap.grid_to_world(coordinate)
        waypoints.append(
            PoseStamped(
                frame_id=costmap.frame_id,
                position=[point.x, point.y, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            )
        )
    return Path(frame_id=costmap.frame_id, poses=waypoints)


def _cell(costmap: OccupancyGrid | CdrOccupancyGrid, value: Point | VectorLike) -> tuple[int, int]:
    vector = Vector3((value.x, value.y, value.z)) if isinstance(value, Point) else Vector3(value)
    if isinstance(costmap, CdrOccupancyGrid):
        x, y = world_to_grid(costmap, Point(x=vector.x, y=vector.y, z=vector.z))
    else:
        coordinates = costmap.world_to_grid(vector)
        x, y = coordinates.x, coordinates.y
    return int(x), int(y)


@overload
def min_cost_astar(
    costmap: CdrOccupancyGrid,
    goal: Point | VectorLike,
    start: Point | VectorLike = (0.0, 0.0),
    cost_threshold: int = 100,
    unknown_penalty: float = 0.8,
    use_cpp: bool = True,
) -> CdrPath | None: ...


@overload
def min_cost_astar(
    costmap: OccupancyGrid,
    goal: Point | VectorLike,
    start: Point | VectorLike = (0.0, 0.0),
    cost_threshold: int = 100,
    unknown_penalty: float = 0.8,
    use_cpp: bool = True,
) -> Path | None: ...


def min_cost_astar(
    costmap: OccupancyGrid | CdrOccupancyGrid,
    goal: Point | VectorLike,
    start: Point | VectorLike = (0.0, 0.0),
    cost_threshold: int = 100,
    unknown_penalty: float = 0.8,
    use_cpp: bool = True,
) -> Path | CdrPath | None:
    start_tuple = _cell(costmap, start)
    goal_tuple = _cell(costmap, goal)
    cells = occupancy_view(costmap) if isinstance(costmap, CdrOccupancyGrid) else costmap.grid
    height, width = cells.shape
    if not (0 <= goal_tuple[0] < width and 0 <= goal_tuple[1] < height):
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

    coordinates = _find_path(cells, start_tuple, goal_tuple, cost_threshold, unknown_penalty)
    if coordinates is None:
        return None
    return _reconstruct_path_from_coords(coordinates, costmap)


def _find_path(
    cells: NDArray[np.int8],
    start_tuple: tuple[int, int],
    goal_tuple: tuple[int, int],
    cost_threshold: int,
    unknown_penalty: float,
) -> list[tuple[int, int]] | None:
    height, width = cells.shape
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
            return _reconstruct_path(parents, current, start_tuple)

        closed_set.add(current)

        for i, (dx, dy) in enumerate(_directions):
            neighbor_x, neighbor_y = current_x + dx, current_y + dy
            neighbor = (neighbor_x, neighbor_y)

            if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                continue

            if neighbor in closed_set:
                continue

            neighbor_val = cells[neighbor_y, neighbor_x]

            if neighbor_val >= cost_threshold:
                continue

            if neighbor_val == CostValues.UNKNOWN:
                cell_cost = cost_threshold * unknown_penalty
                if cell_cost >= cost_threshold:
                    continue
            elif neighbor_val == CostValues.FREE:
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
