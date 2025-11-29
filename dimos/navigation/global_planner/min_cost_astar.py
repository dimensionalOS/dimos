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

import heapq

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, VectorLike
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid, Path

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
    start_tuple: tuple[int, int],
    goal_tuple: tuple[int, int],
) -> Path:
    # Reconstruct the path
    waypoints: list[PoseStamped] = []
    while current in parents:
        world_point = costmap.grid_to_world(current)
        # Create PoseStamped with identity quaternion (no orientation)
        pose = PoseStamped(
            frame_id="world",
            position=[world_point.x, world_point.y, 0.0],
            orientation=Quaternion(0, 0, 0, 1),  # Identity quaternion
        )
        waypoints.append(pose)
        current = parents[current]

    # Add the start position
    start_world_point = costmap.grid_to_world(start_tuple)
    start_pose = PoseStamped(
        frame_id="world",
        position=[start_world_point.x, start_world_point.y, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )
    waypoints.append(start_pose)

    # Reverse the path (start to goal)
    waypoints.reverse()

    # Add the goal position if it's not already included
    goal_point = costmap.grid_to_world(goal_tuple)

    if (
        not waypoints
        or (waypoints[-1].x - goal_point.x) ** 2 + (waypoints[-1].y - goal_point.y) ** 2 > 1e-10
    ):
        goal_pose = PoseStamped(
            frame_id="world",
            position=[goal_point.x, goal_point.y, 0.0],
            orientation=Quaternion(0, 0, 0, 1),
        )
        waypoints.append(goal_pose)

    return Path(frame_id="world", poses=waypoints)


def min_cost_astar(
    costmap: OccupancyGrid,
    goal: VectorLike,
    start: VectorLike = (0.0, 0.0),
    cost_threshold: int = 100,
    unknown_penalty: float = 0.8,
) -> Path | None:
    # Convert world coordinates to grid coordinates directly using vector-like inputs
    start_vector = costmap.world_to_grid(start)
    goal_vector = costmap.world_to_grid(goal)

    # Store positions as tuples for dictionary keys
    start_tuple = (int(start_vector.x), int(start_vector.y))
    goal_tuple = (int(goal_vector.x), int(goal_vector.y))

    # Check if goal is out of bounds
    if not (0 <= goal_tuple[0] < costmap.width and 0 <= goal_tuple[1] < costmap.height):
        return None

    # A* algorithm implementation
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

    # Track nodes already in open set to avoid duplicates
    open_set_hash: set[tuple[int, int]] = {start_tuple}

    while open_set:
        # Get the node with the lowest priority (cost first, then distance)
        _current_cost, _current_dist, current = heapq.heappop(open_set)
        current_x, current_y = current

        # Remove from open set hash
        if current in open_set_hash:
            open_set_hash.remove(current)

        # Skip if already processed (can happen with duplicate entries)
        if current in closed_set:
            continue

        # Check if we've reached the goal
        if current == goal_tuple:
            return _reconstruct_path(parents, current, costmap, start_tuple, goal_tuple)

        # Add current node to closed set
        closed_set.add(current)

        # Explore neighbors
        for i, (dx, dy) in enumerate(_directions):
            neighbor_x, neighbor_y = current_x + dx, current_y + dy
            neighbor = (neighbor_x, neighbor_y)

            # Check if the neighbor is valid
            if not (0 <= neighbor_x < costmap.width and 0 <= neighbor_y < costmap.height):
                continue

            # Check if the neighbor is already explored
            if neighbor in closed_set:
                continue

            # Get the neighbor's cost value
            neighbor_val = costmap.grid[neighbor_y, neighbor_x]

            # Skip if it's a hard obstacle
            if neighbor_val >= cost_threshold:
                continue

            # Calculate movement cost with penalties
            # Unknown cells get half the penalty of obstacles
            if neighbor_val == CostValues.UNKNOWN:  # Unknown cell (-1)
                # Unknown cells have a moderate traversal cost (half of obstacle threshold)
                cell_cost = cost_threshold * unknown_penalty
            elif neighbor_val == CostValues.FREE:  # Free space (0)
                # Free cells have minimal cost
                cell_cost = 0.0
            else:
                # Other cells use their actual cost value (1-99)
                cell_cost = neighbor_val

            # Calculate tentative scores: accumulate cost and distance separately
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
                # Only add if not already in open set to reduce duplicates
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (priority_cost, priority_dist, neighbor))
                    open_set_hash.add(neighbor)

    return None
