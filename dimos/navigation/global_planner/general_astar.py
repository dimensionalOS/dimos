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
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def general_astar(
    costmap: OccupancyGrid,
    goal: VectorLike,
    start: VectorLike = (0.0, 0.0),
    cost_threshold: int = 90,
    unknown_penalty: float = 0.8,
) -> Path | None:
    """
    A* path planning algorithm from start to goal position.

    Args:
        costmap: Costmap object containing the environment
        goal: Goal position as any vector-like object
        start: Start position as any vector-like object (default: origin [0,0])
        cost_threshold: Cost threshold above which a cell is considered an obstacle

    Returns:
        Path object containing waypoints, or None if no path found
    """

    # Convert world coordinates to grid coordinates directly using vector-like inputs
    start_vector = costmap.world_to_grid(start)
    goal_vector = costmap.world_to_grid(goal)
    logger.debug(f"ASTAR {costmap} {start_vector} -> {goal_vector}")

    # Store positions as tuples for dictionary keys
    start_tuple = (int(start_vector.x), int(start_vector.y))
    goal_tuple = (int(goal_vector.x), int(goal_vector.y))

    # Check if goal is out of bounds
    if not (0 <= goal_tuple[0] < costmap.width and 0 <= goal_tuple[1] < costmap.height):
        return None

    # Define possible movements (8-connected grid with diagonal movements)
    directions = [
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
    sc = 1.0  # Straight cost
    dc = 1.42  # Diagonal cost (approximately sqrt(2))
    movement_costs = [sc, sc, sc, sc, dc, dc, dc, dc]

    # A* algorithm implementation
    open_set = []  # type: ignore[var-annotated]  # Priority queue for nodes to explore
    closed_set = set()  # Set of explored nodes

    # Dictionary to store cost from start and parents for each node
    g_score = {start_tuple: 0}
    parents = {}  # type: ignore[var-annotated]

    # Heuristic function (Octile distance for 8-connected grid)
    def heuristic(x1, y1, x2, y2):  # type: ignore[no-untyped-def]
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        # Octile distance: optimal for 8-connected grids with diagonal movement
        return (dx + dy) + (dc - 2 * sc) * min(dx, dy)

    # Start with the starting node
    f_score = g_score[start_tuple] + heuristic(  # type: ignore[no-untyped-call]
        start_tuple[0], start_tuple[1], goal_tuple[0], goal_tuple[1]
    )
    heapq.heappush(open_set, (f_score, start_tuple))

    # Track nodes already in open set to avoid duplicates
    open_set_hash = {start_tuple}

    while open_set:
        # Get the node with the lowest f_score
        _current_f, current = heapq.heappop(open_set)
        current_x, current_y = current

        # Remove from open set hash
        if current in open_set_hash:
            open_set_hash.remove(current)

        # Skip if already processed (can happen with duplicate entries)
        if current in closed_set:
            continue

        # Check if we've reached the goal
        if current == goal_tuple:
            # Reconstruct the path
            waypoints = []
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
                or (waypoints[-1].x - goal_point.x) ** 2 + (waypoints[-1].y - goal_point.y) ** 2
                > 1e-10
            ):
                goal_pose = PoseStamped(
                    frame_id="world",
                    position=[goal_point.x, goal_point.y, 0.0],
                    orientation=Quaternion(0, 0, 0, 1),
                )
                waypoints.append(goal_pose)

            return Path(frame_id="world", poses=waypoints)

        # Add current node to closed set
        closed_set.add(current)

        # Explore neighbors
        for i, (dx, dy) in enumerate(directions):
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

            # Calculate cost penalty based on cell cost (higher cost = higher penalty)
            # This encourages the planner to prefer lower-cost paths
            cost_penalty = cell_cost / CostValues.OCCUPIED  # Normalized penalty (divide by 100)

            tentative_g_score = g_score[current] + movement_costs[i] * (1.0 + cost_penalty)

            # Get the current g_score for the neighbor or set to infinity if not yet explored
            neighbor_g_score = g_score.get(neighbor, float("inf"))

            # If this path to the neighbor is better than any previous one
            if tentative_g_score < neighbor_g_score:
                # Update the neighbor's scores and parent
                parents[neighbor] = current
                g_score[neighbor] = tentative_g_score  # type: ignore[assignment]
                f_score = tentative_g_score + heuristic(  # type: ignore[no-untyped-call]
                    neighbor_x, neighbor_y, goal_tuple[0], goal_tuple[1]
                )

                # Add the neighbor to the open set with its f_score
                # Only add if not already in open set to reduce duplicates
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score, neighbor))
                    open_set_hash.add(neighbor)

    # If we get here, no path was found
    return None
