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

from collections import deque

import numpy as np

from dimos.msgs.geometry_msgs import Vector3, VectorLike
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid


def find_safe_goal(
    costmap: OccupancyGrid,
    goal: VectorLike,
    algorithm: str = "bfs",
    cost_threshold: int = 50,
    min_clearance: float = 0.3,
    max_search_distance: float = 5.0,
    connectivity_check_radius: int = 3,
) -> Vector3 | None:
    """
    Find a safe goal position when the original goal is in collision or too close to obstacles.

    Args:
        costmap: The occupancy grid/costmap
        goal: Original goal position in world coordinates
        algorithm: Algorithm to use ("bfs", "spiral", "voronoi", "gradient_descent")
        cost_threshold: Maximum acceptable cost for a safe position (default: 50)
        min_clearance: Minimum clearance from obstacles in meters (default: 0.3m)
        max_search_distance: Maximum distance to search from original goal in meters (default: 5.0m)
        connectivity_check_radius: Radius in cells to check for connectivity (default: 3)

    Returns:
        Safe goal position in world coordinates, or None if no safe position found
    """

    if algorithm == "bfs":
        return _find_safe_goal_bfs(
            costmap,
            goal,
            cost_threshold,
            min_clearance,
            max_search_distance,
            connectivity_check_radius,
        )
    elif algorithm == "bfs_contiguous":
        return _find_safe_goal_bfs_contiguous(
            costmap,
            goal,
            cost_threshold,
            min_clearance,
            max_search_distance,
            connectivity_check_radius,
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")


def _find_safe_goal_bfs(
    costmap: OccupancyGrid,
    goal: VectorLike,
    cost_threshold: int,
    min_clearance: float,
    max_search_distance: float,
    connectivity_check_radius: int,
) -> Vector3 | None:
    """
    BFS-based search for nearest safe goal position.
    This guarantees finding the closest valid position.

    Pros:
    - Guarantees finding the closest safe position
    - Can check connectivity to avoid isolated spots
    - Efficient for small to medium search areas

    Cons:
    - Can be slower for large search areas
    - Memory usage scales with search area
    """

    # Convert goal to grid coordinates
    goal_grid = costmap.world_to_grid(goal)
    gx, gy = int(goal_grid.x), int(goal_grid.y)

    # Convert distances to grid cells
    clearance_cells = int(np.ceil(min_clearance / costmap.resolution))
    max_search_cells = int(np.ceil(max_search_distance / costmap.resolution))

    # BFS queue and visited set
    queue = deque([(gx, gy, 0)])
    visited = set([(gx, gy)])

    # 8-connected neighbors
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    while queue:
        x, y, dist = queue.popleft()

        # Check if we've exceeded max search distance
        if dist > max_search_cells:
            break

        # Check if position is valid
        if _is_position_safe(
            costmap, x, y, cost_threshold, clearance_cells, connectivity_check_radius
        ):
            # Convert back to world coordinates
            return costmap.grid_to_world((x, y))

        # Add neighbors to queue
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy

            # Check bounds
            if 0 <= nx < costmap.width and 0 <= ny < costmap.height:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

    return None


def _find_safe_goal_bfs_contiguous(
    costmap: OccupancyGrid,
    goal: VectorLike,
    cost_threshold: int,
    min_clearance: float,
    max_search_distance: float,
    connectivity_check_radius: int,
) -> Vector3 | None:
    """
    BFS-based search for nearest safe goal position, only following passable cells.
    Unlike regular BFS, this only expands through cells with occupancy < 100,
    ensuring the path doesn't cross through impassable obstacles.

    Pros:
    - Guarantees finding the closest safe position reachable without crossing obstacles
    - Ensures connectivity to the goal through passable space
    - Good for finding safe positions in the same "room" or connected area

    Cons:
    - May not find nearby safe spots if they're on the other side of a wall
    - Slightly slower than regular BFS due to additional checks
    """

    # Convert goal to grid coordinates
    goal_grid = costmap.world_to_grid(goal)
    gx, gy = int(goal_grid.x), int(goal_grid.y)

    # Convert distances to grid cells
    clearance_cells = int(np.ceil(min_clearance / costmap.resolution))
    max_search_cells = int(np.ceil(max_search_distance / costmap.resolution))

    # BFS queue and visited set
    queue = deque([(gx, gy, 0)])
    visited = set([(gx, gy)])

    # 8-connected neighbors
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    while queue:
        x, y, dist = queue.popleft()

        # Check if we've exceeded max search distance
        if dist > max_search_cells:
            break

        # Check if position is valid
        if _is_position_safe(
            costmap, x, y, cost_threshold, clearance_cells, connectivity_check_radius
        ):
            # Convert back to world coordinates
            return costmap.grid_to_world((x, y))

        # Add neighbors to queue
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy

            # Check bounds
            if 0 <= nx < costmap.width and 0 <= ny < costmap.height:
                if (nx, ny) not in visited:
                    # Only expand through passable cells (occupancy < 100)
                    if costmap.grid[ny, nx] < 100:
                        visited.add((nx, ny))
                        queue.append((nx, ny, dist + 1))

    return None


def _is_position_safe(
    costmap: OccupancyGrid,
    x: int,
    y: int,
    cost_threshold: int,
    clearance_cells: int,
    connectivity_check_radius: int,
) -> bool:
    """
    Check if a position is safe based on multiple criteria.

    Args:
        costmap: The occupancy grid
        x, y: Grid coordinates to check
        cost_threshold: Maximum acceptable cost
        clearance_cells: Minimum clearance in cells
        connectivity_check_radius: Radius to check for connectivity

    Returns:
        True if position is safe, False otherwise
    """

    # Check bounds first
    if not (0 <= x < costmap.width and 0 <= y < costmap.height):
        return False

    # Check if position itself is free
    if costmap.grid[y, x] >= cost_threshold or costmap.grid[y, x] == CostValues.UNKNOWN:
        return False

    # Check clearance around position
    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < costmap.width and 0 <= ny < costmap.height:
                # Check if within circular clearance
                if dx * dx + dy * dy <= clearance_cells * clearance_cells:
                    if costmap.grid[ny, nx] >= cost_threshold:
                        return False

    # Check connectivity (not surrounded by obstacles)
    # Count free neighbors in a larger radius
    free_count = 0
    total_count = 0

    for dy in range(-connectivity_check_radius, connectivity_check_radius + 1):
        for dx in range(-connectivity_check_radius, connectivity_check_radius + 1):
            if dx == 0 and dy == 0:
                continue

            nx, ny = x + dx, y + dy
            if 0 <= nx < costmap.width and 0 <= ny < costmap.height:
                total_count += 1
                if (
                    costmap.grid[ny, nx] < cost_threshold
                    and costmap.grid[ny, nx] != CostValues.UNKNOWN
                ):
                    free_count += 1

    # Require at least 50% of neighbors to be free (not surrounded)
    if total_count > 0 and free_count < total_count * 0.5:
        return False

    return True
