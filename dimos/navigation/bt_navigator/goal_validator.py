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

from collections import deque

import numpy as np

from dimos.mapping.occupancy.gradient import gradient
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
    elif algorithm == "spiral":
        return _find_safe_goal_spiral(
            costmap,
            goal,
            cost_threshold,
            min_clearance,
            max_search_distance,
            connectivity_check_radius,
        )
    elif algorithm == "voronoi":
        return _find_safe_goal_voronoi(
            costmap, goal, cost_threshold, min_clearance, max_search_distance
        )
    elif algorithm == "gradient_descent":
        return _find_safe_goal_gradient(
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


def _find_safe_goal_spiral(
    costmap: OccupancyGrid,
    goal: VectorLike,
    cost_threshold: int,
    min_clearance: float,
    max_search_distance: float,
    connectivity_check_radius: int,
) -> Vector3 | None:
    """
    Spiral search pattern from goal outward.

    Pros:
    - Simple and predictable pattern
    - Memory efficient
    - Good for uniformly distributed obstacles

    Cons:
    - May not find the absolute closest safe position
    - Can miss nearby safe spots due to spiral pattern
    """

    # Convert goal to grid coordinates
    goal_grid = costmap.world_to_grid(goal)
    cx, cy = int(goal_grid.x), int(goal_grid.y)

    # Convert distances to grid cells
    clearance_cells = int(np.ceil(min_clearance / costmap.resolution))
    max_radius = int(np.ceil(max_search_distance / costmap.resolution))

    # Spiral outward
    for radius in range(0, max_radius + 1):
        if radius == 0:
            # Check center point
            if _is_position_safe(
                costmap, cx, cy, cost_threshold, clearance_cells, connectivity_check_radius
            ):
                return costmap.grid_to_world((cx, cy))
        else:
            # Check points on the square perimeter at this radius
            points = []

            # Top and bottom edges
            for x in range(cx - radius, cx + radius + 1):
                points.append((x, cy - radius))  # Top
                points.append((x, cy + radius))  # Bottom

            # Left and right edges (excluding corners to avoid duplicates)
            for y in range(cy - radius + 1, cy + radius):
                points.append((cx - radius, y))  # Left
                points.append((cx + radius, y))  # Right

            # Check each point
            for x, y in points:
                if 0 <= x < costmap.width and 0 <= y < costmap.height:
                    if _is_position_safe(
                        costmap, x, y, cost_threshold, clearance_cells, connectivity_check_radius
                    ):
                        return costmap.grid_to_world((x, y))

    return None


def _find_safe_goal_voronoi(
    costmap: OccupancyGrid,
    goal: VectorLike,
    cost_threshold: int,
    min_clearance: float,
    max_search_distance: float,
) -> Vector3 | None:
    """
    Find safe position using Voronoi diagram (ridge points equidistant from obstacles).

    Pros:
    - Finds positions maximally far from obstacles
    - Good for narrow passages
    - Natural safety margin

    Cons:
    - More computationally expensive
    - May find positions unnecessarily far from obstacles
    - Requires scipy for efficient implementation
    """

    from scipy import ndimage  # type: ignore[import-untyped]
    from skimage.morphology import skeletonize  # type: ignore[import-not-found]

    # Convert goal to grid coordinates
    goal_grid = costmap.world_to_grid(goal)
    gx, gy = int(goal_grid.x), int(goal_grid.y)

    # Create binary obstacle map
    free_map = (costmap.grid < cost_threshold) & (costmap.grid != CostValues.UNKNOWN)

    # Compute distance transform
    distance_field = ndimage.distance_transform_edt(free_map)

    # Find skeleton/medial axis (approximation of Voronoi diagram)
    skeleton = skeletonize(free_map)

    # Filter skeleton points by minimum clearance
    clearance_cells = int(np.ceil(min_clearance / costmap.resolution))
    valid_skeleton = skeleton & (distance_field >= clearance_cells)  # type: ignore[operator]

    if not np.any(valid_skeleton):
        # Fall back to BFS if no valid skeleton points
        return _find_safe_goal_bfs(
            costmap, goal, cost_threshold, min_clearance, max_search_distance, 3
        )

    # Find nearest valid skeleton point to goal
    skeleton_points = np.argwhere(valid_skeleton)
    if len(skeleton_points) == 0:
        return None

    # Calculate distances from goal to all skeleton points
    distances = np.sqrt((skeleton_points[:, 1] - gx) ** 2 + (skeleton_points[:, 0] - gy) ** 2)

    # Filter by max search distance
    max_search_cells = max_search_distance / costmap.resolution
    valid_indices = distances <= max_search_cells

    if not np.any(valid_indices):
        return None

    # Find closest valid point
    valid_distances = distances[valid_indices]
    valid_points = skeleton_points[valid_indices]
    closest_idx = np.argmin(valid_distances)
    best_y, best_x = valid_points[closest_idx]

    return costmap.grid_to_world((best_x, best_y))


def _find_safe_goal_gradient(
    costmap: OccupancyGrid,
    goal: VectorLike,
    cost_threshold: int,
    min_clearance: float,
    max_search_distance: float,
    connectivity_check_radius: int,
) -> Vector3 | None:
    """
    Use gradient descent on the costmap to find a safe position.

    Pros:
    - Naturally flows away from obstacles
    - Works well with gradient costmaps
    - Can handle complex cost distributions

    Cons:
    - Can get stuck in local minima
    - Requires a gradient costmap
    - May not find globally optimal position
    """

    # Convert goal to grid coordinates
    goal_grid = costmap.world_to_grid(goal)
    x, y = goal_grid.x, goal_grid.y

    # Convert distances to grid cells
    clearance_cells = int(np.ceil(min_clearance / costmap.resolution))
    max_search_cells = int(np.ceil(max_search_distance / costmap.resolution))

    # Create gradient if needed (assuming costmap might already be a gradient)
    if np.all((costmap.grid == 0) | (costmap.grid == 100) | (costmap.grid == -1)):
        # Binary map, create gradient
        gradient_map = gradient(
            costmap, obstacle_threshold=cost_threshold, max_distance=min_clearance * 2
        )
        grid = gradient_map.grid
    else:
        grid = costmap.grid

    # Gradient descent with momentum
    momentum = 0.9
    learning_rate = 1.0
    vx, vy = 0.0, 0.0

    best_x, best_y = None, None
    best_cost = float("inf")

    for iteration in range(100):  # Max iterations
        ix, iy = int(x), int(y)

        # Check if current position is valid
        if 0 <= ix < costmap.width and 0 <= iy < costmap.height:
            current_cost = grid[iy, ix]

            # Check distance from original goal
            dist = np.sqrt((x - goal_grid.x) ** 2 + (y - goal_grid.y) ** 2)
            if dist > max_search_cells:
                break

            # Check if position is safe
            if _is_position_safe(
                costmap, ix, iy, cost_threshold, clearance_cells, connectivity_check_radius
            ):
                if current_cost < best_cost:
                    best_x, best_y = ix, iy
                    best_cost = current_cost

                    # If cost is very low, we found a good spot
                    if current_cost < 10:
                        break

        # Compute gradient using finite differences
        gx, gy = 0.0, 0.0

        if 0 < ix < costmap.width - 1:
            gx = (grid[iy, min(ix + 1, costmap.width - 1)] - grid[iy, max(ix - 1, 0)]) / 2.0

        if 0 < iy < costmap.height - 1:
            gy = (grid[min(iy + 1, costmap.height - 1), ix] - grid[max(iy - 1, 0), ix]) / 2.0

        # Update with momentum
        vx = momentum * vx - learning_rate * gx
        vy = momentum * vy - learning_rate * gy

        # Update position
        x += vx
        y += vy

        # Add small random noise to escape local minima
        if iteration % 20 == 0:
            x += np.random.randn() * 0.5
            y += np.random.randn() * 0.5

    if best_x is not None and best_y is not None:
        return costmap.grid_to_world((best_x, best_y))

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
