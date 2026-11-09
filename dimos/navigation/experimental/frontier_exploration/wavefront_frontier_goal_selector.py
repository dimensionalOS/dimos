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

"""
Simple wavefront frontier exploration algorithm implementation using dimos types.

This module provides frontier detection and exploration goal selection
for autonomous navigation using the dimos Costmap and Vector types.
"""

from collections import deque
from dataclasses import dataclass
from enum import IntFlag
import heapq
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
import numpy as np
from reactivex.disposable import Disposable
from scipy.ndimage import binary_dilation, label

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import get_distance

logger = setup_logger()


class PointClassification(IntFlag):
    """Point classification flags for frontier detection algorithm."""

    NoInformation = 0
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


@dataclass
class GridPoint:
    """Represents a point in the grid map with classification."""

    x: int
    y: int
    classification: int = PointClassification.NoInformation


class FrontierCache:
    """Cache for grid points to avoid duplicate point creation."""

    def __init__(self) -> None:
        self.points = {}  # type: ignore[var-annotated]

    def get_point(self, x: int, y: int) -> GridPoint:
        """Get or create a grid point at the given coordinates."""
        key = (x, y)
        if key not in self.points:
            self.points[key] = GridPoint(x, y)
        return self.points[key]  # type: ignore[no-any-return]

    def clear(self) -> None:
        """Clear the point cache."""
        self.points.clear()


class WavefrontConfig(ModuleConfig):
    min_frontier_perimeter: float = 0.5
    # Obstacles are grown by this much for planning. It also sets how far a frontier
    # may sit from unknown space, so it is one number rather than two that can disagree.
    costmap_inflation: float = 0.25
    occupancy_threshold: int = 99
    safe_distance: float = 3.0
    lookahead_distance: float = 5.0
    max_explored_distance: float = 10.0
    info_gain_threshold: float = 0.03
    num_no_gain_attempts: int = 2
    goal_timeout: float = 15.0
    # Budget for the leg itself: goal_timeout plus path length / this speed, so a
    # far frontier is not declared a failure just because it is far.
    goal_speed: float = 0.4
    # A target that timed out is cooled off for this long instead of being picked again.
    failure_cooldown: float = 60.0
    # A goal already reached is refused as a repeat for this long, inside this radius.
    # It expires because reaching the next part of a building often means driving back
    # through ground already crossed; refusing it forever ends the run early.
    visited_exclusion_radius: float = 0.6
    visited_exclusion_ttl: float = 30.0


def reachable_distances(grid: OccupancyGrid, pose: Vector3) -> np.ndarray[Any, Any]:
    """Dijkstra over free cells; diagonal steps cannot cut occupied corners.

    Returns metres of *travel* cost from the robot to every cell, ``inf`` where the
    cell cannot be reached. Straight-line distance is not enough: a frontier on the
    far side of a wall is close, but unreachable.

    """
    free = grid.grid == CostValues.FREE
    distances = np.full(free.shape, np.inf)
    start = grid.world_to_grid(pose)
    x, y = math.floor(start.x), math.floor(start.y)
    if not (0 <= x < grid.width and 0 <= y < grid.height and free[y, x]):
        return distances
    distances[y, x] = 0
    queue = [(0.0, y, x)]
    while queue:
        distance, row, col = heapq.heappop(queue)
        if distance != distances[row, col]:
            continue
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            ny, nx = row + dy, col + dx
            if not (0 <= nx < grid.width and 0 <= ny < grid.height and free[ny, nx]):
                continue
            if dx and dy and not (free[row, nx] and free[ny, col]):
                continue
            candidate = distance + grid.resolution * (math.sqrt(2) if dx and dy else 1)
            if candidate < distances[ny, nx]:
                distances[ny, nx] = candidate
                heapq.heappush(queue, (candidate, ny, nx))
    return distances


def _open_start(grid: OccupancyGrid, pose: Vector3, metres: float = 0.3) -> OccupancyGrid:
    """A copy of the map with a drivable hole opened around ``pose``.

    The robot is usually standing inside the inflation buffer of an obstacle it is
    passing, which makes its own cell read as occupied and the whole map unreachable.
    """

    opened = grid.copy()
    centre = grid.world_to_grid(pose)
    row, col = math.floor(centre.y), math.floor(centre.x)
    if not (0 <= row < opened.height and 0 <= col < opened.width):
        return opened
    reach = max(0, math.ceil(metres / grid.resolution))
    rows, cols = np.indices(opened.grid.shape)
    near = (rows - row) ** 2 + (cols - col) ** 2 <= reach**2
    opened.grid[near & (opened.grid != CostValues.UNKNOWN)] = CostValues.FREE
    return opened


class WavefrontFrontierExplorer(Module):
    """
    Wavefront frontier exploration algorithm implementation.

    This class encapsulates the frontier detection and exploration goal selection
    functionality using the wavefront algorithm with BFS exploration.

    Inputs:
        - costmap: Current costmap for frontier detection
        - odometry: Current robot pose

    Outputs:
        - goal_request: Exploration goals sent to the navigator
    """

    config: WavefrontConfig

    # LCM inputs
    global_costmap: In[OccupancyGrid]
    odom: In[PoseStamped]
    goal_reached: In[Bool]
    explore_cmd: In[Bool]
    stop_explore_cmd: In[Bool]
    stop_movement: In[Bool]

    # LCM outputs
    goal_request: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        """
        Initialize the frontier explorer.

        Args:
            min_frontier_perimeter: Minimum perimeter in meters to consider a valid frontier
            occupancy_threshold: Cost threshold above which a cell is considered occupied (0-255)
            safe_distance: Safe distance from obstacles for scoring (meters)
            info_gain_threshold: Minimum percentage increase in costmap information required to continue exploration (0.05 = 5%)
            num_no_gain_attempts: Maximum number of consecutive attempts with no information gain
        """
        super().__init__(**kwargs)
        self._cache = FrontierCache()
        self.explored_goals = []  # type: ignore[var-annotated]  # list of explored goals
        self.exploration_direction = Vector3(0.0, 0.0, 0.0)  # current exploration direction
        self.last_costmap = None  # store last costmap for information comparison
        self.no_gain_counter = 0  # track consecutive no-gain attempts

        # Latest data
        self.latest_costmap: OccupancyGrid | None = None
        self.latest_odometry: PoseStamped | None = None

        # Targets that timed out: (goal, monotonic deadline). Re-selecting a target the
        # navigator could not reach is the main source of re-walked ground.
        self._failed: list[tuple[Vector3, float]] = []
        # Goals already reached, with the moment they stop being treated as repeats.
        self._visited: list[tuple[Vector3, float]] = []
        self._last_goal_cost = 0.0

        # Goal reached event
        self.goal_reached_event = threading.Event()

        # Exploration state
        self.exploration_active = False
        self.exploration_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        unsub = self.global_costmap.subscribe(self._on_costmap)
        self.register_disposable(Disposable(unsub))

        unsub = self.odom.subscribe(self._on_odometry)
        self.register_disposable(Disposable(unsub))

        if self.goal_reached.transport is not None:
            unsub = self.goal_reached.subscribe(self._on_goal_reached)
            self.register_disposable(Disposable(unsub))

        if self.explore_cmd.transport is not None:
            unsub = self.explore_cmd.subscribe(self._on_explore_cmd)
            self.register_disposable(Disposable(unsub))

        if self.stop_explore_cmd.transport is not None:
            unsub = self.stop_explore_cmd.subscribe(self._on_stop_explore_cmd)
            self.register_disposable(Disposable(unsub))

        if self.stop_movement.transport is not None:
            unsub = self.stop_movement.subscribe(self._on_stop_movement)
            self.register_disposable(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        self.stop_exploration()
        super().stop()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        """Handle incoming costmap messages."""
        self.latest_costmap = msg

    def _on_odometry(self, msg: PoseStamped) -> None:
        """Handle incoming odometry messages."""
        self.latest_odometry = msg

    def _on_goal_reached(self, msg: Bool) -> None:
        """Handle goal reached messages."""
        if msg.data:
            self.goal_reached_event.set()

    def _on_explore_cmd(self, msg: Bool) -> None:
        """Handle exploration command messages."""
        if msg.data:
            logger.info("Received exploration start command via LCM")
            self.explore()

    def _on_stop_explore_cmd(self, msg: Bool) -> None:
        """Handle stop exploration command messages."""
        if msg.data:
            logger.info("Received exploration stop command via LCM")
            self.stop_exploration()

    def _on_stop_movement(self, msg: Bool) -> None:
        """Handle stop movement from teleop — cancel active exploration."""
        if msg.data and self.exploration_active:
            logger.info("WavefrontFrontierExplorer: stop_movement received, stopping exploration")
            self.stop_exploration()

    def _count_costmap_information(self, costmap: OccupancyGrid) -> int:
        """
        Count the amount of information in a costmap (free space + obstacles).

        Args:
            costmap: Costmap to analyze

        Returns:
            Number of cells that are free space or obstacles (not unknown)
        """
        free_count = np.sum(costmap.grid == CostValues.FREE)
        obstacle_count = np.sum(costmap.grid >= self.config.occupancy_threshold)
        return int(free_count + obstacle_count)

    def _get_neighbors(self, point: GridPoint, costmap: OccupancyGrid) -> list[GridPoint]:
        """Get valid neighboring points for a given grid point."""
        neighbors = []

        # 8-connected neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                nx, ny = point.x + dx, point.y + dy

                # Check bounds
                if 0 <= nx < costmap.width and 0 <= ny < costmap.height:
                    neighbors.append(self._cache.get_point(nx, ny))

        return neighbors

    def _is_frontier_point(self, point: GridPoint, costmap: OccupancyGrid) -> bool:
        """
        Check if a point is a frontier point.
        A frontier point is an unknown cell adjacent to at least one free cell
        and not adjacent to any occupied cells.
        """
        # Point must be unknown
        cost = costmap.grid[point.y, point.x]
        if cost != CostValues.UNKNOWN:
            return False

        has_free = False

        for neighbor in self._get_neighbors(point, costmap):
            neighbor_cost = costmap.grid[neighbor.y, neighbor.x]

            # If adjacent to occupied space, not a frontier
            if neighbor_cost > self.config.occupancy_threshold:
                return False

            # Check if adjacent to free space
            if neighbor_cost == CostValues.FREE:
                has_free = True

        return has_free

    def _find_free_space(
        self, start_x: int, start_y: int, costmap: OccupancyGrid
    ) -> tuple[int, int]:
        """
        Find the nearest free space point using BFS from the starting position.
        """
        queue = deque([self._cache.get_point(start_x, start_y)])
        visited = set()

        while queue:
            point = queue.popleft()

            if (point.x, point.y) in visited:
                continue
            visited.add((point.x, point.y))

            # Check if this point is free space
            if costmap.grid[point.y, point.x] == CostValues.FREE:
                return (point.x, point.y)

            # Add neighbors to search
            for neighbor in self._get_neighbors(point, costmap):
                if (neighbor.x, neighbor.y) not in visited:
                    queue.append(neighbor)

        # If no free space found, return original position
        return (start_x, start_y)

    def _compute_centroid(self, frontier_points: list[Vector3]) -> Vector3:
        """Compute the centroid of a list of frontier points."""
        if not frontier_points:
            return Vector3(0.0, 0.0, 0.0)

        # Vectorized approach using numpy
        points_array = np.array([[point.x, point.y] for point in frontier_points])
        centroid = np.mean(points_array, axis=0)

        return Vector3(centroid[0], centroid[1], 0.0)

    def detect_frontiers(self, robot_pose: Vector3, costmap: OccupancyGrid) -> list[Vector3]:
        """
        Main frontier detection algorithm using wavefront exploration.

        Args:
            robot_pose: Current robot position in world coordinates
            costmap: Costmap for frontier detection

        Returns:
            List of frontier centroids in world coordinates
        """
        self._cache.clear()

        # Convert robot pose to grid coordinates
        grid_pos = costmap.world_to_grid(robot_pose)
        grid_x, grid_y = int(grid_pos.x), int(grid_pos.y)

        # Find nearest free space to start exploration
        free_x, free_y = self._find_free_space(grid_x, grid_y, costmap)
        start_point = self._cache.get_point(free_x, free_y)
        start_point.classification = PointClassification.MapOpen

        # Main exploration queue - explore ALL reachable free space
        map_queue = deque([start_point])
        frontiers = []
        frontier_sizes = []

        points_checked = 0
        frontier_candidates = 0

        while map_queue:
            current_point = map_queue.popleft()
            points_checked += 1

            # Skip if already processed
            if current_point.classification & PointClassification.MapClosed:
                continue

            # Mark as processed
            current_point.classification |= PointClassification.MapClosed

            # Check if this point starts a new frontier
            if self._is_frontier_point(current_point, costmap):
                frontier_candidates += 1
                current_point.classification |= PointClassification.FrontierOpen
                frontier_queue = deque([current_point])
                new_frontier = []

                # Explore this frontier region using BFS
                while frontier_queue:
                    frontier_point = frontier_queue.popleft()

                    # Skip if already processed
                    if frontier_point.classification & PointClassification.FrontierClosed:
                        continue

                    # If this is still a frontier point, add to current frontier
                    if self._is_frontier_point(frontier_point, costmap):
                        new_frontier.append(frontier_point)

                        # Add neighbors to frontier queue
                        for neighbor in self._get_neighbors(frontier_point, costmap):
                            if not (
                                neighbor.classification
                                & (
                                    PointClassification.FrontierOpen
                                    | PointClassification.FrontierClosed
                                )
                            ):
                                neighbor.classification |= PointClassification.FrontierOpen
                                frontier_queue.append(neighbor)

                    frontier_point.classification |= PointClassification.FrontierClosed

                # Check if we found a large enough frontier
                # Convert minimum perimeter to minimum number of cells based on resolution
                min_cells = int(self.config.min_frontier_perimeter / costmap.resolution)
                if len(new_frontier) >= min_cells:
                    world_points = []
                    for point in new_frontier:
                        world_pos = costmap.grid_to_world(
                            Vector3(float(point.x), float(point.y), 0.0)
                        )
                        world_points.append(world_pos)

                    # Compute centroid in world coordinates (already correctly scaled)
                    centroid = self._compute_centroid(world_points)
                    frontiers.append(centroid)  # Store centroid
                    frontier_sizes.append(len(new_frontier))  # Store frontier size

            # Add ALL neighbors to main exploration queue to explore entire free space
            for neighbor in self._get_neighbors(current_point, costmap):
                if not (
                    neighbor.classification
                    & (PointClassification.MapOpen | PointClassification.MapClosed)
                ):
                    # Check if neighbor is free space or unknown (explorable)
                    neighbor_cost = costmap.grid[neighbor.y, neighbor.x]

                    # Add free space and unknown space to exploration queue
                    if neighbor_cost == CostValues.FREE or neighbor_cost == CostValues.UNKNOWN:
                        neighbor.classification |= PointClassification.MapOpen
                        map_queue.append(neighbor)

        # Extract just the centroids for ranking
        frontier_centroids = frontiers

        if not frontier_centroids:
            return []

        # Rank frontiers using original costmap for proper filtering
        ranked_frontiers = self._rank_frontiers(
            frontier_centroids, frontier_sizes, robot_pose, costmap
        )

        return ranked_frontiers

    def _update_exploration_direction(
        self, robot_pose: Vector3, goal_pose: Vector3 | None = None
    ) -> None:
        """Update the current exploration direction based on robot movement or selected goal."""
        if goal_pose is not None:
            # Calculate direction from robot to goal
            direction = Vector3(goal_pose.x - robot_pose.x, goal_pose.y - robot_pose.y, 0.0)
            magnitude = np.sqrt(direction.x**2 + direction.y**2)
            if magnitude > 0.1:  # Avoid division by zero for very close goals
                self.exploration_direction = Vector3(
                    direction.x / magnitude, direction.y / magnitude, 0.0
                )

    def _compute_direction_momentum_score(self, frontier: Vector3, robot_pose: Vector3) -> float:
        """Compute direction momentum score for a frontier."""
        if self.exploration_direction.x == 0 and self.exploration_direction.y == 0:
            return 0.0  # No momentum if no previous direction

        # Calculate direction from robot to frontier
        frontier_direction = Vector3(frontier.x - robot_pose.x, frontier.y - robot_pose.y, 0.0)
        magnitude = np.sqrt(frontier_direction.x**2 + frontier_direction.y**2)

        if magnitude < 0.1:
            return 0.0  # Too close to calculate meaningful direction

        # Normalize frontier direction
        frontier_direction = Vector3(
            frontier_direction.x / magnitude, frontier_direction.y / magnitude, 0.0
        )

        # Calculate dot product for directional alignment
        dot_product = (
            self.exploration_direction.x * frontier_direction.x
            + self.exploration_direction.y * frontier_direction.y
        )

        # Return momentum score (higher for same direction, lower for opposite)
        return max(0.0, dot_product)  # Only positive momentum, no penalty for different directions

    def _compute_distance_to_explored_goals(self, frontier: Vector3) -> float:
        """Compute distance from frontier to the nearest explored goal."""
        if not self.explored_goals:
            return 5.0  # Default consistent value when no explored goals
        # Calculate distance to nearest explored goal
        min_distance = float("inf")
        for goal in self.explored_goals:
            distance = np.sqrt((frontier.x - goal.x) ** 2 + (frontier.y - goal.y) ** 2)
            min_distance = min(min_distance, distance)

        return min_distance

    def _compute_distance_to_obstacles(self, frontier: Vector3, costmap: OccupancyGrid) -> float:
        """
        Compute the minimum distance from a frontier point to the nearest obstacle.

        Args:
            frontier: Frontier point in world coordinates
            costmap: Costmap to check for obstacles

        Returns:
            Minimum distance to nearest obstacle in meters
        """
        # Convert frontier to grid coordinates
        grid_pos = costmap.world_to_grid(frontier)
        grid_x, grid_y = int(grid_pos.x), int(grid_pos.y)

        # Check if frontier is within costmap bounds
        if grid_x < 0 or grid_x >= costmap.width or grid_y < 0 or grid_y >= costmap.height:
            return 0.0  # Consider out-of-bounds as obstacle

        min_distance = float("inf")
        search_radius = (
            int(self.config.safe_distance / costmap.resolution) + 5
        )  # Search a bit beyond minimum

        # Search in a square around the frontier point
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy

                # Skip if out of bounds
                if (
                    check_x < 0
                    or check_x >= costmap.width
                    or check_y < 0
                    or check_y >= costmap.height
                ):
                    continue

                # Check if this cell is an obstacle
                if costmap.grid[check_y, check_x] >= self.config.occupancy_threshold:
                    # Calculate distance in meters
                    distance = np.sqrt(dx**2 + dy**2) * costmap.resolution
                    min_distance = min(min_distance, distance)

        # If no obstacles found within search radius, return the safe distance
        # This indicates the frontier is safely away from obstacles
        return min_distance if min_distance != float("inf") else self.config.safe_distance

    def _compute_comprehensive_frontier_score(
        self, frontier: Vector3, frontier_size: int, robot_pose: Vector3, costmap: OccupancyGrid
    ) -> float:
        """Compute comprehensive score considering multiple criteria."""

        # 1. Distance from robot (preference for moderate distances)
        robot_distance = get_distance(frontier, robot_pose)

        # Distance score: prefer moderate distances (not too close, not too far)
        # Normalized to 0-1 range
        distance_score = 1.0 / (1.0 + abs(robot_distance - self.config.lookahead_distance))

        # 2. Information gain (frontier size)
        # Normalize by a reasonable max frontier size
        max_expected_frontier_size = self.config.min_frontier_perimeter / costmap.resolution * 10
        info_gain_score = min(frontier_size / max_expected_frontier_size, 1.0)

        # 3. Distance to explored goals (bonus for being far from explored areas)
        # Normalize by a reasonable max distance (e.g., 10 meters)
        explored_goals_distance = self._compute_distance_to_explored_goals(frontier)
        explored_goals_score = min(explored_goals_distance / self.config.max_explored_distance, 1.0)

        # 4. Distance to obstacles (score based on safety)
        # 0 = too close to obstacles, 1 = at or beyond safe distance
        obstacles_distance = self._compute_distance_to_obstacles(frontier, costmap)
        if obstacles_distance >= self.config.safe_distance:
            obstacles_score = 1.0  # Fully safe
        else:
            obstacles_score = obstacles_distance / self.config.safe_distance  # Linear penalty

        # 5. Direction momentum (already in 0-1 range from dot product)
        momentum_score = self._compute_direction_momentum_score(frontier, robot_pose)

        logger.debug(
            f"Distance score: {distance_score:.2f}, Info gain: {info_gain_score:.2f}, Explored goals: {explored_goals_score:.2f}, Obstacles: {obstacles_score:.2f}, Momentum: {momentum_score:.2f}"
        )

        # Combine scores with consistent scaling
        total_score = (
            0.3 * info_gain_score  # 30% information gain
            + 0.3 * explored_goals_score  # 30% distance from explored goals
            + 0.2 * distance_score  # 20% distance optimization
            + 0.15 * obstacles_score  # 15% distance from obstacles
            + 0.05 * momentum_score  # 5% direction momentum
        )

        return total_score

    def _rank_frontiers(
        self,
        frontier_centroids: list[Vector3],
        frontier_sizes: list[int],
        robot_pose: Vector3,
        costmap: OccupancyGrid,
    ) -> list[Vector3]:
        """
        Find the single best frontier using comprehensive scoring and filtering.

        Args:
            frontier_centroids: List of frontier centroids
            frontier_sizes: List of frontier sizes
            robot_pose: Current robot position
            costmap: Costmap for additional analysis

        Returns:
            List containing single best frontier, or empty list if none suitable
        """
        if not frontier_centroids:
            return []

        valid_frontiers = []

        for i, frontier in enumerate(frontier_centroids):
            # Compute comprehensive score
            frontier_size = frontier_sizes[i] if i < len(frontier_sizes) else 1
            score = self._compute_comprehensive_frontier_score(
                frontier, frontier_size, robot_pose, costmap
            )

            valid_frontiers.append((frontier, score))

        logger.debug(f"Valid frontiers: {len(valid_frontiers)}")

        if not valid_frontiers:
            return []

        # Sort by score and return all valid frontiers (highest scores first)
        valid_frontiers.sort(key=lambda x: x[1], reverse=True)

        # Extract just the frontiers (remove scores) and return as list
        return [frontier for frontier, _ in valid_frontiers]

    def mark_explored_goal(self, goal: Vector3) -> None:
        """Mark a goal as explored."""
        self.explored_goals.append(goal)
        self._visited.append((goal, time.monotonic() + self.config.visited_exclusion_ttl))

    @rpc
    def mark_failed_goal(self, goal: Vector3) -> None:
        """Cool off a frontier the robot could not reach.

        Without this, a frontier the navigator cannot route to keeps winning the
        selection every cycle, and the robot walks the same ground indefinitely.

        Args:
            goal: The frontier that timed out or was reported unreachable.
        """
        deadline = time.monotonic() + self.config.failure_cooldown
        self._failed.append((goal, deadline))
        logger.info(
            "Cooling off unreachable frontier",
            cooldown_s=self.config.failure_cooldown,
            x=round(goal.x, 2),
            y=round(goal.y, 2),
        )

    def _active_failures(self) -> list[tuple[Vector3, float]]:
        """Cooled-off frontiers that have not expired yet."""
        now = time.monotonic()
        self._failed = [entry for entry in self._failed if entry[1] > now]
        return self._failed

    def select_goal(self, pose: Vector3, grid: OccupancyGrid) -> Vector3 | None:
        """Choose the cheapest *reachable* frontier.

        Scoring frontiers by straight-line distance keeps sending the robot at walls:
        a frontier on the far side of a closed door is close, so it wins, the
        navigator cannot route to it, the goal times out, and next cycle the same
        frontier wins again. This ranks by travel cost over the costmap, so frontiers
        the robot cannot get to are never candidates, and cooled-off targets are
        blocked out while they are fresh.

        Args:
            pose: Robot position in world coordinates.
            grid: Inflated costmap to plan over.

        Returns:
            The frontier to walk to, or None when nothing new is reachable.
        """
        failures = self._active_failures()
        goal = self._select_frontier(pose, grid, failures, block_failures=True)
        if goal is None and failures:
            # Cooling off unreachable targets can wall the robot in: it stopped right
            # where it gave up, and the blocking disc can sit in its only doorway.
            # Release the blocking so it can drive away. The frontier exclusions still
            # apply, so it will not simply walk back into that spot.
            logger.warning("No frontier around the cooled-off goals; releasing the area blocking")
            goal = self._select_frontier(pose, grid, failures, block_failures=False)
        return goal

    def _select_frontier(
        self,
        pose: Vector3,
        grid: OccupancyGrid,
        failures: list[tuple[Vector3, float]],
        *,
        block_failures: bool,
    ) -> Vector3 | None:
        """Rank reachable frontiers once; see :meth:`select_goal` for why this runs twice."""

        known = grid
        if failures and block_failures:
            grid = grid.copy()
            yy, xx = np.indices(grid.grid.shape)
            for previous, _ in failures:
                cell = grid.world_to_grid(previous)
                limit = (0.75 / grid.resolution) ** 2
                grid.grid[(xx - cell.x) ** 2 + (yy - cell.y) ** 2 < limit] = CostValues.OCCUPIED

        distances = reachable_distances(grid, pose)
        if not np.any(np.isfinite(distances)):
            # The robot normally stands inside the inflation buffer of an obstacle it
            # is passing. Opening a small hole around it keeps exploration going
            # instead of reading the whole map as unreachable.
            grid = _open_start(grid, pose)
            distances = reachable_distances(grid, pose)
        if not np.any(np.isfinite(distances)) and failures:
            # Cooling off unreachable targets can wall the robot in, since it stopped
            # right where it gave up. Release the blocking so it can drive away; the
            # goal exclusions still keep it from walking back into that spot.
            logger.warning("Cooled-off goals enclose the robot; releasing the area blocking")
            grid = known
            distances = reachable_distances(grid, pose)
        if not np.any(np.isfinite(distances)):
            return None

        frontier, band = self._frontier_mask(distances, grid)
        if not frontier.any():
            return None

        # Exclusions apply to the points worth driving to, not to the map. Carving them
        # out of the map also carves out the corridor that leads onward, and the explorer
        # then reports the building finished with most of it unmapped.
        excluded = self._excluded_cells(pose, failures, grid)

        clusters, count = label(frontier)
        min_cells = max(1, int(self.config.min_frontier_perimeter / grid.resolution))
        groups = [
            (rows, cols)
            for rows, cols in (np.where(clusters == cid) for cid in range(1, count + 1))
            if len(rows)
        ]
        kept = [group for group in groups if len(group[0]) >= min_cells]
        if not kept and groups:
            # The size floor exists to reject sensor noise. Once the easy ground is
            # mapped, what is left is often reachable only through a door thin enough to
            # leave a frontier under that floor; rejecting those ends the run early.
            logger.info(
                "Only frontiers below the size floor remain; keeping the largest",
                cells=max(len(rows) for rows, _ in groups),
                floor=min_cells,
            )
            kept = [max(groups, key=lambda group: len(group[0]))]

        goals: list[Vector3] = []
        costs: list[float] = []
        sizes: list[int] = []
        for rows, cols in kept:
            keep = ~excluded[rows, cols]
            open_rows, open_cols = rows[keep], cols[keep]
            if not len(open_rows):
                continue
            # Cheapest cell to walk to inside the cluster, not a centroid that can land
            # in unknown space or behind the very obstacle being routed around.
            index = int(np.argmin(distances[open_rows, open_cols]))
            row, col = int(open_rows[index]), int(open_cols[index])
            world = grid.grid_to_world((col, row, 0))
            goals.append(Vector3(world.x, world.y, 0.0))
            costs.append(float(distances[row, col]))
            sizes.append(len(open_rows))

        if not goals:
            return None

        # Information gain per metre walked: a wide opening a few metres away beats a
        # crack in a wall at our feet, but not by much.
        best = min(range(len(goals)), key=lambda i: -(math.sqrt(sizes[i]) / (costs[i] + 0.5)))
        self._last_goal_cost = costs[best]
        logger.info(
            f"Frontier selected at ({goals[best].x:.2f}, {goals[best].y:.2f})",
            path_cost_m=round(self._last_goal_cost, 2),
            candidates=len(goals),
            cooling_targets=len(failures),
            frontier_band_cells=band,
        )
        return goals[best]

    def _excluded_cells(
        self, pose: Vector3, failures: list[tuple[Vector3, float]], grid: OccupancyGrid
    ) -> np.ndarray[Any, Any]:
        """Cells that should not be aimed at again yet: reached goals and cooling ones."""

        discs = [
            *((spot, self.config.visited_exclusion_radius) for spot, _ in self._active_visited()),
            *((spot, 0.75) for spot, _ in failures),
        ]
        excluded = np.zeros(grid.grid.shape, dtype=bool)
        if not discs:
            return excluded
        yy, xx = np.indices(grid.grid.shape)
        for spot, radius in discs:
            cell = grid.world_to_grid(spot)
            reach = radius / grid.resolution
            excluded |= (xx - cell.x) ** 2 + (yy - cell.y) ** 2 < reach**2
        return excluded

    def _active_visited(self) -> list[tuple[Vector3, float]]:
        """Reached goals still recent enough to count as repeats."""

        now = time.monotonic()
        self._visited = [entry for entry in self._visited if entry[1] > now]
        return self._visited

    def _frontier_mask(
        self, distances: np.ndarray[Any, Any], grid: OccupancyGrid
    ) -> tuple[np.ndarray[Any, Any], int]:
        """Frontier cells: drivable cells from which unknown space can be reached.

        The map is inflated for planning, and inflation turns the unknown belt hugging a
        wall into wall too, so the nearest remaining unknown is at least twice the
        inflation plus the wall's own thickness away from any drivable cell. Requiring a
        frontier to touch unknown therefore finds nothing in a corridor that is barely
        wide enough for the robot, and the explorer reports the building finished after
        a third of it. The band grows until it reaches a drivable cell instead of being
        a constant that a different corridor width breaks.
        """
        unknown = grid.grid == CostValues.UNKNOWN
        drivable = np.isfinite(distances)
        widest = max(2, math.ceil((2 * self.config.costmap_inflation + 0.2) / grid.resolution))
        for band in range(1, widest + 1):
            near_unknown = binary_dilation(
                unknown, structure=np.ones((3, 3), dtype=bool), iterations=band
            )
            frontier = drivable & near_unknown
            if frontier.any():
                return frontier, band
        return frontier, widest

    def reset_exploration_session(self) -> None:
        """
        Reset all exploration state variables for a new exploration session.

        Call this method when starting a new exploration or when the robot
        needs to forget its previous exploration history.
        """
        self.explored_goals.clear()  # Clear all previously explored goals
        self._visited.clear()  # Forget which goals count as repeats
        self.exploration_direction = Vector3(0.0, 0.0, 0.0)  # Reset exploration direction
        self.last_costmap = None  # Clear last costmap comparison
        self.no_gain_counter = 0  # Reset no-gain attempt counter
        self._failed.clear()  # Forget cooled-off targets
        self._cache.clear()  # Clear frontier point cache

        logger.info("Exploration session reset - all state variables cleared")

    @rpc
    def explore(self) -> bool:
        """
        Start autonomous frontier exploration.

        Returns:
            bool: True if exploration started, False if already exploring
        """
        if self.exploration_active:
            logger.warning("Exploration already active")
            return False

        self.exploration_active = True
        self.stop_event.clear()

        # Start exploration thread
        self.exploration_thread = threading.Thread(target=self._exploration_loop, daemon=True)
        self.exploration_thread.start()

        logger.info("Started autonomous frontier exploration")
        return True

    @rpc
    def stop_exploration(self) -> bool:
        """
        Stop autonomous frontier exploration.

        Returns:
            bool: True if exploration was stopped, False if not exploring
        """
        if not self.exploration_active:
            return False

        self.exploration_active = False
        self.no_gain_counter = 0  # Reset counter when exploration stops
        self.stop_event.set()

        # Only join if we're NOT being called from the exploration thread itself
        if (
            self.exploration_thread
            and self.exploration_thread.is_alive()
            and threading.current_thread() != self.exploration_thread
        ):
            self.exploration_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        # Publish current location as goal to stop the robot.
        if self.latest_odometry is not None:
            goal = PoseStamped(
                position=self.latest_odometry.position,
                orientation=self.latest_odometry.orientation,
                frame_id="world",
                ts=self.latest_odometry.ts,
            )
            self.goal_request.publish(goal)

        logger.info("Stopped autonomous frontier exploration")
        return True

    @rpc
    def is_exploration_active(self) -> bool:
        return self.exploration_active

    def _exploration_loop(self) -> None:
        """Run the exploration loop and release the movement hold on exit.

        Exploration can end several ways: explicit `end_exploration`, no-gain or
        consecutive-failure self-termination, or an unexpected error. Closing the
        tool-stream in `finally` releases the `movement` capability in every case
        (`end_exploration`'s own `stop_tool` is then a safe no-op).
        """
        try:
            self._run_exploration_loop()
        finally:
            self.stop_tool("begin_exploration")

    def _run_exploration_loop(self) -> None:
        """Main exploration loop running in separate thread."""
        # Track number of goals published
        goals_published = 0
        consecutive_failures = 0
        max_consecutive_failures = 10  # Allow more attempts before giving up

        while self.exploration_active and not self.stop_event.is_set():
            # Check if we have required data
            if self.latest_costmap is None or self.latest_odometry is None:
                threading.Event().wait(0.5)
                continue

            # Get robot pose from odometry
            robot_pose = Vector3(
                self.latest_odometry.position.x, self.latest_odometry.position.y, 0.0
            )

            # Get exploration goal
            costmap = simple_inflate(self.latest_costmap, self.config.costmap_inflation)
            goal = self.select_goal(robot_pose, costmap)

            if goal:
                # Publish goal to navigator
                goal_msg = PoseStamped()
                goal_msg.position.x = goal.x
                goal_msg.position.y = goal.y
                goal_msg.position.z = 0.0
                goal_msg.orientation.w = 1.0  # No rotation
                goal_msg.frame_id = "world"
                goal_msg.ts = self.latest_costmap.ts

                self.goal_request.publish(goal_msg)
                logger.info(f"Published frontier goal: ({goal.x:.2f}, {goal.y:.2f})")

                goals_published += 1
                consecutive_failures = 0  # Reset failure counter on success

                # Clear the goal reached event for next iteration
                self.goal_reached_event.clear()

                # The budget grows with the leg, so a frontier is never called failed
                # merely for being far away; a robot that stops making progress still
                # gives up on it and cools it off.
                budget = self.config.goal_timeout + self._last_goal_cost / max(
                    self.config.goal_speed, 0.05
                )
                logger.info(f"Waiting up to {budget:.0f}s for the goal to be reached...")
                goal_reached = self.goal_reached_event.wait(timeout=budget)

                if goal_reached:
                    logger.info("Goal reached, finding next frontier")
                    self.mark_explored_goal(goal)
                else:
                    logger.warning(
                        f"Goal not reached within {budget:.0f}s; cooling it off and replanning"
                    )
                    self.mark_failed_goal(goal)
            else:
                consecutive_failures += 1

                # Only give up if we've published at least 2 goals AND had many consecutive failures
                if goals_published >= 2 and consecutive_failures >= max_consecutive_failures:
                    logger.info(
                        f"Exploration complete after {goals_published} goals and {consecutive_failures} consecutive failures finding new frontiers"
                    )
                    self.exploration_active = False
                    break
                elif goals_published < 2:
                    logger.info(
                        f"No frontier found, but only {goals_published} goals published so far. Retrying in 2 seconds..."
                    )
                    threading.Event().wait(2.0)
                else:
                    logger.info(
                        f"No frontier found (attempt {consecutive_failures}/{max_consecutive_failures}). Retrying in 2 seconds..."
                    )
                    threading.Event().wait(2.0)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def begin_exploration(self) -> str:
        """Command the robot to move around and explore the area. Cancelled with end_exploration."""
        # Open (or re-stamp, on a same-tool takeover) the tool-stream before the
        # loop starts, so the movement hold is always carried by a live stream
        # and gets released whether exploration ends via `end_exploration` or
        # self-terminates (see `_exploration_loop`'s finally). Opening it first
        # also avoids a race where a fast-failing loop could `stop_tool` before
        # the stream exists.
        self.start_tool("begin_exploration")
        started = self.explore()
        if not started:
            return "Exploration skill is already active. Use end_exploration to stop before starting again."
        return "Started exploration skill. The robot is now moving. Use end_exploration to stop."

    @skill
    def end_exploration(self) -> str:
        """Cancel the exploration. The robot will stop moving and remain where it is."""
        stopped = self.stop_exploration()
        self.stop_tool("begin_exploration")
        if stopped:
            return "Stopped exploration. The robot has stopped moving."
        else:
            return "Exploration skill was not active, so nothing was stopped."
