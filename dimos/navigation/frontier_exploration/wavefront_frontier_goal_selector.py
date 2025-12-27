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

"""
Simple wavefront frontier exploration algorithm implementation using dimos types.

This module provides frontier detection and exploration goal selection
for autonomous navigation using the dimos Costmap and Vector types.
"""

from collections import deque
from dataclasses import dataclass
from enum import IntFlag
import threading

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid
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

    # LCM inputs
    global_costmap: In[OccupancyGrid]
    odom: In[PoseStamped]
    goal_reached: In[Bool]
    explore_cmd: In[Bool]
    stop_explore_cmd: In[Bool]

    # LCM outputs
    goal_request: Out[PoseStamped]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        min_frontier_perimeter: float = 0.5,
        occupancy_threshold: int = 99,
        safe_distance: float = 3.0,
        lookahead_distance: float = 5.0,
        max_explored_distance: float = 10.0,
        info_gain_threshold: float = 0.03,
        num_no_gain_attempts: int = 2,
        goal_timeout: float = 15.0,
        **kwargs,
    ) -> None:
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
        self.min_frontier_perimeter = min_frontier_perimeter
        self.occupancy_threshold = occupancy_threshold
        self.safe_distance = safe_distance
        self.max_explored_distance = max_explored_distance
        self.lookahead_distance = lookahead_distance
        self.info_gain_threshold = info_gain_threshold
        self.num_no_gain_attempts = num_no_gain_attempts
        self._cache = FrontierCache()
        self.explored_goals = []  # type: ignore[var-annotated]  # list of explored goals
        self.exploration_direction = Vector3(0.0, 0.0, 0.0)  # current exploration direction
        self.last_costmap = None  # store last costmap for information comparison
        self.no_gain_counter = 0  # track consecutive no-gain attempts
        self.goal_timeout = goal_timeout

        # Latest data
        self.latest_costmap: OccupancyGrid | None = None
        self.latest_odometry: PoseStamped | None = None

        # Goal reached event
        self.goal_reached_event = threading.Event()

        # Exploration state
        self.exploration_active = False
        self.exploration_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        logger.info("WavefrontFrontierExplorer module initialized")

    @rpc
    def start(self) -> None:
        super().start()

        unsub = self.global_costmap.subscribe(self._on_costmap)
        self._disposables.add(Disposable(unsub))

        unsub = self.odom.subscribe(self._on_odometry)
        self._disposables.add(Disposable(unsub))

        if self.goal_reached.transport is not None:
            unsub = self.goal_reached.subscribe(self._on_goal_reached)
            self._disposables.add(Disposable(unsub))

        if self.explore_cmd.transport is not None:
            unsub = self.explore_cmd.subscribe(self._on_explore_cmd)
            self._disposables.add(Disposable(unsub))

        if self.stop_explore_cmd.transport is not None:
            unsub = self.stop_explore_cmd.subscribe(self._on_stop_explore_cmd)
            self._disposables.add(Disposable(unsub))

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

    def _count_costmap_information(self, costmap: OccupancyGrid) -> int:
        """
        Count the amount of information in a costmap (free space + obstacles).

        Args:
            costmap: Costmap to analyze

        Returns:
            Number of cells that are free space or obstacles (not unknown)
        """
        free_count = np.sum(costmap.grid == CostValues.FREE)
        obstacle_count = np.sum(costmap.grid >= self.occupancy_threshold)
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
            if neighbor_cost > self.occupancy_threshold:
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
                min_cells = int(self.min_frontier_perimeter / costmap.resolution)
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
            int(self.safe_distance / costmap.resolution) + 5
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
                if costmap.grid[check_y, check_x] >= self.occupancy_threshold:
                    # Calculate distance in meters
                    distance = np.sqrt(dx**2 + dy**2) * costmap.resolution
                    min_distance = min(min_distance, distance)

        # If no obstacles found within search radius, return the safe distance
        # This indicates the frontier is safely away from obstacles
        return min_distance if min_distance != float("inf") else self.safe_distance

    def _compute_comprehensive_frontier_score(
        self, frontier: Vector3, frontier_size: int, robot_pose: Vector3, costmap: OccupancyGrid
    ) -> float:
        """Compute comprehensive score considering multiple criteria."""

        # 1. Distance from robot (preference for moderate distances)
        robot_distance = get_distance(frontier, robot_pose)

        # Distance score: prefer moderate distances (not too close, not too far)
        # Normalized to 0-1 range
        distance_score = 1.0 / (1.0 + abs(robot_distance - self.lookahead_distance))

        # 2. Information gain (frontier size)
        # Normalize by a reasonable max frontier size
        max_expected_frontier_size = self.min_frontier_perimeter / costmap.resolution * 10
        info_gain_score = min(frontier_size / max_expected_frontier_size, 1.0)

        # 3. Distance to explored goals (bonus for being far from explored areas)
        # Normalize by a reasonable max distance (e.g., 10 meters)
        explored_goals_distance = self._compute_distance_to_explored_goals(frontier)
        explored_goals_score = min(explored_goals_distance / self.max_explored_distance, 1.0)

        # 4. Distance to obstacles (score based on safety)
        # 0 = too close to obstacles, 1 = at or beyond safe distance
        obstacles_distance = self._compute_distance_to_obstacles(frontier, costmap)
        if obstacles_distance >= self.safe_distance:
            obstacles_score = 1.0  # Fully safe
        else:
            obstacles_score = obstacles_distance / self.safe_distance  # Linear penalty

        # 5. Direction momentum (already in 0-1 range from dot product)
        momentum_score = self._compute_direction_momentum_score(frontier, robot_pose)

        logger.info(
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

        logger.info(f"Valid frontiers: {len(valid_frontiers)}")

        if not valid_frontiers:
            return []

        # Sort by score and return all valid frontiers (highest scores first)
        valid_frontiers.sort(key=lambda x: x[1], reverse=True)

        # Extract just the frontiers (remove scores) and return as list
        return [frontier for frontier, _ in valid_frontiers]

    def get_exploration_goal(self, robot_pose: Vector3, costmap: OccupancyGrid) -> Vector3 | None:
        """
        Get the single best exploration goal using comprehensive frontier scoring.

        Args:
            robot_pose: Current robot position in world coordinates
            costmap: Costmap for additional analysis

        Returns:
            Single best frontier goal in world coordinates, or None if no suitable frontiers found
        """
        # Check if we should compare costmaps for information gain
        if len(self.explored_goals) > 5 and self.last_costmap is not None:
            current_info = self._count_costmap_information(costmap)
            last_info = self._count_costmap_information(self.last_costmap)

            # Check if information increase meets minimum percentage threshold
            if last_info > 0:  # Avoid division by zero
                info_increase_percent = (current_info - last_info) / last_info
                if info_increase_percent < self.info_gain_threshold:
                    logger.info(
                        f"Information increase ({info_increase_percent:.2f}) below threshold ({self.info_gain_threshold:.2f})"
                    )
                    logger.info(
                        f"Current information: {current_info}, Last information: {last_info}"
                    )
                    self.no_gain_counter += 1
                    if self.no_gain_counter >= self.num_no_gain_attempts:
                        logger.info(
                            f"No information gain for {self.no_gain_counter} consecutive attempts"
                        )
                        self.no_gain_counter = 0  # Reset counter when stopping due to no gain
                        self.stop_exploration()
                        return None
                else:
                    self.no_gain_counter = 0

        # Always detect new frontiers to get most up-to-date information
        # The new algorithm filters out explored areas and returns only the best frontier
        frontiers = self.detect_frontiers(robot_pose, costmap)

        if not frontiers:
            # Store current costmap before returning
            self.last_costmap = costmap  # type: ignore[assignment]
            self.reset_exploration_session()
            return None

        # Update exploration direction based on best goal selection
        if frontiers:
            self._update_exploration_direction(robot_pose, frontiers[0])

            # Store the selected goal as explored
            selected_goal = frontiers[0]
            self.mark_explored_goal(selected_goal)

            # Store current costmap for next comparison
            self.last_costmap = costmap  # type: ignore[assignment]

            return selected_goal

        # Store current costmap before returning
        self.last_costmap = costmap  # type: ignore[assignment]
        return None

    def mark_explored_goal(self, goal: Vector3) -> None:
        """Mark a goal as explored."""
        self.explored_goals.append(goal)

    def reset_exploration_session(self) -> None:
        """
        Reset all exploration state variables for a new exploration session.

        Call this method when starting a new exploration or when the robot
        needs to forget its previous exploration history.
        """
        self.explored_goals.clear()  # Clear all previously explored goals
        self.exploration_direction = Vector3(0.0, 0.0, 0.0)  # Reset exploration direction
        self.last_costmap = None  # Clear last costmap comparison
        self.no_gain_counter = 0  # Reset no-gain attempt counter
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
            self.exploration_thread.join(timeout=2.0)

        logger.info("Stopped autonomous frontier exploration")
        return True

    @rpc
    def is_exploration_active(self) -> bool:
        return self.exploration_active

    def _exploration_loop(self) -> None:
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
            costmap = simple_inflate(self.latest_costmap, 0.25)
            goal = self.get_exploration_goal(robot_pose, costmap)

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

                # Wait for goal to be reached or timeout
                logger.info("Waiting for goal to be reached...")
                goal_reached = self.goal_reached_event.wait(timeout=self.goal_timeout)

                if goal_reached:
                    logger.info("Goal reached, finding next frontier")
                else:
                    logger.warning("Goal timeout after 30 seconds, finding next frontier anyway")
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


wavefront_frontier_explorer = WavefrontFrontierExplorer.blueprint

__all__ = ["WavefrontFrontierExplorer", "wavefront_frontier_explorer"]
