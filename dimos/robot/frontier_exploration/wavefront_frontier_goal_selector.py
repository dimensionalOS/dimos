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

from typing import List, Tuple, Optional, Callable
from collections import deque
import numpy as np
from dataclasses import dataclass
from enum import IntFlag
import threading
from dimos.utils.logging_config import setup_logger

from dimos.types.costmap import Costmap, CostValues
from dimos.types.vector import Vector
from dimos.robot.frontier_exploration.utils import smooth_costmap_for_frontiers

logger = setup_logger("dimos.robot.unitree.frontier_exploration")


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

    def __init__(self):
        self.points = {}

    def get_point(self, x: int, y: int) -> GridPoint:
        """Get or create a grid point at the given coordinates."""
        key = (x, y)
        if key not in self.points:
            self.points[key] = GridPoint(x, y)
        return self.points[key]

    def clear(self):
        """Clear the point cache."""
        self.points.clear()


class WavefrontFrontierExplorer:
    """
    Wavefront frontier exploration algorithm implementation.

    This class encapsulates the frontier detection and exploration goal selection
    functionality using the wavefront algorithm with BFS exploration.
    """

    def __init__(
        self,
        min_frontier_size: int = 10,
        occupancy_threshold: int = 65,
        subsample_resolution: int = 3,
        min_distance_from_robot: float = 0.5,
        explored_area_buffer: float = 0.5,
        min_distance_from_obstacles: float = 0.6,
        info_gain_threshold: float = 0.03,
        num_no_gain_attempts: int = 4,
        set_goal: Optional[Callable] = None,
        get_costmap: Optional[Callable] = None,
        get_robot_pos: Optional[Callable] = None,
    ):
        """
        Initialize the frontier explorer.

        Args:
            min_frontier_size: Minimum number of points to consider a valid frontier
            occupancy_threshold: Cost threshold above which a cell is considered occupied (0-255)
            subsample_resolution: Factor by which to subsample the costmap for faster processing (1=no subsampling, 2=half resolution, 4=quarter resolution)
            min_distance_from_robot: Minimum distance frontier must be from robot (meters)
            explored_area_buffer: Buffer distance around free areas to consider as explored (meters)
            min_distance_from_obstacles: Minimum distance frontier must be from obstacles (meters)
            info_gain_threshold: Minimum percentage increase in costmap information required to continue exploration (0.05 = 5%)
            num_no_gain_attempts: Maximum number of consecutive attempts with no information gain
            set_goal: Callable to set navigation goal, signature: (goal: Vector, stop_event: Optional[threading.Event]) -> bool
            get_costmap: Callable to get current costmap, signature: () -> Costmap
            get_robot_pos: Callable to get current robot position, signature: () -> Vector
        """
        self.min_frontier_size = min_frontier_size
        self.occupancy_threshold = occupancy_threshold
        self.subsample_resolution = subsample_resolution
        self.min_distance_from_robot = min_distance_from_robot
        self.explored_area_buffer = explored_area_buffer
        self.min_distance_from_obstacles = min_distance_from_obstacles
        self.info_gain_threshold = info_gain_threshold
        self.num_no_gain_attempts = num_no_gain_attempts
        self.set_goal = set_goal
        self.get_costmap = get_costmap
        self.get_robot_pos = get_robot_pos
        self._cache = FrontierCache()
        self.explored_goals = []  # list of explored goals
        self.exploration_direction = Vector([0.0, 0.0])  # current exploration direction
        self.last_costmap = None  # store last costmap for information comparison

    def _count_costmap_information(self, costmap: Costmap) -> int:
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

    def _get_neighbors(self, point: GridPoint, costmap: Costmap) -> List[GridPoint]:
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

    def _is_frontier_point(self, point: GridPoint, costmap: Costmap) -> bool:
        """
        Check if a point is a frontier point.
        A frontier point is an unknown cell adjacent to at least one free cell
        and not adjacent to any occupied cells.
        """
        # Point must be unknown
        world_pos = costmap.grid_to_world(Vector([float(point.x), float(point.y)]))
        cost = costmap.get_value(world_pos)
        if cost != CostValues.UNKNOWN:
            return False

        has_free = False

        for neighbor in self._get_neighbors(point, costmap):
            neighbor_world = costmap.grid_to_world(Vector([float(neighbor.x), float(neighbor.y)]))
            neighbor_cost = costmap.get_value(neighbor_world)

            # If adjacent to occupied space, not a frontier
            if neighbor_cost and neighbor_cost > self.occupancy_threshold:
                return False

            # Check if adjacent to free space
            if neighbor_cost == CostValues.FREE:
                has_free = True

        return has_free

    def _find_free_space(self, start_x: int, start_y: int, costmap: Costmap) -> Tuple[int, int]:
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
            world_pos = costmap.grid_to_world(Vector([float(point.x), float(point.y)]))
            if costmap.get_value(world_pos) == CostValues.FREE:
                return (point.x, point.y)

            # Add neighbors to search
            for neighbor in self._get_neighbors(point, costmap):
                if (neighbor.x, neighbor.y) not in visited:
                    queue.append(neighbor)

        # If no free space found, return original position
        return (start_x, start_y)

    def _compute_centroid(self, frontier_points: List[Vector]) -> Vector:
        """Compute the centroid of a list of frontier points."""
        if not frontier_points:
            return Vector([0.0, 0.0])

        # Vectorized approach using numpy
        points_array = np.array([[point.x, point.y] for point in frontier_points])
        centroid = np.mean(points_array, axis=0)

        return Vector([centroid[0], centroid[1]])

    def detect_frontiers(self, robot_pose: Vector, costmap: Costmap) -> List[Vector]:
        """
        Main frontier detection algorithm using wavefront exploration.

        Args:
            robot_pose: Current robot position in world coordinates (Vector with x, y)
            costmap: Costmap for additional analysis

        Returns:
            List of frontier centroids in world coordinates
        """
        self._cache.clear()

        # Apply filtered costmap (now default)
        working_costmap = smooth_costmap_for_frontiers(costmap)

        # Subsample the costmap for faster processing
        if self.subsample_resolution > 1:
            subsampled_costmap = working_costmap.subsample(self.subsample_resolution)
        else:
            subsampled_costmap = working_costmap

        # Convert robot pose to subsampled grid coordinates
        subsampled_grid_pos = subsampled_costmap.world_to_grid(robot_pose)
        grid_x, grid_y = int(subsampled_grid_pos.x), int(subsampled_grid_pos.y)

        # Find nearest free space to start exploration
        free_x, free_y = self._find_free_space(grid_x, grid_y, subsampled_costmap)
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
            if self._is_frontier_point(current_point, subsampled_costmap):
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
                    if self._is_frontier_point(frontier_point, subsampled_costmap):
                        new_frontier.append(frontier_point)

                        # Add neighbors to frontier queue
                        for neighbor in self._get_neighbors(frontier_point, subsampled_costmap):
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
                if len(new_frontier) >= self.min_frontier_size:
                    world_points = []
                    for point in new_frontier:
                        world_pos = subsampled_costmap.grid_to_world(
                            Vector([float(point.x), float(point.y)])
                        )
                        world_points.append(world_pos)

                    # Compute centroid in world coordinates (already correctly scaled)
                    centroid = self._compute_centroid(world_points)
                    frontiers.append(centroid)  # Store centroid
                    frontier_sizes.append(len(new_frontier))  # Store frontier size

            # Add ALL neighbors to main exploration queue to explore entire free space
            for neighbor in self._get_neighbors(current_point, subsampled_costmap):
                if not (
                    neighbor.classification
                    & (PointClassification.MapOpen | PointClassification.MapClosed)
                ):
                    # Check if neighbor is free space or unknown (explorable)
                    neighbor_world = subsampled_costmap.grid_to_world(
                        Vector([float(neighbor.x), float(neighbor.y)])
                    )
                    neighbor_cost = subsampled_costmap.get_value(neighbor_world)

                    # Add free space and unknown space to exploration queue
                    if neighbor_cost is not None and (
                        neighbor_cost == CostValues.FREE or neighbor_cost == CostValues.UNKNOWN
                    ):
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

    def _update_exploration_direction(self, robot_pose: Vector, goal_pose: Optional[Vector] = None):
        """Update the current exploration direction based on robot movement or selected goal."""
        if goal_pose is not None:
            # Calculate direction from robot to goal
            direction = Vector([goal_pose.x - robot_pose.x, goal_pose.y - robot_pose.y])
            magnitude = np.sqrt(direction.x**2 + direction.y**2)
            if magnitude > 0.1:  # Avoid division by zero for very close goals
                self.exploration_direction = Vector(
                    [direction.x / magnitude, direction.y / magnitude]
                )

    def _compute_direction_momentum_score(self, frontier: Vector, robot_pose: Vector) -> float:
        """Compute direction momentum score for a frontier."""
        if self.exploration_direction.x == 0 and self.exploration_direction.y == 0:
            return 0.0  # No momentum if no previous direction

        # Calculate direction from robot to frontier
        frontier_direction = Vector([frontier.x - robot_pose.x, frontier.y - robot_pose.y])
        magnitude = np.sqrt(frontier_direction.x**2 + frontier_direction.y**2)

        if magnitude < 0.1:
            return 0.0  # Too close to calculate meaningful direction

        # Normalize frontier direction
        frontier_direction = Vector(
            [frontier_direction.x / magnitude, frontier_direction.y / magnitude]
        )

        # Calculate dot product for directional alignment
        dot_product = (
            self.exploration_direction.x * frontier_direction.x
            + self.exploration_direction.y * frontier_direction.y
        )

        # Return momentum score (higher for same direction, lower for opposite)
        return max(0.0, dot_product)  # Only positive momentum, no penalty for different directions

    def _compute_distance_to_explored_goals(self, frontier: Vector) -> float:
        """Compute distance from frontier to the nearest explored goal."""
        if not self.explored_goals:
            return 5.0  # Default consistent value when no explored goals
        # Calculate distance to nearest explored goal
        min_distance = float("inf")
        for goal in self.explored_goals:
            distance = np.sqrt((frontier.x - goal.x) ** 2 + (frontier.y - goal.y) ** 2)
            min_distance = min(min_distance, distance)

        return min_distance

    def _compute_distance_to_obstacles(self, frontier: Vector, costmap: Costmap) -> float:
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
            int(self.min_distance_from_obstacles / costmap.resolution) + 5
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

        return min_distance if min_distance != float("inf") else float("inf")

    def _compute_comprehensive_frontier_score(
        self, frontier: Vector, frontier_size: int, robot_pose: Vector, costmap: Costmap
    ) -> float:
        """Compute comprehensive score considering multiple criteria."""

        # 1. Distance from robot (preference for moderate distances)
        robot_distance = np.sqrt(
            (frontier.x - robot_pose.x) ** 2 + (frontier.y - robot_pose.y) ** 2
        )

        # Distance score: prefer moderate distances (not too close, not too far)
        optimal_distance = 4.0  # meters
        distance_score = 1.0 / (1.0 + abs(robot_distance - optimal_distance))

        # 2. Information gain (frontier size)
        info_gain_score = frontier_size

        # 3. Distance to explored goals (bonus for being far from explored areas)
        explored_goals_distance = self._compute_distance_to_explored_goals(frontier)
        explored_goals_score = explored_goals_distance

        # 4. Distance to obstacles (penalty for being too close)
        obstacles_distance = self._compute_distance_to_obstacles(frontier, costmap)
        obstacles_score = obstacles_distance

        # 5. Direction momentum (if we have a current direction)
        momentum_score = self._compute_direction_momentum_score(frontier, robot_pose)

        # Combine scores with consistent scaling (no arbitrary multipliers)
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
        frontier_centroids: List[Vector],
        frontier_sizes: List[int],
        robot_pose: Vector,
        costmap: Costmap,
    ) -> List[Vector]:
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
            robot_distance = np.sqrt(
                (frontier.x - robot_pose.x) ** 2 + (frontier.y - robot_pose.y) ** 2
            )

            # Filter 1: Skip frontiers too close to robot
            if robot_distance < self.min_distance_from_robot:
                continue

            # Filter 2: Skip frontiers too close to obstacles
            obstacle_distance = self._compute_distance_to_obstacles(frontier, costmap)
            if obstacle_distance < self.min_distance_from_obstacles:
                continue

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

    def get_exploration_goal(self, robot_pose: Vector, costmap: Costmap) -> Optional[Vector]:
        """
        Get the single best exploration goal using comprehensive frontier scoring.

        Args:
            robot_pose: Current robot position in world coordinates (Vector with x, y)
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
                    self.num_no_gain_attempts += 1
                    if self.num_no_gain_attempts >= self.num_no_gain_attempts:
                        logger.info(
                            "No information gain for {} consecutive attempts, skipping frontier selection".format(
                                self.num_no_gain_attempts
                            )
                        )
                        self.reset_exploration_session()
                        return None

        # Always detect new frontiers to get most up-to-date information
        # The new algorithm filters out explored areas and returns only the best frontier
        frontiers = self.detect_frontiers(robot_pose, costmap)

        if not frontiers:
            # Store current costmap before returning
            self.last_costmap = costmap
            self.reset_exploration_session()
            return None

        # Update exploration direction based on best goal selection
        if frontiers:
            self._update_exploration_direction(robot_pose, frontiers[0])

            # Store the selected goal as explored
            selected_goal = frontiers[0]
            self.mark_explored_goal(selected_goal)

            # Store current costmap for next comparison
            self.last_costmap = costmap

            return selected_goal

        # Store current costmap before returning
        self.last_costmap = costmap
        return None

    def mark_explored_goal(self, goal: Vector):
        """Mark a goal as explored."""
        self.explored_goals.append(goal)

    def reset_exploration_session(self):
        """
        Reset all exploration state variables for a new exploration session.

        Call this method when starting a new exploration or when the robot
        needs to forget its previous exploration history.
        """
        self.explored_goals.clear()  # Clear all previously explored goals
        self.exploration_direction = Vector([0.0, 0.0])  # Reset exploration direction
        self.last_costmap = None  # Clear last costmap comparison
        self.num_no_gain_attempts = 0  # Reset no-gain attempt counter
        self._cache.clear()  # Clear frontier point cache

        logger.info("Exploration session reset - all state variables cleared")

    def explore(self, stop_event: Optional[threading.Event] = None) -> bool:
        """
        Perform autonomous frontier exploration by continuously finding and navigating to frontiers.

        Args:
            stop_event: Optional threading.Event to signal when exploration should stop

        Returns:
            bool: True if exploration completed successfully, False if stopped or failed
        """

        logger.info("Starting autonomous frontier exploration")

        while True:
            # Check if stop event is set
            if stop_event and stop_event.is_set():
                logger.info("Exploration stopped by stop event")
                return False

            # Get fresh robot position and costmap data
            robot_pose = self.get_robot_pos()
            costmap = self.get_costmap()

            # Get the next frontier goal
            next_goal = self.get_exploration_goal(robot_pose, costmap)
            if not next_goal:
                logger.info("No more frontiers found, exploration complete")
                return True

            # Navigate to the frontier
            logger.info(f"Navigating to frontier at {next_goal}")
            navigation_successful = self.set_goal(next_goal, stop_event=stop_event)

            if not navigation_successful:
                logger.warning("Failed to navigate to frontier, continuing exploration")
                # Continue to try other frontiers instead of stopping
                continue
