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

from typing import List, Tuple, Optional
from collections import deque
import numpy as np
from dataclasses import dataclass
import time
from enum import IntFlag

from dimos.types.costmap import Costmap, CostValues, smooth_costmap_for_frontiers
from dimos.types.vector import Vector

import os
import glob
import pickle
from dimos.robot.frontier_exploration.utils import costmap_to_pil_image, draw_frontiers_on_image


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


@dataclass
class PersistentFrontier:
    """Represents a persistent frontier with metadata."""

    centroid: Vector
    size: int
    last_seen: float  # timestamp
    attempts: int = 0  # number of exploration attempts
    accessible: bool = True  # whether frontier is reachable
    map_hash: int = 0  # hash of local map region for change detection


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
        min_frontier_size: int = 20,
        occupancy_threshold: int = 65,
        subsample_resolution: int = 3,
        min_distance_from_robot: float = 0.5,
        explored_area_buffer: float = 0.5,
        use_filtered_costmap: bool = True,
        costmap_save_dir: Optional[str] = None,
    ):
        """
        Initialize the frontier explorer.

        Args:
            min_frontier_size: Minimum number of points to consider a valid frontier
            occupancy_threshold: Cost threshold above which a cell is considered occupied (0-255)
            subsample_resolution: Factor by which to subsample the costmap for faster processing (1=no subsampling, 2=half resolution, 4=quarter resolution)
            min_distance_from_robot: Minimum distance frontier must be from robot (meters)
            explored_area_buffer: Buffer distance around free areas to consider as explored (meters)
            use_filtered_costmap: Whether to use a filtered costmap for exploration
            costmap_save_dir: Directory to save costmaps for debugging (optional)
        """
        self.min_frontier_size = min_frontier_size
        self.occupancy_threshold = occupancy_threshold
        self.subsample_resolution = subsample_resolution
        self.min_distance_from_robot = min_distance_from_robot
        self.explored_area_buffer = explored_area_buffer
        self.use_filtered_costmap = use_filtered_costmap
        self.costmap_save_dir = costmap_save_dir
        self._cache = FrontierCache()
        self.persistent_frontiers = {}  # map of frontier centroids to PersistentFrontier objects
        self.exploration_direction = Vector([0.0, 0.0])  # current exploration direction
        self._costmap_save_counter = 0  # Counter for costmap file naming

        # Create save directory if it doesn't exist
        if self.costmap_save_dir and not os.path.exists(self.costmap_save_dir):
            os.makedirs(self.costmap_save_dir)

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
            costmap: Occupancy grid costmap (dimos Costmap)

        Returns:
            List of frontier centroids in world coordinates
        """
        self._cache.clear()

        # Apply filtered costmap if enabled
        working_costmap = costmap
        if self.use_filtered_costmap:
            working_costmap = smooth_costmap_for_frontiers(
                costmap, alpha=4.0, free_space_dilation_size=2
            )
            print(f"DEBUG: Applied costmap filtering for frontier exploration")

        # Save costmap if directory is specified
        if self.costmap_save_dir:
            self._costmap_save_counter += 1
            # Save pickle file
            pickle_path = os.path.join(
                self.costmap_save_dir, f"{self._costmap_save_counter:04d}.pickle"
            )
            with open(pickle_path, "wb") as f:
                pickle.dump(working_costmap, f)

            # Save image file
            image_path = os.path.join(
                self.costmap_save_dir, f"{self._costmap_save_counter:04d}.jpg"
            )
            working_costmap.costmap_to_image(image_path)
            print(
                f"DEBUG: Saved filtered costmap #{self._costmap_save_counter} to {self.costmap_save_dir}"
            )

        # Subsample the costmap for faster processing
        if self.subsample_resolution > 1:
            subsampled_costmap = working_costmap.subsample(self.subsample_resolution)
            print(
                f"DEBUG: Original costmap - Width: {working_costmap.width}, Height: {working_costmap.height}"
            )
            print(
                f"DEBUG: Subsampled costmap - Width: {subsampled_costmap.width}, Height: {subsampled_costmap.height}, Subsample factor: {self.subsample_resolution}"
            )
        else:
            subsampled_costmap = working_costmap
            print(
                f"DEBUG: No subsampling - Width: {working_costmap.width}, Height: {working_costmap.height}"
            )

        # Debug: Print costmap statistics
        print(
            f"DEBUG: Costmap percentages - Occupied: {subsampled_costmap.occupied_percent:.1f}%, Free: {subsampled_costmap.free_percent:.1f}%, Unknown: {subsampled_costmap.unknown_percent:.1f}%"
        )
        print(
            f"DEBUG: CostValues - FREE: {CostValues.FREE}, UNKNOWN: {CostValues.UNKNOWN}, OCCUPIED: {CostValues.OCCUPIED}"
        )

        # Convert robot pose to subsampled grid coordinates
        subsampled_grid_pos = subsampled_costmap.world_to_grid(robot_pose)
        grid_x, grid_y = int(subsampled_grid_pos.x), int(subsampled_grid_pos.y)

        print(
            f"DEBUG: Robot pose - World: ({robot_pose.x:.2f}, {robot_pose.y:.2f}), Subsampled Grid: ({grid_x}, {grid_y})"
        )

        # Find nearest free space to start exploration
        free_x, free_y = self._find_free_space(grid_x, grid_y, subsampled_costmap)
        print(f"DEBUG: Found free space at subsampled grid: ({free_x}, {free_y})")

        start_point = self._cache.get_point(free_x, free_y)
        start_point.classification = PointClassification.MapOpen

        # Main exploration queue - explore ALL reachable free space
        map_queue = deque([start_point])
        frontiers = []

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
                    print(
                        f"DEBUG: Found valid frontier with {len(new_frontier)} points at {centroid}"
                    )

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

        print(
            f"DEBUG: Frontier detection complete - Points checked: {points_checked}, Frontier candidates: {frontier_candidates}, Valid frontiers: {len(frontiers)}"
        )

        # Extract just the centroids for ranking
        frontier_centroids = frontiers

        if not frontier_centroids:
            return []

        # Update persistent frontiers
        self._update_persistent_frontiers(frontier_centroids, time.time())

        # Rank frontiers using original costmap for proper filtering
        ranked_frontiers = self._rank_frontiers_by_information_gain(
            frontier_centroids, [1] * len(frontier_centroids), robot_pose, costmap
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

    def _update_persistent_frontiers(self, new_frontiers: List[Vector], current_time: float):
        """Update the persistent frontier database with newly detected frontiers."""
        # Convert new frontiers to set for efficient lookups
        new_frontier_positions = set((round(f.x, 2), round(f.y, 2)) for f in new_frontiers)

        # Update existing frontiers
        to_remove = []
        for key, persistent_frontier in self.persistent_frontiers.items():
            rounded_pos = (
                round(persistent_frontier.centroid.x, 2),
                round(persistent_frontier.centroid.y, 2),
            )
            if rounded_pos in new_frontier_positions:
                # Frontier still exists, update timestamp
                persistent_frontier.last_seen = current_time
                new_frontier_positions.remove(rounded_pos)  # Mark as processed
            else:
                # Frontier no longer exists, check if it should be removed
                if current_time - persistent_frontier.last_seen > 30.0:  # Remove after 30 seconds
                    to_remove.append(key)

        # Remove outdated frontiers
        for key in to_remove:
            del self.persistent_frontiers[key]

        # Add new frontiers
        for frontier in new_frontiers:
            key = f"{frontier.x:.2f},{frontier.y:.2f}"
            if key not in self.persistent_frontiers:
                self.persistent_frontiers[key] = PersistentFrontier(
                    centroid=frontier,
                    size=1,  # We lost size info in current implementation, could be improved
                    last_seen=current_time,
                    attempts=0,
                    accessible=True,
                )

    def _get_persistent_frontiers(
        self, robot_pose: Vector, max_age: float = 60.0
    ) -> List[PersistentFrontier]:
        """Get valid persistent frontiers that haven't been attempted too many times."""
        current_time = time.time()
        valid_frontiers = []

        for persistent_frontier in self.persistent_frontiers.values():
            # Check age
            if current_time - persistent_frontier.last_seen > max_age:
                continue

            # Check attempt count (avoid frontiers that have failed multiple times)
            if persistent_frontier.attempts > 3:
                continue

            # Check accessibility
            if not persistent_frontier.accessible:
                continue

            valid_frontiers.append(persistent_frontier)

        return valid_frontiers

    def mark_frontier_attempted(
        self, frontier_goal: Vector, success: bool = False, radius: float = 4.0
    ):
        """Mark frontiers in an area around the attempted goal and update their accessibility."""
        marked_count = 0

        for key, persistent_frontier in self.persistent_frontiers.items():
            # Calculate distance from attempted goal to this persistent frontier
            distance = np.sqrt(
                (persistent_frontier.centroid.x - frontier_goal.x) ** 2
                + (persistent_frontier.centroid.y - frontier_goal.y) ** 2
            )

            # Mark all frontiers within radius as attempted
            if distance <= radius:
                persistent_frontier.attempts += 1
                marked_count += 1

                if not success:
                    # If exploration failed multiple times, mark as potentially inaccessible
                    if persistent_frontier.attempts >= 2:
                        persistent_frontier.accessible = False

        print(
            f"DEBUG: Marked {marked_count} frontiers within {radius}m of attempted goal (success: {success})"
        )

    def _is_in_explored_area(self, frontier: Vector, costmap: Costmap) -> bool:
        """Check if a frontier is in a significantly explored area (requires high density of free cells)."""
        # Convert frontier to grid coordinates
        grid_pos = costmap.world_to_grid(frontier)
        grid_x, grid_y = int(grid_pos.x), int(grid_pos.y)

        # Check neighborhood around frontier
        buffer_cells = int(self.explored_area_buffer / costmap.resolution)

        free_count = 0
        total_count = 0

        for dx in range(-buffer_cells, buffer_cells + 1):
            for dy in range(-buffer_cells, buffer_cells + 1):
                check_x, check_y = grid_x + dx, grid_y + dy

                # Check bounds
                if 0 <= check_x < costmap.width and 0 <= check_y < costmap.height:
                    world_pos = costmap.grid_to_world(Vector([float(check_x), float(check_y)]))
                    cell_value = costmap.get_value(world_pos)

                    total_count += 1
                    if cell_value == CostValues.FREE:
                        free_count += 1

        # Only consider it "explored" if significant portion (>60%) is free space
        if total_count > 0:
            free_ratio = free_count / total_count
            is_explored = free_ratio > 0.6
            if is_explored:
                print(f"DEBUG: Frontier {frontier} in explored area - {free_ratio:.2f} free ratio")
            return is_explored

        return False

    def _compute_distance_to_costmap_border(self, frontier: Vector, costmap: Costmap) -> float:
        """Compute distance from frontier to the nearest unknown border of the costmap."""
        grid_pos = costmap.world_to_grid(frontier)
        grid_x, grid_y = int(grid_pos.x), int(grid_pos.y)

        # Distance to physical edges of costmap
        edge_distances = [
            grid_x,  # distance to left edge
            costmap.width - grid_x,  # distance to right edge
            grid_y,  # distance to bottom edge
            costmap.height - grid_y,  # distance to top edge
        ]

        min_edge_distance = min(edge_distances)

        # Convert to world units
        return min_edge_distance * costmap.resolution

    def _compute_comprehensive_frontier_score(
        self, frontier: Vector, frontier_size: int, robot_pose: Vector, costmap: Costmap
    ) -> float:
        """Compute comprehensive score considering multiple criteria."""

        # 1. Distance from robot (penalty for being too close, bonus for reasonable distance)
        robot_distance = np.sqrt(
            (frontier.x - robot_pose.x) ** 2 + (frontier.y - robot_pose.y) ** 2
        )

        if robot_distance < self.min_distance_from_robot:
            return -1000.0  # Heavy penalty for being too close

        # Distance score: prefer moderate distances (not too close, not too far)
        optimal_distance = 4.0  # meters
        distance_score = 1.0 / (1.0 + abs(robot_distance - optimal_distance))

        # 2. Information gain (frontier size)
        info_gain_score = frontier_size

        # 3. Border proximity (prefer frontiers closer to unexplored edges)
        border_distance = self._compute_distance_to_costmap_border(frontier, costmap)
        border_score = 1.0 / (1.0 + border_distance)  # Higher score for closer to border

        # 4. Direction momentum (if we have a current direction)
        momentum_score = self._compute_direction_momentum_score(frontier, robot_pose)

        # Combine scores with weights
        total_score = (
            0.4 * info_gain_score  # 40% information gain
            + 0.3 * border_score * 100  # 30% border proximity (scaled up)
            + 0.2 * distance_score * 50  # 20% distance optimization (scaled up)
            + 0.1 * momentum_score * 20  # 10% direction momentum (scaled up)
        )

        return total_score

    def _rank_frontiers_by_information_gain(
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
            print("DEBUG: No frontier centroids provided to ranking")
            return []

        print(f"DEBUG: Starting frontier ranking with {len(frontier_centroids)} candidates")
        valid_frontiers = []

        for i, frontier in enumerate(frontier_centroids):
            robot_distance = np.sqrt(
                (frontier.x - robot_pose.x) ** 2 + (frontier.y - robot_pose.y) ** 2
            )
            print(f"DEBUG: Evaluating frontier {i}: {frontier}, distance: {robot_distance:.2f}m")

            # Filter 1: Skip frontiers too close to explored areas (TEMPORARILY DISABLED for debugging)
            # if self._is_in_explored_area(frontier, costmap):
            #     print(f"DEBUG: Skipping frontier {frontier} - too close to explored area")
            #     continue

            # Filter 2: Skip frontiers too close to robot
            if robot_distance < self.min_distance_from_robot:
                print(
                    f"DEBUG: Skipping frontier {frontier} - too close to robot ({robot_distance:.2f}m < {self.min_distance_from_robot}m)"
                )
                continue

            # Compute comprehensive score
            frontier_size = frontier_sizes[i] if i < len(frontier_sizes) else 1
            score = self._compute_comprehensive_frontier_score(
                frontier, frontier_size, robot_pose, costmap
            )

            valid_frontiers.append((frontier, score))
            print(
                f"DEBUG: Frontier {frontier} - Score: {score:.2f}, Size: {frontier_size}, Distance: {robot_distance:.2f}m"
            )

        if not valid_frontiers:
            print("DEBUG: No valid frontiers found after filtering")
            return []

        # Sort by score and return only the best one
        valid_frontiers.sort(key=lambda x: x[1], reverse=True)
        best_frontier = valid_frontiers[0][0]

        print(
            f"DEBUG: Selected best frontier: {best_frontier} with score {valid_frontiers[0][1]:.2f}"
        )
        return [best_frontier]  # Return list with single best frontier

    def _rank_frontiers_by_distance(
        self, frontiers: List[Vector], robot_pose: Vector
    ) -> List[Tuple[Vector, float]]:
        """
        Rank frontiers by distance from robot position.

        Args:
            frontiers: List of frontier centroids
            robot_pose: Current robot position

        Returns:
            List of (frontier, distance) pairs sorted by distance (closest first)
        """
        if not frontiers:
            return []

        # Vectorized distance calculation using numpy
        frontiers_array = np.array([[frontier.x, frontier.y] for frontier in frontiers])
        robot_array = np.array([robot_pose.x, robot_pose.y])

        # Compute all distances at once using broadcasting
        distances = np.linalg.norm(frontiers_array - robot_array, axis=1)

        # Create list of (frontier, distance) pairs
        frontier_distances = list(zip(frontiers, distances))

        # Sort by distance (closest first)
        frontier_distances.sort(key=lambda x: x[1])
        return frontier_distances

    def get_exploration_goals(
        self, robot_pose: Vector, costmap: Costmap, num_goals: int = 1
    ) -> List[Vector]:
        """
        Get the single best exploration goal using comprehensive frontier scoring.

        Args:
            robot_pose: Current robot position in world coordinates (Vector with x, y)
            costmap: Costmap for additional analysis
            num_goals: Ignored - always returns 0 or 1 goals

        Returns:
            List with single best frontier goal, or empty list if no suitable frontiers found
        """
        # Always detect new frontiers to get most up-to-date information
        # The new algorithm filters out explored areas and returns only the best frontier
        frontiers = self.detect_frontiers(robot_pose, costmap)

        if not frontiers:
            print("DEBUG: No suitable frontiers found")
            return []

        # Update exploration direction based on best goal selection
        if frontiers:
            self._update_exploration_direction(robot_pose, frontiers[0])
            print(f"DEBUG: Updated exploration direction toward {frontiers[0]}")

        return frontiers  # Already contains only the single best frontier

    def get_best_exploration_goal(self, robot_pose: Vector, costmap: Costmap) -> Optional[Vector]:
        """
        Get the single best exploration goal (highest information gain frontier).

        Args:
            robot_pose: Current robot position in world coordinates (Vector with x, y)
            costmap: Costmap for additional analysis

        Returns:
            Best frontier goal position in world coordinates, or None if no frontiers found
        """
        goals = self.get_exploration_goals(robot_pose, costmap, num_goals=1)
        return goals[0] if goals else None


def test_frontier_detection_visual():
    """
    Visual unit test for frontier detection using saved costmaps.
    Shows frontier detection results with different colors:
    - All unfiltered results: light green
    - Top 5 results: green
    - Best result: red
    """

    # Path to saved costmaps
    saved_maps_dir = os.path.join(os.getcwd(), "assets", "saved_maps")

    if not os.path.exists(saved_maps_dir):
        print(f"Error: Saved maps directory not found: {saved_maps_dir}")
        return

    # Get all pickle files
    pickle_files = sorted(glob.glob(os.path.join(saved_maps_dir, "*.pickle")))

    if not pickle_files:
        print(f"No pickle files found in {saved_maps_dir}")
        return

    print(f"Found {len(pickle_files)} costmap files for testing")

    # Initialize frontier explorer with more lenient settings for testing
    explorer = WavefrontFrontierExplorer(
        min_frontier_size=10,  # Smaller minimum size for testing
        min_distance_from_robot=0.3,  # Closer minimum distance
        subsample_resolution=2,  # Faster processing
    )

    # Process each costmap
    for i, pickle_file in enumerate(pickle_files):
        print(
            f"\n--- Processing costmap {i + 1}/{len(pickle_files)}: {os.path.basename(pickle_file)} ---"
        )

        try:
            # Load the costmap
            costmap = Costmap.from_pickle(pickle_file)
            print(f"Loaded costmap: {costmap}")

            # Use center of costmap as robot position for testing
            center_world = costmap.grid_to_world(Vector([costmap.width / 2, costmap.height / 2]))
            robot_pose = Vector([center_world.x, center_world.y])

            print(f"Using robot position: {robot_pose}")

            # Detect all frontiers (unfiltered) - need to call the internal detection method
            print("Detecting unfiltered frontiers...")

            # Clear cache and detect frontiers manually to get all results
            explorer._cache.clear()

            # Use the internal detect_frontiers method but bypass the ranking filter
            if explorer.subsample_resolution > 1:
                subsampled_costmap = costmap.subsample(explorer.subsample_resolution)
            else:
                subsampled_costmap = costmap

            # Convert robot pose to subsampled grid coordinates
            subsampled_grid_pos = subsampled_costmap.world_to_grid(robot_pose)
            grid_x, grid_y = int(subsampled_grid_pos.x), int(subsampled_grid_pos.y)

            # Find nearest free space to start exploration
            free_x, free_y = explorer._find_free_space(grid_x, grid_y, subsampled_costmap)
            start_point = explorer._cache.get_point(free_x, free_y)
            start_point.classification = PointClassification.MapOpen

            # Run the core frontier detection to get ALL frontiers
            from collections import deque

            map_queue = deque([start_point])
            all_detected_frontiers = []

            while map_queue:
                current_point = map_queue.popleft()

                # Skip if already processed
                if current_point.classification & PointClassification.MapClosed:
                    continue

                # Mark as processed
                current_point.classification |= PointClassification.MapClosed

                # Check if this point starts a new frontier
                if explorer._is_frontier_point(current_point, subsampled_costmap):
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
                        if explorer._is_frontier_point(frontier_point, subsampled_costmap):
                            new_frontier.append(frontier_point)

                            # Add neighbors to frontier queue
                            for neighbor in explorer._get_neighbors(
                                frontier_point, subsampled_costmap
                            ):
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
                    if len(new_frontier) >= 5:  # Lower threshold for visualization
                        world_points = []
                        for point in new_frontier:
                            world_pos = subsampled_costmap.grid_to_world(
                                Vector([float(point.x), float(point.y)])
                            )
                            world_points.append(world_pos)

                        # Compute centroid in world coordinates
                        centroid = explorer._compute_centroid(world_points)
                        all_detected_frontiers.append(centroid)

                # Add ALL neighbors to main exploration queue to explore entire free space
                for neighbor in explorer._get_neighbors(current_point, subsampled_costmap):
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

            # Get filtered/ranked frontiers (top 5) using the normal method
            print("Getting top exploration goals...")
            ranked_frontiers = explorer._rank_frontiers_by_information_gain(
                all_detected_frontiers, [1] * len(all_detected_frontiers), robot_pose, costmap
            )

            # Take top 5 from ranked results
            top_5_frontiers = (
                ranked_frontiers[:5] if len(ranked_frontiers) >= 5 else ranked_frontiers
            )

            print(f"Found {len(all_detected_frontiers)} unfiltered frontiers")
            print(f"Selected {len(top_5_frontiers)} top frontiers")
            print(f"Best frontier: {ranked_frontiers[0] if ranked_frontiers else 'None'}")

            # Convert costmap to image
            scale_factor = 3  # Scale up for better visibility
            img = costmap_to_pil_image(costmap, scale_factor)

            # Draw frontiers on image
            img_with_frontiers = draw_frontiers_on_image(
                img, costmap, top_5_frontiers, scale_factor, all_detected_frontiers
            )

            # Show image and wait for user input
            title = f"Frame {i + 1:04d} - Frontier Detection Results"
            img_with_frontiers.show(title=title)

        except Exception as e:
            print(f"Error processing {pickle_file}: {e}")
            continue

    print("\n=== Frontier Detection Test Complete ===")


if __name__ == "__main__":
    # Run the visual test
    test_frontier_detection_visual()
