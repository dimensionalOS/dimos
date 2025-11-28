#!/usr/bin/env python3

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
Gradient-Augmented Look-Ahead Pursuit (GLAP) holonomic local planner.
"""

import numpy as np

from dimos.core import rpc
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.navigation.local_planner.local_planner import BaseLocalPlanner
from dimos.utils.transform_utils import get_distance, normalize_angle, quaternion_to_euler


class HolonomicLocalPlanner(BaseLocalPlanner):
    """
    Gradient-Augmented Look-Ahead Pursuit (GLAP) holonomic local planner.

    This planner combines path following with obstacle avoidance using
    costmap gradients to produce smooth holonomic velocity commands.

    Args:
        lookahead_dist: Look-ahead distance in meters (default: 1.0)
        k_rep: Repulsion gain for obstacle avoidance (default: 1.0)
        alpha: Low-pass filter coefficient [0-1] (default: 0.5)
        v_max: Maximum velocity per component in m/s (default: 0.8)
        goal_tolerance: Distance threshold to consider goal reached (default: 0.5)
        control_frequency: Control loop frequency in Hz (default: 10.0)
    """

    def __init__(  # type: ignore[no-untyped-def]
        self,
        lookahead_dist: float = 1.0,
        k_rep: float = 0.5,
        k_angular: float = 0.75,
        alpha: float = 0.5,
        v_max: float = 0.8,
        goal_tolerance: float = 0.5,
        orientation_tolerance: float = 0.2,
        control_frequency: float = 10.0,
        **kwargs,
    ) -> None:
        """Initialize the GLAP planner with specified parameters."""
        super().__init__(
            goal_tolerance=goal_tolerance,
            orientation_tolerance=orientation_tolerance,
            control_frequency=control_frequency,
            **kwargs,
        )

        # Algorithm parameters
        self.lookahead_dist = lookahead_dist
        self.k_rep = k_rep
        self.alpha = alpha
        self.v_max = v_max
        self.k_angular = k_angular

        # Previous velocity for filtering (vx, vy, vtheta)
        self.v_prev = np.array([0.0, 0.0, 0.0])

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def compute_velocity(self) -> Twist | None:
        """
        Compute velocity commands using GLAP algorithm.

        Returns:
            Twist with linear and angular velocities in robot frame
        """
        if self.latest_odom is None or self.latest_path is None or self.latest_costmap is None:
            return None

        pose = np.array([self.latest_odom.position.x, self.latest_odom.position.y])

        euler = quaternion_to_euler(self.latest_odom.orientation)
        robot_yaw = euler.z

        path_points = []
        for pose_stamped in self.latest_path.poses:
            path_points.append([pose_stamped.position.x, pose_stamped.position.y])

        if len(path_points) == 0:
            return None

        path = np.array(path_points)

        costmap = self.latest_costmap.grid

        v_follow_odom = self._compute_path_following(pose, path)

        v_rep_odom = self._compute_obstacle_repulsion(pose, costmap)

        v_odom = v_follow_odom + v_rep_odom

        # Transform velocity from odom frame to robot frame
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)

        v_robot_x = cos_yaw * v_odom[0] + sin_yaw * v_odom[1]
        v_robot_y = -sin_yaw * v_odom[0] + cos_yaw * v_odom[1]

        # Compute angular velocity
        closest_idx, _ = self._find_closest_point_on_path(pose, path)

        # Check if we're near the final goal
        goal_pose = self.latest_path.poses[-1]
        distance_to_goal = get_distance(self.latest_odom, goal_pose)

        if distance_to_goal < self.goal_tolerance:
            # Near goal - rotate to match final goal orientation
            goal_euler = quaternion_to_euler(goal_pose.orientation)
            desired_yaw = goal_euler.z
        else:
            # Not near goal - align with path direction
            lookahead_point = self._find_lookahead_point(path, closest_idx)
            dx = lookahead_point[0] - pose[0]
            dy = lookahead_point[1] - pose[1]
            desired_yaw = np.arctan2(dy, dx)

        yaw_error = normalize_angle(desired_yaw - robot_yaw)
        k_angular = self.k_angular
        v_theta = k_angular * yaw_error

        # Slow down linear velocity when turning
        # Scale linear velocity based on angular velocity magnitude
        angular_speed = abs(v_theta)
        max_angular_speed = self.v_max

        # Calculate speed reduction factor (1.0 when not turning, 0.2 when at max turn rate)
        turn_slowdown = 1.0 - 0.8 * min(angular_speed / max_angular_speed, 1.0)

        # Apply speed reduction to linear velocities
        v_robot_x = np.clip(v_robot_x * turn_slowdown, -self.v_max, self.v_max)
        v_robot_y = np.clip(v_robot_y * turn_slowdown, -self.v_max, self.v_max)
        v_theta = np.clip(v_theta, -self.v_max, self.v_max)

        v_raw = np.array([v_robot_x, v_robot_y, v_theta])
        v_filtered = self.alpha * v_raw + (1 - self.alpha) * self.v_prev
        self.v_prev = v_filtered

        return Twist(
            linear=Vector3(v_filtered[0], v_filtered[1], 0.0),
            angular=Vector3(0.0, 0.0, v_filtered[2]),
        )

    def _compute_path_following(self, pose: np.ndarray, path: np.ndarray) -> np.ndarray:  # type: ignore[type-arg]
        """
        Compute path following velocity using pure pursuit.

        Args:
            pose: Current robot position [x, y]
            path: Path waypoints as Nx2 array

        Returns:
            Path following velocity vector [vx, vy]
        """
        closest_idx, _ = self._find_closest_point_on_path(pose, path)

        carrot = self._find_lookahead_point(path, closest_idx)

        direction = carrot - pose
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return np.zeros(2)

        v_follow = self.v_max * direction / distance

        return v_follow  # type: ignore[no-any-return]

    def _compute_obstacle_repulsion(self, pose: np.ndarray, costmap: np.ndarray) -> np.ndarray:  # type: ignore[type-arg]
        """
        Compute obstacle repulsion velocity from costmap gradient.

        Args:
            pose: Current robot position [x, y]
            costmap: 2D costmap array

        Returns:
            Repulsion velocity vector [vx, vy]
        """
        grid_point = self.latest_costmap.world_to_grid(pose)  # type: ignore[union-attr]
        grid_x = int(grid_point.x)
        grid_y = int(grid_point.y)

        height, width = costmap.shape
        if not (1 <= grid_x < width - 1 and 1 <= grid_y < height - 1):
            return np.zeros(2)

        # Compute gradient using central differences
        # Note: costmap is in row-major order (y, x)
        gx = (costmap[grid_y, grid_x + 1] - costmap[grid_y, grid_x - 1]) / (
            2.0 * self.latest_costmap.resolution  # type: ignore[union-attr]
        )
        gy = (costmap[grid_y + 1, grid_x] - costmap[grid_y - 1, grid_x]) / (
            2.0 * self.latest_costmap.resolution  # type: ignore[union-attr]
        )

        # Gradient points towards higher cost, so negate for repulsion
        v_rep = -self.k_rep * np.array([gx, gy])

        return v_rep

    def _find_closest_point_on_path(
        self,
        pose: np.ndarray,  # type: ignore[type-arg]
        path: np.ndarray,  # type: ignore[type-arg]
    ) -> tuple[int, np.ndarray]:  # type: ignore[type-arg]
        """
        Find the closest point on the path to current pose.

        Args:
            pose: Current position [x, y]
            path: Path waypoints as Nx2 array

        Returns:
            Tuple of (closest_index, closest_point)
        """
        distances = np.linalg.norm(path - pose, axis=1)
        closest_idx = np.argmin(distances)
        return closest_idx, path[closest_idx]  # type: ignore[return-value]

    def _find_lookahead_point(self, path: np.ndarray, start_idx: int) -> np.ndarray:  # type: ignore[type-arg]
        """
        Find look-ahead point on path at specified distance.

        Args:
            path: Path waypoints as Nx2 array
            start_idx: Starting index for search

        Returns:
            Look-ahead point [x, y]
        """
        accumulated_dist = 0.0

        for i in range(start_idx, len(path) - 1):
            segment_dist = np.linalg.norm(path[i + 1] - path[i])

            if accumulated_dist + segment_dist >= self.lookahead_dist:
                remaining_dist = self.lookahead_dist - accumulated_dist
                t = remaining_dist / segment_dist
                carrot = path[i] + t * (path[i + 1] - path[i])
                return carrot  # type: ignore[no-any-return]

            accumulated_dist += segment_dist  # type: ignore[assignment]

        return path[-1]  # type: ignore[no-any-return]

    def _clip(self, v: np.ndarray) -> np.ndarray:  # type: ignore[type-arg]
        """Instance method to clip velocity with access to v_max."""
        return np.clip(v, -self.v_max, self.v_max)


holonomic_local_planner = HolonomicLocalPlanner.blueprint

__all__ = ["HolonomicLocalPlanner", "holonomic_local_planner"]
