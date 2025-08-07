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

from typing import Optional, Tuple

import numpy as np

from dimos.msgs.geometry_msgs import Vector3
from dimos.navigation.local_planner import BaseLocalPlanner
from dimos.utils.transform_utils import quaternion_to_euler, normalize_angle


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

    def __init__(
        self,
        lookahead_dist: float = 1.0,
        k_rep: float = 0.5,
        alpha: float = 0.5,
        v_max: float = 0.8,
        goal_tolerance: float = 0.5,
        control_frequency: float = 10.0,
        **kwargs,
    ):
        """Initialize the GLAP planner with specified parameters."""
        super().__init__(
            goal_tolerance=goal_tolerance, control_frequency=control_frequency, **kwargs
        )

        # Algorithm parameters
        self.lookahead_dist = lookahead_dist
        self.k_rep = k_rep
        self.alpha = alpha
        self.v_max = v_max

        # Previous velocity for filtering (vx, vy, vtheta)
        self.v_prev = np.array([0.0, 0.0, 0.0])

    def compute_velocity(self) -> Optional[Vector3]:
        """
        Compute velocity commands using GLAP algorithm.

        Returns:
            Vector3 with x, y velocities in robot frame and z as angular velocity
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

        # Compute angular velocity to align with path direction
        closest_idx, _ = self._find_closest_point_on_path(pose, path)
        lookahead_point = self._find_lookahead_point(path, closest_idx)

        dx = lookahead_point[0] - pose[0]
        dy = lookahead_point[1] - pose[1]
        desired_yaw = np.arctan2(dy, dx)

        yaw_error = normalize_angle(desired_yaw - robot_yaw)
        k_angular = 2.0  # Angular gain
        v_theta = k_angular * yaw_error

        v_robot_x = np.clip(v_robot_x, -self.v_max, self.v_max)
        v_robot_y = np.clip(v_robot_y, -self.v_max, self.v_max)
        v_theta = np.clip(v_theta, -self.v_max, self.v_max)

        v_raw = np.array([v_robot_x, v_robot_y, v_theta])
        v_filtered = self.alpha * v_raw + (1 - self.alpha) * self.v_prev
        self.v_prev = v_filtered

        return Vector3(v_filtered[0], v_filtered[1], v_filtered[2])

    def _compute_path_following(self, pose: np.ndarray, path: np.ndarray) -> np.ndarray:
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

        return v_follow

    def _compute_obstacle_repulsion(self, pose: np.ndarray, costmap: np.ndarray) -> np.ndarray:
        """
        Compute obstacle repulsion velocity from costmap gradient.

        Args:
            pose: Current robot position [x, y]
            costmap: 2D costmap array

        Returns:
            Repulsion velocity vector [vx, vy]
        """
        grid_point = self.latest_costmap.world_to_grid(pose)
        grid_x = int(grid_point.x)
        grid_y = int(grid_point.y)

        height, width = costmap.shape
        if not (1 <= grid_x < width - 1 and 1 <= grid_y < height - 1):
            return np.zeros(2)

        # Compute gradient using central differences
        # Note: costmap is in row-major order (y, x)
        gx = (costmap[grid_y, grid_x + 1] - costmap[grid_y, grid_x - 1]) / (
            2.0 * self.latest_costmap.resolution
        )
        gy = (costmap[grid_y + 1, grid_x] - costmap[grid_y - 1, grid_x]) / (
            2.0 * self.latest_costmap.resolution
        )

        # Gradient points towards higher cost, so negate for repulsion
        v_rep = -self.k_rep * np.array([gx, gy])

        return v_rep

    def _find_closest_point_on_path(
        self, pose: np.ndarray, path: np.ndarray
    ) -> Tuple[int, np.ndarray]:
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
        return closest_idx, path[closest_idx]

    def _find_lookahead_point(self, path: np.ndarray, start_idx: int) -> np.ndarray:
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
                return carrot

            accumulated_dist += segment_dist

        return path[-1]

    def _clip(self, v: np.ndarray) -> np.ndarray:
        """Instance method to clip velocity with access to v_max."""
        return np.clip(v, -self.v_max, self.v_max)
