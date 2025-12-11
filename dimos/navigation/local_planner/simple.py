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

from enum import Enum, auto

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.core import In, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import PoseStamped, Twist, Vector3
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid, Path
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.navigation.local_planner.local_planner import BaseLocalPlanner
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle, quaternion_to_euler

logger = setup_logger()


class NavigationStage(Enum):
    INITIAL_ROTATION = auto()
    PATH_FOLLOWING = auto()
    FINAL_ROTATION = auto()


class SimplePlanner(BaseLocalPlanner):
    global_costmap: In[OccupancyGrid] = None  # type: ignore[assignment]
    goal_request: In[PoseStamped] = None  # type: ignore[assignment]
    target: Out[PoseStamped] = None  # type: ignore[assignment]

    _global_config: GlobalConfig
    _latest_costmap: OccupancyGrid | None = None

    def __init__(
        self,
        speed: float = 1.0,
        k_angular: float = 2.0,
        lookahead_dist: float = 0.5,
        goal_tolerance: float = 0.3,
        orientation_tolerance: float = 0.2,
        control_frequency: float = 20.0,
        global_config: GlobalConfig | None = None,
    ) -> None:
        super().__init__(
            goal_tolerance=goal_tolerance,
            orientation_tolerance=orientation_tolerance,
            control_frequency=control_frequency,
        )

        self._global_config = global_config or GlobalConfig()

        self.speed = self._global_config.planner_robot_speed or speed
        self.k_angular = k_angular
        self.lookahead_dist = lookahead_dist
        self._stage = NavigationStage.INITIAL_ROTATION
        self._goal_reached = False

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.goal_request.subscribe(self._set_target)))

        def set_latest_costmap(msg: OccupancyGrid) -> None:
            self._latest_costmap = msg

        self._disposables.add(Disposable(self.global_costmap.subscribe(set_latest_costmap)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_path(self, msg: Path) -> None:
        self._stage = NavigationStage.INITIAL_ROTATION
        self._goal_reached = False
        super()._on_path(msg)

    def compute_velocity(self) -> Twist | None:
        if self._goal_reached:
            return None

        if self.latest_odom is None or self.latest_path is None:
            return None

        if not len(self.latest_path.poses):
            return None

        if self._stage == NavigationStage.INITIAL_ROTATION:
            return self._compute_initial_rotation()
        elif self._stage == NavigationStage.PATH_FOLLOWING:
            return self._compute_path_following()
        else:  # FINAL_ROTATION
            return self._compute_final_rotation()

    def _compute_initial_rotation(self) -> Twist:
        assert self.latest_path is not None
        assert self.latest_odom is not None
        first_pose = self.latest_path.poses[0]
        first_yaw = quaternion_to_euler(first_pose.orientation).z
        robot_yaw = self.latest_odom.orientation.euler[2]
        yaw_error = normalize_angle(first_yaw - robot_yaw)

        if abs(yaw_error) < self.orientation_tolerance:
            self._stage = NavigationStage.PATH_FOLLOWING
            return self._compute_path_following()

        max_angular_speed = self.speed
        angular_velocity = np.clip(
            self.k_angular * yaw_error, -max_angular_speed, max_angular_speed
        )

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _compute_path_following(self) -> Twist:
        assert self.latest_path is not None
        assert self.latest_odom is not None
        current_pose = np.array([self.latest_odom.position.x, self.latest_odom.position.y])
        path = np.array([[p.position.x, p.position.y] for p in self.latest_path.poses])

        # Check if we've reached the final position
        goal_position = path[-1]
        distance_to_goal = np.linalg.norm(goal_position - current_pose)

        if distance_to_goal < self.goal_tolerance:
            logger.info("Reached goal position, starting final rotation")
            self._stage = NavigationStage.FINAL_ROTATION
            return self._compute_final_rotation()

        closest_idx = self._find_closest_point_idx(current_pose, path)
        lookahead_point = self._find_lookahead_point(current_pose, path, closest_idx)

        direction = lookahead_point - current_pose
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return Twist()

        robot_yaw = self.latest_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = normalize_angle(desired_yaw - robot_yaw)

        max_angular_speed = 0.5
        angular_velocity = np.clip(
            self.k_angular * yaw_error, -max_angular_speed, max_angular_speed
        )

        # Linear velocity - reduce speed when turning
        angular_slowdown = 1.0 - 0.7 * min(abs(angular_velocity) / max_angular_speed, 1.0)
        linear_velocity = self.speed * angular_slowdown

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _compute_final_rotation(self) -> Twist:
        assert self.latest_path is not None
        assert self.latest_odom is not None
        last_pose = self.latest_path.poses[-1]
        goal_yaw = quaternion_to_euler(last_pose.orientation).z
        robot_yaw = self.latest_odom.orientation.euler[2]
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        if abs(yaw_error) < self.orientation_tolerance:
            logger.info("Final rotation complete, goal reached")
            self._goal_reached = True
            return Twist()

        max_angular_speed = self.speed
        angular_velocity = np.clip(
            self.k_angular * yaw_error, -max_angular_speed, max_angular_speed
        )

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _find_closest_point_idx(self, pose: NDArray[np.float64], path: NDArray[np.float64]) -> int:
        """Find the index of the closest point on the path."""
        distances = np.linalg.norm(path - pose, axis=1)
        return int(np.argmin(distances))

    def _find_lookahead_point(
        self, pose: NDArray[np.float64], path: NDArray[np.float64], start_idx: int
    ) -> NDArray[np.float64]:
        """Find a lookahead point on the path at the specified distance."""
        accumulated_dist: float = 0.0

        for i in range(start_idx, len(path) - 1):
            segment_dist = float(np.linalg.norm(path[i + 1] - path[i]))

            if accumulated_dist + segment_dist >= self.lookahead_dist:
                remaining_dist = self.lookahead_dist - accumulated_dist
                if segment_dist > 0:
                    t = remaining_dist / segment_dist
                    result: NDArray[np.float64] = path[i] + t * (path[i + 1] - path[i])
                    return result
                result_i: NDArray[np.float64] = path[i]
                return result_i

            accumulated_dist += segment_dist

        # Return the last point if we've traversed the whole path
        result_last: NDArray[np.float64] = path[-1]
        return result_last

    def _set_target(self, pose_stamped: PoseStamped) -> None:
        if self._latest_costmap is None:
            logger.warning("Cannot set target: missing costmap")
            return

        safe_goal = find_safe_goal(
            self._latest_costmap,
            pose_stamped.position,
            algorithm="bfs_contiguous",
            cost_threshold=CostValues.OCCUPIED,
            min_clearance=self._global_config.robot_rotation_diameter / 2,
            max_search_distance=5.0,
        )

        if safe_goal is None:
            logger.warning("No safe goal found near requested target")
            return
        logger.info("Set safe goal", x=safe_goal.x, y=safe_goal.y)
        self.target.publish(pose_stamped)


simple_planner = SimplePlanner.blueprint

__all__ = ["SimplePlanner", "simple_planner"]
