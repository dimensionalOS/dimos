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

from threading import RLock

from reactivex import Subject
from reactivex.disposable import CompositeDisposable

from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.navigation.global_planner.astar import astar
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner, StopMessage
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GlobalPlanner(Resource):
    path: Subject[Path]

    _current_odom: PoseStamped | None = None
    _current_goal: PoseStamped | None = None
    _goal_reached: bool = False
    _global_config: GlobalConfig
    _navigation_map: NavigationMap
    _local_planner: LocalPlanner
    _disposables: CompositeDisposable
    _lock: RLock

    def __init__(self, global_config: GlobalConfig) -> None:
        self.path = Subject()
        self._global_config = global_config
        self._navigation_map = NavigationMap(self._global_config)
        self._local_planner = LocalPlanner(self._global_config, self._navigation_map)
        self._disposables = CompositeDisposable()
        self._lock = RLock()

    def start(self) -> None:
        self._local_planner.start()
        self._disposables.add(
            self._local_planner.stopped_navigating.subscribe(self._on_stopped_navigating)
        )

    def stop(self) -> None:
        self.cancel_goal()
        self._local_planner.stop()
        self._disposables.dispose()

    def handle_odom(self, msg: PoseStamped) -> None:
        self._current_odom = msg
        self._local_planner.handle_odom(msg)

    def handle_global_costmap(self, msg: OccupancyGrid) -> None:
        self._navigation_map.update(msg)

    def handle_goal_request(self, goal: PoseStamped) -> None:
        self._current_goal = goal
        with self._lock:
            self._goal_reached = False
        self._plan_path(self._current_goal)

    def cancel_goal(self) -> None:
        self.path.on_next(Path())
        self._local_planner.stop_planning()

    def get_state(self) -> NavigationState:
        return self._local_planner.get_state()

    def is_goal_reached(self) -> bool:
        with self._lock:
            return self._goal_reached

    def _on_stopped_navigating(self, stop_message: StopMessage) -> None:
        self.path.on_next(Path())
        if stop_message == "obstacle_found":
            logger.info("Replanning path due to obstacle found")
            assert self._current_goal is not None
            self._plan_path(self._current_goal)
        elif stop_message == "arrived":
            logger.info("Arrived at goal")
            with self._lock:
                self._goal_reached = True
        elif stop_message == "error":
            logger.info("Failure in navigation")

    def _plan_path(self, goal: PoseStamped) -> None:
        self.cancel_goal()

        if self._current_odom is None:
            logger.warning("Cannot handle goal request: missing odometry")
            return

        safe_goal = find_safe_goal(
            self._navigation_map.binary_costmap,
            goal.position,
            algorithm="bfs_contiguous",
            cost_threshold=CostValues.OCCUPIED,
            min_clearance=self._global_config.robot_rotation_diameter / 2,
            max_search_distance=5.0,
        )

        if safe_goal is None:
            logger.warning("No safe goal found near requested target")
            return

        logger.info("Found safe goal", x=round(safe_goal.x, 2), y=round(safe_goal.y, 2))

        robot_pos = self._current_odom.position
        costmap = self._navigation_map.gradient_costmap
        path = astar(self._global_config.astar_algorithm, costmap, safe_goal, robot_pos)

        if not path:
            logger.warning(
                "No path found to the goal.", x=round(safe_goal.x, 3), y=round(safe_goal.y, 3)
            )
            return

        resampled_path = smooth_resample_path(path, goal, 0.1)

        self.path.on_next(resampled_path)

        self._local_planner.start_planning(resampled_path)

    @property
    def cmd_vel(self) -> Subject[Twist]:
        return self._local_planner.cmd_vel

    @property
    def debug_navigation(self) -> Subject[Image]:
        return self._local_planner.debug_navigation
