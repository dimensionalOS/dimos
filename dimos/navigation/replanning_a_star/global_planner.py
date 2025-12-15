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

from reactivex import Subject
from reactivex.disposable import CompositeDisposable

from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.mapping.occupancy.path_resampling import simple_resample_path
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.navigation.global_planner.astar import astar
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GlobalPlanner(Resource):
    path: Subject[Path]

    _current_odom: PoseStamped | None = None
    _current_global_costmap: OccupancyGrid | None = None
    _local_planner: LocalPlanner
    _global_config: GlobalConfig
    _last_global_costmap_used: OccupancyGrid | None = None
    _current_navigation_map: OccupancyGrid | None = None
    _disposables: CompositeDisposable

    def __init__(self, global_config: GlobalConfig) -> None:
        self.path = Subject()
        self._local_planner = LocalPlanner()
        self._global_config = global_config
        self._disposables = CompositeDisposable()

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
        self._current_global_costmap = msg

    def handle_goal_request(self, goal: PoseStamped) -> None:
        self.cancel_goal()

        if self._current_global_costmap is None:
            logger.warning("Cannot handle goal request: missing costmap")
            return

        if self._current_odom is None:
            logger.warning("Cannot handle goal request: missing odometry")
            return

        safe_goal = find_safe_goal(
            self._current_global_costmap,
            goal.position,
            algorithm="bfs_contiguous",
            cost_threshold=CostValues.OCCUPIED,
            min_clearance=self._global_config.robot_rotation_diameter / 2,
            max_search_distance=5.0,
        )

        if safe_goal is None:
            logger.warning("No safe goal found near requested target")
            return

        logger.info("Found safe goal", x=safe_goal.x, y=safe_goal.y)

        costmap = self._get_latest_navigation_map(self._current_global_costmap)
        robot_pos = self._current_odom.position
        path = astar(self._global_config.astar_algorithm, costmap, safe_goal, robot_pos)

        if not path:
            logger.warning("No path found to the goal.", x=safe_goal.x, y=safe_goal.y)
            return

        resampled_path = simple_resample_path(path, goal, 0.1)

        self.path.on_next(resampled_path)

        self._local_planner.start_planning(resampled_path)

    def _get_latest_navigation_map(self, global_costmap: OccupancyGrid) -> OccupancyGrid:
        # "is", not "==", to check for reference equality
        if global_costmap is self._last_global_costmap_used:
            assert self._current_navigation_map is not None
            return self._current_navigation_map

        self._current_navigation_map = make_navigation_map(
            global_costmap,
            self._global_config.robot_width,
            strategy=self._global_config.planner_strategy,
        )
        self._last_global_costmap_used = global_costmap

        return self._current_navigation_map

    def cancel_goal(self) -> None:
        self.path.on_next(Path())
        self._local_planner.stop_planning()

    def get_state(self) -> NavigationState:
        return self._local_planner.get_state()

    def is_goal_reached(self) -> bool:
        return self._local_planner.is_goal_reached()

    def _on_stopped_navigating(self, _: None) -> None:
        self.path.on_next(Path())

    @property
    def cmd_vel(self) -> Subject[Twist]:
        return self._local_planner.cmd_vel
