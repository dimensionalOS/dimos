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

import os
from typing import Any

from dimos_lcm.std_msgs import Bool, String
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ReplanningAStarPlannerConfig(ModuleConfig):
    robot_width: float | None = None
    robot_rotation_diameter: float | None = None
    path_length_weight: float = Field(default=1.0, ge=0.0)
    path_cell_cost_weight: float = Field(default=3.0, ge=0.0)
    publish_raw_path: bool = False
    constrained_path_smoothing_enabled: bool = False
    path_smoothing_performance_logging_enabled: bool = False
    path_smoothing_iterations: int = Field(default=40, ge=0)
    path_smoothing_data_weight: float = Field(default=0.02, ge=0.0, le=1.0)
    path_smoothing_smoothness_weight: float = Field(default=0.45, ge=0.0, le=0.5)
    path_smoothing_max_deviation_m: float = Field(default=0.1, ge=0.0)
    path_smoothing_collision_sample_spacing_m: float = Field(default=0.05, gt=0.0)
    path_smoothing_max_cost_increase: float = Field(default=2.0, ge=0.0)
    path_smoothing_backtracking_factor: float = Field(default=0.5, gt=0.0, lt=1.0)
    path_smoothing_max_backtracking_steps: int = Field(default=3, ge=0)
    path_resample_spacing_m: float = Field(default=0.1, gt=0.0)


class ReplanningAStarPlanner(Module, NavigationInterface):
    config: ReplanningAStarPlannerConfig

    odom: In[PoseStamped]  # TODO: Use TF.
    odometry: In[Odometry]
    global_costmap: In[OccupancyGrid]
    goal_request: In[PoseStamped]
    clicked_point: In[PointStamped]
    target: In[PoseStamped]
    stop_movement: In[Bool]

    goal_reached: Out[Bool]
    navigation_state: Out[String]  # TODO: set it
    nav_cmd_vel: Out[Twist]
    path: Out[Path]
    raw_path: Out[Path]
    navigation_costmap: Out[OccupancyGrid]

    _planner: GlobalPlanner

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        overrides = {
            name: value
            for name, value in (
                ("robot_width", self.config.robot_width),
                ("robot_rotation_diameter", self.config.robot_rotation_diameter),
            )
            if value is not None
        }
        effective_global_config = (
            self.config.g.model_copy(update=overrides) if overrides else self.config.g
        )
        self._planner = GlobalPlanner(
            effective_global_config,
            path_length_weight=self.config.path_length_weight,
            path_cell_cost_weight=self.config.path_cell_cost_weight,
            publish_raw_path=self.config.publish_raw_path,
            constrained_path_smoothing_enabled=self.config.constrained_path_smoothing_enabled,
            path_smoothing_performance_logging_enabled=(
                self.config.path_smoothing_performance_logging_enabled
            ),
            path_smoothing_iterations=self.config.path_smoothing_iterations,
            path_smoothing_data_weight=self.config.path_smoothing_data_weight,
            path_smoothing_smoothness_weight=self.config.path_smoothing_smoothness_weight,
            path_smoothing_max_deviation_m=self.config.path_smoothing_max_deviation_m,
            path_smoothing_collision_sample_spacing_m=(
                self.config.path_smoothing_collision_sample_spacing_m
            ),
            path_smoothing_max_cost_increase=self.config.path_smoothing_max_cost_increase,
            path_smoothing_backtracking_factor=self.config.path_smoothing_backtracking_factor,
            path_smoothing_max_backtracking_steps=(
                self.config.path_smoothing_max_backtracking_steps
            ),
            path_resample_spacing_m=self.config.path_resample_spacing_m,
        )

    @rpc
    def start(self) -> None:
        super().start()

        self.register_disposable(Disposable(self.odom.subscribe(self._planner.handle_odom)))
        self.register_disposable(
            Disposable(
                self.odometry.subscribe(
                    lambda msg: self._planner.handle_odom(msg.to_pose_stamped())
                )
            )
        )
        self.register_disposable(
            Disposable(self.global_costmap.subscribe(self._planner.handle_global_costmap))
        )
        self.register_disposable(
            Disposable(self.goal_request.subscribe(self._planner.handle_goal_request))
        )
        self.register_disposable(
            Disposable(self.target.subscribe(self._planner.handle_goal_request))
        )

        self.register_disposable(
            Disposable(
                self.clicked_point.subscribe(
                    lambda pt: self._planner.handle_goal_request(pt.to_pose_stamped())
                )
            )
        )

        if self.stop_movement.transport is not None:
            self.register_disposable(
                Disposable(self.stop_movement.subscribe(self._on_stop_movement))
            )

        self.register_disposable(self._planner.path.subscribe(self.path.publish))
        self.register_disposable(self._planner.raw_path.subscribe(self.raw_path.publish))

        self.register_disposable(self._planner.cmd_vel.subscribe(self.nav_cmd_vel.publish))

        self.register_disposable(self._planner.goal_reached.subscribe(self.goal_reached.publish))

        if "DEBUG_NAVIGATION" in os.environ:
            self.register_disposable(
                self._planner.navigation_costmap.subscribe(self.navigation_costmap.publish)
            )

        self._planner.start()

    @rpc
    def stop(self) -> None:
        self.cancel_goal()
        self._planner.stop()

        super().stop()

    def _on_stop_movement(self, msg: Bool) -> None:
        if msg.data:
            self.cancel_goal()

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        self._planner.handle_goal_request(goal)
        return True

    @rpc
    def get_state(self) -> NavigationState:
        return self._planner.get_state()

    @rpc
    def is_goal_reached(self) -> bool:
        return self._planner.is_goal_reached()

    @rpc
    def cancel_goal(self) -> bool:
        self._planner.cancel_goal()
        return True

    @rpc
    def set_replanning_enabled(self, enabled: bool) -> None:
        self._planner.set_replanning_enabled(enabled)

    @rpc
    def set_safe_goal_clearance(self, clearance: float) -> None:
        self._planner.set_safe_goal_clearance(clearance)

    @rpc
    def reset_safe_goal_clearance(self) -> None:
        self._planner.reset_safe_goal_clearance()
