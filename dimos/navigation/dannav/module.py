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
from typing import Any, Literal

from dimos_lcm.std_msgs import Bool, String
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
from dimos.navigation.dannav.global_planner import GlobalPlanner
from dimos.navigation.holonomic_trajectory_controller.trajectory_run_profiles import GO2_RUN_PROFILES, RunProfileError
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DannavPlannerConfig(ModuleConfig):
    robot_width: float | None = None
    robot_rotation_diameter: float | None = None
    replan_on_costmap_update: bool = False
    path_controller: Literal["differential", "holonomic"] | None = None


class DannavPlanner(Module, NavigationInterface):
    config: DannavPlannerConfig

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
    navigation_costmap: Out[OccupancyGrid]

    _planner: GlobalPlanner

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        overrides = {
            name: value
            for name, value in (
                ("robot_width", self.config.robot_width),
                ("robot_rotation_diameter", self.config.robot_rotation_diameter),
                ("local_planner_path_controller", self.config.path_controller or "holonomic"),
            )
            if value is not None
        }
        effective_global_config = (
            self.config.g.model_copy(update=overrides) if overrides else self.config.g
        )
        self._planner = GlobalPlanner(
            effective_global_config,
            replan_on_costmap_update=self.config.replan_on_costmap_update,
        )
        logger.info(
            "DannavPlanner configured.",
            path_controller=effective_global_config.local_planner_path_controller,
            replan_on_costmap_update=self.config.replan_on_costmap_update,
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
    def set_goal(self, goal: PoseStamped, profile: str | None = None) -> bool:
        self._planner.handle_goal_request(goal, run_profile=profile)
        return True

    @rpc
    def set_run_profile(self, profile: str) -> bool:
        """Set the session-default movement envelope for subsequent goals.

        Validated against the run-profile registry so a bad name cannot poison
        the planner config.
        """
        try:
            GO2_RUN_PROFILES.get(profile)
        except RunProfileError as exc:
            logger.warning("Rejected run profile.", profile=profile, reason=str(exc))
            return False
        self._planner.set_run_profile(profile)
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
