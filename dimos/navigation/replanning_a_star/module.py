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

from dataclasses import dataclass
import math
import os
from typing import Any

from dimos_lcm.std_msgs import Bool, String
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import PointStamped, PoseStamped, Twist
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner


@dataclass
class Config(ModuleConfig):
    max_linear_speed: float = 0.55               # Max linear velocity (m/s)
    max_angular_speed: float = 0.55              # Max angular velocity (rad/s)
    control_frequency: float = 10                # Command loop rate (Hz)
    k_angular: float = 0.5                       # Proportional gain for yaw correction
    rotation_threshold: float = math.radians(90) # Above: rotate in place. Below: drive+rotate

    def __post_init__(self) -> None:
        if self.max_linear_speed <= 0:
            raise ValueError(f"max_linear_speed must be positive, got {self.max_linear_speed}")
        if self.max_angular_speed <= 0:
            raise ValueError(f"max_angular_speed must be positive, got {self.max_angular_speed}")
        if self.control_frequency <= 0:
            raise ValueError(f"control_frequency must be positive, got {self.control_frequency}")
        if self.k_angular <= 0:
            raise ValueError(f"k_angular must be positive, got {self.k_angular}")
        if self.rotation_threshold <= 0:
            raise ValueError(f"rotation_threshold must be positive, got {self.rotation_threshold}")


class ReplanningAStarPlanner(Module, NavigationInterface):
    default_config = Config
    config: Config

    odom: In[PoseStamped]  # TODO: Use TF.
    global_costmap: In[OccupancyGrid]
    goal_request: In[PoseStamped]
    clicked_point: In[PointStamped]
    target: In[PoseStamped]

    goal_reached: Out[Bool]
    navigation_state: Out[String]  # TODO: set it
    cmd_vel: Out[Twist]
    path: Out[Path]
    navigation_costmap: Out[OccupancyGrid]

    _planner: GlobalPlanner

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._planner = GlobalPlanner(self.config.g)

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(Disposable(self.odom.subscribe(self._planner.handle_odom)))
        self._disposables.add(
            Disposable(self.global_costmap.subscribe(self._planner.handle_global_costmap))
        )
        self._disposables.add(
            Disposable(self.goal_request.subscribe(self._planner.handle_goal_request))
        )
        self._disposables.add(Disposable(self.target.subscribe(self._planner.handle_goal_request)))

        self._disposables.add(
            Disposable(
                self.clicked_point.subscribe(
                    lambda pt: self._planner.handle_goal_request(pt.to_pose_stamped())
                )
            )
        )

        self._disposables.add(self._planner.path.subscribe(self.path.publish))

        self._disposables.add(self._planner.cmd_vel.subscribe(self.cmd_vel.publish))

        self._disposables.add(self._planner.goal_reached.subscribe(self.goal_reached.publish))

        if "DEBUG_NAVIGATION" in os.environ:
            self._disposables.add(
                self._planner.navigation_costmap.subscribe(self.navigation_costmap.publish)
            )

        self._planner.start()

    @rpc
    def stop(self) -> None:
        self.cancel_goal()
        self._planner.stop()

        super().stop()

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


replanning_a_star_planner = ReplanningAStarPlanner.blueprint

__all__ = ["ReplanningAStarPlanner", "replanning_a_star_planner"]
