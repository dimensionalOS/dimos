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

import os

from dimos_lcm.std_msgs import Bool, String  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner


class ReplanningAStarPlanner(Module, NavigationInterface):
    odom: In[PoseStamped]  # TODO: Use TF.
    global_costmap: In[OccupancyGrid]
    goal_request: In[PoseStamped]
    target: In[PoseStamped]

    goal_reached: Out[Bool]
    navigation_state: Out[String]  # TODO: set it
    cmd_vel: Out[Twist]
    path: Out[Path]
    debug_navigation: Out[Image]

    _planner: GlobalPlanner
    _global_config: GlobalConfig

    def __init__(self, global_config: GlobalConfig | None = None) -> None:
        super().__init__()
        self._global_config = global_config or GlobalConfig()
        self._planner = GlobalPlanner(self._global_config)

    @rpc
    def start(self) -> None:
        super().start()

        unsub = self.odom.subscribe(self._planner.handle_odom)
        self._disposables.add(Disposable(unsub))

        unsub = self.global_costmap.subscribe(self._planner.handle_global_costmap)
        self._disposables.add(Disposable(unsub))

        unsub = self.goal_request.subscribe(self._planner.handle_goal_request)
        self._disposables.add(Disposable(unsub))

        unsub = self.target.subscribe(self._planner.handle_goal_request)
        self._disposables.add(Disposable(unsub))

        self._disposables.add(self._planner.path.subscribe(self.path.publish))

        self._disposables.add(self._planner.cmd_vel.subscribe(self.cmd_vel.publish))

        self._disposables.add(self._planner.goal_reached.subscribe(self.goal_reached.publish))

        if "DEBUG_NAVIGATION" in os.environ:
            self._disposables.add(
                self._planner.debug_navigation.subscribe(self.debug_navigation.publish)
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
