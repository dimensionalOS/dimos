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

from dataclasses import dataclass
from abc import abstractmethod
from typing import Callable, Optional
import threading

from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class Planner(Visualizable):
    set_local_nav: Callable[[Path, Optional[threading.Event]], bool]

    @abstractmethod
    def plan(self, goal: VectorLike) -> Path: ...

    def set_goal(
        self, goal: VectorLike, goal_theta: Optional[float] = None, stop_event: Optional[threading.Event] = None
    ):
        goal = to_vector(goal).to_2d()
        path = self.plan(goal)
        if not path:
            logger.warning("No path found to the goal.")
            return False
        return self.set_local_nav(path, stop_event=stop_event, goal_theta=goal_theta)


@dataclass
class AstarPlanner(Planner):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Vector]
    set_local_nav: Callable[[Path], bool]
    conservativism: int = 3

    def plan(self, goal: VectorLike) -> Path:
        pos = self.get_robot_pos()
        costmap = self.get_costmap().smudge(iterations=self.conservativism)

        self.vis("planner_costmap", costmap)
        self.vis("target", goal)

        path = astar(costmap, goal, pos)

        if path:
            path = path.resample(0.1)
            self.vis("a*", path)
            return path

        logger.warning("No path found to the goal.")
