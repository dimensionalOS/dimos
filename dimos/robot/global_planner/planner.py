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

import threading
from abc import abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional

from dimos.core import In, Module, Out
from dimos.msgs.geometry_msgs import Vector3
from dimos.robot.global_planner.algo import astar
from dimos.types.costmap import Costmap
from dimos.types.path import Path
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class Planner(Visualizable, Module):
    target: In[Vector3] = None
    path: Out[Path] = None

    def __init__(self):
        Module.__init__(self)
        Visualizable.__init__(self)

    # def set_goal(
    #     self,
    #     goal: VectorLike,
    #     goal_theta: Optional[float] = None,
    #     stop_event: Optional[threading.Event] = None,
    # ):
    #     path = self.plan(goal)
    #     if not path:
    #         logger.warning("No path found to the goal.")
    #         return False

    #     print("pathing success", path)
    #     return self.set_local_nav(path, stop_event=stop_event, goal_theta=goal_theta)


class AstarPlanner(Planner):
    target: In[Vector3] = None
    path: Out[Path] = None

    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Vector3]

    conservativism: int = 8

    def __init__(
        self,
        get_costmap: Callable[[], Costmap],
        get_robot_pos: Callable[[], Vector3],
    ):
        super().__init__()
        self.get_costmap = get_costmap
        self.get_robot_pos = get_robot_pos

    async def start(self):
        self.target.subscribe(self.plan)

    def plan(self, goal: VectorLike) -> Path:
        print("planning path to goal", goal)
        goal = to_vector(goal).to_2d()
        pos = self.get_robot_pos()
        print("current pos", pos)
        costmap = self.get_costmap().smudge()

        print("current costmap", costmap)
        self.vis("target", goal)

        print("ASTAR ", costmap, goal, pos)
        path = astar(costmap, goal, pos)

        if path:
            path = path.resample(0.1)
            self.vis("a*", path)
            self.path.publish(path)
            return path
        logger.warning("No path found to the goal.")
