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

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, PoseLike, PoseStamped, Vector3, to_pose
from dimos.robot.global_planner.algo import astar
from dimos.types.costmap import Costmap
from dimos.types.path import Path
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class Planner(Visualizable, Module):
    target: In[PoseStamped] = None
    path: Out[Path] = None

    def __init__(self):
        Module.__init__(self)
        Visualizable.__init__(self)

    @rpc
    def set_goal(
        self,
        goal: VectorLike,
        goal_theta: Optional[float] = None,
        stop_event: Optional[threading.Event] = None,
    ):
        path = self.plan(goal)
        if not path:
            logger.warning("No path found to the goal.")
            return False

        print("pathing success", path)


class AstarPlanner(Planner):
    target: In[Vector3] = None
    path: Out[Path] = None

    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Vector3]
    set_local_nav: Callable[[Path, Optional[threading.Event], Optional[float]], bool] = None

    conservativism: int = 8

    def __init__(
        self,
        get_costmap: Callable[[], Costmap],
        get_robot_pos: Callable[[], Vector3],
        set_local_nav: Callable[[Path, Optional[threading.Event], Optional[float]], bool] = None,
    ):
        super().__init__()
        self.get_costmap = get_costmap
        self.get_robot_pos = get_robot_pos
        self.set_local_nav = set_local_nav

    @rpc
    def start(self):
        self.target.subscribe(self.plan)

    def plan(self, goallike: PoseLike) -> Path:
        goal = to_pose(goallike)
        logger.info(f"planning path to goal {goal}")
        pos = self.get_robot_pos()
        print("current pos", pos)
        costmap = self.get_costmap()

        print("current costmap", costmap)

        self.vis("target", goal)

        path = astar(costmap, goal.position, pos)

        if path:
            path = path.resample(0.1)
            self.vis("a*", path)
            self.path.publish(path)
            if hasattr(self, "set_local_nav") and self.set_local_nav:
                self.set_local_nav(path)
            return path
        logger.warning("No path found to the goal.")
