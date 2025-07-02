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
from dimos.robot.module_utils import robot_module
from dimos.robot.capabilities import Move, Lidar, Odometry
from abc import abstractmethod
from typing import Callable, Optional
import threading

from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable
from dimos.robot.local_planner.local_planner import navigate_path_local
from dimos.utils.reactive import getter_streaming
from dimos.robot.unitree_webrtc.type.map import Map

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class Planner(Visualizable):
    set_local_nav: Callable[[Path, Optional[threading.Event]], bool]

    @abstractmethod
    def plan(self, goal: VectorLike) -> Path: ...

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
        return self.set_local_nav(path, stop_event=stop_event, goal_theta=goal_theta)


@robot_module
class AstarPlanner(Planner):
    REQUIRES = (Move, Lidar, Odometry)

    def __init__(self, conservativism: int = 8):
        self.conservativism = conservativism

    def setup(self, robot):
        # Build lambdas that access robot state lazily
        self.get_costmap = lambda: Map(voxel_size=0.2).costmap()
        self.get_robot_pos = lambda: getter_streaming(robot.odom_stream()).pos
        self.set_local_nav = lambda path, stop_event=None, goal_theta=None: navigate_path_local(
            robot, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event
        )

    def plan(self, goal: VectorLike) -> Path:
        goal = to_vector(goal).to_2d()
        pos = self.get_robot_pos().to_2d()
        costmap = self.get_costmap().smudge()

        # self.vis("costmap", costmap)
        self.vis("target", goal)

        print("ASTAR ", costmap, goal, pos)
        path = astar(costmap, goal, pos)

        if path:
            path = path.resample(0.1)
            self.vis("a*", path)
            return path

        logger.warning("No path found to the goal.")
