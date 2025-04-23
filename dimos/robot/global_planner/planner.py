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

import logging

from dataclasses import dataclass
from abc import ABC, abstractmethod

from dimos.robot.robot import Robot
from dimos.types.vector import VectorLike, to_vector
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from nav_msgs import msg

logger = setup_logger("dimos.robot.unitree.global_planner", level=logging.DEBUG)


@dataclass
class Planner(ABC):
    robot: Robot

    @abstractmethod
    def plan(self, goal: VectorLike) -> Path: ...

    # actually we might want to rewrite this into rxpy
    def walk_loop(self, path: Path) -> bool:
        # pop the next goal from the path
        local_goal = path.head()
        print("path head", local_goal)
        result = self.robot.navigate_to_goal_local(local_goal.to_list(), is_robot_frame=False)

        if not result:
            # do we need to re-plan here?
            logger.warning("Failed to navigate to the local goal.")
            return False

        # get the rest of the path (btw here we can globally replan also)
        tail = path.tail()
        print("path tail", tail)
        if not tail:
            logger.info("Reached the goal.")
            return True

        # continue walking down the rest of the path
        # does python support tail calling haha?
        self.walk_loop(tail)

    def set_goal(self, goal: VectorLike):
        goal = to_vector(goal).to_2d()
        path = self.plan(goal)
        if not path:
            logger.warning("No path found to the goal.")
            return False

        return self.walk_loop(path)


class AstarPlanner(Planner):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        self.costmap = self.robot.ros_control.topic_latest("map", msg.OccupancyGrid)

    def start(self):
        return self

    def stop(self):
        if hasattr(self, "costmap"):
            self.costmap.dispose()
            del self.costmap

    def plan(self, goal: VectorLike) -> Path:
        [pos, rot] = self.robot.ros_control.transform_euler("base_link")
        return astar(Costmap.from_msg(self.costmap()).smudge(), goal, pos)
