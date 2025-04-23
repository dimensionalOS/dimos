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

    def walk_loop(self, path: Path) -> bool:
        """Navigate through a path of waypoints.
        
        This method now passes the entire path to the local planner at once,
        utilizing the waypoint following capabilities.
        
        Args:
            path: Path object containing waypoints
            
        Returns:
            bool: True if successfully reached the goal, False otherwise
        """
        if not path or len(path) == 0:
            logger.warning("Cannot follow empty path")
            return False
            
        logger.info(f"Following path with {len(path)} waypoints")
        
        # Use the robot's waypoint navigation capability
        result = self.robot.navigate_path_local(path)
        
        if not result:
            logger.warning("Failed to navigate the path")
            return False
            
        logger.info("Successfully reached the goal")
        return True

    def set_goal(self, goal: VectorLike):
        """Plan and navigate to a goal position.
        
        Args:
            goal: Goal position as a vector-like object
            
        Returns:
            bool: True if planning and navigation succeeded, False otherwise
        """
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
        return astar(Costmap.from_msg(self.costmap()).smudge(kernel_size=3), goal, pos)
