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
from typing import Callable, Optional, List
import threading
import os
import time

from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable
from dimos.robot.frontier_exploration.utils import CostmapSaver

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

        current_costmap = self.get_costmap()
        self.costmap_saver.save_costmap(current_costmap)

        navigation_successful = self.set_local_nav(
            path, stop_event=stop_event, goal_theta=goal_theta
        )

        next_goal = self.get_frontiers()
        if next_goal:
            self.vis("frontier_goal", next_goal)

        return navigation_successful


@dataclass
class AstarPlanner(Planner):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Vector]
    set_local_nav: Callable[[Path], bool]
    get_frontiers: Optional[Callable[[], Vector]] = None
    conservativism: int = 8
    save_costmaps: bool = False
    costmap_save_dir: Optional[str] = None

    def __post_init__(self):
        """Initialize costmap saver if saving is enabled."""
        if self.save_costmaps and self.costmap_save_dir:
            self.costmap_saver = CostmapSaver(self.costmap_save_dir)
        else:
            self.costmap_saver = None

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
