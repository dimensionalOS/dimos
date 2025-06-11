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
        navigation_successful = self.set_local_nav(
            path, stop_event=stop_event, goal_theta=goal_theta
        )

        # Provide feedback to frontier explorer if this was a frontier goal
        if hasattr(self, "get_frontiers") and self.get_frontiers is not None:
            # Mark the goal as attempted (success or failure) for frontier persistence
            try:
                # Get the frontier explorer instance from the get_frontiers callable
                import inspect

                if hasattr(self.get_frontiers, "__self__"):
                    frontier_explorer = self.get_frontiers.__self__
                    if hasattr(frontier_explorer, "mark_frontier_attempted"):
                        frontier_explorer.mark_frontier_attempted(goal, True)
            except Exception as e:
                logger.warning(f"Could not provide frontier feedback: {e}")

        # AFTER navigation is complete, detect and visualize frontiers for future exploration
        if (
            navigation_successful
            and hasattr(self, "get_frontiers")
            and self.get_frontiers is not None
        ):
            try:
                logger.info("Goal reached! Detecting frontiers for future exploration...")
                frontiers = self.get_frontiers()
                if frontiers:
                    # Visualize all detected frontiers
                    for i, frontier in enumerate(frontiers):
                        self.vis(f"frontier_{i}", frontier)
                    logger.info(
                        f"Post-goal completion: Detected {len(frontiers)} frontiers for exploration"
                    )
                else:
                    logger.info("Post-goal completion: No frontiers detected")
            except Exception as e:
                logger.error(f"Failed to detect frontiers after goal completion: {e}")

        return navigation_successful


@dataclass
class AstarPlanner(Planner):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Vector]
    set_local_nav: Callable[[Path], bool]
    get_frontiers: Optional[Callable[[], List[Vector]]] = None
    conservativism: int = 8

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
