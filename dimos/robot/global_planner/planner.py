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
from typing import Tuple, Callable, Optional
import threading

from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class Planner(Visualizable):
    local_nav: Callable[[Path, Optional[threading.Event]], bool]

    @abstractmethod
    def plan(self, goal: VectorLike) -> Path: ...

    def set_goal(self, goal: VectorLike, stop_event: Optional[threading.Event] = None):
        goal = to_vector(goal).to_2d()
        path = self.plan(goal)
        if not path:
            logger.warning("No path found to the goal.")
            return False

        return self.local_nav(path.resample(1.0), stop_event=stop_event)


class AstarPlanner(Planner):
    def __init__(
        self,
        costmap: Callable[[], Costmap],
        base_link: Callable[[], Tuple[Vector, Vector]],
        local_nav: Callable[[Vector], bool],
    ):
        super().__init__(local_nav)
        self.base_link = base_link
        self.costmap = costmap

    def plan(self, goal: VectorLike) -> Path:
        [pos, rot] = self.base_link()
        costmap = self.costmap()
        smudge = costmap.smudge()

        self.vis("planner_costmap", smudge)
        self.vis("target", goal)

        path = astar(smudge, goal, pos)

        if path:
            path = path.resample(0.5)
            self.vis("a*", path)
            return path

        logger.warning("No path found to the goal.")
