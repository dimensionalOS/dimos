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
from abc import abstractmethod
from typing import Tuple, Callable

from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.types import Drawable, Visualizable
from reactivex.subject import Subject
from reactivex import Observable

logger = setup_logger("dimos.robot.unitree.global_planner", level=logging.DEBUG)


@dataclass
class Planner(Visualizable):
    local_nav: Callable[[VectorLike], bool]

    @abstractmethod
    def plan(self, goal: VectorLike) -> Path: ...

    def start(self):
        return self

    def stop(self):
        if hasattr(self, "costmap"):
            self.costmap.dispose()
            del self.costmap
        if hasattr(self, "_vis_subject"):
            self._vis_subject.dispose()

    def vis_stream(self) -> Observable[Tuple[str, Drawable]]:
        # check if we have self._vis_subject
        if not hasattr(self, "_vis_subject"):
            self._vis_subject = Subject()
        return self._vis_subject

    def vis(self, name: str, drawable: Drawable) -> None:
        if not hasattr(self, "_vis_subject"):
            return
        self._vis_subject.on_next((name, drawable))

    # actually we might want to rewrite this into rxpy
    def walk_loop(self, path: Path) -> bool:
        # pop the next goal from the path
        local_goal = path.head()
        print("path head", local_goal)
        result = self.local_nav(local_goal)

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
        costmap.save_pickle("costmap3.pickle")
        smudge = costmap.smudge()
        self.vis("global_costmap", smudge)
        self.vis("pos", pos)
        path = astar(smudge, goal, pos)
        if path:
            path = path.resample(0.25)
            self.vis("global_target", path)
        return path
