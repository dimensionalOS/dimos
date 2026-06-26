# Copyright 2025-2026 Dimensional Inc.
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

"""Intended for testing and experimenting only

Loop A* global planner: replans on every new occupancy grid.

``_on_costmap`` and ``_on_goal`` both call ``_replan``; the stored costmap, goal
(from ``clicked_point``), and robot start (from ``odom``) feed ``min_cost_astar``
and the resulting ``Path`` is published on ``path``. There is no goal-following
state machine here: a fresh grid means a fresh A* run, and the downstream
``dannav.HolonomicController`` tracks whatever path is published.
"""

from __future__ import annotations

import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import VectorLike
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SimpleGlobalPlannerConfig(ModuleConfig):
    goal_tolerance: float = 0.2


class SimpleGlobalPlanner(Module):
    config: SimpleGlobalPlannerConfig

    global_costmap: In[OccupancyGrid]
    clicked_point: In[PointStamped]
    odom: In[PoseStamped]

    path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._costmap: OccupancyGrid | None = None
        self._goal: PoseStamped | None = None
        self._odom: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self.register_disposable(
            Disposable(
                self.clicked_point.subscribe(
                    lambda point: self._on_goal(point.to_pose_stamped())
                )
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    def plan(
        self,
        costmap: OccupancyGrid,
        goal: PoseStamped,
        start: VectorLike,
    ) -> Path | None:
        return min_cost_astar(costmap, goal.position, start)

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._odom = msg

    def _on_costmap(self, costmap: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = costmap
        self._replan()

    def _on_goal(self, goal: PoseStamped) -> None:
        logger.info("New goal", x=round(goal.x, 3), y=round(goal.y, 3))
        with self._lock:
            self._goal = goal
        self._replan()

    def _replan(self) -> None:
        with self._lock:
            costmap = self._costmap
            goal = self._goal
            odom = self._odom

        if costmap is None or goal is None or odom is None:
            return

        start = (float(odom.position.x), float(odom.position.y))
        if goal.position.distance(odom.position) < self.config.goal_tolerance:
            return

        path = self.plan(costmap, goal, start)
        if path is None or not path.poses:
            logger.warning(
                "No path found.",
                goal_x=round(goal.x, 3),
                goal_y=round(goal.y, 3),
            )
            return

        self.path.publish(path)
