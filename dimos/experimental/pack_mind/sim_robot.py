# Copyright 2026 Dimensional Inc.
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

"""PACK MIND sim — SimRobot: one autonomous agent in the sim loop.

Uses CoveragePatrolRouter (scores candidate goals by NEW coverage along the A*
path) instead of FrontierPatrolRouter (which thrashed: farthest-point goals laid
thin re-walked trails and plateaued ~41%). The router reads its `_visitation`
mask — shared across robots in PACK mode, private in INDEPENDENT mode — so the
only thing that changes between the two experiments is whether memory is shared.
"""

from __future__ import annotations

from math import hypot

from dimos.experimental.pack_mind.world import pose_at
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.patrolling.routers.coverage_patrol_router import CoveragePatrolRouter
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

_MAX_GOAL_FAILURES = 3  # consecutive next_goal/plan failures before the robot stops


class SimRobot:
    """One simulated robot: coverage router + A* path follower."""

    def __init__(
        self,
        name: str,
        router: CoveragePatrolRouter,
        world: OccupancyGrid,
        start_xy: tuple[float, float],
        step_m: float = 0.12,
    ) -> None:
        self.name = name
        self.router = router
        self.world = world
        self.step_m = step_m
        self.x, self.y = start_xy
        self.path: list[tuple[float, float]] | None = None
        self.idx = 0
        self.done = False
        self._failures = 0
        # Throttle: handle_occupancy_grid is rate-limited to once/60s, so call
        # it exactly once per router here at init.
        self.router.handle_occupancy_grid(world)
        self.router.handle_odom(pose_at(self.x, self.y))

    def _request_goal(self) -> bool:
        g = self.router.next_goal()
        if g is None:
            return False
        p = min_cost_astar(
            self.world,
            (g.position.x, g.position.y),
            (self.x, self.y),
            use_cpp=False,
        )
        if p is None or len(p.poses) == 0:
            return False
        self.path = [(po.position.x, po.position.y) for po in p.poses]
        self.idx = 0
        return True

    def tick(self) -> None:
        if self.done:
            return
        if self.path is None or self.idx >= len(self.path):
            if not self._request_goal():
                self._failures += 1
                if self._failures >= _MAX_GOAL_FAILURES:
                    self.done = True
                return
            self._failures = 0
        tx, ty = self.path[self.idx]
        dx, dy = tx - self.x, ty - self.y
        d = hypot(dx, dy)
        if d <= self.step_m or d == 0:
            self.x, self.y = tx, ty
            self.idx += 1
        else:
            self.x += self.step_m * dx / d
            self.y += self.step_m * dy / d
        self.router.handle_odom(pose_at(self.x, self.y))  # stamps the visited mask
