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

"""Replanning global planner: A* a fresh path over each new costmap.

Stateless — the costmap tick is the replan trigger; ``goal`` is the latest
requested destination and ``position`` the robot pose sampled from tf at the
costmap's ts. Each tick runs :func:`min_cost_astar` (the C++-FFI A*, Python
fallback) from the robot to the goal and emits a ``Path``; a tick with no
reachable path (or a goal off the grid) skips silently, leaving the last path
standing. Both endpoints resolve in ``frame_id`` — the costmap's frame — so the
plan is frame-consistent with the map it was cut from.
"""

from __future__ import annotations

from dimos import pure as pm
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

__all__ = ["Planner"]


class Planner(pm.PureModule):
    """A* a path from the robot to the goal point over each new costmap."""

    frame_id: str = "world"  # plan frame; costmap, goal, and position all live here
    body_frame: str = "base_link"  # robot frame; position samples frame_id <- body_frame
    cost_threshold: int = 100  # cells at/above this cost are impassable
    unknown_penalty: float = 0.8  # unknown-cell cost as a fraction of cost_threshold

    class In(pm.In):
        costmap: OccupancyGrid = pm.tick(expect_hz=2)
        position: Transform = pm.tf("{frame_id}", "{body_frame}")  # robot pose at tick ts
        goal_point: PointStamped | None = pm.latest(default=None)  # destination; silent until set

    class Out(pm.Out):
        path: Path  # no cadence contract: the planner is silent with no goal / no path

    def step(self, i: In) -> Out | None:
        """Plan robot -> goal_point on this costmap; skip the tick with no goal or no path."""
        if i.goal_point is None:
            return None  # no destination yet
        path = min_cost_astar(
            i.costmap,
            goal=Vector3(i.goal_point.x, i.goal_point.y, i.goal_point.z),
            start=i.position.translation,
            cost_threshold=self.cost_threshold,
            unknown_penalty=self.unknown_penalty,
        )
        return None if path is None else Planner.Out(path=path)
