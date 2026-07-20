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

"""Planner: A* over synthetic costmaps through over()."""

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.pure.modules.planner import Planner


def _grid(cells: np.ndarray, ts: float) -> OccupancyGrid:
    cm = OccupancyGrid(grid=cells.astype(np.int8), resolution=0.1, frame_id="world")
    cm.ts = ts
    return cm


def _goal(x: float, y: float, ts: float) -> PoseStamped:
    g = PoseStamped(frame_id="world", position=[x, y, 0.0])
    g.ts = ts
    return g


def _at(x: float, y: float, *ts: float) -> list[Any]:
    """world <- base_link tf holding the robot at (x, y), sampled at each ts."""
    out = []
    for t in ts:
        tf = Transform(translation=Vector3(x, y, 0.0), frame_id="world", child_frame_id="base_link")
        tf.ts = t  # ctor swaps an explicit ts=0.0 for wall clock; force the sample ts
        out.append(tf)
    return out


def test_plans_robot_to_goal_over_free_grid() -> None:
    m = Planner()
    cm = _grid(np.zeros((20, 20)), ts=1.0)
    rows = list(m.over(costmap=[cm], goal=[_goal(1.5, 0.5, ts=0.5)], tf=_at(0.2, 0.2, 0.0, 2.0)))
    (row,) = rows
    poses = row.path.poses
    assert row.path.frame_id == "world"
    assert (round(poses[0].x, 2), round(poses[0].y, 2)) == (0.2, 0.2)  # starts at the robot
    assert (round(poses[-1].x, 2), round(poses[-1].y, 2)) == (1.5, 0.5)  # ends at the goal


def test_unreachable_goal_skips_tick() -> None:
    # a full wall at column x=10 with no gap seals the goal off
    cells = np.zeros((20, 20))
    cells[:, 10] = 100
    m = Planner()
    rows = list(
        m.over(
            costmap=[_grid(cells, ts=1.0)],
            goal=[_goal(1.5, 0.5, ts=0.5)],
            tf=_at(0.2, 0.2, 0.0, 2.0),
        )
    )
    assert rows == []  # no path -> the tick emits nothing


def test_no_goal_yet_emits_nothing() -> None:
    # required latest goal: ticks drop until a goal has been seen
    m = Planner()
    rows = list(
        m.over(costmap=[_grid(np.zeros((20, 20)), ts=1.0)], goal=[], tf=_at(0.2, 0.2, 0.0, 2.0))
    )
    assert rows == []


def test_replans_each_costmap() -> None:
    # two costmaps on one goal -> two fresh plans (the replanning trigger is the costmap)
    m = Planner()
    cms = [_grid(np.zeros((20, 20)), ts=1.0), _grid(np.zeros((20, 20)), ts=2.0)]
    rows = list(
        m.over(costmap=cms, goal=[_goal(1.5, 0.5, ts=0.5)], tf=_at(0.2, 0.2, 0.0, 1.5, 2.5))
    )
    assert [r.ts for r in rows] == [1.0, 2.0]
