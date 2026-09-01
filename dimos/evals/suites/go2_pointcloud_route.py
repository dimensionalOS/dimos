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


"""Path-to-goal VQA over the go2 replays, in sensing terms.

Rows (``go2_pointcloud_route_vqa.json``) are pure data emitted by
:func:`rows` — ground truth a flood fill over the full-resolution cloud,
quizzing whatever lossy encoding the agent receives for a ``PointCloud2``.

``route`` asks whether a goal 3 m out can be reached along some path across
the floor, and what that path has to cross. Three answers, all defined by
what the lidar measured: ``measured`` if a path exists through cells that
were swept and hold nothing above 0.15 m; ``unmeasured`` if every path has to
cross cells with no return at all; ``blocked`` if no path exists even
counting unmeasured cells as passable. The 0.15 m edge and the 0.2 m margin
are this question's definitions, stated in the prompt; the encoder carries
neither.

Earlier this family's truth came from the navigation stack's planner and its
answers were ``reachable`` / ``unknown`` / ``blocked`` — words that ask the
encoder to be a planner. The frames and goals are unchanged; the truth is
now the sensing definition above, and 10 of 36 answers moved with it.


Regenerate (needs both recordings)::

    uv run python -m dimos.evals.suites.go2_pointcloud_route
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy import ndimage

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_route_vqa.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

_SHORT_TS = [3.0 + i * 2.0 for i in range(29)]
_OFFICE_TS = [22.0 + i * 3.0 for i in range(38)]


# -- paths ----------------------------------------------------------------------
#
# Straight-line clearance is half of what a robot asks; the other half is whether
# a path exists when the direct line does not. The truth here is a flood fill on
# a grid of what the lidar measured: a cell is passable when nothing above LOW_Z
# was returned within MARGIN of it, and it is swept when it holds a return at
# all. Two fills -- swept cells alone, then any passable cell -- give the three
# answers. No planner, no cost model, no robot-specific notion of traversable.

GOAL_RANGE = 3.0  # how far out the goal sits, meters
CELL = 0.1  # grid the path is traced on
SWEPT_CELL = 0.2  # grid "holds a return" is judged on; 0.1 m cells speckle
LOW_Z = 0.15  # returns above this are in the way
MARGIN = 0.2  # a path keeps this far from any return above LOW_Z
BODY = 0.35  # returns within this of the robot are its own body
CHOICES = ("measured", "unmeasured", "blocked")


def _question(goal: np.ndarray, name: str) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: "
        "+x is east, +y is north, coordinates in meters). You want to reach the point at "
        f"({goal[0]:.2f}, {goal[1]:.2f}), {GOAL_RANGE:g} m due {name} of you, along any "
        f"path across the floor. The floor is divided into {CELL:g} m cells. A cell is "
        f"passable when no lidar return above z = {LOW_Z} m lies within {MARGIN:g} m of its "
        f"centre (returns within {BODY:g} m of your own position are your body and do not "
        f"count). A passable cell is swept when the {SWEPT_CELL:g} m square containing it "
        f"(squares aligned to multiples of {SWEPT_CELL:g} m) holds at least one lidar return "
        "at any height, and unmeasured when that square holds none. A path is a chain of "
        "passable cells, each touching the next at an edge or a corner, from your position "
        "to the goal. Using only the mapped "
        "point cloud, answer with exactly one word: measured if some path runs through swept "
        "cells alone; unmeasured if every path has to cross unmeasured cells; blocked if there "
        "is no path even through unmeasured cells."
    )


def verdict(pts: np.ndarray, robot: np.ndarray, goal: np.ndarray) -> str:
    """``measured`` / ``unmeasured`` / ``blocked`` for a goal, by flood fill.

    The grid is widened to hold the goal, so a goal past the sensing window
    sits in unmeasured cells rather than off the map. Start and goal are
    taken as any cell within ``BODY`` / ``MARGIN`` of them. Connectivity is
    8-way; the blocked ring round a return is two cells thick, so a path
    cannot slip diagonally between two returns.
    """
    bounds = (np.minimum(robot, goal) - 0.5, np.maximum(robot, goal) + 0.5)
    origin, count, _, zmax = generate._cell_grid(pts, CELL, bounds=bounds)
    wx, wy = generate._cell_centers(origin, CELL, count.shape)
    from_robot = np.hypot(wx - robot[0], wy - robot[1])
    from_goal = np.hypot(wx - goal[0], wy - goal[1])
    obstacle = (zmax > LOW_Z) & (from_robot > BODY)
    passable = ndimage.distance_transform_edt(~obstacle) * CELL > MARGIN
    # swept is judged on the coarser grid, looked up per fine cell
    coarse_origin, coarse_count, _, _ = generate._cell_grid(pts, SWEPT_CELL, bounds=bounds)
    cj = np.floor((wy - coarse_origin[1]) / SWEPT_CELL).astype(np.int64)
    ci = np.floor((wx - coarse_origin[0]) / SWEPT_CELL).astype(np.int64)
    inside = (cj >= 0) & (cj < coarse_count.shape[0]) & (ci >= 0) & (ci < coarse_count.shape[1])
    held = np.zeros_like(passable)
    held[inside] = coarse_count[cj[inside], ci[inside]] > 0
    swept = passable & held
    start = from_robot <= BODY
    end = from_goal <= MARGIN
    eight = np.ones((3, 3), dtype=bool)

    def joined(mask: np.ndarray) -> bool:
        labels, _ = ndimage.label(mask, structure=eight)
        a = set(labels[mask & start].tolist()) - {0}
        b = set(labels[mask & end].tolist()) - {0}
        return bool(a & b)

    if joined(swept):
        return "measured"
    if joined(passable):
        return "unmeasured"
    return "blocked"


def route_rows(
    dataset: str,
    timestamps: Sequence[float],
    *,
    goal_range: float = GOAL_RANGE,
) -> list[generate.Row]:
    """What a path to a goal 3 m out has to cross, for every compass goal.

    Every candidate is returned; the committed set keeps the goals chosen
    when the family was first built, whose direct line the planner of the
    day found blocked, so a straight-line reader is not handed the answer.
    """
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            for i, name in enumerate(generate.COMPASS):
                th = np.radians(i * 45.0)
                goal = robot + goal_range * np.array([np.cos(th), np.sin(th)])
                rows.append(
                    {
                        "id": f"{dataset}_route_t{t:g}_{name}",
                        "family": "route",
                        "type": "mcq",
                        "q": _question(goal, name),
                        "a": verdict(pts, robot, goal),
                        "choices": list(CHOICES),
                        "context": [
                            *context,
                            ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                        ],
                        "dataset": dataset,
                    }
                )
        return rows


# The dataset: the 36 goals chosen when the family was first built, unchanged.
_CASES = (
    "go2_short_route_t7_southeast",
    "go2_short_route_t21_southwest",
    "go2_short_route_t31_northwest",
    "go2_short_route_t41_south",
    "go2_short_route_t53_south",
    "go2_china_office_route_t34_west",
    "go2_china_office_route_t61_southeast",
    "go2_china_office_route_t73_west",
    "go2_china_office_route_t85_southwest",
    "go2_china_office_route_t97_southeast",
    "go2_china_office_route_t121_northwest",
    "go2_china_office_route_t133_south",
    "go2_short_route_t3_east",
    "go2_short_route_t13_southeast",
    "go2_short_route_t31_west",
    "go2_short_route_t47_west",
    "go2_short_route_t57_east",
    "go2_china_office_route_t28_southwest",
    "go2_china_office_route_t46_west",
    "go2_china_office_route_t61_northwest",
    "go2_china_office_route_t79_southeast",
    "go2_china_office_route_t97_northwest",
    "go2_china_office_route_t121_northeast",
    "go2_china_office_route_t133_southeast",
    "go2_short_route_t5_north",
    "go2_short_route_t13_west",
    "go2_short_route_t21_north",
    "go2_short_route_t29_east",
    "go2_short_route_t37_northeast",
    "go2_short_route_t51_south",
    "go2_china_office_route_t25_south",
    "go2_china_office_route_t37_southeast",
    "go2_china_office_route_t55_northeast",
    "go2_china_office_route_t67_northeast",
    "go2_china_office_route_t106_west",
    "go2_china_office_route_t130_west",
)


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    candidates = {
        r["id"]: r
        for r in (
            *route_rows("go2_short", _SHORT_TS),
            *route_rows("go2_china_office", _OFFICE_TS),
        )
    }
    return [candidates[i] for i in _CASES]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
