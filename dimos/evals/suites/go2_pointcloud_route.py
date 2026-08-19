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


"""Route reachability VQA over the go2 replays.

Rows (``go2_pointcloud_route_vqa.json``) are pure data emitted by
:func:`rows` — ground truth computed by the navigation stack the robot
actually uses (``height_cost_occupancy`` + ``NavigationMap`` +
``min_cost_astar``), wired the way ``GlobalPlanner._find_wide_path`` wires it,
quizzing whatever lossy encoding the agent receives for a ``PointCloud2``.

``route`` asks whether a goal 3 m out is reachable. Straight-line clearance
(:mod:`dimos.evals.suites.go2_pointcloud_clearance`) is half of what a robot
asks; the other half is whether a route exists when the direct line does not.
Every case has a blocked direct line, so a straight-line reader answers them
all "blocked" and is right on exactly the third that are.

The planner runs twice per goal: ``unknown_penalty=1.0`` makes unmeasured
cells impassable, ``0.8`` (production) lets the search cross them. Reachable
under both is ``reachable``, unreachable under both is ``blocked``, and the
gap between them is ``unknown`` — a route that exists only if unmeasured
space happens to be clear. That third answer is the one an obstacle-list
encoding cannot express.

Rows are sliced train / holdout / spare by :mod:`dimos.evals.split`.

Regenerate (needs both recordings)::

    python -m dimos.evals.suites.go2_pointcloud_route
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.evals import generate, split
from dimos.evals.types import Suite
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap

_JSON = Path(__file__).parent / "go2_pointcloud_route_vqa.json"

SUITE: Suite = generate.cases(
    split.assign(json.loads(_JSON.read_text())), tags=frozenset({"pointcloud"})
)

_SHORT_TS = [3.0 + i * 2.0 for i in range(29)]
_OFFICE_TS = [22.0 + i * 3.0 for i in range(38)]


# -- routing ---------------------------------------------------------------------
#
# Straight-line clearance is half of what a robot asks; the other half is whether
# a route exists when the direct line does not. Ground truth is the stack the
# robot actually navigates with, wired the way `GlobalPlanner._find_wide_path`
# wires it: height-cost occupancy (per-cell height *step*, so a desk with clear
# floor under it is passable rather than a wall), a voronoi gradient costmap
# inflated by robot width, the robot's own footprint cleared, then `min_cost_astar`.
# The verdict is "would our planner get there", not "does some hand-rolled flood
# fill get there".

GOAL_RANGE = 3.0  # how far out the goal sits, meters
GRID_RES = 0.05  # OccupancyConfig default
ROBOT_INCREASE = 1.1  # GlobalPlanner._find_wide_path sizes_to_try
MIN_LETHAL_CELLS = 3  # a grazed corner is not a blocked line


def _plan_costmap(cloud: Any, origin: np.ndarray) -> Any:
    """The costmap the global planner would plan this frame on.

    Mirrors ``GlobalPlanner._find_wide_path`` for a goal beyond 1.5 m: the
    voronoi navigation map at ``ROBOT_INCREASE`` robot widths, with the cells
    under the robot dropped to high-but-passable. Without that last step a
    robot standing inside a fresh obstacle's inflation envelope has no passable
    start neighbour and every goal scores "blocked".
    """
    config = GlobalConfig()
    nav = NavigationMap(config, "voronoi")
    nav.update(height_cost_occupancy(cloud, resolution=GRID_RES))
    costmap = nav.make_gradient_costmap(ROBOT_INCREASE)
    binary = nav.binary_costmap
    if binary.grid.shape == costmap.grid.shape and binary.origin == costmap.origin:
        center = costmap.world_to_grid(Vector3(origin[0], origin[1], 0.0))
        cx, cy = int(center.x), int(center.y)
        cells = int(config.robot_rotation_diameter / 2 / costmap.resolution) + 1
        h, w = costmap.grid.shape
        y0, y1 = max(0, cy - cells), min(h, cy + cells + 1)
        x0, x1 = max(0, cx - cells), min(w, cx + cells + 1)
        if y0 < y1 and x0 < x1:
            rows, cols = np.ogrid[y0:y1, x0:x1]
            disc = (rows - cy) ** 2 + (cols - cx) ** 2 <= cells**2
            region = costmap.grid[y0:y1, x0:x1]
            clearable = (
                disc
                & (region >= CostValues.OCCUPIED)
                & (binary.grid[y0:y1, x0:x1] < CostValues.OCCUPIED)
            )
            region[clearable] = CostValues.OCCUPIED - 1
    return costmap


def _straight_blocked(grid: Any, start: np.ndarray, goal: np.ndarray) -> bool:
    """Does the direct segment robustly cross lethal cells?

    Sampled at half-cell steps, and it takes ``MIN_LETHAL_CELLS`` distinct
    lethal cells to count. One grazed corner is not a blocked line — and a case
    whose premise turns on a single cell flips when the goal coordinate is
    rounded for printing, which would make the question text state something
    the geometry does not support.
    """
    steps = max(2, int(float(np.hypot(*(goal - start))) / (grid.resolution * 0.5)))
    lethal: set[tuple[int, int]] = set()
    for k in range(1, steps + 1):
        cell = grid.world_to_grid(start + (goal - start) * k / steps)
        x, y = int(cell.x), int(cell.y)
        if not (0 <= x < grid.width and 0 <= y < grid.height):
            return False  # segment leaves the map — not a clean blocked line
        if grid.grid[y, x] >= CostValues.OCCUPIED:
            lethal.add((x, y))
    return len(lethal) >= MIN_LETHAL_CELLS


def route_rows(
    dataset: str,
    timestamps: Sequence[float],
    *,
    goal_range: float = GOAL_RANGE,
) -> list[generate.Row]:
    """Can the robot reach a goal its straight line cannot?

    Only goals whose direct segment robustly crosses lethal cells become cases,
    so the family cannot be answered by a straight-line check — that check calls
    all of them blocked and is right only on the ones that are.

    The planner runs twice on the same costmap. ``unknown_penalty=1.0`` makes
    unmeasured cells impassable, so a path found there runs through
    confirmed-traversable space alone. The production ``0.8`` lets the search
    cross unmeasured space. Reachable under both is ``reachable``; unreachable
    under both is ``blocked``; the gap between them is ``unknown`` — a route
    that exists only if unmeasured space happens to be clear.

    Every candidate is returned.
    """
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            cloud, context = generate._frame_at(store, t)
            odom = store.streams.odom.range_time(0, t).to_list()[-1].data.position
            origin = np.array([float(odom.x), float(odom.y)])
            costmap = _plan_costmap(cloud, origin)
            for i, name in enumerate(generate.COMPASS):
                th = np.radians(i * 45.0)
                goal = origin + goal_range * np.array([np.cos(th), np.sin(th)])
                cell = costmap.world_to_grid(goal)
                if not (0 <= int(cell.x) < costmap.width and 0 <= int(cell.y) < costmap.height):
                    continue  # goal is off the mapped area — nothing to ask
                if not _straight_blocked(costmap, origin, goal):
                    continue  # direct line is clear — no detour to reason about
                strict = min_cost_astar(costmap, goal, origin, unknown_penalty=1.0)
                loose = min_cost_astar(costmap, goal, origin, unknown_penalty=0.8)
                label = (
                    "reachable"
                    if strict is not None
                    else ("unknown" if loose is not None else "blocked")
                )
                rows.append(
                    {
                        "id": f"{dataset}_route_t{t:g}_{name}",
                        "family": "route",
                        "type": "mcq",
                        "q": "You are the robot; your current pose is the odom "
                        "observation shown (world frame: +x is east, +y is north, "
                        "coordinates in meters). You want to reach the point at "
                        f"({goal[0]:.2f}, {goal[1]:.2f}), {goal_range:g} m due {name} of "
                        "you, and you may follow any route across the floor. Using only "
                        "the mapped point cloud, can you get there? Answer with exactly "
                        "one word: "
                        "reachable if some route reaches it entirely through space the "
                        "sensor confirmed is traversable; blocked if no route reaches it "
                        "even when space the sensor never measured is treated as "
                        "traversable; unknown if every route that reaches it has to cross "
                        "space the sensor never measured.",
                        "a": label,
                        "choices": ["reachable", "blocked", "unknown"],
                        "context": [
                            *context,
                            ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                        ],
                        "dataset": dataset,
                    }
                )
        return rows


# The dataset: an even answer mix, spread across both recordings.
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
