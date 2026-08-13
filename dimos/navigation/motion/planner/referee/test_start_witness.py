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

"""A pose the robot actually occupies may always be departed.

The SE(2) search names its seed node by the cell the start snaps to, and used
to read that cell's clearance to decide whether the start was feasible at all.
The snap moves the body by up to half a cell diagonal (~85 mm), which is enough
to veto a start whose real pose is fine — `door_side` is the world that caught
it (true pose 0.083 m of union clearance against a 0.05 m margin, snapped cell
0.043 m) and gold refused a route that exists.

Both sides of the rule are pinned here: the pose decides, and a pose that is
genuinely not feasible still refuses.

The shape the seed reads is the STANDING body since planner/revision.md's
standing-witness amendment — the intersection of the envelope's rows, not the
union of the swept walking boxes. That is 0.089 m narrower per side than the
union laterally, which is more than the ~0.085 m a snap can move the body, so
`door_side` no longer witnesses the snap and a wall BEHIND does: the standing
box is only 0.010 m shorter than the union at the back.
"""

from __future__ import annotations

import math

import numpy as np

from dimos.navigation.motion.embodiment import GO2, box_offsets
from dimos.navigation.motion.scenarios import CELL, SCENARIOS, Box, se2_path

DOOR_SIDE = next(sc for sc in SCENARIOS if sc.name == "door_side")


def union_clear(boxes: list[Box], pose: tuple[float, float, float]) -> float:
    """Exact all-gait-union clearance at a pose — no grid, no lattice snap."""
    return _clear(boxes, pose, GO2.offsets())


def stand_clear(boxes: list[Box], pose: tuple[float, float, float]) -> float:
    """The same reading on the STANDING body, which is what the seed uses."""
    return _clear(boxes, pose, box_offsets(GO2.stand_box()))


def _clear(boxes: list[Box], pose: tuple[float, float, float], off: np.ndarray) -> float:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    pts = np.column_stack(
        [
            pose[0] + c * off[:, 0] - s * off[:, 1],
            pose[1] + s * off[:, 0] + c * off[:, 1],
            np.zeros(len(off)),
        ]
    )
    return float(np.min([b.sdf2d(pts) for b in boxes]))


def test_a_pose_the_robot_occupies_may_be_departed() -> None:
    """The cell would have refused; the pose the robot is in did not.

    A wall behind, and a start half a cell in front of the lattice line it snaps
    back onto: 0.060 m of standing clearance where the robot stands, 0.001 m at
    the cell that names the seed.
    """
    boxes = [Box(-0.93, 0.0, 1.0, 4.0)]  # face at x = -0.43
    start = (CELL / 2.0 - 0.001, 0.0, 0.0)  # snaps back onto x = 0
    snapped = (0.0, 0.0, 0.0)
    assert stand_clear(boxes, start) > GO2.precision
    assert stand_clear(boxes, snapped) < GO2.precision, (
        "the snap no longer witnesses anything: move the wall, not the assertion"
    )
    gold = se2_path(boxes, start, (2.0, 0.0), GO2)
    assert gold is not None, "the seed read the cell instead of the pose"
    # ...and the cell still NAMES the seed: gold's first vertex is the snap.
    assert abs(float(gold[0][0]) - snapped[0]) < 1e-6


def test_door_side_routes_from_beside_the_door() -> None:
    """The world that caught the rule. Its snap no longer witnesses it — the
    standing body absorbs the lateral half-cell — so it pins the route now."""
    sc = DOOR_SIDE
    gold = se2_path(sc.boxes, sc.start, sc.goal, sc.emb)
    assert gold is not None, "door_side has a route and gold must find it"
    snapped = (float(gold[0][0]), float(gold[0][1]), sc.start[2])
    assert union_clear(sc.boxes, snapped) < sc.emb.precision < union_clear(sc.boxes, sc.start)
    assert stand_clear(sc.boxes, snapped) > sc.emb.precision


def test_a_start_inside_an_obstacle_still_refuses() -> None:
    """Negative true clearance is not a pose the robot occupies — it refuses."""
    boxes = [Box(0.0, 0.0, 1.0, 1.0)]
    start = (0.0, 0.0, 0.0)
    assert union_clear(boxes, start) < 0.0
    assert se2_path(boxes, start, (3.0, 0.0), GO2) is None


def test_a_start_under_the_margin_still_refuses() -> None:
    """Clearance below control precision is fiction, and the seed reads it so.

    Measured against the STANDING body — the intersection of the envelope's
    rows, 0.416 m wide against the union's 0.593 — since planner/revision.md's
    standing-witness amendment. A corridor tight only on the union is one the
    robot walks down nose-first, and the seed is right to accept it.
    """
    stand = GO2.stand_box()
    half = stand[1] / 2.0 + 0.02  # 0.02 m of room per side: real, but not trusted
    boxes = [Box(0.0, half + 0.5, 4.0, 1.0), Box(0.0, -half - 0.5, 4.0, 1.0)]
    start = (0.0, 0.0, 0.0)
    assert 0.0 < stand_clear(boxes, start) < GO2.precision
    assert se2_path(boxes, start, (1.5, 0.0), GO2) is None
