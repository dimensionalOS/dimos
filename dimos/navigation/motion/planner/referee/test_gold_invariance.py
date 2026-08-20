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

"""Where the lattice happens to sit may not change what the gold oracle says.

`test_grid_invariance.py` pins the same property for the RUST CANDIDATE, where
it is still a strict xfail (phase 2). This is the gold side of
planner/revision.md: `se2_path` anchors its grids at
`floor(corner / PERIOD) * PERIOD` in the scenario's own frame, so an obstacle
appearing or vanishing can add whole rows at the edge and can never move a
sample position.

PERIOD (0.24 m) is 3 map voxels, 2 lattice cells and 6 fine SDF samples, which
makes it the translation all three pitches map onto themselves under. A world
shifted by a whole period is the same world sampled in the same places, so the
answer has to shift with it and do nothing else.

The world is a 0.60 m doorway, deliberately sized between the two footprints
the revision put in play: the 0.593 m all-gait union cannot fit through it with
a 0.05 m margin, and the 0.416 m nose-first row can. So these also witness that
the per-heading envelope is live in the search rather than merely stored.
"""

from __future__ import annotations

import numpy as np

from dimos.navigation.motion.embodiment import Embodiment
from dimos.navigation.motion.scenarios import PERIOD, Box, se2_path

# Doorway at x = 0.72, 0.60 m between the leaves. Start beside the axis facing
# across it, so the route is a real maneuver and not a single straight edge.
DOOR = [Box(0.72, 0.66, 0.12, 0.72), Box(0.72, -0.66, 0.12, 0.72)]
START = (-0.2, 0.45, -1.2)
GOAL = (1.68, 0.0)
PAD = 0.6  # compact working area: this is a per-solve second, not a battery


def gold(
    boxes: list[Box] | None = None,
    shift: float = 0.0,
    emb: Embodiment | None = None,
) -> np.ndarray:
    boxes = boxes or DOOR
    moved = [Box(b.cx + shift, b.cy + shift, b.sx, b.sy, b.yaw, b.height) for b in boxes]
    start = (START[0] + shift, START[1] + shift, START[2])
    goal = (GOAL[0] + shift, GOAL[1] + shift)
    out = se2_path(moved, start, goal, **({"emb": emb} if emb else {}), pad=PAD)
    assert out is not None, "the reference world has to have an answer to compare"
    return out


def test_the_doorway_is_a_per_heading_win_not_a_union_one() -> None:
    """The premise: the union walks around this door, the nose-first row through."""
    through = gold()
    around = gold(emb=Embodiment(tag="union-only"))  # no envelope rows: union everywhere
    assert float(np.abs(through[:, 1]).max()) < 0.5, "envelope body did not take the door"
    assert float(np.abs(around[:, 1]).max()) > 1.0, "union body squeezed through the door"


def test_a_whole_period_translation_translates_the_gold_exactly() -> None:
    base = gold()
    for k in (1, -2):
        moved = gold(shift=k * PERIOD)
        assert moved.shape == base.shape, f"{k} periods changed the state count"
        off = np.array([k * PERIOD, k * PERIOD, 0.0])
        assert np.allclose(moved, base + off, atol=1e-9, rtol=0.0), (
            f"shifting the world by {k} whole lattice periods moved the answer by "
            f"{np.abs(moved - base - off).max():.3e} m more than the world"
        )


def test_a_far_obstacle_does_not_change_the_gold() -> None:
    """A box past the working area's low corner: rows appear, samples do not move."""
    base = gold()
    far = gold([*DOOR, Box(-6.0, -5.0, 0.4, 0.4)])
    assert far.shape == base.shape, "a distant box changed the state count"
    assert np.allclose(far, base, atol=1e-9, rtol=0.0), (
        f"a box 7 m off the route moved the answer by {np.abs(far - base).max():.3e} m"
    )
