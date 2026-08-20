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

"""An obstacle the route never approaches must not change the route.

The working area — and with it the 0.12 m lattice and the 0.04 m distance
field — used to be laid out from `min(goal, cloud) - PAD` (`build_world`). That
corner is a continuous function of the cloud, so a single return past it, metres
away from anything the route touches, slid the whole grid by an arbitrary
sub-cell amount and the search answered a differently-sampled question.
`build_world` already quantised the pose-driven *growth* for precisely this
reason — "the grid only ever gains or loses whole rows at the edge" — but the
corner it grew from was itself unquantised, so the same failure arrived via the
cloud. Both corners are now floored onto the world frame's own `PERIOD` lattice
(0.24 m = 2 cells = 3 voxels = 6 fine samples) and every index downstream is an
absolute count, so a distant point can add rows and nothing else.

The world here is a 0.75 m doorway in a wall raked 60 deg off the lattice axes:
0.08 m of slack per side against a 0.593 m body and a 0.05 m margin, comfortably
threadable, and the planner does thread it. One point thirteen metres away,
past the cloud's low corner, used to buy a 7.5 m detour instead. Points past the
*high* corner were always harmless — they add rows without moving the origin —
which is what placed the defect at the corner rather than in the point count or
the field itself.

The gap and the far point both track the body: they were 0.66 m and
(-11.71, -4.0) against the 0.50 m union planner/revision.md re-baselined away.
The witness is a knife-edge by nature — a sub-cell grid shift deciding a
marginal doorway — so a wider body needs the corner pushed further out to move
the origin by as much. Keep it that way: a witness that clears comfortably
stops witnessing anything.

Seen on the robot as a route flip through a doorway 5.4 m away, replaying
`ml-trajectory-research/door.zenoh.mcap` at t=7.093 (13.27 m around, vs 1.96 m
through once one distant point was added).
"""

from __future__ import annotations

import dimos_motion2_target
import numpy as np

from dimos.navigation.motion.embodiment import EMBODIMENTS

EMB = EMBODIMENTS["go2"]
RAKE = np.radians(60.0)
GAP = 0.75  # body is 0.593 wide, margin 0.05: 0.08 m of slack per side
FAR = (-11.7, -6.0)  # past the cloud's low corner, ~13 m off the route


def _wall() -> np.ndarray:
    """A raked wall with a doorway at the origin, sampled like a lidar band."""
    t = np.arange(-2.5, 2.5 + 0.025, 0.05)
    t = t[np.abs(t) > GAP / 2.0]
    return np.column_stack([-np.sin(RAKE) * t, np.cos(RAKE) * t])


def _route(points: np.ndarray) -> tuple[str, float]:
    """Plan across the doorway; return (how it crossed the wall, arc length)."""
    out = dimos_motion2_target.plan(
        np.ascontiguousarray(points, dtype=np.float64),
        (-1.5 * np.cos(RAKE), -1.5 * np.sin(RAKE), RAKE),
        (1.5 * np.cos(RAKE), 1.5 * np.sin(RAKE)),
        (
            EMB.length,
            EMB.width,
            EMB.center_off,
            EMB.comfort,
            EMB.precision,
            EMB.strafe,
            EMB.reverse,
            EMB.yaw_w,
            EMB.envelope,
            EMB.arc_inflate,
        ),
        0.1,
    )
    if out is None or len(out) < 2:
        return "hold", 0.0
    xy = out[:, :2]
    arc = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    along = xy @ np.array([np.cos(RAKE), np.sin(RAKE)])
    for a, b, da, db in zip(xy[:-1], xy[1:], along[:-1], along[1:], strict=False):
        if da * db <= 0 and da != db:
            hit = a + (-da / (db - da)) * (b - a)
            return ("through" if float(np.hypot(*hit)) < 1.0 else "around"), arc
    return "hold", arc


def test_the_doorway_is_threadable_at_all() -> None:
    """The premise: with nothing else in the world the planner takes the door."""
    how, arc = _route(_wall())
    assert how == "through", f"doorway not threaded at all ({how}, {arc:.2f} m)"
    assert arc < 4.0, f"threaded the door but wandered {arc:.2f} m"


def test_a_far_obstacle_does_not_change_the_route() -> None:
    wall = _wall()
    base_how, base_arc = _route(wall)
    far_how, far_arc = _route(np.vstack([wall, [FAR]]))
    assert far_how == base_how, (
        f"one point at {FAR}, {np.hypot(*FAR):.1f} m from the door, turned a "
        f"{base_arc:.2f} m {base_how} route into a {far_arc:.2f} m {far_how} one"
    )


def test_a_far_obstacle_past_the_high_corner_is_harmless() -> None:
    """The mirror image, which already holds: the far side only adds rows."""
    wall = _wall()
    base_how, _ = _route(wall)
    for far in ((-FAR[0], -FAR[1]), (FAR[0], -FAR[1])):
        how, arc = _route(np.vstack([wall, [far]]))
        assert how == base_how, f"a point at {far} changed the route to {how} ({arc:.2f} m)"
