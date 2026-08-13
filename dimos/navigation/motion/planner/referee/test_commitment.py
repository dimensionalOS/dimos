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

"""The route already published is an input, and it decides near-ties.

The search is an argmin over a quantized lattice, so the continuous pose enters
as a seed state — one of 16 yaw bins, one 0.12 m cell — and two poses nobody
could tell apart seed two different queries. Where two routes near-tie, that
flips the winner: gen001 at (3.33, -2.89) swaps between a 41-pose 2.44 m answer
and a 32-pose 2.33 m one across a tenth of a degree of the -11.25 deg bin edge,
and 25 of its 51 replans re-roll while the yaw sweeps five boundaries. The
planner was right every time; it simply had no reason to prefer the route it had
already published, because that was not one of its inputs.

What is pinned here, in the order it matters:

- the defect itself, so this file fails the day the lattice stops being one;
- the fix: fed its own previous answer, the planner returns that answer;
- the SAFETY rail the margin may never eat — an obstacle appearing across the
  incumbent diverts the very next query, with no confirmation, no debounce and
  no tick of hold-through;
- re-validation more generally: a route the map does not permit is not a route,
  whatever it costs;
- and the price function the whole comparison rests on, which has to read the
  same number off a route whether it arrives smoothed or densified.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.geometry import NEAR_FIELD_M, AvoidanceConfig, near_field_diff
from dimos.navigation.motion.obstacles import hard_points, load as load_model
from dimos.navigation.motion.planner.planners.base import load
from dimos.navigation.motion.planner.planners.gold import densify_states, states_of
from dimos.navigation.motion.scenarios import (
    COMMIT_MARGIN,
    Box,
    Scenario,
    generated,
    path_cost,
    trim_to_pose,
    truth_grid,
)

from .sim import CLOUD_STEP, SWITCH_M

RES = AvoidanceConfig().resolution
NEAR = round(NEAR_FIELD_M / RES)
# gen001's pinned tie: the -11.25 deg bin edge at (3.33, -2.89), a tenth of a
# degree either side. Both answers are correct; they are not the same route.
GEN001_XY = (3.33, -2.89)
GEN001_YAW = math.radians(-11.25)
EPS = math.radians(0.1)
# Arc a route may gain or lose and still be the same route: a trim lands on a
# waypoint rather than on the pose, and a carried route is re-densified, both
# worth millimetres. gen001's tie is 0.114 m apart, so nothing here is close.
ARC_EPS = 0.05
# Candidates only. Gold is the same code with a box-exact field and seconds per
# solve, and `test_gold_invariance` is where it earns its keep. The python port
# spec is checked where one query answers the question and left off the chains,
# which are six solves deep and minutes long through it.
CANDIDATES = ["target", "target-py"]


def _obstacles(sc: Scenario) -> np.ndarray:
    """What an honest candidate is allowed to see — sim.py's own cloud rule."""
    if not sc.boxes:
        return np.empty((0, 2))
    pts = np.concatenate([b.surface(CLOUD_STEP) for b in sc.boxes]).astype(np.float32)
    return np.ascontiguousarray(
        hard_points(load_model("raw_band", sc.emb), pts, 0.0)[:, :2], dtype=np.float64
    )


def _arc(path: Path) -> float:
    xy = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


def _clearance(path: Path, box: Box) -> float:
    """Closest the published route's own waypoints come to one box."""
    xy = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
    return float(np.min(box.sdf2d(xy)))


def _same_route(a: Path, b: Path) -> bool:
    """Is this the same route, or a different one?

    `near_field_diff` alone will not answer that here. It is a MEAN offset onto
    the other polyline, so two routes that thread the same corridor and part
    company only at the end read ~0 through it — which is exactly gen001's tie:
    41 waypoints over 2.442 m against 32 over 2.328 m, running side by side the
    whole way and committing to different ends of the same obstacle. Arc length
    is what separates those; near-field offset is what separates a route that
    moved under the follower's nose. A route is the same when both agree.
    """
    return near_field_diff(a, b, NEAR) < SWITCH_M and abs(_arc(a) - _arc(b)) < ARC_EPS


# The defect, and the answer to it.


@pytest.mark.parametrize("planner", CANDIDATES)
def test_a_bin_edge_flips_the_route_and_the_incumbent_settles_it(planner: str) -> None:
    sc = generated(40)[1]
    obstacles = _obstacles(sc)
    ep = load(planner)(sc, AvoidanceConfig())
    ep.reset()

    below = ep.plan(obstacles, (*GEN001_XY, GEN001_YAW - EPS), sc.goal)
    above = ep.plan(obstacles, (*GEN001_XY, GEN001_YAW + EPS), sc.goal)
    # The defect. This assertion is the pre-fix state of the world, and it is
    # deliberately the first thing here: a stateless argmin over a lattice
    # answers 0.2 deg of yaw with two different routes.
    assert not _same_route(below, above), (
        "gen001's tie no longer flips at the bin edge — the lattice, the costs "
        "or the world moved, and this file's premise needs re-measuring"
    )

    # ...and the fix. Handed the answer it published a tenth of a degree ago,
    # the planner returns that answer rather than the coin it just re-flipped.
    held = ep.plan(obstacles, (*GEN001_XY, GEN001_YAW + EPS), sc.goal, below)
    assert _same_route(below, held)
    assert not _same_route(above, held)


@pytest.mark.parametrize("planner", CANDIDATES)
def test_the_margin_covers_the_tie_it_was_measured_for(planner: str) -> None:
    """The two gen001 answers differ in price by less than the margin — which
    is the whole claim: a difference this size is not a reason to change route."""
    sc = generated(40)[1]
    obstacles = _obstacles(sc)
    ep = load(planner)(sc, AvoidanceConfig())
    ep.reset()
    fgx, fgy, grid, _ = truth_grid(sc.boxes, sc.start, sc.goal)
    here = (*GEN001_XY, GEN001_YAW)
    prices = [
        path_cost(
            fgx,
            fgy,
            grid,
            np.vstack([[list(here)], states_of(ep.plan(obstacles, (*GEN001_XY, y), sc.goal))]),
            sc.emb,
        )
        for y in (GEN001_YAW - EPS, GEN001_YAW + EPS)
    ]
    assert abs(prices[1] - prices[0]) < COMMIT_MARGIN


@pytest.mark.parametrize("planner", ["target"])
@pytest.mark.parametrize("idx", [0, 1, 2, 3, 4])
def test_a_chain_of_replans_keeps_the_route_it_published(planner: str, idx: int) -> None:
    """Walk the pose down the published route and replan, each query fed the one
    before it. The world is static and the goal is fixed, so any change of route
    is a change nothing in the world asked for."""
    sc = generated(40)[idx]
    obstacles = _obstacles(sc)
    ep = load(planner)(sc, AvoidanceConfig())
    ep.reset()
    out = ep.plan(obstacles, sc.start, sc.goal)
    if _arc(out) < 0.5:
        pytest.skip(f"{sc.name}: no route to hold on to")
    for _ in range(6):
        xy = np.array([[p.position.x, p.position.y] for p in out.poses])
        if len(xy) < 4:
            break
        k = max(1, min(len(xy) // 4, len(xy) - 2))
        tang = xy[k + 1] - xy[k - 1]
        spot = (float(xy[k][0]), float(xy[k][1]), math.atan2(tang[1], tang[0]))
        remainder = Path(ts=0.0, frame_id=out.frame_id, poses=out.poses[k:])
        new = ep.plan(obstacles, spot, sc.goal, out)
        assert _same_route(remainder, new), (
            f"{sc.name}: a replan from its own route published a different one"
        )
        out = new


# The rail the margin may never eat.


@pytest.mark.parametrize("planner", CANDIDATES)
def test_an_obstacle_across_the_route_diverts_the_very_next_query(planner: str) -> None:
    """No confirmation, no debounce, no tick of hold-through.

    Delaying belief in a detected obstacle is a robustness layer priced in
    collisions. Map noise flapping a corridor is perception's ledger — the
    diagnose churn pass names it same-map vs new-map — and never something the
    planner absorbs by walking into things for one more tick.
    """
    sc = Scenario("open", [], goal=(4.0, 0.0))
    ep = load(planner)(sc, AvoidanceConfig())
    ep.reset()
    straight = ep.plan(_obstacles(sc), sc.start, sc.goal)
    assert _arc(straight) > 3.0, "the empty world's route is the straight line"

    # A wall drops across the route, ahead of the robot, with a way round.
    wall = Box(2.0, 0.0, 0.2, 2.0)
    blocked = Scenario("blocked", [wall], goal=(4.0, 0.0))
    out = ep.plan(_obstacles(blocked), sc.start, sc.goal, straight)
    assert near_field_diff(straight, out, NEAR) > SWITCH_M, "held a blocked route"
    assert _clearance(out, wall) > 0.0, "the diversion goes through the wall"


@pytest.mark.parametrize("planner", CANDIDATES)
def test_a_route_this_map_forbids_is_not_an_incumbent(planner: str) -> None:
    """Re-validation is the search's own per-row test on the CURRENT map, so a
    handed-in route that walks through a wall buys nothing at all."""
    wall = Box(2.0, 0.0, 0.2, 2.0)
    sc = Scenario("blocked", [wall], goal=(4.0, 0.0))
    obstacles = _obstacles(sc)
    ep = load(planner)(sc, AvoidanceConfig())
    ep.reset()
    honest = ep.plan(obstacles, sc.start, sc.goal)
    open_world = Scenario("open", [], goal=(4.0, 0.0))
    through = ep.plan(_obstacles(open_world), sc.start, sc.goal)
    out = ep.plan(obstacles, sc.start, sc.goal, through)
    assert near_field_diff(honest, out, NEAR) < SWITCH_M
    assert _clearance(out, wall) > 0.0


# The price the comparison is made on.


@pytest.mark.parametrize("idx", [0, 1])
def test_a_route_prices_the_same_however_many_points_it_arrives_with(idx: int) -> None:
    """An incumbent comes back at path resolution and a fresh answer is a
    handful of smoothed vertices. Sampling by ARC rather than by vertex is what
    makes those two numbers comparable at all."""
    sc = generated(40)[idx]
    obstacles = _obstacles(sc)
    ep = load("target-py")(sc, AvoidanceConfig())
    ep.reset()
    states = states_of(ep.plan(obstacles, sc.start, sc.goal))
    assert states is not None
    fgx, fgy, grid, _ = truth_grid(sc.boxes, sc.start, sc.goal)
    dense = np.array(densify_states(states, 0.03))
    assert path_cost(fgx, fgy, grid, dense, sc.emb) == pytest.approx(
        path_cost(fgx, fgy, grid, states, sc.emb), abs=1e-9
    )


def test_trimming_keeps_the_remainder_and_nothing_else() -> None:
    route = np.array([[float(i) * 0.5, 0.0, 0.0] for i in range(10)])
    # (2.0, 0, 0) is index 4: everything BEFORE it goes, and it stays — the
    # route's own head pose, not the robot's.
    assert np.array_equal(trim_to_pose(route, (2.1, 0.3, 0.2)), route[4:])
    assert np.array_equal(trim_to_pose(route, (99.0, 0.0, 0.0)), route[9:])
    assert np.array_equal(trim_to_pose(route, (-99.0, 0.0, 0.0)), route)
