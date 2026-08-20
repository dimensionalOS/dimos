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

"""Replanning from your own emitted route may never refuse.

The per-heading envelope made route feasibility motion-conditioned, but the
seed-entry tests kept reading the all-gait UNION: the planner threaded a gap
only a narrow drift row clears, the robot followed the plan in, and the next
replan from mid-gap refused — a single-pose stub, the follower runs out of path,
the robot holds until the world times out. `--score --gen 40 -s 'gen*'` failed
6 of 40 that way (5 timeouts + a collision); gen000 froze at (2.63, 1.56,
yaw 0.147) reading union +0.033 against a 0.05 m margin while the nose row that
justified the route read +0.121. Pre-envelope this was structurally impossible:
one shape meant "the plan accepted this pose" and "the witness accepts it" were
the same test.

The fix restores that identity by construction rather than by repair. Standing
occupies the static body — the INTERSECTION of the envelope's rows, mirrored in
both drift signs — which is nested in every shape an edge can be cleared by. So
any pose whose row clears the margin passes the witness, and the seed can never
reject a pose the planner itself published.

Both halves are pinned here: the nesting, which is the whole argument, and the
end-to-end replay — every k-th pose along every emitted path, replayed as the
start of a fresh query against the same world, must come back with a plan. Gold
paths replay through gold (box-exact truth SDF), candidate paths through the
candidate (its own cloud-built one), because a refusal is only interesting
against the world model that emitted the route.

`door_side` is the cheapest witness of the pre-fix defect: gold's own first
emitted vertex is the snapped start cell, 0.043 m of union clearance against a
0.05 m margin, and replanning from it returned None.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.navigation.motion.embodiment import EMBODIMENTS, box_offsets
from dimos.navigation.motion.geometry import AvoidanceConfig
from dimos.navigation.motion.obstacles import hard_points, load as load_model
from dimos.navigation.motion.planner.planners.gold import densify_states
from dimos.navigation.motion.scenarios import SCENARIOS, Scenario, generated, se2_path

from .sim import CLOUD_STEP

# Every 5th published waypoint, so ~0.5 m of route per replay at the 0.1 m
# path resolution. Gold is seconds per solve and cached on disk by (world,
# query), so the first run of this file is minutes and every later one is not.
STRIDE = 5
RES = AvoidanceConfig().resolution
# A pose this close to the end of the route has arrived; a short answer there is
# an answer, not a stub, so it carries no information about refusals.
ARRIVED_M = 0.5

GEN_WORLDS = 10
# Everything but the sealed worlds: a "safe" label still publishes a route, and
# `narrow_gap` / `zigzag_room` are exactly the tight ones this is about.
WORLDS = [sc for sc in SCENARIOS if sc.expect != "refuse"] + generated(40)[:GEN_WORLDS]
IDS = [sc.name for sc in WORLDS]


def _emb_tuple(sc: Scenario) -> tuple[object, ...]:
    e = sc.emb
    return (
        e.length,
        e.width,
        e.center_off,
        e.comfort,
        e.precision,
        e.strafe,
        e.reverse,
        e.yaw_w,
        e.envelope,
        e.arc_inflate,
    )


def _obstacles(sc: Scenario) -> np.ndarray:
    """What an honest candidate is allowed to see — sim.py's own cloud rule."""
    if not sc.boxes:
        return np.empty((0, 2))
    pts = np.concatenate([b.surface(CLOUD_STEP) for b in sc.boxes]).astype(np.float32)
    return np.ascontiguousarray(
        hard_points(load_model("raw_band", sc.emb), pts, 0.0)[:, :2], dtype=np.float64
    )


def _replay_poses(path: np.ndarray) -> list[tuple[int, tuple[float, float, float]]]:
    """Every STRIDE-th pose of an emitted path, minus the ones already arrived."""
    xy = path[:, :2]
    if len(xy) < 2:
        return []
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    left = np.concatenate([np.cumsum(seg[::-1])[::-1], [0.0]])
    return [
        (k, (float(path[k][0]), float(path[k][1]), float(path[k][2])))
        for k in range(0, len(path), STRIDE)
        if left[k] > ARRIVED_M
    ]


# ---- the argument: the standing shape is nested in every shape an edge uses --


def test_the_standing_body_is_nested_in_every_envelope_row() -> None:
    """Intersection, in both drift signs — which is what makes the pin hold."""
    emb = EMBODIMENTS["go2"]
    sl, sw, sx, sy = emb.stand_box()
    assert (sl, sw, sx, sy) != emb.box(None), "the go2's standing box is still the union"
    assert sy == 0.0, "mirroring the rows cannot leave a lateral offset behind"
    for deg, length, width, off_x, off_y in emb.envelope:
        for sign in (1.0, -1.0):
            oy = sign * off_y
            assert off_x - length / 2.0 <= sx - sl / 2.0 + 1e-12, f"{deg} deg: back edge"
            assert off_x + length / 2.0 >= sx + sl / 2.0 - 1e-12, f"{deg} deg: front edge"
            assert oy - width / 2.0 <= sy - sw / 2.0 + 1e-12, f"{deg} deg: right edge"
            assert oy + width / 2.0 >= sy + sw / 2.0 - 1e-12, f"{deg} deg: left edge"


def test_an_unmeasured_body_stands_in_its_union() -> None:
    """No rows to intersect: nothing changes for a body nobody measured."""
    for tag in ("go2-payload", "slim", "diffdrive"):
        emb = EMBODIMENTS[tag]
        assert not emb.envelope
        assert emb.stand_box() == emb.box(None), tag


def test_the_standing_witness_reads_at_least_what_any_row_reads() -> None:
    """The consequence, on a world: no row can clear where standing does not.

    Up to the footprint SAMPLING, which is where the exactness stops: a box
    nested in another still tiles its own extent at OFFSET_STEP, so the two
    sample sets are offset from each other and their minima can disagree by a
    fraction of one step. Half a step is the bound; the worst this world
    produces is 2.2 mm, against the 88 mm the union used to read short by.
    """
    tol = 0.5 * 0.05  # half of Embodiment.offsets' sample step
    sc = next(s for s in SCENARIOS if s.name == "door_side")
    emb = sc.emb
    stand = box_offsets(emb.stand_box())
    rng = np.random.default_rng(0)
    poses = np.column_stack(
        [
            rng.uniform(0.0, 4.0, 400),
            rng.uniform(-1.5, 1.5, 400),
            rng.uniform(-math.pi, math.pi, 400),
        ]
    )
    for x, y, th in poses:
        c, s = math.cos(th), math.sin(th)

        def clear(off: np.ndarray, x: float = x, y: float = y, c: float = c, s: float = s) -> float:
            pts = np.column_stack(
                [
                    x + c * off[:, 0] - s * off[:, 1],
                    y + s * off[:, 0] + c * off[:, 1],
                    np.zeros(len(off)),
                ]
            )
            return float(np.min([b.sdf2d(pts) for b in sc.boxes]))

        witness = clear(stand)
        for row in emb.envelope:
            for sign in (1.0, -1.0):
                got = clear(box_offsets((row[1], row[2], row[3], sign * row[4])))
                assert witness >= got - tol, (
                    f"at ({x:.2f}, {y:.2f}, {th:.2f}) the {row[0]} deg row reads {got:.4f} "
                    f"and the standing witness only {witness:.4f}"
                )


# The end-to-end pin: replay every k-th pose of every emitted route.


@pytest.mark.parametrize("sc", WORLDS, ids=IDS)
def test_gold_replans_from_every_pose_it_published(sc: Scenario) -> None:
    base = se2_path(sc.boxes, sc.start, sc.goal, sc.emb)
    if base is None:
        assert sc.expect != "clear", f"{sc.name} is labeled clear and gold must find a route"
        pytest.skip(f"{sc.name}: gold refuses the world itself, so there is no route to replay")
    path = np.array(densify_states(base, RES))
    bad = []
    for k, pose in _replay_poses(path):
        again = se2_path(sc.boxes, pose, sc.goal, sc.emb)
        if again is None or len(again) < 2:
            bad.append((k, pose, None if again is None else len(again)))
    assert not bad, (
        f"{sc.name}: gold refused from {len(bad)} pose(s) on its own {len(path)}-pose route, "
        f"first at index {bad[0][0]} {tuple(round(v, 3) for v in bad[0][1])}"
    )


@pytest.mark.parametrize("sc", WORLDS, ids=IDS)
def test_the_candidate_replans_from_every_pose_it_published(sc: Scenario) -> None:
    dimos_motion2_target = pytest.importorskip("dimos_motion2_target")
    obstacles = _obstacles(sc)
    emb = _emb_tuple(sc)
    base = dimos_motion2_target.plan(obstacles, sc.start, sc.goal, emb, RES)
    if base is None or len(base) < 2:
        pytest.skip(f"{sc.name}: the candidate refuses the world itself — a different question")
    bad = []
    for k, pose in _replay_poses(np.asarray(base)):
        again = dimos_motion2_target.plan(obstacles, pose, sc.goal, emb, RES)
        if again is None or len(again) < 2:
            bad.append((k, pose, None if again is None else len(again)))
    assert not bad, (
        f"{sc.name}: the candidate refused from {len(bad)} pose(s) on its own {len(base)}-pose "
        f"route, first at index {bad[0][0]} {tuple(round(v, 3) for v in bad[0][1])}"
    )


def test_door_side_is_the_witness_the_union_failed() -> None:
    """The one that named the defect: gold's own first vertex is the snapped
    start cell, and on the union the seed refused to stand where it had just
    published a route from."""
    sc = next(s for s in SCENARIOS if s.name == "door_side")
    base = se2_path(sc.boxes, sc.start, sc.goal, sc.emb)
    assert base is not None
    vertex = (float(base[0][0]), float(base[0][1]), float(base[0][2]))
    union = box_offsets(sc.emb.box(None))
    c, s = math.cos(vertex[2]), math.sin(vertex[2])
    pts = np.column_stack(
        [
            vertex[0] + c * union[:, 0] - s * union[:, 1],
            vertex[1] + s * union[:, 0] + c * union[:, 1],
            np.zeros(len(union)),
        ]
    )
    assert float(np.min([b.sdf2d(pts) for b in sc.boxes])) < sc.emb.precision, (
        "door_side no longer reads under the margin on the union: pick another witness"
    )
    assert se2_path(sc.boxes, vertex, sc.goal, sc.emb) is not None, (
        "gold refused to replan from the first vertex of its own published route"
    )
