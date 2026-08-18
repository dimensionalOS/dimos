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


"""Gap-crossing VQA over the go2 replays — the glass probe.

Rows (``go2_pointcloud_glass_vqa.json``) are pure data emitted by :func:`rows`.
Every case shows a gap in the body-height point cloud that is wider than the
robot, and asks whether walking into it gets you through. Half the gaps are
real floor; half are glass.

``crossing`` is deliberately small and deliberately correlated — a probe, not a
benchmark. Read :data:`PANES` before using the numbers for anything.

Why this shape and not a routing question. Glass produces no routing failure in
these recordings: the lidar stream is a 6.4 m rolling local window, so any
barrier short enough to fit inside it has both ends inside it too, and the
planner walks around. Painting a human-confirmed pane into the costmap — even
extended along its own line to the full run of coplanar support, turning a
1.1 m panel into a 4-7 m wall — changed the planner's answer on 0 of 12 goals.
Straight-line clearance fares no better: the office is cluttered enough that a
corridor aimed at a pane is already blocked by furniture, so the right answer
comes out for the wrong reason. What is left is local. At the pane itself the
map shows a hole the robot is told it fits through, and that is the question
asked here.

Ground truth is split by class, because glass cannot be labelled from lidar —
lidar is what glass defeats.

``barrier`` is human-adjudicated. YOLO-E (:class:`Yoloe2DDetector`, prompt mode,
"glass wall" / "glass door" / "window" / "glass partition") segmented candidate
panes in the camera stream; each was placed in world coordinates by casting
rays through the mask's bottom edge onto the floor plane the lidar measured,
and every candidate was then confirmed or rejected by a person against the
camera frame. Only panes that leave a usable trace in the cloud survive: a pane
returning nothing at all cannot be reported by *any* encoding of that cloud, so
scoring it would reward guessing. Of ten confirmed panes, seven returned
nothing; :data:`PANES` is what remains.

``open`` is analytic. The gate is a slice across the robot's own direction of
travel at a point its base later occupied — it walked through, so the gap was
real. Both classes are sampled to the same gate width and standoff so the
geometry cannot separate them.

Regenerate (needs both recordings)::

    python -m dimos.evals.suites.go2_pointcloud_glass
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from dimos.evals import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_glass_vqa.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))


# -- geometry --------------------------------------------------------------------

MIN_GAP = 0.5  # Go2 footprint inflated by the planner; narrower reads as solid
MAX_GAP = 1.3  # wider stops being a "gap" and starts being open floor
NEAR = 0.25  # half-width of the corridor counting as support for a pane
GATE_LO = 0.8  # nearest a gate may sit to the robot, meters
GATE_HI = 3.0  # farthest; beyond this the camera could not adjudicate the pane
SAMPLE = 2.0  # seconds between sampled frames
OPEN_SAMPLE = 1.0  # open gates are the larger pool; sample them finer to match widths
LOOKAHEAD = 25.0  # how far ahead of a frame the trajectory certificate may look
SLICE = 0.15  # half-thickness of the trajectory slice defining an open gate
FLANK = 1.5  # a gate needs mapped obstacle within this on both sides


# Human-confirmed glass. Each entry is (dataset, tag, end, end, time window).
#
# Every pane here was confirmed by a person against the camera frame, and left
# enough trace in the point cloud that an encoding could in principle report it.
# Two entries share the tag ``partition_a``: the reviewer identified the panes
# found at t=38 and t=122 as one physical surface, so the suite treats them as
# one and the ids stay distinguishable by time.
#
# ponytail: three distinct physical surfaces, all in go2_china_office. The
# go2_short pane the reviewer confirmed never presents a gap in the MIN_GAP
# range, so it contributes no cases. Treat per-pane results as three samples,
# not sixteen.
PANES: tuple[
    tuple[str, str, tuple[float, float], tuple[float, float], tuple[float, float]], ...
] = (
    ("go2_china_office", "partition_a", (1.76, 3.80), (2.72, 5.06), (30.0, 46.0)),
    ("go2_china_office", "partition_a", (1.76, 3.80), (2.72, 5.06), (114.0, 130.0)),
    ("go2_china_office", "partition_b", (0.44, 4.52), (1.94, 3.88), (100.0, 116.0)),
    ("go2_china_office", "meeting_room", (2.40, 6.85), (3.50, 6.53), (126.0, 138.0)),
)

_OPEN_DATASETS = ("go2_china_office", "go2_short")


def _question(gate: np.ndarray, gap: float, rng: float) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown "
        "(world frame: +x is east, +y is north, coordinates in meters). The "
        f"mapped points leave a gap centred at ({gate[0]:.2f}, {gate[1]:.2f}), "
        f"{rng:.1f} m from you: the nearest mapped points on either side of it are "
        f"{gap:.2f} m apart, which is wider than your body. Using only the mapped "
        "point cloud, would you get through if you walked into it? Answer with "
        "exactly one word: open if the gap is floor you would pass through, or "
        "barrier if something standing in it would stop you."
    )


def _row(
    dataset: str,
    ident: str,
    label: str,
    gate: np.ndarray,
    gap: float,
    rng: float,
    t: float,
    context: list[list[object]],
) -> generate.Row:
    return {
        "id": ident,
        "family": "crossing",
        "type": "mcq",
        "q": _question(gate, gap, rng),
        "a": label,
        "choices": ["open", "barrier"],
        "context": [*context, ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]]],
        "dataset": dataset,
    }


def _widest_hole(band: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[float, np.ndarray]:
    """Longest stretch of a pane with no body-height return, and its midpoint.

    Glass returns intermittently — mullions and near-normal incidence give
    points, the pane between them gives none — so a confirmed pane still shows
    holes. The widest one is what the robot would be told it fits through.
    """
    length = float(np.linalg.norm(b - a))
    u = (b - a) / length
    n = np.array([-u[1], u[0]])
    w = band - a
    along = w @ u
    lateral = np.abs(w @ n)
    support = np.sort(along[(lateral <= NEAR) & (along >= -0.1) & (along <= length + 0.1)])
    edges = np.concatenate(([0.0], support, [length]))
    holes = np.diff(edges)
    if not holes.size:
        return length, a + u * (length / 2)
    k = int(np.argmax(holes))
    return float(holes[k]), a + u * float((edges[k] + edges[k + 1]) / 2)


def barrier_rows() -> list[generate.Row]:
    """Confirmed glass panes, at every frame where the map still shows a hole.

    A pane only becomes a case where the cloud presents a gap in the crossing
    range: the point is that the map reads as passable, so a frame where the
    pane looks solid is not a failure to elicit.
    """
    rows: list[generate.Row] = []
    for dataset in sorted({p[0] for p in PANES}):
        with generate._dataset(dataset) as store:
            for _, tag, pa, pb, (t0, t1) in [p for p in PANES if p[0] == dataset]:
                a, b = np.array(pa), np.array(pb)
                for t in np.arange(t0, t1 + 1e-9, SAMPLE):
                    pts, context = generate._cloud_at(store, float(t))
                    lo, hi = pts[:, :2].min(axis=0), pts[:, :2].max(axis=0)
                    if not ((lo <= np.minimum(a, b)).all() and (np.maximum(a, b) <= hi).all()):
                        continue  # pane has rolled out of the local window
                    band = pts[
                        (pts[:, 2] >= generate.BODY_Z[0]) & (pts[:, 2] <= generate.BODY_Z[1])
                    ]
                    gap, gate = _widest_hole(band[:, :2], a, b)
                    odom = store.streams.odom.range_time(0, float(t)).to_list()[-1].data.position
                    origin = np.array([float(odom.x), float(odom.y)])
                    rng = float(np.hypot(*(gate - origin)))
                    if not (MIN_GAP <= gap <= MAX_GAP and GATE_LO <= rng <= GATE_HI):
                        continue
                    rows.append(
                        _row(
                            dataset,
                            f"{dataset}_crossing_{tag}_t{t:g}",
                            "barrier",
                            gate,
                            gap,
                            rng,
                            float(t),
                            context,
                        )
                    )
    return rows


def open_rows() -> list[generate.Row]:
    """Gaps certified by the robot's own trajectory: it walked through them.

    The gate is a slice taken across the direction of travel at a point the
    base later occupied, so "passable" is not an inference — it happened. Gate
    width and standoff are held to the same ranges as the barrier class.
    """
    rows: list[generate.Row] = []
    for dataset in _OPEN_DATASETS:
        with generate._dataset(dataset) as store:
            odom = store.streams.odom
            first = odom.first().ts
            traj = [
                (
                    obs.ts - first,
                    np.array([float(obs.data.position.x), float(obs.data.position.y)]),
                )
                for obs in odom.to_list()
            ]
            for t in np.arange(4.0, traj[-1][0] - 4.0, OPEN_SAMPLE):
                pts, context = generate._cloud_at(store, float(t))
                band = pts[(pts[:, 2] >= generate.BODY_Z[0]) & (pts[:, 2] <= generate.BODY_Z[1])]
                band = band[:, :2]
                pos = odom.range_time(0, float(t)).to_list()[-1].data.position
                origin = np.array([float(pos.x), float(pos.y)])
                future = [(tt, p) for tt, p in traj if t < tt <= t + LOOKAHEAD]
                seen: set[tuple[int, int]] = set()
                for k in range(1, len(future)):
                    gate = future[k][1]
                    rng = float(np.hypot(*(gate - origin)))
                    if not (GATE_LO <= rng <= GATE_HI):
                        continue
                    heading = future[k][1] - future[k - 1][1]
                    norm = float(np.linalg.norm(heading))
                    if norm < 1e-3:
                        continue
                    u = heading / norm
                    n = np.array([-u[1], u[0]])
                    w = band - gate
                    side = (w @ n)[np.abs(w @ u) <= SLICE]
                    left, right = side[side > 0], side[side < 0]
                    if not left.size or not right.size:
                        continue  # no flanking structure — that is open floor, not a gap
                    gap = float(left.min() - right.max())
                    if not (
                        MIN_GAP <= gap <= MAX_GAP and left.min() < FLANK and -right.max() < FLANK
                    ):
                        continue
                    cell = (int(gate[0] / 0.5), int(gate[1] / 0.5))
                    if cell in seen:
                        continue  # the trajectory lingers; one gate per spot per frame
                    seen.add(cell)
                    rows.append(
                        _row(
                            dataset,
                            f"{dataset}_crossing_open_t{t:g}_{len(seen)}",
                            "open",
                            gate,
                            gap,
                            rng,
                            float(t),
                            context,
                        )
                    )
    return rows


# The dataset: every barrier case, and an equal number of open cases chosen to
# match the barrier gate widths so the answer cannot be read off the geometry.
_CASES = (
    "go2_china_office_crossing_partition_a_t34",
    "go2_china_office_crossing_partition_a_t36",
    "go2_china_office_crossing_partition_a_t120",
    "go2_china_office_crossing_partition_a_t122",
    "go2_china_office_crossing_partition_b_t106",
    "go2_china_office_crossing_partition_b_t110",
    "go2_china_office_crossing_partition_b_t112",
    "go2_china_office_crossing_partition_b_t114",
    "go2_china_office_crossing_partition_b_t116",
    "go2_china_office_crossing_meeting_room_t126",
    "go2_china_office_crossing_meeting_room_t128",
    "go2_china_office_crossing_meeting_room_t130",
    "go2_china_office_crossing_meeting_room_t132",
    "go2_china_office_crossing_meeting_room_t134",
    "go2_china_office_crossing_meeting_room_t136",
    "go2_china_office_crossing_meeting_room_t138",
)


def _matched_open(
    barrier: list[generate.Row], candidates: list[generate.Row]
) -> list[generate.Row]:
    """One open case per barrier case, nearest in gate width, no reuse.

    Matching on width keeps the two classes indistinguishable by the number the
    question states; without it a reader could learn "wide means open".
    """

    def width(row: generate.Row) -> float:
        return float(str(row["q"]).split(" m apart")[0].rsplit(" are ", 1)[1])

    pool = sorted(candidates, key=lambda r: str(r["id"]))
    picked: list[generate.Row] = []
    used: set[str] = set()
    for b in barrier:
        target = width(b)
        best = min(
            (r for r in pool if str(r["id"]) not in used),
            key=lambda r: abs(width(r) - target),
            default=None,
        )
        if best is not None:
            used.add(str(best["id"]))
            picked.append(best)
    return picked


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    barrier = [r for r in barrier_rows() if str(r["id"]) in _CASES]
    return [*barrier, *_matched_open(barrier, open_rows())]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
