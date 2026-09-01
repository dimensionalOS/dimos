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
Each case names one coordinate and asks whether the robot could stand there.
The body-height cloud leaves a robot-width gap at every one; half are floor,
half are glass.

``barrier`` panes are hand-labelled from the camera, lidar being the sensor
glass defeats. The verdicts, rejections included, are in
``go2_glass_labels.json`` beside this file and cannot be regenerated.
``open`` gates are ones the robot drove through.


Regenerate (needs both recordings)::

    uv run python -m dimos.evals.suites.go2_pointcloud_glass
"""

from __future__ import annotations

from collections.abc import Iterable
import json
from pathlib import Path
from typing import NamedTuple

import numpy as np

from dimos.evals.suites.lib import generate
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
SPOT = 0.30  # gates closer than this are the same place on the same surface
STANDOFF = 0.50  # robot body length; nearer poses read the same spot the same way


class Candidate(NamedTuple):
    """A case plus the geometry the width match and the duplicate rule read."""

    gap: float
    gate: np.ndarray
    origin: np.ndarray
    row: generate.Row


# (dataset, tag, end, end, time window) for the six ``glass and learnable``
# entries of go2_glass_labels.json. Both ``partition_a`` rows are one surface.
#
# 11 barrier cases but only 3 surfaces, all go2_china_office. Read
# per-pane, not per-case.
PANES: tuple[
    tuple[str, str, tuple[float, float], tuple[float, float], tuple[float, float]], ...
] = (
    ("go2_china_office", "partition_a", (1.76, 3.80), (2.72, 5.06), (30.0, 46.0)),
    ("go2_china_office", "partition_a", (1.76, 3.80), (2.72, 5.06), (114.0, 130.0)),
    ("go2_china_office", "partition_b", (0.44, 4.52), (1.94, 3.88), (100.0, 116.0)),
    ("go2_china_office", "meeting_room", (2.40, 6.85), (3.50, 6.53), (126.0, 138.0)),
)

_OPEN_DATASETS = ("go2_china_office", "go2_short")


def _question(gate: np.ndarray) -> str:
    """Names a coordinate and never says what is mapped there."""
    return (
        "You are the robot; your current pose is the odom observation shown "
        "(world frame: +x is east, +y is north, coordinates in meters). Using "
        f"only the mapped point cloud, is ({gate[0]:.2f}, {gate[1]:.2f}) a spot "
        "you could stand? Answer with exactly one word: open if you could, or "
        "barrier if you could not."
    )


def _row(
    dataset: str,
    ident: str,
    label: str,
    gate: np.ndarray,
    t: float,
    context: list[list[object]],
) -> generate.Row:
    return {
        "id": ident,
        "family": "crossing",
        "type": "mcq",
        "q": _question(gate),
        "a": label,
        "choices": ["open", "barrier"],
        "context": [*context, ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]]],
        "dataset": dataset,
    }


def _widest_hole(band: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[float, np.ndarray]:
    """Longest stretch of a pane with no body-height return, and its midpoint."""
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


def barrier_rows() -> list[Candidate]:
    """Confirmed panes, at each frame where the map still shows a robot-width hole."""
    rows: list[Candidate] = []
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
                        Candidate(
                            gap,
                            gate,
                            origin,
                            _row(
                                dataset,
                                f"{dataset}_crossing_{tag}_t{t:g}",
                                "barrier",
                                gate,
                                float(t),
                                context,
                            ),
                        )
                    )
    return rows


def open_rows() -> list[Candidate]:
    """Gates the robot's base later occupied, at the barrier widths and standoffs."""
    rows: list[Candidate] = []
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
                        Candidate(
                            gap,
                            gate,
                            origin,
                            _row(
                                dataset,
                                f"{dataset}_crossing_open_t{t:g}_{len(seen)}",
                                "open",
                                gate,
                                float(t),
                                context,
                            ),
                        )
                    )
    return rows


# Frames of the confirmed panes that clear the gap and range gates. _distinct
# thins them, and _matched_open pairs each survivor with an open case of the
# same gate width.
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


def _distinct(candidates: Iterable[Candidate]) -> list[Candidate]:
    """Drop a case whose gate and robot pose both repeat an earlier one.

    Frames come every ``SAMPLE`` seconds and the robot walks slowly, so a gap
    stays in view for several of them. Same gate, same standoff, same answer:
    the later frames re-ask a question already in the set.
    """
    kept: list[Candidate] = []
    for c in candidates:
        if not any(
            np.hypot(*(c.gate - k.gate)) < SPOT and np.hypot(*(c.origin - k.origin)) < STANDOFF
            for k in kept
        ):
            kept.append(c)
    return kept


def _matched_open(barrier: list[Candidate], candidates: list[Candidate]) -> list[generate.Row]:
    """One open case per barrier case, nearest in gate width, no reuse.

    Width is visible in the cloud; unmatched, it would separate the classes.
    """
    pool = sorted(candidates, key=lambda c: str(c.row["id"]))
    picked: list[generate.Row] = []
    used: set[str] = set()
    for target in barrier:
        best = min(
            (c for c in pool if str(c.row["id"]) not in used),
            key=lambda c: abs(c.gap - target.gap),
            default=None,
        )
        if best is not None:
            used.add(str(best.row["id"]))
            picked.append(best.row)
    return picked


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    barrier = _distinct(c for c in barrier_rows() if str(c.row["id"]) in _CASES)
    return [*(c.row for c in barrier), *_matched_open(barrier, _distinct(open_rows()))]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
