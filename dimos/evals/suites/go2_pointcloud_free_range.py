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


"""Longest measured, empty bearing from a point — VQA over the go2 replays.

Rows (``go2_pointcloud_free_range_vqa.json``) are pure data emitted by
:func:`rows` — ground truth ray-marched on the full-resolution cloud, quizzing
whatever lossy encoding the agent receives for a ``PointCloud2``.

``free_range`` asks which of the eight compass bearings from a point runs
furthest through lidar-swept floor with nothing above 0.15 m before it meets
a return, a stretch nobody measured, or the cap. The cap is the distance to
the nearest edge of the sensing window, the same for every bearing, so a
diagonal that merely reaches further into the corner of the box cannot win.
The 0.15 m edge is this question's definition of "nothing on the floor"; the
encoder carries no such edge.

Rows are sliced train / holdout / spare by :mod:`dimos.evals.temp.split`.

Regenerate (needs the three recordings)::

    python -m dimos.evals.suites.go2_pointcloud_free_range
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np

from dimos.evals import generate
from dimos.evals.temp import split
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_free_range_vqa.json"

SUITE: Suite = generate.cases(
    split.assign(json.loads(_JSON.read_text())), tags=frozenset({"pointcloud"})
)

LOW_Z = 0.15  # the question's "nothing on the floor" edge, world z
HALF_WIDTH = 0.3  # half the lane width; ~Go2 body width plus margin
SELF_RETURN = 0.15  # returns closer than this along the lane are the robot's own body
START = 0.3  # the lane is judged from here out; under the robot is not asked about
BIN = 0.2  # a stretch of lane this long with no return at all ends the run
MIN_MARGIN = 0.4  # the winner must beat the runner-up by this much
MIN_RUN = 1.0  # and run at least this far, or the frame is clutter
QUERY_RADIUS = 1.0  # query points sit this close to the window centre
QUERY_CELL = 0.25
PER_FRAME = 6  # candidate points per frame before selection

_AGENTIC_TS = [
    100.0,
    300.0,
    375.0,
    500.0,
    750.0,
    900.0,
    1125.0,
    1200.0,
    1325.0,
    1350.0,
    1375.0,
    1400.0,
    1425.0,
    1450.0,
    1500.0,
]
_SHORT_TS = [5.0, 12.0, 20.0, 28.0, 36.0, 44.0, 52.0, 58.0]
_OFFICE_TS = [
    25.0,
    40.0,
    48.0,
    55.0,
    62.0,
    70.0,
    78.0,
    85.0,
    93.0,
    100.0,
    108.0,
    115.0,
    122.0,
    130.0,
]


def _question(point: np.ndarray, cap: float) -> str:
    bearings = ", ".join(generate.COMPASS)
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: "
        "+x is east, +y is north, coordinates in meters). From the world point "
        f"({point[0]:.2f}, {point[1]:.2f}) consider the eight compass bearings ({bearings}). "
        f"Along each bearing walk outward in a lane {2 * HALF_WIDTH:g} m wide, starting "
        f"{START:g} m out from the point, until the lane meets a lidar return above "
        f"z = {LOW_Z} m, meets a {BIN:g} m stretch with no lidar return at any height, or "
        f"reaches {cap:.1f} m from the point (the distance to the nearest edge of the mapped "
        "cloud, the same cap for every bearing). Using only the mapped point cloud, which "
        "bearing goes furthest before it stops? Answer with exactly one word, the bearing."
    )


def runs(pts: np.ndarray, point: np.ndarray) -> tuple[np.ndarray, float]:
    """Per compass bearing, how far the lane runs before it stops, and the cap.

    The lane stops at the first return above LOW_Z (past SELF_RETURN),
    at the first BIN of lane with no return at any height, or at the cap —
    the inscribed radius of the cloud's x-y window about the point.
    """
    xy = pts[:, :2]
    z = pts[:, 2]
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    cap = float(
        np.floor(min(point[0] - lo[0], hi[0] - point[0], point[1] - lo[1], hi[1] - point[1]) / 0.1)
        * 0.1
    )
    d = xy - point
    out = np.zeros(len(generate.COMPASS))
    n_bins = int(np.ceil(cap / BIN))
    for i in range(len(generate.COMPASS)):
        theta = np.radians(i * 45.0)
        u = np.array([np.cos(theta), np.sin(theta)])
        along = d @ u
        lane = np.abs(d @ np.array([-u[1], u[0]])) <= HALF_WIDTH
        hit = lane & (z > LOW_Z) & (along > SELF_RETURN)
        d_hit = float(along[hit].min()) if hit.any() else np.inf
        bins = np.floor(along[lane & (along >= 0)] / BIN).astype(int)
        measured = np.zeros(n_bins + 1, dtype=bool)
        measured[np.clip(bins, 0, n_bins)] = True
        first = int(START // BIN)
        empty = np.flatnonzero(~measured[first:n_bins])
        d_empty = (first + int(empty[0])) * BIN if empty.size else np.inf
        out[i] = min(d_hit, d_empty, cap)
    return out, cap


def _nearest_bearing(pts: np.ndarray, point: np.ndarray) -> int | None:
    """Compass index of the nearest return above LOW_Z — the bearing an
    obstacle-only reading is anchored on."""
    high = pts[pts[:, 2] > LOW_Z]
    d = np.hypot(high[:, 0] - point[0], high[:, 1] - point[1])
    keep = d > SELF_RETURN
    if not keep.any():
        return None
    q = high[keep][d[keep].argmin()]
    return int(np.round(np.degrees(np.arctan2(q[1] - point[1], q[0] - point[0])) / 45.0)) % 8


def _query_points(pts: np.ndarray, rng: np.random.Generator) -> list[np.ndarray]:
    """Centres of swept, empty cells near the middle of the window, shuffled."""
    xy = pts[:, :2]
    centre = (xy.min(axis=0) + xy.max(axis=0)) / 2
    origin, count, _, zmax = generate._cell_grid(pts, QUERY_CELL)
    wx, wy = generate._cell_centers(origin, QUERY_CELL, count.shape)
    ok = (count >= 3) & (zmax <= LOW_Z) & (np.hypot(wx - centre[0], wy - centre[1]) <= QUERY_RADIUS)
    points = [np.array([float(x), float(y)]) for x, y in zip(wx[ok], wy[ok], strict=True)]
    rng.shuffle(points)  # type: ignore[arg-type]
    return points[:PER_FRAME]


def free_range_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Which bearing runs furthest: one row per query point that has a clean winner.

    Built adversarially, as far as a single sweep allows: the winner must not
    be the bearing of the nearest return above LOW_Z, so a reading that
    carries obstacle distance alone and nothing about what was swept is not
    handed the answer. It must also win by MIN_MARGIN and run MIN_RUN.

    Every candidate is returned.
    """
    rng = np.random.default_rng(3)
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            for k, point in enumerate(_query_points(pts, rng)):
                lengths, cap = runs(pts, point)
                order = np.argsort(-lengths)
                win = int(order[0])
                margin = float(lengths[order[0]] - lengths[order[1]])
                if margin < MIN_MARGIN or lengths[win] < MIN_RUN:
                    continue  # no clean winner, or a point buried in clutter
                if win == _nearest_bearing(pts, point):
                    continue  # the nearest obstacle gives it away
                rows.append(
                    {
                        "id": f"{dataset}_freerange_t{t:g}_p{k}",
                        "family": "free_range",
                        "type": "mcq",
                        "q": _question(point, cap),
                        "a": generate.COMPASS[win],
                        "choices": list(generate.COMPASS),
                        "context": [
                            *context,
                            ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                        ],
                        "dataset": dataset,
                    }
                )
        return rows


# The dataset: answers spread across their range, at most three rows per scene.
_CASES = (
    "go2_china_office_freerange_t85_p1",
    "go2_agentic_20260819_freerange_t1125_p3",
    "go2_agentic_20260819_freerange_t1200_p5",
    "go2_china_office_freerange_t85_p5",
    "go2_agentic_20260819_freerange_t1125_p1",
    "go2_china_office_freerange_t108_p2",
    "go2_agentic_20260819_freerange_t1375_p5",
    "go2_china_office_freerange_t25_p1",
    "go2_china_office_freerange_t85_p2",
    "go2_agentic_20260819_freerange_t1375_p1",
    "go2_agentic_20260819_freerange_t1425_p4",
    "go2_agentic_20260819_freerange_t1125_p2",
    "go2_china_office_freerange_t108_p5",
    "go2_agentic_20260819_freerange_t1450_p3",
    "go2_china_office_freerange_t48_p5",
    "go2_short_freerange_t28_p2",
    "go2_agentic_20260819_freerange_t1425_p3",
    "go2_agentic_20260819_freerange_t1450_p1",
    "go2_agentic_20260819_freerange_t1375_p0",
    "go2_short_freerange_t28_p3",
    "go2_china_office_freerange_t55_p1",
    "go2_short_freerange_t28_p5",
    "go2_short_freerange_t52_p0",
    "go2_agentic_20260819_freerange_t1500_p1",
    "go2_agentic_20260819_freerange_t900_p1",
    "go2_agentic_20260819_freerange_t300_p4",
    "go2_short_freerange_t58_p3",
    "go2_short_freerange_t52_p4",
    "go2_agentic_20260819_freerange_t375_p1",
    "go2_agentic_20260819_freerange_t900_p5",
    "go2_agentic_20260819_freerange_t375_p3",
    "go2_agentic_20260819_freerange_t900_p0",
    "go2_china_office_freerange_t122_p0",
    "go2_china_office_freerange_t122_p2",
)


def candidates() -> dict[str, generate.Row]:
    return {
        str(r["id"]): r
        for r in (
            *free_range_rows("go2_agentic_20260819", _AGENTIC_TS),
            *free_range_rows("go2_short", _SHORT_TS),
            *free_range_rows("go2_china_office", _OFFICE_TS),
        )
    }


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    found = candidates()
    return [found[i] for i in _CASES]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
