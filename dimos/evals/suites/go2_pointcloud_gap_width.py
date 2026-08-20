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


"""Narrowest gap between structures near a point — VQA over the go2 replays.

Rows (``go2_pointcloud_gap_width_vqa.json``) are pure data emitted by
:func:`rows` — ground truth by single-linkage grouping of the
full-resolution returns, quizzing whatever lossy encoding the agent receives
for a ``PointCloud2``.

``gap_width`` asks how wide the narrowest gap is between two different
groups of returns above 0.15 m within 2 m of a point. Groups are returns
within 0.4 m of one another, chained; groups under ten returns are ignored.
Frames whose narrowest gap is under 0.5 m are dropped — that is one wall
fragmenting at range, not a gap — and so are frames with one group. The
answer is a width, nothing is said about what the gap is in.

Rows are sliced train / holdout / spare by :mod:`dimos.evals.temp.split`.

Regenerate (needs the three recordings)::

    python -m dimos.evals.suites.go2_pointcloud_gap_width
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from dimos.evals import generate
from dimos.evals.temp import split
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_gap_width_vqa.json"

SUITE: Suite = generate.cases(
    split.assign(json.loads(_JSON.read_text())), tags=frozenset({"pointcloud"})
)

LOW_Z = 0.15  # returns above this are structure
REACH = 2.0  # returns within this of the point are considered
LINK = 0.4  # returns this close are one group
MIN_PTS = 10  # smaller groups are ignored
BODY = 0.35  # returns within this of the robot are its own body
MIN_GAP, MAX_GAP = 0.5, 2.0  # rows outside this are dropped
BAND = 0.3  # within() band on the answer, meters
PER_FRAME = 3  # query points per frame: the robot and two offsets

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


def _question(point: np.ndarray) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: "
        "+x is east, +y is north, coordinates in meters). Consider the lidar returns above "
        f"z = {LOW_Z} m within {REACH:g} m of the world point ({point[0]:.2f}, {point[1]:.2f}), "
        f"leaving out returns within {BODY:g} m of your own position (your body). Group them: "
        f"two returns are in the same group when they are within {LINK:g} m of each other, "
        f"directly or through a chain of such returns; ignore groups with fewer than {MIN_PTS} "
        "returns. Using only the mapped point cloud, what is the narrowest gap between two "
        "different groups — the smallest horizontal distance from a return in one group to a "
        "return in another — in meters? Answer with a single number."
    )


def narrowest_gap(pts: np.ndarray, point: np.ndarray, robot: np.ndarray) -> float | None:
    """Smallest distance between two different groups, or None with fewer than two groups."""
    near = np.hypot(pts[:, 0] - point[0], pts[:, 1] - point[1]) <= REACH
    body = np.hypot(pts[:, 0] - robot[0], pts[:, 1] - robot[1]) <= BODY
    high = pts[near & ~body & (pts[:, 2] > LOW_Z)][:, :2].astype(np.float64)
    if high.shape[0] < 2 * MIN_PTS:
        return None
    tree = cKDTree(high)
    _, label = connected_components(tree.sparse_distance_matrix(tree, LINK), directed=False)
    groups = [high[label == g] for g in np.flatnonzero(np.bincount(label) >= MIN_PTS)]
    if len(groups) < 2:
        return None
    gaps = []
    for a in range(len(groups)):
        tree_a = cKDTree(groups[a])
        for b in range(a + 1, len(groups)):
            d, _ = tree_a.query(groups[b])
            gaps.append(float(d.min()))
    return min(gaps)


def gap_width_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """One row per (frame, point) with at least two groups and a gap worth asking about."""
    rng = np.random.default_rng(5)
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            points = [robot] + [robot + rng.uniform(-1.5, 1.5, 2) for _ in range(PER_FRAME - 1)]
            for k, point in enumerate(points):
                gap = narrowest_gap(pts, point, robot)
                if gap is None or not MIN_GAP <= gap <= MAX_GAP:
                    continue
                rows.append(
                    {
                        "id": f"{dataset}_gapwidth_t{t:g}_p{k}",
                        "family": "gap_width",
                        "type": "numeric",
                        "q": _question(point),
                        "a": round(gap, 2),
                        "band": BAND,
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
    "go2_agentic_20260819_gapwidth_t1125_p1",
    "go2_agentic_20260819_gapwidth_t100_p2",
    "go2_agentic_20260819_gapwidth_t1125_p0",
    "go2_agentic_20260819_gapwidth_t1125_p2",
    "go2_agentic_20260819_gapwidth_t1375_p0",
    "go2_agentic_20260819_gapwidth_t1400_p0",
    "go2_agentic_20260819_gapwidth_t1350_p2",
    "go2_agentic_20260819_gapwidth_t1375_p1",
    "go2_china_office_gapwidth_t115_p2",
    "go2_agentic_20260819_gapwidth_t1400_p2",
    "go2_agentic_20260819_gapwidth_t1375_p2",
    "go2_china_office_gapwidth_t130_p0",
    "go2_agentic_20260819_gapwidth_t1450_p0",
    "go2_agentic_20260819_gapwidth_t1500_p1",
    "go2_china_office_gapwidth_t40_p0",
    "go2_agentic_20260819_gapwidth_t1450_p1",
    "go2_agentic_20260819_gapwidth_t1500_p2",
    "go2_china_office_gapwidth_t40_p2",
    "go2_agentic_20260819_gapwidth_t1450_p2",
    "go2_agentic_20260819_gapwidth_t375_p2",
    "go2_agentic_20260819_gapwidth_t1500_p0",
    "go2_china_office_gapwidth_t100_p0",
    "go2_agentic_20260819_gapwidth_t300_p0",
    "go2_china_office_gapwidth_t108_p0",
    "go2_agentic_20260819_gapwidth_t300_p2",
    "go2_china_office_gapwidth_t48_p0",
    "go2_agentic_20260819_gapwidth_t750_p2",
    "go2_china_office_gapwidth_t70_p1",
    "go2_china_office_gapwidth_t130_p1",
    "go2_china_office_gapwidth_t85_p1",
    "go2_china_office_gapwidth_t130_p2",
    "go2_china_office_gapwidth_t85_p2",
    "go2_short_gapwidth_t20_p0",
    "go2_short_gapwidth_t12_p0",
    "go2_short_gapwidth_t20_p1",
    "go2_short_gapwidth_t36_p0",
)


def candidates() -> dict[str, generate.Row]:
    return {
        str(r["id"]): r
        for r in (
            *gap_width_rows("go2_agentic_20260819", _AGENTIC_TS),
            *gap_width_rows("go2_short", _SHORT_TS),
            *gap_width_rows("go2_china_office", _OFFICE_TS),
        )
    }


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    found = candidates()
    return [found[i] for i in _CASES]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
