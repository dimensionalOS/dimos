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


"""Gap between things you could pass through — VQA over the go2 replays.

Rows (``go2_pointcloud_gap_width.json``) are pure data emitted by
:func:`rows` — ground truth by single-linkage grouping of the
full-resolution returns, quizzing whatever lossy encoding the agent receives
for a ``PointCloud2``.

The task: near a point, are there two or more separate things standing above
the floor with a gap between them — the kind of gap a robot picks its way
through? If so, where is the narrowest gap and how wide is it. Ten rows, five
with two or more separate things and five with at most one; the prompt does
not say which. The answer locates the narrowest gap and its width, or is
``none``.

Ten hand-picked rows, tagged ``train``: too few to slice.

Regenerate (needs the three recordings)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud_gap_width
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from dimos.evals.suites.pointcloud.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_gap_width.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

LOW_Z = 0.15  # returns above this are structure
REACH = 2.0  # returns within this of the point are considered
LINK = 0.4  # returns this close are one thing
MIN_PTS = 10  # smaller groups are ignored
BODY = 0.35  # returns within this of the robot are its own body
MIN_GAP, MAX_GAP = 0.5, 2.0  # a real gap sits in this range; narrower is one wall fragmenting
RADIUS = 1.0  # match tolerance on the gap's location, meters
VALUE_BAND = 0.3  # band on the width
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
    1400.0,
    1450.0,
    1500.0,
]
_SHORT_TS = [5.0, 20.0, 36.0, 44.0, 52.0, 58.0]
_OFFICE_TS = [25.0, 40.0, 55.0, 62.0, 78.0, 93.0, 108.0, 122.0, 130.0]


def _question(point: np.ndarray) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: +x is "
        f"east, +y is north, coordinates in meters). Within {REACH:g} m of the world point "
        f"({point[0]:.2f}, {point[1]:.2f}), are there two or more separate things standing above "
        f"the floor (returns above z = {LOW_Z} m, not your own body within {BODY:g} m of you) "
        "with a gap between them? If so, give the location of the narrowest gap between two "
        "separate things and how wide it is, as x,y,width on one line. If there is at most one "
        "such thing, or none, answer none. Use only the mapped point cloud."
    )


def narrowest_gap(
    pts: np.ndarray, point: np.ndarray, robot: np.ndarray
) -> tuple[float, np.ndarray] | None:
    """Narrowest gap ``(width, midpoint)`` between two things, or None with fewer than two."""
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
    best: tuple[float, np.ndarray] | None = None
    for a in range(len(groups)):
        tree_a = cKDTree(groups[a])
        for b in range(a + 1, len(groups)):
            d, idx = tree_a.query(groups[b])
            m = int(d.argmin())
            if best is None or d[m] < best[0]:
                best = (float(d[m]), (groups[a][idx[m]] + groups[b][m]) / 2)
    return best


def gap_width_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Candidate rows, each tagged ``kind`` gap or none for the split in :func:`rows`."""
    rng = np.random.default_rng(5)
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            points = [robot] + [robot + rng.uniform(-1.5, 1.5, 2) for _ in range(PER_FRAME - 1)]
            for k, point in enumerate(points):
                found = narrowest_gap(pts, point, robot)
                if found is not None and MIN_GAP <= found[0] <= MAX_GAP:
                    mid = found[1]
                    kind, answer = (
                        "gap",
                        [[round(float(mid[0]), 2), round(float(mid[1]), 2), round(found[0], 2)]],
                    )
                elif found is None:
                    kind, answer = "none", []
                else:
                    continue  # a gap outside the band — ambiguous, dropped
                rows.append(
                    {
                        "id": f"{dataset}_gapwidth_t{t:g}_p{k}",
                        "family": "gap_width",
                        "type": "coords",
                        "q": _question(point),
                        "a": answer,
                        "radius": RADIUS,
                        "value_band": VALUE_BAND,
                        "kind": kind,
                        "context": [
                            *context,
                            ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                        ],
                        "dataset": dataset,
                    }
                )
        return rows


# Five positive and five negative, one per scene; holdout shares no scene block.
_TRAIN = (
    "go2_agentic_20260819_gapwidth_t1400_p0",
    "go2_agentic_20260819_gapwidth_t1450_p0",
    "go2_agentic_20260819_gapwidth_t1500_p2",
    "go2_agentic_20260819_gapwidth_t750_p2",
    "go2_china_office_gapwidth_t108_p0",
    "go2_agentic_20260819_gapwidth_t300_p1",
    "go2_agentic_20260819_gapwidth_t500_p0",
    "go2_agentic_20260819_gapwidth_t900_p2",
    "go2_china_office_gapwidth_t62_p2",
    "go2_short_gapwidth_t20_p1",
)
_HOLDOUT = (
    "go2_agentic_20260819_gapwidth_t100_p2",
    "go2_agentic_20260819_gapwidth_t1125_p0",
    "go2_agentic_20260819_gapwidth_t1200_p1",
    "go2_agentic_20260819_gapwidth_t1325_p0",
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
    """The committed rows: curated ids, each tagged with its split."""
    found = candidates()
    out = []
    for split, ids in (("train", _TRAIN), ("holdout", _HOLDOUT)):
        for i in ids:
            row = dict(found[i])
            row.pop("kind", None)
            row["split"] = split
            out.append(row)
    return out


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
