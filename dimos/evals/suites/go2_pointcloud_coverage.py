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


"""Fraction of a square the lidar reached — VQA over the go2 replays.

Rows (``go2_pointcloud_coverage_vqa.json``) are pure data emitted by
:func:`rows` — ground truth a cell mask over the full-resolution cloud,
quizzing whatever lossy encoding the agent receives for a ``PointCloud2``.

``coverage`` asks what fraction of a named 3 m square holds any lidar
return at all, in 0.25 m cells. It is the plainest form of the
measured-versus-not distinction: no heights, no structure, just where the
sensor reached. Squares are chosen so the answers spread from partly covered
to full.

Rows are sliced train / holdout / spare by :mod:`dimos.evals.temp.split`.

Regenerate (needs the three recordings)::

    python -m dimos.evals.suites.go2_pointcloud_coverage
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np

from dimos.evals import generate
from dimos.evals.temp import split
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_coverage_vqa.json"

SUITE: Suite = generate.cases(
    split.assign(json.loads(_JSON.read_text())), tags=frozenset({"pointcloud"})
)

SIDE = 3.0
CELL = 0.25
BAND = 0.15  # within() band on the fraction
PER_FRAME = 4  # squares per frame before selection
MIN_FRACTION = 0.2  # squares mostly outside the cloud are dropped

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


def _question(corner: np.ndarray) -> str:
    n = round(SIDE / CELL)
    return (
        "You are shown the point cloud the robot mapped (world frame: +x is east, +y is north, "
        f"coordinates in meters). Consider the {SIDE:g} m by {SIDE:g} m square with south-west "
        f"corner ({corner[0]:.2f}, {corner[1]:.2f}) and north-east corner "
        f"({corner[0] + SIDE:.2f}, {corner[1] + SIDE:.2f}), divided into {CELL:g} m cells, "
        f"{n} by {n}. Using only the mapped point cloud, what fraction of those {n * n} cells "
        "contain at least one lidar return, at any height? Answer with a single number "
        "between 0 and 1."
    )


def fraction(pts: np.ndarray, corner: np.ndarray) -> float:
    """Share of the square's cells holding a return."""
    n = round(SIDE / CELL)
    ij = np.floor((pts[:, :2] - corner) / CELL).astype(np.int64)
    inside = (ij >= 0).all(axis=1) & (ij < n).all(axis=1)
    return float(np.unique(ij[inside], axis=0).shape[0]) / (n * n)


def coverage_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Squares around the robot, corners snapped to the cell grid."""
    rng = np.random.default_rng(11)
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            for k in range(PER_FRAME):
                corner = np.floor((robot + rng.uniform(-SIDE - 0.5, 0.5, 2)) / CELL) * CELL
                share = fraction(pts, corner)
                if share < MIN_FRACTION:
                    continue
                rows.append(
                    {
                        "id": f"{dataset}_coverage_t{t:g}_s{k}",
                        "family": "coverage",
                        "type": "numeric",
                        "q": _question(corner),
                        "a": round(share, 2),
                        "band": BAND,
                        "context": context,
                        "dataset": dataset,
                    }
                )
        return rows


# The dataset: answers spread across their range, at most three rows per scene.
_CASES = (
    "go2_agentic_20260819_coverage_t100_s0",
    "go2_agentic_20260819_coverage_t100_s1",
    "go2_agentic_20260819_coverage_t1125_s0",
    "go2_agentic_20260819_coverage_t1125_s1",
    "go2_agentic_20260819_coverage_t1375_s0",
    "go2_agentic_20260819_coverage_t1200_s2",
    "go2_agentic_20260819_coverage_t1325_s1",
    "go2_agentic_20260819_coverage_t1200_s0",
    "go2_agentic_20260819_coverage_t1500_s2",
    "go2_agentic_20260819_coverage_t1375_s1",
    "go2_agentic_20260819_coverage_t1325_s2",
    "go2_agentic_20260819_coverage_t1400_s0",
    "go2_china_office_coverage_t55_s2",
    "go2_agentic_20260819_coverage_t1450_s0",
    "go2_agentic_20260819_coverage_t1400_s2",
    "go2_agentic_20260819_coverage_t1425_s0",
    "go2_short_coverage_t52_s2",
    "go2_agentic_20260819_coverage_t1450_s2",
    "go2_agentic_20260819_coverage_t1425_s1",
    "go2_agentic_20260819_coverage_t300_s0",
    "go2_agentic_20260819_coverage_t1500_s1",
    "go2_agentic_20260819_coverage_t300_s3",
    "go2_agentic_20260819_coverage_t375_s0",
    "go2_agentic_20260819_coverage_t375_s2",
    "go2_agentic_20260819_coverage_t500_s3",
    "go2_agentic_20260819_coverage_t750_s1",
    "go2_agentic_20260819_coverage_t500_s0",
    "go2_agentic_20260819_coverage_t750_s2",
    "go2_agentic_20260819_coverage_t900_s1",
    "go2_agentic_20260819_coverage_t900_s2",
    "go2_china_office_coverage_t100_s0",
    "go2_china_office_coverage_t100_s2",
    "go2_china_office_coverage_t25_s1",
    "go2_china_office_coverage_t122_s1",
    "go2_china_office_coverage_t40_s3",
    "go2_china_office_coverage_t130_s0",
)


def candidates() -> dict[str, generate.Row]:
    return {
        str(r["id"]): r
        for r in (
            *coverage_rows("go2_agentic_20260819", _AGENTIC_TS),
            *coverage_rows("go2_short", _SHORT_TS),
            *coverage_rows("go2_china_office", _OFFICE_TS),
        )
    }


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    found = candidates()
    return [found[i] for i in _CASES]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
