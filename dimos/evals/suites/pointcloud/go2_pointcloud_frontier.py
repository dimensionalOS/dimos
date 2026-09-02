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


"""Unexplored space: reachable or walled off — VQA over the go2 replays.

Rows (``go2_pointcloud_frontier.json``) are pure data emitted by
:func:`rows` — ground truth by flood fill over the full-resolution cloud,
quizzing whatever lossy encoding the agent receives for a ``PointCloud2``.

The task is an exploring robot's: the lidar returned nothing from a spot, so
it is unmapped — but is it unmapped because it is *open floor you have not
driven to yet* (a place worth exploring), or because it is *behind a wall or
inside something solid* (nowhere you could go)? The agent reasons from the
mapped floor and obstacles whether an open path leads to the spot. Ten train
rows, five reachable and five walled off, plus a small holdout; the prompt
says the spot is unmapped but never which kind it is.

Regenerate (needs the recordings)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud_frontier
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy import ndimage

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_frontier.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

LOW_Z = 0.15  # returns above this are obstacles standing on the floor
CELL = 0.2
MARGIN = 0.2  # a path keeps this far from an obstacle
BODY = 0.35  # returns within this of the robot are its own body
MIN_CELLS = 6  # an unmapped patch smaller than this is not worth asking about
MIN_OPENING = 4  # cells (~0.8 m) of free floor touching a patch: a clear way in
DEDUP_M = 0.7  # patches whose centres are closer than this are the same spot
CHOICES = ("reachable", "walled")

_MEMORY_TS = [30.0, 90.0, 150.0, 240.0, 360.0, 480.0, 600.0, 660.0]
_AGENTIC_TS = [100.0, 300.0, 500.0, 900.0, 1200.0, 1450.0]
_OFFICE_TS = [25.0, 40.0, 55.0, 70.0, 93.0, 108.0, 122.0, 130.0]
_SHORT_TS = [5.0, 20.0, 36.0, 44.0, 52.0, 58.0]


def _question(point: np.ndarray) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: +x is "
        f"east, +y is north, coordinates in meters). The lidar returned nothing from the area "
        f"around the world point ({point[0]:.2f}, {point[1]:.2f}), so it is unmapped. Reasoning "
        "only from the floor and obstacles the lidar did map, could you drive there across mapped "
        "floor to explore it, or is it closed off from you behind walls or other obstacles? Answer "
        "with one word: reachable if an open path over mapped floor leads to it, or walled if it "
        "is blocked off. Use only the mapped point cloud."
    )


def classify(pts: np.ndarray, robot: np.ndarray) -> list[tuple[float, float, str, int]]:
    """Every sizeable unmapped patch, as ``(x, y, reachable|walled, cells)``.

    Free floor the robot can reach is flood-filled from its own cell over
    swept-empty cells that stay ``MARGIN`` from any obstacle. An unmapped
    patch touching that reachable floor is ``reachable``; one that instead
    abuts an obstacle, with no reachable floor beside it, is ``walled``.
    Patches that are neither (unreachable but not against structure) are left
    out — the question is only asked where the answer is clean.
    """
    xy = pts[:, :2]
    origin = np.floor(xy.min(axis=0) / CELL) * CELL
    shape = np.floor((xy.max(axis=0) - origin) / CELL).astype(np.int64) + 1
    nx, ny = int(shape[0]), int(shape[1])
    ij = np.floor((xy - origin) / CELL).astype(np.int64)
    lin = ij[:, 1] * nx + ij[:, 0]
    count = np.bincount(lin, minlength=nx * ny).reshape(ny, nx)
    zmax: np.ndarray = np.full(nx * ny, -np.inf)
    np.maximum.at(zmax, lin, pts[:, 2])
    zmax = zmax.reshape(ny, nx)
    wx, wy = generate._cell_centers(origin, CELL, (ny, nx))
    r = np.hypot(wx - robot[0], wy - robot[1])
    measured = count > 0
    obstacle = (zmax > LOW_Z) & (r > BODY)
    passable = ndimage.distance_transform_edt(~obstacle) * CELL > MARGIN
    free = measured & passable
    labels, _ = ndimage.label(free, structure=np.ones((3, 3)))
    start = {int(v) for v in labels[r <= BODY] if v}
    reachable_free = np.isin(labels, list(start)) if start else np.zeros_like(free)
    near_reach = ndimage.binary_dilation(
        reachable_free, structure=ndimage.generate_binary_structure(2, 1)
    )
    near_obstacle = ndimage.binary_dilation(obstacle)
    empty = ~measured
    patch_labels, n = ndimage.label(empty, structure=np.ones((3, 3)))
    out: list[tuple[float, float, str, int]] = []
    for label in range(1, n + 1):
        cells = patch_labels == label
        size = int(cells.sum())
        if size < MIN_CELLS:
            continue
        touches_reach = bool((cells & near_reach & passable).any())
        touches_obstacle = bool((cells & near_obstacle).any())
        if touches_reach:
            kind = "reachable"
        elif touches_obstacle:
            kind = "walled"
        else:
            continue
        out.append(
            (round(float(wx[cells].mean()), 2), round(float(wy[cells].mean()), 2), kind, size)
        )
    return out


def frontier_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Candidate rows, each tagged ``kind`` reachable or walled for the split in :func:`rows`."""
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            for x, y, kind, _size in classify(pts, robot):
                point = np.array([x, y])
                rows.append(
                    {
                        "id": f"{dataset}_frontier_t{t:g}_{x:g}_{y:g}",
                        "family": "frontier",
                        "type": "mcq",
                        "q": _question(point),
                        "a": kind,
                        "choices": list(CHOICES),
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
    "go2_agentic_20260819_frontier_t1450_-0.55_-3.06",
    "go2_agentic_memory_20260820_frontier_t150_-1.43_9.79",
    "go2_agentic_memory_20260820_frontier_t30_4.49_6.51",
    "go2_agentic_memory_20260820_frontier_t480_3.58_10",
    "go2_agentic_memory_20260820_frontier_t600_-0.04_2.7",
    "go2_agentic_20260819_frontier_t900_-4.3_-0.54",
    "go2_china_office_frontier_t108_4.1_-0.34",
    "go2_china_office_frontier_t25_3.6_-2.02",
    "go2_china_office_frontier_t55_6.92_1.46",
    "go2_china_office_frontier_t93_1.54_2.3",
)
_HOLDOUT = (
    "go2_agentic_20260819_frontier_t100_-1.15_0.18",
    "go2_agentic_20260819_frontier_t1200_-0.28_6.83",
    "go2_agentic_20260819_frontier_t300_-3.66_2.69",
    "go2_agentic_20260819_frontier_t500_-0.82_-5.13",
)


def candidates() -> dict[str, generate.Row]:
    return {
        str(r["id"]): r
        for r in (
            *frontier_rows("go2_agentic_memory_20260820", _MEMORY_TS),
            *frontier_rows("go2_agentic_20260819", _AGENTIC_TS),
            *frontier_rows("go2_china_office", _OFFICE_TS),
            *frontier_rows("go2_short", _SHORT_TS),
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
