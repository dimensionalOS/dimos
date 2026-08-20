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


"""Largest swept, empty circle near the robot — VQA over the go2 replays.

Rows (``go2_pointcloud_free_disk_vqa.json``) are pure data emitted by
:func:`rows` — ground truth from a distance transform over the
full-resolution cloud, quizzing whatever lossy encoding the agent receives
for a ``PointCloud2``.

``free_disk`` asks for the centre and radius of the largest circle of
cells that the lidar swept and found nothing above 0.15 m in. An unmeasured
cell ruins a circle as surely as a return does, so the reading has to carry
the difference between floor that was seen empty and floor nobody looked at.
Scored by :func:`~dimos.evals.scorers.matched_set` at 1.0 m on the centre
with a 0.4 m band on the radius. Frames whose largest circle has several
equally good centres more than 1.0 m apart are dropped rather than asked.

Rows are sliced train / holdout / spare by :mod:`dimos.evals.temp.split`.

Regenerate (needs the three recordings)::

    python -m dimos.evals.suites.go2_pointcloud_free_disk
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy import ndimage

from dimos.evals import generate
from dimos.evals.temp import split
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_free_disk_vqa.json"

SUITE: Suite = generate.cases(
    split.assign(json.loads(_JSON.read_text())), tags=frozenset({"pointcloud"})
)

LOW_Z = 0.15  # the question's "nothing on the floor" edge, world z
CELL = 0.2
NEAR = 2.5  # the centre lies within this of the named point
BODY = 0.35  # returns within this of the robot are its own body
MIN_RADIUS = 0.8  # smaller circles are not worth asking about
RADIUS = 1.0  # match tolerance on the centre, meters
VALUE_BAND = 0.4  # band on the radius
SPREAD = 1.0  # the plateau of best centres must fit in this
PER_FRAME = 3  # named points per frame: the robot and two offsets

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
        "You are the robot; your current pose is the odom observation shown (world frame: +x is "
        f"east, +y is north, coordinates in meters). Cells are {CELL:g} m squares aligned to "
        f"multiples of {CELL:g} m. Call a cell swept if it holds at least one lidar return and "
        f"none of its returns is above z = {LOW_Z} m; cells within {BODY:g} m of your own "
        "position hold your body's returns and count as swept if they hold any return. Consider "
        f"circles whose centre is within {NEAR:g} m of the world point ({point[0]:.2f}, "
        f"{point[1]:.2f}) and which cover swept cells only: no cell inside the circle may be "
        f"without returns or hold a return above z = {LOW_Z} m, and the circle may not reach "
        "past the edge of the mapped cloud. Using only the mapped point cloud, give the largest "
        "such circle as its centre and radius, x,y,r on one line, nothing else."
    )


def largest_disk(
    pts: np.ndarray, robot: np.ndarray, point: np.ndarray
) -> tuple[float, float, float, float]:
    """``(x, y, radius, spread)`` of the largest swept, empty disk centred near ``point``.

    spread is how far apart the centres within one cell of the best
    radius lie — large when the answer is a plateau rather than a point.
    """
    origin, count, _, zmax = generate._cell_grid(pts, CELL)
    wx, wy = generate._cell_centers(origin, CELL, count.shape)
    r = np.hypot(wx - robot[0], wy - robot[1])
    measured = count > 0
    swept = measured & ((zmax <= LOW_Z) | (r <= BODY))
    edt = ndimage.distance_transform_edt(np.pad(swept, 1))[1:-1, 1:-1] * CELL
    edt = np.where(np.hypot(wx - point[0], wy - point[1]) <= NEAR, edt, 0.0)
    j, i = np.unravel_index(int(edt.argmax()), edt.shape)
    best = float(edt[j, i])
    near_best = edt >= best - CELL
    spread = float(np.hypot(wx[near_best] - wx[j, i], wy[near_best] - wy[j, i]).max())
    return float(wx[j, i]), float(wy[j, i]), best, spread


def free_disk_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """One row per (frame, point) whose largest swept disk is big and has one centre."""
    rng = np.random.default_rng(7)
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            points = [robot] + [robot + rng.uniform(-1.5, 1.5, 2) for _ in range(PER_FRAME - 1)]
            for k, point in enumerate(points):
                x, y, radius, spread = largest_disk(pts, robot, point)
                if radius < MIN_RADIUS or spread > SPREAD:
                    continue
                rows.append(
                    {
                        "id": f"{dataset}_freedisk_t{t:g}_p{k}",
                        "family": "free_disk",
                        "type": "coords",
                        "q": _question(point),
                        "a": [[round(x, 2), round(y, 2), round(radius, 2)]],
                        "radius": RADIUS,
                        "value_band": VALUE_BAND,
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
    "go2_agentic_20260819_freedisk_t100_p2",
    "go2_china_office_freedisk_t108_p1",
    "go2_short_freedisk_t36_p2",
    "go2_agentic_20260819_freedisk_t1125_p0",
    "go2_china_office_freedisk_t122_p0",
    "go2_short_freedisk_t52_p1",
    "go2_agentic_20260819_freedisk_t1125_p1",
    "go2_china_office_freedisk_t122_p1",
    "go2_agentic_20260819_freedisk_t1200_p0",
    "go2_china_office_freedisk_t122_p2",
    "go2_agentic_20260819_freedisk_t1200_p1",
    "go2_china_office_freedisk_t25_p2",
    "go2_agentic_20260819_freedisk_t1200_p2",
    "go2_china_office_freedisk_t48_p1",
    "go2_agentic_20260819_freedisk_t1350_p0",
    "go2_china_office_freedisk_t55_p0",
    "go2_agentic_20260819_freedisk_t1350_p1",
    "go2_china_office_freedisk_t55_p1",
    "go2_agentic_20260819_freedisk_t1350_p2",
    "go2_china_office_freedisk_t62_p0",
    "go2_agentic_20260819_freedisk_t1375_p0",
    "go2_china_office_freedisk_t62_p1",
    "go2_agentic_20260819_freedisk_t1400_p0",
    "go2_china_office_freedisk_t62_p2",
    "go2_agentic_20260819_freedisk_t1400_p1",
    "go2_china_office_freedisk_t93_p0",
    "go2_agentic_20260819_freedisk_t1400_p2",
    "go2_china_office_freedisk_t93_p2",
    "go2_agentic_20260819_freedisk_t1425_p0",
    "go2_agentic_20260819_freedisk_t1425_p1",
    "go2_agentic_20260819_freedisk_t1425_p2",
    "go2_agentic_20260819_freedisk_t1450_p0",
    "go2_agentic_20260819_freedisk_t1450_p1",
    "go2_agentic_20260819_freedisk_t1450_p2",
    "go2_agentic_20260819_freedisk_t375_p0",
    "go2_agentic_20260819_freedisk_t375_p1",
)


def candidates() -> dict[str, generate.Row]:
    return {
        str(r["id"]): r
        for r in (
            *free_disk_rows("go2_agentic_20260819", _AGENTIC_TS),
            *free_disk_rows("go2_short", _SHORT_TS),
            *free_disk_rows("go2_china_office", _OFFICE_TS),
        )
    }


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    found = candidates()
    return [found[i] for i in _CASES]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
