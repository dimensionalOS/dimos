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


"""Room to set something down — VQA over the go2 replays.

Rows (``go2_pointcloud_free_disk_vqa.json``) are pure data emitted by
:func:`rows` — ground truth from a distance transform over the
full-resolution cloud, quizzing whatever lossy encoding the agent receives
for a ``PointCloud2``.

The task: the robot needs a clear open spot near a point to set down what it
carries. Is there a patch of floor the lidar mapped and found nothing on
(above 0.15 m) large enough for a circle at least 1.8 m across, within about
2.5 m of the point? If so, where is the centre of the largest such spot. Ten
rows, five with a spot and five without; the prompt does not say which. Floor
the sensor never swept cannot be a place to set something down, so it does
not count.

Ten hand-picked rows, tagged ``train``.

Regenerate (needs the three recordings)::

    uv run python -m dimos.evals.suites.go2_pointcloud_free_disk
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy import ndimage

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_free_disk_vqa.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

LOW_Z = 0.15  # the question's "nothing on the floor" edge, world z
CELL = 0.2
NEAR = 2.5  # the centre lies within this of the named point
BODY = 0.35  # returns within this of the robot are its own body
BIG = 1.1  # a spot exists when the largest clear circle's radius is at least this
SMALL = 0.9  # and there is none when it is under this — the negative
SPREAD = 1.0  # a positive's plateau of best centres must fit in this, or it is ambiguous
RADIUS = 1.0  # match tolerance on the centre, meters
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
    1400.0,
    1450.0,
    1500.0,
]
_SHORT_TS = [5.0, 20.0, 36.0, 44.0, 52.0, 58.0]
_OFFICE_TS = [25.0, 40.0, 55.0, 62.0, 78.0, 93.0, 108.0, 122.0, 130.0]


def _question(point: np.ndarray) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: +x is "
        f"east, +y is north, coordinates in meters). You need to set down what you are carrying "
        f"on clear, open floor near the world point ({point[0]:.2f}, {point[1]:.2f}). Is there a "
        f"patch of floor the lidar has mapped and found nothing standing on, above z = {LOW_Z} m, "
        f"within about {NEAR:g} m of that point, big enough to hold a circle at least "
        f"{2 * BIG:g} m across? If so, give the centre of the largest such patch as x,y on one "
        "line. If there is no clear mapped patch that large, answer none. Floor the lidar never "
        "reached does not count. Use only the mapped point cloud."
    )


def largest_disk(
    pts: np.ndarray, robot: np.ndarray, point: np.ndarray
) -> tuple[float, float, float, float]:
    """``(x, y, radius, spread)`` of the largest swept, empty disk centred near ``point``.

    A cell is swept when it holds a return and none above ``LOW_Z``; cells
    within ``BODY`` of the robot hold its own body and count as swept if they
    hold any return. The radius is the distance to the nearest non-swept cell,
    less half a cell so the circle stops at that cell's near edge. ``spread``
    is how far apart the near-best centres lie — large when the best circle is
    a plateau rather than a point.
    """
    origin, count, _, zmax = generate._cell_grid(pts, CELL)
    wx, wy = generate._cell_centers(origin, CELL, count.shape)
    r = np.hypot(wx - robot[0], wy - robot[1])
    measured = count > 0
    swept = measured & ((zmax <= LOW_Z) | (r <= BODY))
    edt = ndimage.distance_transform_edt(np.pad(swept, 1))[1:-1, 1:-1] * CELL - CELL / 2
    edt = np.where(np.hypot(wx - point[0], wy - point[1]) <= NEAR, edt, 0.0)
    j, i = np.unravel_index(int(edt.argmax()), edt.shape)
    best = float(edt[j, i])
    near_best = edt >= best - CELL
    spread = float(np.hypot(wx[near_best] - wx[j, i], wy[near_best] - wy[j, i]).max())
    return float(wx[j, i]), float(wy[j, i]), best, spread


def free_disk_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Candidate rows, each tagged ``kind`` spot or none for the split in :func:`rows`."""
    rng = np.random.default_rng(7)
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            points = [robot] + [robot + rng.uniform(-1.5, 1.5, 2) for _ in range(PER_FRAME - 1)]
            for k, point in enumerate(points):
                x, y, radius, spread = largest_disk(pts, robot, point)
                if radius >= BIG and spread <= SPREAD:
                    kind, answer = "spot", [[round(x, 2), round(y, 2)]]
                elif radius < SMALL:
                    kind, answer = "none", []
                else:
                    continue
                rows.append(
                    {
                        "id": f"{dataset}_freedisk_t{t:g}_p{k}",
                        "family": "free_disk",
                        "type": "coords",
                        "q": _question(point),
                        "a": answer,
                        "radius": RADIUS,
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
    "go2_agentic_20260819_freedisk_t375_p0",
    "go2_china_office_freedisk_t122_p0",
    "go2_china_office_freedisk_t55_p0",
    "go2_china_office_freedisk_t78_p0",
    "go2_china_office_freedisk_t130_p0",
    "go2_agentic_20260819_freedisk_t500_p0",
    "go2_agentic_20260819_freedisk_t750_p0",
    "go2_china_office_freedisk_t25_p2",
    "go2_short_freedisk_t20_p0",
    "go2_agentic_20260819_freedisk_t750_p1",
)
_HOLDOUT = (
    "go2_agentic_20260819_freedisk_t1200_p0",
    "go2_agentic_20260819_freedisk_t1400_p0",
    "go2_agentic_20260819_freedisk_t1450_p0",
    "go2_agentic_20260819_freedisk_t1500_p0",
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
