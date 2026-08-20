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


"""Lowest return at a point, relative to the robot's cell — VQA over the go2 replays.

Rows (``go2_pointcloud_floor_height_vqa.json``) are pure data emitted by
:func:`rows` — ground truth read off the full-resolution cloud, quizzing
whatever lossy encoding the agent receives for a ``PointCloud2``.

``floor_height`` asks for the lowest lidar return in the 0.25 m cell at a
named point minus the lowest return in the cell the robot stands in. Half the
rows sit on the same level as the robot; the other half are cells a patch of
something higher or lower, so a reading that carries no elevation per cell
can be right on neither half by knowing the other. The band is 0.2 m: per-cell
min z on flat floor wanders by about one 0.1 m level between neighbours.

``go2_stairs_20260819`` carries the raised patch the hand-authored floor
rows label; the frames here are the moving stretches between those rows'
frames. ``go2_teleop_20260819`` is absent on purpose: its map has a false
depression from the robot being carried before the recording started.

Rows are sliced train / holdout / spare by :mod:`dimos.evals.temp.split`.

Regenerate (needs the four recordings)::

    python -m dimos.evals.suites.go2_pointcloud_floor_height
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np

from dimos.evals import generate
from dimos.evals.temp import split
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_floor_height_vqa.json"

SUITE: Suite = generate.cases(
    split.assign(json.loads(_JSON.read_text())), tags=frozenset({"pointcloud"})
)

CELL = 0.25
BAND = 0.2  # within() band on the answer, meters
MIN_PTS = 10  # returns a cell needs before its lowest is trusted
ROBOT_MIN_PTS = 5
NEAR, FAR = 0.5, 3.0  # where query cells are taken from, meters from the robot
FLAT = 0.05  # |dz| at or under this is the robot's own level
OFF = 0.2  # |dz| at or over this is a different level
PATCH = 3  # an off-level cell needs this many like neighbours, or it is speckle

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
_SHORT_TS = [5.0, 20.0, 40.0, 58.0]
_OFFICE_TS = [25.0, 48.0, 62.0, 85.0, 100.0, 122.0, 130.0]
_STAIRS_TS = [20.0, 30.0, 50.0, 60.0, 70.0, 80.0, 110.0]


def _question(point: np.ndarray) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: "
        "+x is east, +y is north, coordinates in meters). Cells are "
        f"{CELL:g} m squares aligned to multiples of {CELL:g} m in world x and y. Take the "
        f"cell containing the world point ({point[0]:.3f}, {point[1]:.3f}) and the cell "
        "containing your own position; in each, find the lowest lidar return (smallest z). "
        "Using only the mapped point cloud, what is the lowest return at the named point "
        "minus the lowest return at your position, in meters? Answer with a single signed number."
    )


def floor_height_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Lowest return per cell relative to the robot's own cell.

    Candidate cells lie NEAR..FAR from the robot and hold MIN_PTS
    returns. A cell counts as off-level when PATCH of its eight neighbours
    agree with it to within 0.1 m — one odd cell is lidar speckle, not a level.
    Every flat and every off-level candidate is returned, labelled by
    kind for the selection in :func:`rows`; the cell between them is not.
    """
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            origin, count, zmin, _ = generate._cell_grid(pts, CELL)
            rj, ri = generate._cell_index(origin, CELL, robot)
            if not (0 <= rj < count.shape[0] and 0 <= ri < count.shape[1]):
                continue
            if count[rj, ri] < ROBOT_MIN_PTS:
                continue  # nothing measured under the robot to measure against
            dz = zmin - zmin[rj, ri]
            wx, wy = generate._cell_centers(origin, CELL, count.shape)
            r = np.hypot(wx - robot[0], wy - robot[1])
            usable = (count >= MIN_PTS) & (r >= NEAR) & (r <= FAR)
            for j, i in zip(*np.nonzero(usable), strict=True):
                value = float(dz[j, i])
                if abs(value) <= FLAT:
                    kind = "flat"
                elif abs(value) >= OFF:
                    block = dz[max(0, j - 1) : j + 2, max(0, i - 1) : i + 2]
                    alike = np.isfinite(block) & (np.abs(block - value) <= 0.1)
                    if int(alike.sum()) - 1 < PATCH:
                        continue
                    kind = "off"
                else:
                    continue
                point = np.array([wx[j, i], wy[j, i]])
                rows.append(
                    {
                        "id": f"{dataset}_floorheight_t{t:g}_c{int(i)}x{int(j)}",
                        "family": "floor_height",
                        "type": "numeric",
                        "q": _question(point),
                        "a": round(value, 2),
                        "band": BAND,
                        "kind": kind,
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
    "go2_agentic_20260819_floorheight_t1125_c10x12",
    "go2_agentic_20260819_floorheight_t1200_c4x20",
    "go2_agentic_20260819_floorheight_t1125_c10x13",
    "go2_agentic_20260819_floorheight_t1200_c5x20",
    "go2_agentic_20260819_floorheight_t1325_c10x1",
    "go2_agentic_20260819_floorheight_t1400_c17x22",
    "go2_agentic_20260819_floorheight_t1325_c10x10",
    "go2_agentic_20260819_floorheight_t1400_c18x16",
    "go2_agentic_20260819_floorheight_t1450_c10x11",
    "go2_agentic_20260819_floorheight_t300_c11x18",
    "go2_agentic_20260819_floorheight_t1450_c10x12",
    "go2_agentic_20260819_floorheight_t300_c11x19",
    "go2_agentic_20260819_floorheight_t1500_c10x10",
    "go2_agentic_20260819_floorheight_t375_c12x24",
    "go2_agentic_20260819_floorheight_t1500_c10x11",
    "go2_agentic_20260819_floorheight_t375_c12x25",
    "go2_agentic_20260819_floorheight_t500_c0x7",
    "go2_agentic_20260819_floorheight_t900_c2x12",
    "go2_agentic_20260819_floorheight_t500_c0x8",
    "go2_agentic_20260819_floorheight_t900_c3x12",
    "go2_agentic_20260819_floorheight_t750_c10x1",
    "go2_china_office_floorheight_t122_c13x25",
    "go2_agentic_20260819_floorheight_t750_c10x10",
    "go2_china_office_floorheight_t122_c18x3",
    "go2_china_office_floorheight_t100_c10x12",
    "go2_china_office_floorheight_t25_c16x6",
    "go2_china_office_floorheight_t100_c10x13",
    "go2_china_office_floorheight_t25_c16x7",
    "go2_china_office_floorheight_t48_c10x10",
    "go2_china_office_floorheight_t48_c15x2",
    "go2_china_office_floorheight_t62_c10x1",
    "go2_china_office_floorheight_t62_c6x20",
    "go2_short_floorheight_t20_c10x10",
    "go2_short_floorheight_t40_c16x6",
    "go2_short_floorheight_t20_c10x11",
    "go2_short_floorheight_t40_c16x7",
)


def candidates() -> dict[str, generate.Row]:
    return {
        str(r["id"]): r
        for r in (
            *floor_height_rows("go2_stairs_20260819", _STAIRS_TS),
            *floor_height_rows("go2_agentic_20260819", _AGENTIC_TS),
            *floor_height_rows("go2_short", _SHORT_TS),
            *floor_height_rows("go2_china_office", _OFFICE_TS),
        )
    }


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    found = candidates()
    out = []
    for i in _CASES:
        row = dict(found[i])
        row.pop("kind")
        out.append(row)
    return out


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
