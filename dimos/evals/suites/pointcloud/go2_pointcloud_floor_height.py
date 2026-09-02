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


"""Off-level floor near the robot — VQA over the go2 replays.

Rows (``go2_pointcloud_floor_height.json``) are pure data emitted by
:func:`rows` — ground truth read off the full-resolution cloud, quizzing
whatever lossy encoding the agent receives for a ``PointCloud2``.

The task: is any floor within a few meters of the robot at a different level
than the floor it stands on — raised or sunken — and if so, where and by how
much. Ten rows, five with a real step or drop and five where the floor is one
level; the prompt does not say which. The answer locates each off-level area
and its height offset, or is ``level``.

``go2_stairs_20260819`` carries the raised area the hand-authored floor rows
label; the frames here are the moving stretches around them.
``go2_teleop_20260819`` is absent — its map has a false depression from the
robot being carried before recording.

Ten hand-picked rows, tagged ``train``: too few to slice.

Regenerate (needs the recordings)::

    uv run python -m dimos.evals.suites.pointcloud.go2_pointcloud_floor_height
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np
from scipy import ndimage

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_floor_height.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

CELL = 0.25
MIN_PTS = 8  # returns a cell needs before its floor height is trusted
ROBOT_MIN_PTS = 5
FAR = 3.0  # off-level areas are looked for within this of the robot
FLAT = 0.08  # |dz| at or under this is the robot's own level
OFF = 0.2  # |dz| at or over this is a different level, above the min-z noise floor
MIN_AREA = 4  # cells an off-level area needs, so one noisy cell is not a step
RADIUS = 1.2  # match tolerance on an area's centre, meters
VALUE_BAND = 0.2  # band on the height offset

_AGENTIC_TS = [100.0, 300.0, 500.0, 900.0, 1200.0, 1450.0]
_SHORT_TS = [5.0, 20.0, 40.0, 58.0]
_OFFICE_TS = [25.0, 48.0, 100.0, 130.0]
_STAIRS_TS = [20.0, 30.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0]

QUESTION = (
    "You are the robot; your current pose is the odom observation shown (world frame: +x is "
    "east, +y is north, coordinates in meters). Is any part of the floor within a few meters "
    "of you at a different level than the floor beneath you — noticeably raised or sunken? If "
    "so, give the middle of each such area and how far it sits above (positive) or below "
    "(negative) your level, as x,y,dz — one area per line. If the floor around you is all one "
    "level, answer level. Use only the mapped point cloud."
)


def off_level_areas(pts: np.ndarray, robot: np.ndarray) -> list[list[float]]:
    """Connected patches of floor near the robot at a different level, as ``[x, y, dz]``.

    Floor height per cell is the lowest return; a cell is off-level when it
    holds ``MIN_PTS`` returns and its floor sits ``OFF`` from the robot's cell.
    Adjacent off-level cells of the same sign are one area, reported by its
    centroid and mean offset; areas under ``MIN_AREA`` cells are dropped.
    """
    origin, count, zmin, _ = generate._cell_grid(pts, CELL)
    rj, ri = generate._cell_index(origin, CELL, robot)
    if not (0 <= rj < count.shape[0] and 0 <= ri < count.shape[1]) or count[rj, ri] < ROBOT_MIN_PTS:
        return []
    base = zmin[rj, ri]
    wx, wy = generate._cell_centers(origin, CELL, count.shape)
    near = (count >= MIN_PTS) & (np.hypot(wx - robot[0], wy - robot[1]) <= FAR)
    dz = np.where(near, zmin - base, 0.0)
    out: list[list[float]] = []
    for sign in (1.0, -1.0):
        mask = near & (sign * dz >= OFF)
        labels, n = ndimage.label(mask, structure=np.ones((3, 3)))
        for label in range(1, n + 1):
            cells = labels == label
            if int(cells.sum()) < MIN_AREA:
                continue
            out.append(
                [
                    round(float(wx[cells].mean()), 2),
                    round(float(wy[cells].mean()), 2),
                    round(float(dz[cells].mean()), 2),
                ]
            )
    return out


def floor_height_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """One row per frame, tagged ``kind`` step or level for the split in :func:`rows`."""
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            robot = generate._odom_at(store, t)
            areas = off_level_areas(pts, robot)
            rows.append(
                {
                    "id": f"{dataset}_floorheight_t{t:g}",
                    "family": "floor_height",
                    "type": "coords",
                    "q": QUESTION,
                    "a": areas,
                    "radius": RADIUS,
                    "value_band": VALUE_BAND,
                    "kind": "step" if areas else "level",
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
    "go2_agentic_20260819_floorheight_t900",
    "go2_china_office_floorheight_t25",
    "go2_short_floorheight_t20",
    "go2_short_floorheight_t58",
    "go2_stairs_20260819_floorheight_t100",
    "go2_agentic_20260819_floorheight_t500",
    "go2_china_office_floorheight_t100",
    "go2_china_office_floorheight_t130",
    "go2_china_office_floorheight_t48",
    "go2_short_floorheight_t5",
)
_HOLDOUT = (
    "go2_agentic_20260819_floorheight_t1200",
    "go2_agentic_20260819_floorheight_t300",
    "go2_agentic_20260819_floorheight_t100",
    "go2_agentic_20260819_floorheight_t1450",
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
