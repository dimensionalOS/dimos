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


"""Longest clear heading from a point — VQA over the go2 replays.

Rows (``go2_pointcloud_free_range_vqa.json``) are pure data emitted by
:func:`rows` — ground truth ray-marched on the full-resolution cloud,
quizzing whatever lossy encoding the agent receives for a ``PointCloud2``.

The task is a robot's: which way is it most open, if any. From a point, the
agent picks the compass heading it could travel furthest along over floor the
lidar mapped and found nothing standing on above 0.15 m — or answers that no
heading is clear that far, when the point is boxed in. Ten rows, five with a
clear heading and five without; the prompt does not say which, and a heading
that merely points into the window's corner cannot win because every heading
is capped at the distance to the nearest edge of the mapped cloud.

Ten hand-picked rows, tagged ``train``. The 0.15 m edge is the eval's,
stated in the prompt; the encoder never sees it.

Regenerate (needs the three recordings)::

    uv run python -m dimos.evals.suites.go2_pointcloud_free_range
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_free_range_vqa.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

LOW_Z = 0.15  # the question's "nothing on the floor" edge, world z
HALF_WIDTH = 0.3  # half the lane width; ~Go2 body width plus margin
SELF_RETURN = 0.15  # returns closer than this along the lane are the robot's own body
START = 0.3  # the lane is judged from here out; under the robot is not asked about
BIN = 0.2  # a stretch of lane this long with no return at all ends the run
CLEAR = 2.0  # a heading "goes far" when it runs at least this far
BOXED = 1.2  # every heading under this and the point is boxed in — the negative
MIN_MARGIN = 0.4  # a clear winner beats the runner-up by this much
QUERY_CELL = 0.25

_AGENTIC_TS = [
    100.0,
    150.0,
    200.0,
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

CHOICES = (*generate.COMPASS, "none")


def _question(point: np.ndarray, cap: float) -> str:
    return (
        "You are the robot; your current pose is the odom observation shown (world frame: "
        "+x is east, +y is north, coordinates in meters). Starting from the world point "
        f"({point[0]:.2f}, {point[1]:.2f}), which single compass direction (north, "
        "north-east, east, and so on) could you travel furthest in a straight line over "
        f"floor the lidar has mapped and found clear — nothing standing on it above z = "
        f"{LOW_Z} m — before you would reach something above that height or run off the edge "
        f"of what the lidar mapped? Measure no further than {cap:.1f} m in any direction, "
        "since the mapped area is closer than that on some sides. Using only the mapped point "
        f"cloud, answer with the single most open direction, or none if no direction stays "
        f"clear for even {CLEAR:g} m."
    )


def runs(pts: np.ndarray, point: np.ndarray) -> tuple[np.ndarray, float]:
    """Per compass heading, how far a lane runs clear before it stops, and the cap.

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
    cap = max(cap, 0.0)
    d = xy - point
    out = np.zeros(len(generate.COMPASS))
    n_bins = max(1, int(np.ceil(cap / BIN)))
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
    """Compass index of the nearest return above LOW_Z — the heading an
    obstacle-only reading is anchored on."""
    high = pts[pts[:, 2] > LOW_Z]
    d = np.hypot(high[:, 0] - point[0], high[:, 1] - point[1])
    keep = d > SELF_RETURN
    if not keep.any():
        return None
    q = high[keep][d[keep].argmin()]
    return int(np.round(np.degrees(np.arctan2(q[1] - point[1], q[0] - point[0])) / 45.0)) % 8


def _query_points(pts: np.ndarray) -> list[np.ndarray]:
    """Centres of swept, empty cells — every candidate stance on mapped floor."""
    origin, count, _, zmax = generate._cell_grid(pts, QUERY_CELL)
    wx, wy = generate._cell_centers(origin, QUERY_CELL, count.shape)
    ok = (count >= 3) & (zmax <= LOW_Z)
    return [np.array([float(x), float(y)]) for x, y in zip(wx[ok], wy[ok], strict=True)]


def free_range_rows(dataset: str, timestamps: Sequence[float]) -> list[generate.Row]:
    """Candidate rows, each tagged kind open or boxed for the split in :func:`rows`.

    A point is open when one heading runs at least CLEAR m, beats the
    runner-up by MIN_MARGIN, and — so an obstacle-distance reader is not
    handed the answer — is not the heading of the nearest return above the
    edge. It is boxed when no heading clears BOXED m. Points between are
    dropped.
    """
    with generate._dataset(dataset) as store:
        rows: list[generate.Row] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            for k, point in enumerate(_query_points(pts)):
                lengths, cap = runs(pts, point)
                if cap < CLEAR:
                    continue  # too near the window edge to judge a clear run
                order = np.argsort(-lengths)
                win = int(order[0])
                margin = float(lengths[order[0]] - lengths[order[1]])
                if (
                    lengths[win] >= CLEAR
                    and margin >= MIN_MARGIN
                    and win != _nearest_bearing(pts, point)
                ):
                    kind, answer = "open", generate.COMPASS[win]
                elif lengths.max() < BOXED:
                    kind, answer = "boxed", "none"
                else:
                    continue
                rows.append(
                    {
                        "id": f"{dataset}_freerange_t{t:g}_p{k}",
                        "family": "free_range",
                        "type": "mcq",
                        "q": _question(point, cap),
                        "a": answer,
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
    "go2_agentic_20260819_freerange_t1400_p155",
    "go2_agentic_20260819_freerange_t1450_p102",
    "go2_agentic_20260819_freerange_t900_p67",
    "go2_china_office_freerange_t100_p152",
    "go2_china_office_freerange_t108_p122",
    "go2_agentic_20260819_freerange_t750_p58",
    "go2_china_office_freerange_t25_p54",
    "go2_china_office_freerange_t48_p88",
    "go2_short_freerange_t12_p187",
    "go2_short_freerange_t44_p106",
)
_HOLDOUT = (
    "go2_agentic_20260819_freerange_t100_p116",
    "go2_agentic_20260819_freerange_t1125_p146",
    "go2_agentic_20260819_freerange_t1200_p233",
    "go2_agentic_20260819_freerange_t500_p30",
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
