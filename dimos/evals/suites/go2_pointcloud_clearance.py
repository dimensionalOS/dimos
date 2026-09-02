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


"""Corridor clearance VQA over the go2 replays.

Rows (``go2_pointcloud_clearance.json``) are pure data emitted by
:func:`rows` — ground truth computed analytically from full-resolution clouds
plus odom, quizzing whatever lossy encoding the agent receives for a
``PointCloud2``.

``clearance`` asks whether a 0.6 m x 2 m corridor at body height is
obstructed. Two-valued on purpose: at 2 m the lidar has swept everywhere
around the robot, so silence really does mean empty. Unmeasured space is a
real question, but it needs a goal far enough out to reach it — that belongs
to :mod:`dimos.evals.suites.go2_pointcloud_route`.


Regenerate (needs both recordings)::

    uv run python -m dimos.evals.suites.go2_pointcloud_clearance
"""

from __future__ import annotations

from collections.abc import Sequence
import json
from pathlib import Path

import numpy as np

from dimos.evals.suites.lib import generate
from dimos.evals.types import Suite

_JSON = Path(__file__).parent / "go2_pointcloud_clearance.json"

SUITE: Suite = generate.cases(json.loads(_JSON.read_text()), tags=frozenset({"pointcloud"}))

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


# -- corridor clearance ----------------------------------------------------------
#
# Obstruction is judged at body height because that is what would hit the robot.
# At 2 m the lidar has swept the floor all round, so a corridor with no
# body-height return really is empty -- see clearance_rows for why this family
# is two-valued and unmeasured space belongs to go2_pointcloud_route.

REACH = 2.0  # corridor length asked about, meters
HALF_WIDTH = 0.3  # half the corridor width; ~Go2 body width plus margin
SELF_RETURN = 0.15  # returns closer than this are the robot's own body
MIN_EVIDENCE = 5  # points needed before a corridor verdict is called


COVERAGE_BINS = 72  # 5 degree sectors
COVERAGE_STEP = 0.1  # centreline sample spacing, meters


def _coverage(pts: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """Per-azimuth range the sensor actually collected data out to.

    Built from **every** return, at any height — floor, ceiling and body
    height alike. A return at range r in some direction means the scan
    reached r there; nothing past it was measured. Directions with no return
    at all get 0: no data was collected that way.
    """
    d = pts[:, :2] - origin
    rng = np.hypot(d[:, 0], d[:, 1])
    az = np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 360.0
    b = np.floor(az / (360.0 / COVERAGE_BINS)).astype(int) % COVERAGE_BINS
    cov = np.zeros(COVERAGE_BINS)
    np.maximum.at(cov, b, rng)
    return cov


def _leaves_coverage(cov: np.ndarray, origin: np.ndarray, bearing_deg: float, reach: float) -> bool:
    """Does walking this corridor take you outside the collected data?

    Samples the centreline and asks, at each step, whether the scan reached
    that far in that direction. One step past the envelope is enough.
    """
    th = np.radians(bearing_deg)
    u = np.array([np.cos(th), np.sin(th)])
    for r in np.arange(SELF_RETURN, reach + 1e-9, COVERAGE_STEP):
        point = origin + r * u
        d = point - origin
        az = np.degrees(np.arctan2(d[1], d[0])) % 360.0
        b = int(np.floor(az / (360.0 / COVERAGE_BINS))) % COVERAGE_BINS
        if r > cov[b]:
            return True
    return False


def _corridor_counts(
    band: np.ndarray, origin: np.ndarray, bearing_deg: float, reach: float = REACH
) -> tuple[int, int]:
    """Body-height points inside vs beyond a corridor, as (inside, beyond).

    The corridor runs ``reach`` meters from ``origin`` along ``bearing_deg``
    and is ``2 * HALF_WIDTH`` wide. "Beyond" is the same corridor continued
    past its end — a return there is proof the beam crossed the corridor.
    """
    theta = np.radians(bearing_deg)
    u = np.array([np.cos(theta), np.sin(theta)])
    d = band - origin
    along = d @ u
    lateral = np.abs(d @ np.array([-u[1], u[0]]))
    in_lane = lateral <= HALF_WIDTH
    inside = in_lane & (along > SELF_RETURN) & (along <= reach)
    beyond = in_lane & (along > reach)
    return int(inside.sum()), int(beyond.sum())


def clearance_rows(
    dataset: str,
    timestamps: Sequence[float],
    *,
    z_band: tuple[float, float] = generate.BODY_Z,
) -> list[generate.Row]:
    """Corridor clearance at body height: clear / blocked.

    Two-valued at this range. Coverage built from **every** return at any
    height shows the lidar sweeps the floor all around the robot, so a 2 m
    corridor has always been scanned and silence at body height means nothing
    is there; by 5 m nothing is clear. Unmeasured space belongs to
    :mod:`dimos.evals.suites.go2_pointcloud_route`, whose goal is far enough
    out to reach it.

    Obstruction is judged at body height because that is what would hit the
    robot. Candidates with fewer than ``MIN_EVIDENCE`` points inside are
    dropped rather than guessed at; those belong to a support-strength family.

    Every candidate is returned.
    """
    with generate._dataset(dataset) as store:
        candidates: list[tuple[str, generate.Row]] = []
        for t in timestamps:
            pts, context = generate._cloud_at(store, t)
            odom = store.streams.odom.range_time(0, t).to_list()[-1].data.position
            origin = np.array([float(odom.x), float(odom.y)])
            band = pts[(pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])][:, :2]
            cov = _coverage(pts, origin)  # all heights — floor returns count as coverage
            for i, name in enumerate(generate.COMPASS):
                inside, _ = _corridor_counts(band, origin, i * 45.0)
                if inside >= MIN_EVIDENCE:
                    label = "blocked"  # something at body height is in the way
                elif inside > 0:
                    continue  # thin evidence — ambiguous, not a clean case
                elif _leaves_coverage(cov, origin, i * 45.0, REACH):
                    # Corridor runs past the scanned region. Measured to never
                    # happen at REACH=2 m — the lidar sweeps the floor all round
                    # the robot, so everything this close has been scanned. Kept
                    # as a guard so a longer REACH cannot silently mislabel.
                    continue
                else:
                    label = "clear"  # scanned throughout, and nothing in the way
                candidates.append(
                    (
                        label,
                        {
                            "id": f"{dataset}_clearance_t{t:g}_{name}",
                            "family": "clearance",
                            "type": "mcq",
                            "q": "You are the robot; your current pose is the odom "
                            "observation shown (world frame: +x is east, +y is north). "
                            f"Consider a {2 * HALF_WIDTH:g} m wide corridor running "
                            f"{REACH:g} m due {name} from your position, at body height "
                            f"(z between {z_band[0]} and {z_band[1]} m). Using only the "
                            "mapped point cloud, is that corridor clear? Answer with "
                            "exactly one word: blocked if mapped points lie inside the "
                            "corridor, or clear if none do.",
                            "a": label,
                            "choices": ["clear", "blocked"],
                            "context": [
                                *context,
                                ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                            ],
                            "dataset": dataset,
                        },
                    )
                )
        return [row for _, row in candidates]


# The dataset: an even answer mix, spread across both recordings.
_CASES = (
    "go2_short_clearance_t5_east",
    "go2_short_clearance_t12_northeast",
    "go2_short_clearance_t20_north",
    "go2_short_clearance_t28_north",
    "go2_short_clearance_t36_northwest",
    "go2_short_clearance_t44_southeast",
    "go2_short_clearance_t52_southeast",
    "go2_china_office_clearance_t25_east",
    "go2_china_office_clearance_t25_southeast",
    "go2_china_office_clearance_t48_west",
    "go2_china_office_clearance_t55_south",
    "go2_china_office_clearance_t62_southeast",
    "go2_china_office_clearance_t78_west",
    "go2_china_office_clearance_t85_southeast",
    "go2_china_office_clearance_t100_southeast",
    "go2_china_office_clearance_t115_east",
    "go2_china_office_clearance_t122_northwest",
    "go2_china_office_clearance_t130_southeast",
    "go2_short_clearance_t5_north",
    "go2_short_clearance_t20_south",
    "go2_short_clearance_t36_north",
    "go2_short_clearance_t44_north",
    "go2_short_clearance_t58_southwest",
    "go2_china_office_clearance_t40_east",
    "go2_china_office_clearance_t40_southeast",
    "go2_china_office_clearance_t48_southwest",
    "go2_china_office_clearance_t62_west",
    "go2_china_office_clearance_t70_northeast",
    "go2_china_office_clearance_t78_east",
    "go2_china_office_clearance_t85_northwest",
    "go2_china_office_clearance_t93_northeast",
    "go2_china_office_clearance_t93_southwest",
    "go2_china_office_clearance_t108_south",
    "go2_china_office_clearance_t115_southwest",
    "go2_china_office_clearance_t130_northeast",
    "go2_china_office_clearance_t130_south",
)


def rows() -> list[generate.Row]:
    """The generator calls behind the committed JSON."""
    candidates = {
        r["id"]: r
        for r in (
            *clearance_rows("go2_short", _SHORT_TS),
            *clearance_rows("go2_china_office", _OFFICE_TS),
        )
    }
    return [candidates[i] for i in _CASES]


if __name__ == "__main__":
    _JSON.write_text(json.dumps(rows(), indent=2) + "\n")
