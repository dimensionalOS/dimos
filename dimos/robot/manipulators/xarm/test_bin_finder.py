# Copyright 2025-2026 Dimensional Inc.
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

"""The bin fit on surface points of stack bins: a wrong yaw or centre sends the gripper to the
wrong wall, so these are the ways the robot would move wrong."""

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.robot.manipulators.xarm.bin_finder import (
    XArm7BinFinder,
    XArm7BinFinderConfig,
    blocks_slot,
)
from dimos.robot.manipulators.xarm.kitting import (
    BIN_HEIGHT_M,
    BIN_LENGTH_M,
    BIN_WIDTH_M,
    XArm7KittingConfig,
)

TABLE_Z = XArm7BinFinderConfig().table_top_z
FRONT_HEIGHT_M = 0.07
LIP_M = 0.012
STEP_M = 0.004


def _bin_points(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """Walls of a stack bin: the back tall with a lip behind its rim, the scoop front low."""
    hl, hw = BIN_LENGTH_M / 2, BIN_WIDTH_M / 2
    pts: list[tuple[float, float, float]] = []
    for a in np.arange(-hl, hl, STEP_M):
        for z in np.arange(0.0, BIN_HEIGHT_M, STEP_M):
            pts += [(float(a), -hw, float(z)), (float(a), hw, float(z))]
    for b in np.arange(-hw, hw, STEP_M):
        for z in np.arange(0.0, BIN_HEIGHT_M, STEP_M):
            pts.append((-hl, float(b), float(z)))
        for z in np.arange(0.0, FRONT_HEIGHT_M, STEP_M):
            pts.append((hl, float(b), float(z)))
        for a in np.arange(-hl - LIP_M, -hl, STEP_M):
            pts.append((float(a), float(b), BIN_HEIGHT_M))
    p = np.array(pts)
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return np.column_stack(
        [x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1], TABLE_Z + p[:, 2]]
    )


def _finder() -> SimpleNamespace:
    shim = SimpleNamespace(config=XArm7BinFinderConfig())
    shim._fit_one = lambda p: XArm7BinFinder._fit_one(shim, p)  # type: ignore[arg-type]
    return shim


@pytest.mark.parametrize("yaw_deg", [0.0, 25.0, 90.0, -70.0, 180.0])
def test_a_bin_is_found_where_it_stands_facing_its_scoop(yaw_deg: float) -> None:
    bins = XArm7BinFinder._fit_bins(_finder(), _bin_points(0.33, -0.14, yaw_deg))  # type: ignore[arg-type]
    assert len(bins) == 1
    b = bins[0]
    assert np.hypot(b["x"] - 0.33, b["y"] + 0.14) < 0.005
    assert abs((b["yaw_deg"] - yaw_deg + 180) % 360 - 180) < 1.0
    assert b["zone"] == "middle" and b["size_ok"]


def test_two_bins_as_close_as_the_done_slots_stay_two_and_zone_by_slot() -> None:
    # the done slots leave 26 mm between bins; these are 27 mm apart, lip to wall
    p = np.concatenate([_bin_points(0.314, 0.505, 90.0), _bin_points(0.314, 0.22, 90.0 - 180)])
    bins = sorted(XArm7BinFinder._fit_bins(_finder(), p), key=lambda b: b["y"])  # type: ignore[arg-type]
    assert [b["zone"] for b in bins] == ["middle", "loading"]
    assert abs(bins[1]["y"] - 0.505) < 0.005
    assert all(b["size_ok"] for b in bins)


def _seen(x: float, y: float, yaw_deg: float) -> dict[str, float]:
    return {"x": x, "y": y, "yaw_deg": yaw_deg, "length_m": BIN_LENGTH_M, "width_m": BIN_WIDTH_M}


@pytest.mark.parametrize(
    ("seen", "blocks"),
    [
        # a neighbour on its own slot leaves this one free
        (_seen(0.40, -0.457, -90.0), False),
        (_seen(0.04, -0.457, -90.0), False),
        # a bin on the slot, or pushed 2 cm off a neighbour toward it, takes it
        (_seen(0.22, -0.457, -90.0), True),
        (_seen(0.38, -0.457, -90.0), True),
        # a bin turned across the gap between two slots takes it
        (_seen(0.31, -0.40, 0.0), True),
        (_seen(0.33, -0.14, 0.0), False),
    ],
)
def test_a_done_slot_is_taken_by_any_bin_in_its_way(seen: dict[str, float], blocks: bool) -> None:
    slot = XArm7KittingConfig().done_slots[0]
    assert blocks_slot(seen, slot.x, slot.y, slot.yaw_deg) is blocks
