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

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.manipulators.xarm.kitting import (
    BIN_LENGTH_M,
    BIN_WALL_M,
    BinPose,
    XArm7KittingModule,
    carry_waypoints,
    grip_point,
    tool_quaternion,
    tool_yaw,
    wrist_friendly_yaw,
)

LOADING = BinPose(x=0.314, y=0.505, yaw_deg=90.0)


def test_the_grip_is_on_the_back_wall_beside_the_parts() -> None:
    (x, y), yaw = grip_point(LOADING, 0.055)
    # back wall toward the arm: the end of the bin nearest y = 0
    assert y == pytest.approx(LOADING.y - BIN_LENGTH_M / 2 + BIN_WALL_M / 2)
    assert x == pytest.approx(LOADING.x - 0.055)
    assert yaw == pytest.approx(math.pi / 2)


@pytest.mark.parametrize("yaw", [0.0, 1.2, -2.5, math.pi])
def test_the_tool_points_down_and_closes_across_the_wall(yaw: float) -> None:
    r = Rotation.from_quat(tool_quaternion(yaw))
    assert r.apply([0, 0, 1]) == pytest.approx([0, 0, -1], abs=1e-9)
    closing = r.apply([0, 1, 0])
    # fingers close along the wall's normal, in either direction
    assert abs(np.dot(closing, [math.cos(yaw), math.sin(yaw), 0])) == pytest.approx(1.0)
    assert math.cos(tool_yaw(tool_quaternion(yaw)) - yaw) == pytest.approx(1.0)


def test_the_carry_ends_at_the_target_with_the_bin_turned() -> None:
    start, end = (0.259, 0.389), (0.275, -0.341)
    turn = math.atan2(end[1], end[0]) - math.atan2(start[1], start[0])
    points = carry_waypoints(start, end, 0.3, -math.pi, turn, 3)
    assert points[-1][0] == end
    final = tool_yaw(points[-1][1])
    assert math.cos(final - (0.3 - math.pi)) == pytest.approx(1.0)
    # every point keeps to the arc about the base: no swing in toward the base or out
    radii = [math.hypot(*xy) for xy, _ in points]
    assert min(radii) >= min(math.hypot(*start), math.hypot(*end)) - 1e-9
    assert max(radii) <= max(math.hypot(*start), math.hypot(*end)) + 1e-9


def test_the_wrist_side_keeps_the_last_joint_clear_of_its_limits() -> None:
    # an empty bin whose wall needs the wrist turned 170 deg from home at the pick, and the carry
    # turns it a further 160 deg relative to the base: either side alone would pass the limit
    pick_angle, place_angle, wrist_home = 0.0, 0.5, 0.0
    bin_turn = place_angle + math.radians(160)
    chosen = wrist_friendly_yaw(math.radians(170), bin_turn, pick_angle, place_angle, wrist_home)
    # the same wall normal axis, from whichever side
    assert math.sin(chosen - math.radians(170)) == pytest.approx(0.0, abs=1e-9)
    at_pick = chosen - pick_angle - wrist_home
    at_place = chosen + bin_turn - place_angle - wrist_home
    assert abs(at_pick) < math.pi - 0.03
    assert abs(at_place) < math.pi - 0.03


@pytest.fixture
def kitting():  # type: ignore[no-untyped-def]
    module = XArm7KittingModule()
    moves: list[tuple[BinPose, BinPose, list[BinPose]]] = []
    flags: list[bool] = []
    offsets: list[float] = []

    def move_bin(
        source: BinPose, target: BinPose, others: list[BinPose], offset_m: float
    ) -> dict[str, bool]:
        moves.append((source, target, others))
        offsets.append(offset_m)
        return {"ok": True}

    module._move_bin = move_bin  # type: ignore[method-assign]
    module.empty_bin_ready.publish = lambda msg: flags.append(msg.data)  # type: ignore[method-assign]
    module.offsets = offsets  # type: ignore[attr-defined]
    yield module, moves, flags
    module.stop()


def test_unload_goes_to_the_first_free_slot_and_keeps_clear_of_the_taken_ones(kitting) -> None:  # type: ignore[no-untyped-def]
    module, moves, flags = kitting
    module.set_done_taken([0])
    assert module._unload()["done_slot"] == 1
    source, target, others = moves[-1]
    assert source == module.config.loading_slot
    assert target == module.config.done_slots[1]
    assert others == [module.config.done_slots[0]]
    assert flags == [False]


def test_the_flag_rises_only_once_an_empty_bin_is_in_the_loading_slot(kitting) -> None:  # type: ignore[no-untyped-def]
    module, moves, flags = kitting
    empty = BinPose(x=0.33, y=-0.14, yaw_deg=25.0)
    assert not module._load(empty)["ok"]  # the slot still holds the filled bin
    assert flags == []
    module._unload()
    assert module._load(empty)["ok"]
    assert moves[-1][1] == module.config.loading_slot
    assert flags == [False, True]


def test_no_unload_when_every_done_slot_is_taken(kitting) -> None:  # type: ignore[no-untyped-def]
    module, moves, _ = kitting
    module.set_done_taken([0, 1, 2])
    assert not module._unload()["ok"]
    assert moves == []


def test_a_full_bin_is_held_beside_its_parts_and_an_empty_one_at_the_middle(kitting) -> None:  # type: ignore[no-untyped-def]
    module, _, _ = kitting
    module._unload()
    module._load(BinPose(x=0.33, y=-0.14, yaw_deg=25.0))
    assert module.offsets == [module.config.grip_offset_m, module.config.empty_grip_offset_m]
    assert module.config.grip_offset_m > 0.04  # clear of parts on the centre line
    assert module.config.empty_grip_offset_m == 0.0
