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

"""Grader smoke for xArm7 table tidy / move-to-center cases (no live sim)."""

import math

from dimos.evals.suites.mujoco_xarm_tidy import (
    APPLE_RADIUS,
    APPLE_XY,
    CUP_FALLEN_QUAT,
    CUP_RADIUS,
    CUP_UPRIGHT_Z,
    MESSY_CUP,
    ORANGE_EVAL_XY,
    ORANGE_RADIUS,
    SUITE,
    TABLE_CENTER,
    at_xy,
    at_xy_upright,
    stayed_xy,
    uprightness,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion


def test_at_xy_scores_table_center_with_a_tenth_meter_band() -> None:
    center = dict(target=TABLE_CENTER, start_z=0.19, end_z=0.19, band=0.10)
    assert at_xy(TABLE_CENTER, **center) == 1.0
    assert at_xy((0.45, -0.05), **center) == 0.5
    assert at_xy((0.45, -0.10), **center) == 0.0
    assert at_xy((0.45, -0.11), **center) == 0.0
    assert at_xy(TABLE_CENTER, **(center | {"end_z": 0.06})) == 0.0
    assert at_xy((0.45, 0.22), **center) == 0.0


def test_stayed_xy_uses_a_tight_band_for_the_already_tidy_control() -> None:
    start = TABLE_CENTER
    z = dict(start_z=0.19, end_z=0.19, band=0.03)
    assert stayed_xy(start, start, **z) == 1.0
    assert stayed_xy(start, (0.45, -0.015), **z) == 0.5
    assert stayed_xy(start, (0.45, -0.03), **z) == 0.0
    assert stayed_xy(start, (0.45, -0.10), **z) == 0.0


def test_uprightness_is_one_standing_and_zero_on_its_side() -> None:
    identity = Quaternion(0.0, 0.0, 0.0, 1.0)
    # CUP_FALLEN_QUAT is MuJoCo wxyz; Quaternion is xyzw.
    w, x, y, z = CUP_FALLEN_QUAT
    fallen = Quaternion(x, y, z, w)
    assert uprightness(identity) == 1.0
    assert uprightness(fallen) == 0.0


def test_at_xy_upright_needs_both_center_and_standing() -> None:
    upright = Quaternion(0.0, 0.0, 0.0, 1.0)
    w, x, y, z = CUP_FALLEN_QUAT
    fallen = Quaternion(x, y, z, w)
    assert at_xy_upright(TABLE_CENTER, TABLE_CENTER, upright, end_z=CUP_UPRIGHT_Z) == 1.0
    assert at_xy_upright(TABLE_CENTER, TABLE_CENTER, fallen, end_z=CUP_UPRIGHT_Z) == 0.0
    assert at_xy_upright(MESSY_CUP, TABLE_CENTER, upright, end_z=CUP_UPRIGHT_Z) == 0.0
    assert at_xy_upright(TABLE_CENTER, TABLE_CENTER, upright, end_z=0.10) == 0.0


def test_messy_cup_is_a_tenth_meter_from_center_and_clear_of_fruit() -> None:
    assert math.isclose(
        math.hypot(MESSY_CUP[0] - TABLE_CENTER[0], MESSY_CUP[1] - TABLE_CENTER[1]),
        0.10,
    )
    margin = 0.01
    assert (
        math.hypot(MESSY_CUP[0] - ORANGE_EVAL_XY[0], MESSY_CUP[1] - ORANGE_EVAL_XY[1])
        > CUP_RADIUS + ORANGE_RADIUS + margin
    )
    assert (
        math.hypot(MESSY_CUP[0] - APPLE_XY[0], MESSY_CUP[1] - APPLE_XY[1])
        > CUP_RADIUS + APPLE_RADIUS + margin
    )


def test_tidy_cup_at_center_clears_eval_orange() -> None:
    margin = 0.01
    assert (
        math.hypot(TABLE_CENTER[0] - ORANGE_EVAL_XY[0], TABLE_CENTER[1] - ORANGE_EVAL_XY[1])
        > CUP_RADIUS + ORANGE_RADIUS + margin
    )


def test_suite_includes_raw_duty_pair_and_fallen_upright() -> None:
    assert [case.id for case in SUITE] == [
        "xarm_move_cup_to_center",
        "xarm_tidy_cup_messy",
        "xarm_tidy_cup_already_tidy",
        "xarm_tidy_cup_fallen",
    ]
    raw, messy, tidy, fallen = SUITE
    assert raw.inputs == "Move the cup to the middle of the table."
    assert messy.inputs == tidy.inputs
    assert "cleaning arm" in messy.inputs
    assert "middle of the table" in messy.inputs
    assert "stand upright" in fallen.inputs
    assert "not lying on its side" in fallen.inputs
    assert "0.10" not in fallen.inputs
    assert "0.45" not in messy.inputs
