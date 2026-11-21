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

from dataclasses import replace
from typing import Any

import pytest

from dimos.control.calibration._toy_arm import toy_arm
from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.spec import HalfSpace

STRETCHED, FOLDED = (0.0, 0.0), (1.5, 1.5)


def on_bench(**guard: Any) -> ArmModel:
    """The toy arm 0.1 m clear of the table, so the floor alone checks a short ramp at its ends."""
    toy = toy_arm()
    arm = ArmModel(replace(toy.spec, workspace=replace(toy.workspace, **guard)))
    arm.clearance_m = 0.1
    return arm


def ahead_of(x_m: float) -> dict[str, Any]:
    return {"keep_out": (HalfSpace((1.0, 0.0, 0.0), x_m),)}


@pytest.mark.parametrize(
    "bend,guard,clear",
    [
        (STRETCHED, ahead_of(0.499), False),
        (STRETCHED, ahead_of(0.501), True),
        (FOLDED, {"min_x_m": -0.1765}, False),
        (FOLDED, {"min_x_m": -0.178}, True),
    ],
)
def test_a_yaw_swing_clear_at_both_ends_passes_only_when_clear_in_between(
    bend: tuple[float, float], guard: dict[str, Any], clear: bool
) -> None:
    arm = on_bench(**guard)
    ends = [-0.1, *bend], [0.1, *bend]
    assert all(arm.pose_ok(q) for q in ends)
    assert arm.pose_ok([0.0, *bend]) == clear
    assert arm.transition_ok(*ends) == clear
