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


import pytest

from dimos.teleop.leader.mapping import (
    GripperMapping,
    JointMapping,
    SideMapping,
    map_side_readings,
)

ARM = SideMapping(
    joints=(
        JointMapping(leader_joint="joint_1", joint="arm/left_joint1", lower=-2.6, upper=3.1),
        JointMapping(
            leader_joint="joint_2", joint="arm/left_joint2", lower=0.0, upper=3.6, scale=-1.0
        ),
        JointMapping(
            leader_joint="joint_3", joint="arm/left_joint3", lower=0.0, upper=3.1, offset=0.5
        ),
    ),
    gripper=GripperMapping(leader_joint="joint_7", travel_radians=1.0),
)


def test_targets_apply_scale_offset_and_clamp() -> None:
    command = map_side_readings(
        "left", ARM, {"joint_1": 0.4, "joint_2": -1.0, "joint_3": 4.0, "joint_7": 0.0}
    )
    assert command.positions_by_joint == {
        "arm/left_joint1": 0.4,
        "arm/left_joint2": 1.0,
        "arm/left_joint3": 3.1,
    }
    assert command.gripper_opening == 1.0


def test_gripper_opening_follows_the_trigger_through_its_travel() -> None:
    readings = {"joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0, "joint_7": 0.25}
    assert map_side_readings("left", ARM, readings).gripper_opening == pytest.approx(0.75)
    readings["joint_7"] = -2.0
    assert map_side_readings("left", ARM, readings).gripper_opening == 0.0
