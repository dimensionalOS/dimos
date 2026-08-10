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

"""HardwareComponent joint array and derived views."""

from __future__ import annotations

import pytest

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_gripper_joints,
    make_joints,
)


def _component(dof: int, gripper_dof: int) -> HardwareComponent:
    gripper = make_gripper_joints("arm", gripper_dof) if gripper_dof else []
    return HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        all_joints=[*make_joints("arm", dof), *gripper],
        gripper_dof=len(gripper),
    )


class TestDerivedViews:
    def test_gripperless_arm_keeps_every_joint_as_an_arm_joint(self) -> None:
        # Negative slicing would return [] here: -0 == 0.
        hw = _component(dof=6, gripper_dof=0)

        assert hw.arm_joints == hw.all_joints
        assert len(hw.arm_joints) == 6
        assert hw.gripper_joints == []

    def test_gripper_trails_the_arm(self) -> None:
        hw = _component(dof=6, gripper_dof=1)

        assert hw.arm_joints == [f"arm/joint{i}" for i in range(1, 7)]
        assert hw.gripper_joints == ["arm/gripper"]
        assert hw.arm_joints + hw.gripper_joints == hw.all_joints

    def test_multi_joint_gripper_trails(self) -> None:
        hw = _component(dof=6, gripper_dof=3)

        assert len(hw.arm_joints) == 6
        assert hw.gripper_joints == ["arm/gripper1", "arm/gripper2", "arm/gripper3"]

    def test_all_gripper_component_has_no_arm(self) -> None:
        hw = HardwareComponent(
            hardware_id="hand",
            hardware_type=HardwareType.MANIPULATOR,
            all_joints=make_gripper_joints("hand", 6),
            gripper_dof=6,
        )

        assert hw.arm_joints == []
        assert hw.gripper_joints == hw.all_joints

    def test_views_are_derived_not_stored(self) -> None:
        hw = _component(dof=6, gripper_dof=1)
        hw.all_joints.insert(0, "arm/joint0")

        assert hw.arm_joints[0] == "arm/joint0"
        assert len(hw.arm_joints) == 7
        assert hw.gripper_joints == ["arm/gripper"]


class TestValidation:
    def test_gripper_dof_may_not_exceed_the_array(self) -> None:
        with pytest.raises(ValueError, match="exceeds all_joints length"):
            HardwareComponent(
                hardware_id="arm",
                hardware_type=HardwareType.MANIPULATOR,
                all_joints=make_joints("arm", 2),
                gripper_dof=3,
            )

    def test_gripper_dof_may_not_be_negative(self) -> None:
        with pytest.raises(ValueError, match="must be >= 0"):
            HardwareComponent(
                hardware_id="arm",
                hardware_type=HardwareType.MANIPULATOR,
                all_joints=make_joints("arm", 6),
                gripper_dof=-1,
            )


class TestMakeGripperJoints:
    def test_single_joint_keeps_the_unnumbered_name(self) -> None:
        assert make_gripper_joints("arm") == ["arm/gripper"]
        assert make_gripper_joints("arm", 1) == ["arm/gripper"]

    def test_multi_joint_numbers_from_one(self) -> None:
        assert make_gripper_joints("hand", 3) == [
            "hand/gripper1",
            "hand/gripper2",
            "hand/gripper3",
        ]

    def test_zero_joints_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="count must be >= 1"):
            make_gripper_joints("arm", 0)
