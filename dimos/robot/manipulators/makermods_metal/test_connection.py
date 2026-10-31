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

"""Literal Metal firmware topology and lazy deployment contracts."""

import pytest

from dimos.robot.manipulators.common.damiao_connection import MotorProfileSpec
from dimos.robot.manipulators.makermods_metal.config import METAL_SOURCE
from dimos.robot.manipulators.makermods_metal.connection import (
    MetalConnection,
    MetalConnectionConfig,
)


def test_metal_preserves_custom_profiles_ids_gains_and_feedback_tolerance():
    robot = MetalConnection.robot
    assert robot.buses == ("metal",)
    assert [(arm.name, arm.bus) for arm in robot.arms] == [("arm", "metal")]
    joints = robot.arms[0].joints
    assert [(joint.name, joint.send_id, joint.recv_id, joint.kp, joint.kd) for joint in joints] == [
        ("joint1", 1, 17, 100.0, 5.0),
        ("joint2", 2, 18, 100.0, 5.0),
        ("joint3", 3, 19, 100.0, 5.0),
        ("joint4", 4, 20, 50.0, 5.0),
        ("joint5", 5, 21, 20.0, 0.6),
        ("joint6", 6, 22, 20.0, 0.6),
    ]
    assert [joint.motor for joint in joints] == [
        MotorProfileSpec(0x8000, 6.28, 10.0, 30.0),
        MotorProfileSpec(0x8001, 6.28, 10.0, 120.0),
        MotorProfileSpec(0x8000, 6.28, 10.0, 30.0),
        MotorProfileSpec(0x8000, 6.28, 10.0, 30.0),
        MotorProfileSpec(0x8002, 6.28, 30.0, 20.0),
        MotorProfileSpec(0x8002, 6.28, 30.0, 20.0),
    ]
    (gripper,) = robot.grippers
    assert (
        gripper.name,
        gripper.bus,
        gripper.send_id,
        gripper.recv_id,
        gripper.direction,
        gripper.current,
        gripper.motor,
    ) == (
        "gripper",
        "metal",
        7,
        23,
        "increasing_position",
        0.1,
        MotorProfileSpec(0x8002, 6.28, 30.0, 20.0),
    )
    assert not robot.gravity_comp
    assert robot.feedback_margin_rad == 0.06
    assert METAL_SOURCE.ref == "bccf3c82a21453a7b1784a94966456e16ff0f008"


@pytest.mark.parametrize("selector", ["", "  "])
def test_metal_rejects_empty_can_selector_before_opening_hardware(selector):
    with pytest.raises(ValueError, match="CAN selector must not be empty"):
        MetalConnectionConfig(can_port=selector)


def test_metal_model_schema_is_lazy_and_namespaces_both_planner_and_gripper(mocker):
    load = mocker.patch(
        "dimos.robot.assets.model.RobotModel.load", side_effect=AssertionError("must stay lazy")
    )
    deployed = MetalConnection.models()
    assert deployed.joints == (
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "arm/gripper",
    )
    assert tuple(deployed.planning.joint_names) == deployed.arm_joints
    assert deployed.planning.planning_groups[0].joint_names == deployed.arm_joints
    assert deployed.planning.gripper_hardware_id == "arm"
    load.assert_not_called()
