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

import numpy as np
import pytest

from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_HOME_JOINTS,
    DUAL_OPENYAM_JOINT_LIMITS,
    DUAL_OPENYAM_LEADERS,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM,
    OPENYAM_ARM_JOINTS,
    OPENYAM_HOME_JOINTS,
    make_openyam_model_config,
)


def test_dual_openyam_model_has_canonical_groups_and_reference_posture() -> None:
    config = dual_openyam_model_config()

    assert config.joint_names == DUAL_OPENYAM_ARM_JOINTS
    assert config.home_joints == DUAL_OPENYAM_HOME_JOINTS
    assert [(group.name, group.tip_link) for group in config.planning_groups] == [
        ("left_manipulator", "left_grasp_frame"),
        ("right_manipulator", "right_grasp_frame"),
    ]


def test_dual_openyam_reuses_the_single_arm_home_posture() -> None:
    assert DUAL_OPENYAM_HOME_JOINTS == [*OPENYAM_HOME_JOINTS, *OPENYAM_HOME_JOINTS]


def test_each_arm_is_compensated_like_a_single_openyam() -> None:
    single = PhysicalModel(OPENYAM.gravity_model, list(OPENYAM_ARM_JOINTS), gravity_comp=True)
    dual = PhysicalModel(
        DUAL_OPENYAM.gravity_model, DUAL_OPENYAM.arm_joint_names, gravity_comp=True
    )
    left = np.asarray([0.3, 0.8, 0.6, -0.3, 0.9, 0.4])
    right = np.asarray(OPENYAM_HOME_JOINTS)

    np.testing.assert_allclose(
        dual.gravity(np.concatenate([left, right])),
        np.concatenate([single.gravity(left), single.gravity(right)]),
        atol=1e-12,
    )


@pytest.mark.self_hosted
def test_both_arms_and_their_leaders_share_the_single_arms_limits() -> None:
    single = make_openyam_model_config().model.load()
    dual = dual_openyam_model_config().model.load()
    for name, limits in zip(OPENYAM_ARM_JOINTS, DUAL_OPENYAM_JOINT_LIMITS, strict=True):
        joint = single.get_joint(name)
        assert joint is not None
        assert (joint.lower, joint.upper) == limits
    for bus in DUAL_OPENYAM_LEADERS.buses:
        for motor, limits in zip(bus.motors, DUAL_OPENYAM_JOINT_LIMITS, strict=True):
            joint = dual.get_joint(motor.joint)
            assert joint is not None
            assert (joint.lower, joint.upper) == limits
            assert (motor.lower, motor.upper) == limits


def test_dual_openyam_connection_declares_both_arms_and_grippers() -> None:
    connection = DualOpenYamConnection(left_can_port="left", right_can_port="right")
    try:
        assert connection.config.source == "dual_openyam"
        assert connection.can_ports() == ["left", "right"]
        assert connection.joint_names == [
            *DUAL_OPENYAM_ARM_JOINTS,
            "left_gripper",
            "right_gripper",
        ]
    finally:
        connection.stop()


def test_dual_openyam_connection_uses_both_explicit_can_ports() -> None:
    connection = DualOpenYamConnection(left_can_port="can8", right_can_port="can9")
    try:
        assert connection.can_ports() == ["can8", "can9"]
        assert connection.robot.gravity is True
    finally:
        connection.stop()


@pytest.mark.parametrize(
    "ports",
    [
        {"left_can_port": "can8"},
        {"right_can_port": "can9"},
    ],
)
def test_dual_openyam_connection_rejects_partial_can_configuration(ports) -> None:
    with pytest.raises(ValueError, match="Field required"):
        DualOpenYamConnection(**ports)


def test_dual_openyam_connection_rejects_duplicate_can_configuration() -> None:
    with pytest.raises(ValueError, match="requires distinct"):
        DualOpenYamConnection(left_can_port="same", right_can_port="same")


@pytest.mark.parametrize(
    "ports",
    [
        {"left_can_port": " ", "right_can_port": "right"},
        {"left_can_port": "left", "right_can_port": ""},
    ],
)
def test_dual_openyam_connection_rejects_empty_can_configuration(ports) -> None:
    with pytest.raises(ValueError, match="must not be empty"):
        DualOpenYamConnection(**ports)
