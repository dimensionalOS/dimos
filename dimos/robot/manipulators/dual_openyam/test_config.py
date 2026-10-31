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

import pytest

from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_JOINTS,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_HOME_JOINTS


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


def test_dual_openyam_connection_defaults_to_complete_mock() -> None:
    connection = DualOpenYamConnection()
    try:
        assert connection.config.source == "dual_openyam"
        assert connection._can_devices() is None
        assert connection.robot.joint_names == (
            *DUAL_OPENYAM_ARM_JOINTS,
            "left_gripper",
            "right_gripper",
        )
        assert connection.robot.mock_positions == (*DUAL_OPENYAM_HOME_JOINTS, 0.0, 0.0)
    finally:
        connection.stop()


def test_dual_openyam_connection_uses_both_explicit_can_ports() -> None:
    connection = DualOpenYamConnection(left_can_port="can8", right_can_port="can9")
    try:
        assert connection._can_devices() == {"left": "can8", "right": "can9"}
        assert connection.robot.gravity_comp is True
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
    with pytest.raises(ValueError, match="requires both"):
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
