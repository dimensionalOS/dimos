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

from collections.abc import Iterator
import runpy

import can_motor_control
import numpy as np
import pinocchio
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.openarm_damiao import adapter as adapter_module
from dimos.hardware.whole_body.openarm_damiao.adapter import OpenArmDamiaoAdapter
from dimos.robot.manipulators.openarm.config import OPENARM_DOF, OPENARM_JOINTS


@pytest.fixture
def openarm_adapter(mocker: MockerFixture) -> Iterator[OpenArmDamiaoAdapter]:
    mocker.patch.object(can_motor_control, "SocketCanBus", can_motor_control.MockCanBus)
    adapter = OpenArmDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    yield adapter
    adapter.disconnect()


def test_import_lazy_gravity_model_does_not_resolve_asset(mocker: MockerFixture) -> None:
    checkout_path = mocker.patch("dimos.robot.assets.source.RobotDescriptionSource.checkout_path")

    runpy.run_path(adapter_module.__file__)

    checkout_path.assert_not_called()


def test_openarm_topology_connects_arms_and_grippers(
    openarm_adapter: OpenArmDamiaoAdapter,
    mocker: MockerFixture,
) -> None:
    robot = openarm_adapter._build_robot()

    assert robot.group_names() == ["left_arm", "right_arm", "left_gripper", "right_gripper"]
    assert robot.bus_names() == ["left", "right"]
    assert isinstance(robot["left_arm"], can_motor_control.Arm)
    assert isinstance(robot["right_arm"], can_motor_control.Arm)
    assert len(robot["left_arm"]) == OPENARM_DOF
    assert len(robot["right_arm"]) == OPENARM_DOF
    assert isinstance(robot["left_gripper"], can_motor_control.Gripper)
    assert isinstance(robot["right_gripper"], can_motor_control.Gripper)
    mocker.patch.object(DamiaoWholeBodyAdapter, "_load_kinematic_model")
    assert openarm_adapter.connect()


def test_openarm_joint_order_matches_hardware_component(
    openarm_adapter: OpenArmDamiaoAdapter,
) -> None:
    """Commands are routed positionally: the config joint list must equal the
    adapter's declared order or motors silently receive each other's targets."""
    assert list(openarm_adapter.joint_names) == OPENARM_JOINTS


def test_openarm_declares_local_normalized_gripper_limits(
    openarm_adapter: OpenArmDamiaoAdapter,
) -> None:
    limits = openarm_adapter.get_limits()

    arm_count = OPENARM_DOF * 2
    assert limits.position_lower == [*([None] * arm_count), 0.0, 0.0]
    assert limits.position_upper == [*([None] * arm_count), 1.0, 1.0]
    assert limits.velocity_max == [None] * len(OPENARM_JOINTS)


@pytest.mark.self_hosted
def test_openarm_feedback_limits_match_urdf_joint_limits(
    openarm_adapter: OpenArmDamiaoAdapter,
) -> None:
    model = pinocchio.buildModelFromXML(openarm_adapter.kinematic_model.load().xml)

    assert tuple(str(name) for name in model.names[1:]) == openarm_adapter.kinematic_joint_names
    assert np.all(np.asarray(model.lowerPositionLimit) < np.asarray(model.upperPositionLimit))
