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
from can_motor_control import damiao
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.seeed_b601_dm import adapter as adapter_module
from dimos.hardware.whole_body.seeed_b601_dm.adapter import SeeedB601DMAdapter
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_DOF, B601_DM_JOINTS


@pytest.fixture
def b601_adapter(mocker: MockerFixture) -> Iterator[SeeedB601DMAdapter]:
    mocker.patch.object(
        SeeedB601DMAdapter,
        "_make_can_bus",
        side_effect=lambda _name: can_motor_control.MockCanBus("b601_dm"),
    )
    adapter = SeeedB601DMAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    yield adapter
    adapter.disconnect()


def test_import_does_not_fetch_the_vendor_model(mocker: MockerFixture) -> None:
    get_data = mocker.patch("dimos.utils.data.get_data")

    runpy.run_path(adapter_module.__file__)

    get_data.assert_not_called()


def test_b601_topology_connects_arm_and_gripper(
    b601_adapter: SeeedB601DMAdapter,
    mocker: MockerFixture,
) -> None:
    robot = b601_adapter._build_robot()

    assert robot.group_names() == ["arm", "gripper"]
    assert isinstance(robot["arm"], can_motor_control.Arm)
    assert len(robot["arm"]) == B601_DM_DOF
    assert isinstance(robot["gripper"], can_motor_control.Gripper)
    assert b601_adapter.joint_names == B601_DM_JOINTS
    mocker.patch.object(DamiaoWholeBodyAdapter, "_load_kinematic_model")
    assert b601_adapter.connect()


def test_b601_motor_table_uses_the_4340_profile_for_4340p(
    b601_adapter: SeeedB601DMAdapter,
    mocker: MockerFixture,
) -> None:
    spec = mocker.spy(can_motor_control, "MotorSpec")

    b601_adapter._build_robot()

    assert [call.args for call in spec.call_args_list] == [
        ("joint1", damiao.MotorType.DM4340, 0x01, 0x11),
        ("joint2", damiao.MotorType.DM4340, 0x02, 0x12),
        ("joint3", damiao.MotorType.DM4340, 0x03, 0x13),
        ("joint4", damiao.MotorType.DM4310, 0x04, 0x14),
        ("joint5", damiao.MotorType.DM4310, 0x05, 0x15),
        ("joint6", damiao.MotorType.DM4310, 0x06, 0x16),
        ("gripper_motor", damiao.MotorType.DM4310, 0x07, 0x17),
    ]


def test_b601_declares_only_its_local_gripper_position_limits(
    b601_adapter: SeeedB601DMAdapter,
) -> None:
    limits = b601_adapter.get_limits()

    assert limits.position_lower == [*([None] * B601_DM_DOF), 0.0]
    assert limits.position_upper == [*([None] * B601_DM_DOF), 1.0]
    assert limits.velocity_max == [None] * (B601_DM_DOF + 1)
