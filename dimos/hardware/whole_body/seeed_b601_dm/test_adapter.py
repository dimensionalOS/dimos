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
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM_JOINTS


@pytest.fixture
def adapter(mocker: MockerFixture) -> Iterator[SeeedB601DMAdapter]:
    mocker.patch.object(
        SeeedB601DMAdapter,
        "_make_can_bus",
        side_effect=lambda _name: can_motor_control.MockCanBus("b601_dm"),
    )
    adapter = SeeedB601DMAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
        gripper_opening_direction="increasing_position",
        gripper_current=0.1,
    )
    yield adapter
    adapter.disconnect()


def test_b601_dm_reuses_shared_driver_and_lifecycle(adapter) -> None:
    assert isinstance(adapter, DamiaoWholeBodyAdapter), (
        "Damiao arms must reuse DamiaoWholeBodyAdapter and can-motor-control; "
        "extend shared profiles/transports rather than introduce a vendor SDK driver"
    )
    for method in (
        "connect",
        "activate",
        "deactivate",
        "read_motor_states",
        "write_motor_commands",
    ):
        assert getattr(SeeedB601DMAdapter, method) is getattr(DamiaoWholeBodyAdapter, method)


def test_official_4340p_uses_equivalent_4340_driver_profile(adapter, mocker) -> None:
    specs = mocker.spy(can_motor_control, "MotorSpec")
    robot = adapter._build_robot()
    assert [call.args[1:] for call in specs.call_args_list] == [
        (damiao.MotorType.DM4340, 0x01, 0x11),
        (damiao.MotorType.DM4340, 0x02, 0x12),
        (damiao.MotorType.DM4340, 0x03, 0x13),
        (damiao.MotorType.DM4310, 0x04, 0x14),
        (damiao.MotorType.DM4310, 0x05, 0x15),
        (damiao.MotorType.DM4310, 0x06, 0x16),
        (damiao.MotorType.DM4310, 0x07, 0x17),
    ]
    assert robot.group_names() == ["arm", "gripper"]
    assert robot.bus_names() == ["b601_dm"]
    assert isinstance(robot["arm"], can_motor_control.Arm)
    assert len(robot["arm"]) == 6
    assert isinstance(robot["gripper"], can_motor_control.Gripper)


def test_native_mock_connect_uses_common_adapter(adapter, mocker) -> None:
    mocker.patch.object(DamiaoWholeBodyAdapter, "_load_kinematic_model")
    assert adapter.connect()
    assert adapter.is_connected()
    assert list(adapter.joint_names) == list(B601_DM_JOINTS)
    assert adapter.get_limits().position_lower == [None, None, None, None, None, None, 0.0]
    assert adapter.get_limits().position_upper == [None, None, None, None, None, None, 1.0]


def test_import_does_not_download_model(mocker) -> None:
    checkout = mocker.patch("dimos.robot.assets.source.RobotDescriptionSource.checkout_path")
    runpy.run_path(adapter_module.__file__)
    checkout.assert_not_called()


@pytest.mark.parametrize("current", [0.0, -0.1, 1.1, float("nan"), float("inf")])
def test_invalid_gripper_current_is_rejected(current) -> None:
    with pytest.raises(ValueError, match="current"):
        SeeedB601DMAdapter(gripper_opening_direction="increasing_position", gripper_current=current)
