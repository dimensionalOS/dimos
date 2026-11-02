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
import pytest
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.seeedstudio_b601_dm import adapter as adapter_module
from dimos.hardware.whole_body.seeedstudio_b601_dm.adapter import SeeedStudioB601DMAdapter
from dimos.robot.manipulators.seeedstudio.joints import SEEEDSTUDIO_DOF, SEEEDSTUDIO_JOINTS


@pytest.fixture
def b601_adapter(mocker: MockerFixture) -> Iterator[SeeedStudioB601DMAdapter]:
    mocker.patch.object(
        SeeedStudioB601DMAdapter,
        "_make_can_bus",
        side_effect=lambda _name: can_motor_control.MockCanBus("b601_dm"),
    )
    adapter = SeeedStudioB601DMAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    yield adapter
    adapter.disconnect()


def test_import_does_not_fetch_the_vendor_model(mocker: MockerFixture) -> None:
    get_data = mocker.patch("dimos.utils.data.get_data")

    runpy.run_path(adapter_module.__file__)

    get_data.assert_not_called()


def test_b601_topology_connects_arm_and_gripper(
    b601_adapter: SeeedStudioB601DMAdapter,
    mocker: MockerFixture,
) -> None:
    robot = b601_adapter._build_robot()

    assert robot.group_names() == ["arm", "gripper"]
    assert isinstance(robot["arm"], can_motor_control.Arm)
    assert len(robot["arm"]) == SEEEDSTUDIO_DOF
    assert isinstance(robot["gripper"], can_motor_control.Gripper)
    assert b601_adapter.joint_names == tuple(SEEEDSTUDIO_JOINTS)
    mocker.patch.object(DamiaoWholeBodyAdapter, "_load_kinematic_model")
    assert b601_adapter.connect()


def test_b601_declares_only_its_local_gripper_position_limits(
    b601_adapter: SeeedStudioB601DMAdapter,
) -> None:
    limits = b601_adapter.get_limits()

    assert limits.position_lower == [*([None] * SEEEDSTUDIO_DOF), 0.0]
    assert limits.position_upper == [*([None] * SEEEDSTUDIO_DOF), 1.0]
    assert limits.velocity_max == [None] * (SEEEDSTUDIO_DOF + 1)


def test_b601_address_selects_the_device_of_its_only_bus() -> None:
    adapter = SeeedStudioB601DMAdapter("/dev/ttyACM0")

    assert adapter._runtime_config.bus_devices == {"b601_dm": "/dev/ttyACM0"}


def test_b601_rejects_an_address_alongside_bus_devices() -> None:
    with pytest.raises(ValueError, match="bus_devices"):
        SeeedStudioB601DMAdapter(
            "/dev/ttyACM0",
            runtime_config=DamiaoRuntimeConfig(bus_devices={"b601_dm": "can0"}),
        )
