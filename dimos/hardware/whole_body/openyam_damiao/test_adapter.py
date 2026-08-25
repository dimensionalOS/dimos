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
from dimos.hardware.whole_body.openyam_damiao import adapter as adapter_module
from dimos.hardware.whole_body.openyam_damiao.adapter import OpenYamDamiaoAdapter
from dimos.robot.manipulators.openyam.config import OPENYAM_DOF


@pytest.fixture
def openyam_adapter(mocker: MockerFixture) -> Iterator[OpenYamDamiaoAdapter]:
    mocker.patch.object(can_motor_control, "SocketCanBus", can_motor_control.MockCanBus)
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )
    yield adapter
    adapter.disconnect()


def test_import_lazy_gravity_model_does_not_resolve_lfs(mocker: MockerFixture) -> None:
    get_data = mocker.patch("dimos.utils.data.get_data")

    runpy.run_path(adapter_module.__file__)

    get_data.assert_not_called()


def test_openyam_topology_connects_arm_and_gripper(
    openyam_adapter: OpenYamDamiaoAdapter,
    mocker: MockerFixture,
) -> None:
    robot = openyam_adapter._build_robot()

    assert robot.group_names() == ["arm", "gripper"]
    assert isinstance(robot["arm"], can_motor_control.Arm)
    assert len(robot["arm"]) == OPENYAM_DOF
    assert isinstance(robot["gripper"], can_motor_control.Gripper)
    mocker.patch.object(DamiaoWholeBodyAdapter, "_load_kinematic_model")
    assert openyam_adapter.connect()


def test_openyam_declares_only_its_local_gripper_position_limits(
    openyam_adapter: OpenYamDamiaoAdapter,
) -> None:
    limits = openyam_adapter.get_limits()

    assert limits.position_lower == [*([None] * OPENYAM_DOF), 0.0]
    assert limits.position_upper == [*([None] * OPENYAM_DOF), 1.0]
    assert limits.velocity_max == [None] * (OPENYAM_DOF + 1)
