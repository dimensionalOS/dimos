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

import runpy

import can_motor_control
from pytest_mock import MockerFixture

from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.openyam_damiao import adapter as adapter_module
from dimos.hardware.whole_body.openyam_damiao.adapter import OpenYamDamiaoAdapter


def test_import_does_not_resolve_gravity_model_lfs(mocker: MockerFixture) -> None:
    get_data = mocker.patch("dimos.utils.data.get_data")

    runpy.run_path(adapter_module.__file__)

    get_data.assert_not_called()


def test_openyam_builds_upstream_arm_and_gripper(mocker: MockerFixture) -> None:
    mocker.patch.object(can_motor_control, "SocketCanBus", can_motor_control.MockCanBus)
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(
            bus_addresses={"openyam": "test_can"},
            gravity_comp=False,
        )
    )

    robot = adapter._build_robot()

    assert robot.bus_names() == ["openyam"]
    assert robot.group_names() == ["arm", "gripper"]
    assert isinstance(robot["arm"], can_motor_control.Arm)
    assert len(robot["arm"]) == 6
    assert isinstance(robot["gripper"], can_motor_control.Gripper)
    assert adapter.joint_names == (
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "arm/gripper",
    )
