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


def test_import_lazy_gravity_model_does_not_resolve_lfs(mocker: MockerFixture) -> None:
    get_data = mocker.patch("dimos.utils.data.get_data")

    runpy.run_path(adapter_module.__file__)

    get_data.assert_not_called()


def test_build_robot_openyam_topology_builds_expected_arm_and_gripper(
    mocker: MockerFixture,
) -> None:
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
    motor_addresses = [
        (
            robot["arm"]["yam_joint1"].name,
            robot["arm"]["yam_joint1"].send_id,
            robot["arm"]["yam_joint1"].recv_id,
        ),
        (
            robot["arm"]["yam_joint2"].name,
            robot["arm"]["yam_joint2"].send_id,
            robot["arm"]["yam_joint2"].recv_id,
        ),
        (
            robot["arm"]["yam_joint3"].name,
            robot["arm"]["yam_joint3"].send_id,
            robot["arm"]["yam_joint3"].recv_id,
        ),
        (
            robot["arm"]["yam_joint4"].name,
            robot["arm"]["yam_joint4"].send_id,
            robot["arm"]["yam_joint4"].recv_id,
        ),
        (
            robot["arm"]["yam_joint5"].name,
            robot["arm"]["yam_joint5"].send_id,
            robot["arm"]["yam_joint5"].recv_id,
        ),
        (
            robot["arm"]["yam_joint6"].name,
            robot["arm"]["yam_joint6"].send_id,
            robot["arm"]["yam_joint6"].recv_id,
        ),
    ]
    assert motor_addresses == [
        ("yam_joint1", 1, 17),
        ("yam_joint2", 2, 18),
        ("yam_joint3", 3, 19),
        ("yam_joint4", 4, 20),
        ("yam_joint5", 5, 21),
        ("yam_joint6", 6, 22),
    ]
    gripper_motor = robot["gripper"].motor
    assert (gripper_motor.name, gripper_motor.send_id, gripper_motor.recv_id) == (
        "yam_gripper",
        8,
        24,
    )
    assert adapter.joint_names == (
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "arm/gripper",
    )


def test_connect_mock_can_bus_validates_real_upstream_groups(
    mocker: MockerFixture,
) -> None:
    mocker.patch.object(can_motor_control, "SocketCanBus", can_motor_control.MockCanBus)
    adapter = OpenYamDamiaoAdapter(
        runtime_config=DamiaoRuntimeConfig(gravity_comp=False),
    )

    assert adapter.connect()
    try:
        assert adapter.is_connected()
        assert not adapter.has_motor_states()
    finally:
        adapter.disconnect()
