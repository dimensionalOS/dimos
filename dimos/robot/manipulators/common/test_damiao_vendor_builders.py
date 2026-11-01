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

"""Vendor native builders preserve motor wiring and emitted MIT payloads."""

from can_motor_control import damiao
import numpy as np
import pytest

from dimos.robot.manipulators.common import damiao_connection as common
from dimos.robot.manipulators.common._damiao_native_test_helpers import LocalTransport
from dimos.robot.manipulators.common._damiao_test_helpers import _local_test_model
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.openyam.connection import OpenYamConnection


@pytest.mark.parametrize(
    "connection_type,ports,buses,groups,prefixes,motors,kp,kd,direction,current",
    [
        (
            OpenYamConnection,
            {"can_port": "can0"},
            ["openyam"],
            ["arm"],
            ["yam_joint"],
            [damiao.MotorType.DM4340] * 3 + [damiao.MotorType.DM4310] * 3,
            [80, 80, 80, 10, 10, 10],
            [5, 5, 5, 1.5, 1.5, 1.5],
            "decreasing_position",
            0.15,
        ),
        (
            DualOpenYamConnection,
            {"left_can_port": "can0", "right_can_port": "can1"},
            ["left", "right"],
            ["left_arm", "right_arm"],
            ["left_joint", "right_joint"],
            [damiao.MotorType.DM4340] * 3 + [damiao.MotorType.DM4310] * 3,
            [80, 80, 80, 10, 10, 10],
            [5, 5, 5, 1.5, 1.5, 1.5],
            "decreasing_position",
            0.15,
        ),
        (
            OpenArmConnection,
            {"left_can_port": "can0", "right_can_port": "can1"},
            ["left", "right"],
            ["left_arm", "right_arm"],
            ["openarm_left_joint", "openarm_right_joint"],
            [damiao.MotorType.DM8009] * 2
            + [damiao.MotorType.DM4340] * 2
            + [damiao.MotorType.DM4310] * 3,
            [100, 100, 80, 80, 60, 60, 60],
            [1.5, 1.5, 1, 1, 0.8, 0.8, 0.8],
            "decreasing_position",
            0.15,
        ),
        (
            MetalConnection,
            {"can_port": "can0"},
            ["metal"],
            ["arm"],
            ["joint"],
            ["low", "shoulder", "low", "low", "high", "high"],
            [100, 100, 100, 50, 20, 20],
            [5, 5, 5, 5, 0.6, 0.6],
            "increasing_position",
            0.1,
        ),
    ],
)
def test_native_wiring_and_commands(
    connection_type,
    ports,
    buses,
    groups,
    prefixes,
    motors,
    kp,
    kd,
    direction,
    current,
    boundary,
    mocker,
    tmp_path,
):
    profile = codec = None
    if connection_type is MetalConnection:
        profile = mocker.patch.object(
            damiao, "MotorProfile", side_effect=["low", "shoulder", "high"]
        )
        codec = mocker.patch.object(damiao, "DamiaoCodec")
    physics = mocker.patch.object(common, "PhysicalModel").return_value
    physics.normalize.side_effect = list
    physics.gravity.return_value = np.zeros(len(groups) * len(motors))
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_type(
        **ports,
        source="arm",
        model=_local_test_model(connection_type, tmp_path / "control.urdf"),
        deadman_timeout_s=10,
    )
    module.set_transport("position_command", LocalTransport())
    module.set_transport("joint_state", LocalTransport())
    native = boundary[0]
    try:
        module.start()
        module.command({f"arm/{prefix}1/position": 0.7 for prefix in prefixes})
        assert [name for name, _, _ in native.bus_specs] == buses
        for (name, bus, actual_motors), group, prefix, expected_bus in zip(
            native.arm_specs, groups, prefixes, buses, strict=True
        ):
            assert (name, bus) == (group, expected_bus)
            assert actual_motors == [
                (f"{prefix}{index}", motor, index, index + 0x10)
                for index, motor in enumerate(motors, 1)
            ]
            rows = native.groups[group].commands[-1]
            np.testing.assert_equal(rows[:, 0], kp)
            np.testing.assert_equal(rows[:, 1], kd)
            np.testing.assert_equal(rows[:, 2], [0.7, *([0.2] * (len(motors) - 1))])
            np.testing.assert_equal(rows[:, 3:], np.zeros((len(motors), 2)))
        for _, config in native.gripper_specs:
            assert config["opening_direction"] == direction
            assert config["default_current"] == current
            assert config["motor"][2:] == (
                (7, 23) if connection_type is MetalConnection else (8, 24)
            )
        if profile is not None:
            assert profile.call_args_list == [
                mocker.call(0x8000, p_max=6.28, v_max=10, t_max=30),
                mocker.call(0x8001, p_max=6.28, v_max=10, t_max=120),
                mocker.call(0x8002, p_max=6.28, v_max=30, t_max=20),
            ]
            codec.assert_called_once_with(profiles=["low", "shoulder", "high"])
    finally:
        module.stop()


def test_missing_metal_profiles_fail_before_opening_a_bus(boundary, monkeypatch):
    monkeypatch.delattr(damiao, "MotorProfile", raising=False)
    module = MetalConnection()
    try:
        with pytest.raises(
            RuntimeError, match="MetalConnection requires can-motor-control>=0.0.11"
        ):
            module._build_robot()
        boundary[1].assert_not_called()
    finally:
        module.stop()
