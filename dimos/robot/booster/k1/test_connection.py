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

import importlib
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.booster.k1.connection import K1Connection, decode_state, encode_command
from dimos.robot.booster.k1.control_config import HOME, JOINTS, KD, KP


@pytest.fixture
def state():
    return SimpleNamespace(
        motor_state_serial=[
            SimpleNamespace(q=q, dq=i * 0.01, tau_est=i * 0.1) for i, q in enumerate(HOME)
        ],
        # Intentionally different: using the parallel ankles would be wrong.
        motor_state_parallel=[SimpleNamespace(q=99, dq=99) for _ in HOME],
        imu_state=SimpleNamespace(rpy=[0, 0, np.pi / 2], gyro=[1, 2, 3], acc=[4, 5, 6]),
    )


@pytest.fixture
def sdk(mocker):
    sdk = mocker.Mock()
    sdk.kJointCntK1 = 22
    sdk.LowCmd.side_effect = SimpleNamespace
    sdk.MotorCmd.side_effect = SimpleNamespace
    sdk.LowCmdType.SERIAL = 1
    sdk.B1LocoClient.return_value.ChangeMode.return_value = 0
    return sdk


@pytest.fixture
def command():
    return MotorCommandArray(q=list(HOME), kp=list(KP), kd=list(KD), timestamp=100)


@pytest.fixture
def connection(mocker, sdk):
    original_import = importlib.import_module
    mocker.patch(
        "dimos.robot.booster.k1.connection.importlib.import_module",
        side_effect=lambda name, *a: sdk
        if name == "booster_robotics_sdk_python"
        else original_import(name, *a),
    )
    original_thread = threading.Thread
    mocker.patch(
        "dimos.robot.booster.k1.connection.threading.Thread",
        side_effect=lambda *a, **kw: mocker.Mock()
        if kw.get("name") == "k1-sdk"
        else original_thread(*a, **kw),
    )
    clock = mocker.Mock(return_value=10)
    mocker.patch(
        "dimos.robot.booster.k1.connection.time", SimpleNamespace(time=lambda: 100, monotonic=clock)
    )
    module = K1Connection(network_interface="test0")
    mocker.patch.object(module.motor_command, "subscribe", return_value=lambda: None)
    mocker.patch.object(module.joint_state, "publish")
    mocker.patch.object(module.imu, "publish")
    try:
        module.start()
        yield module, clock
    finally:
        module.stop()


def test_serial_feedback_and_imu_share_one_timestamp(state):
    joints, imu = decode_state(state, 123)
    assert joints.name == list(JOINTS)
    assert joints.position == list(HOME)
    assert joints.velocity[-1] == 0.21
    assert joints.effort[-1] == 2.1
    assert joints.ts == imu.ts == 123
    assert [imu.orientation.w, imu.orientation.z] == pytest.approx([2**-0.5] * 2)
    assert [imu.angular_velocity.x, imu.linear_acceleration.z] == [1, 6]


def test_command_uses_serial_layout_and_all_five_motor_fields(sdk, command):
    command.dq[15], command.tau[15] = 0.4, 0.2
    msg = encode_command(sdk, command)
    assert msg.cmd_type == sdk.LowCmdType.SERIAL
    assert len(msg.motor_cmd) == 22
    motor = msg.motor_cmd[15]
    assert (motor.q, motor.dq, motor.kp, motor.kd, motor.tau, motor.weight) == (
        0,
        0.4,
        65,
        1,
        0.2,
        1,
    )


@pytest.mark.parametrize("damage", ["count", "nan", "gain", "limit"])
def test_bad_commands_are_rejected_before_sdk_construction(sdk, command, damage):
    if damage == "count":
        command.q.pop()
    elif damage == "nan":
        command.dq[0] = float("nan")
    elif damage == "gain":
        command.kp[0] = -1
    else:
        command.q[0] = 10
    with pytest.raises(ValueError):
        encode_command(sdk, command)
    sdk.LowCmd.assert_not_called()


async def test_passive_start_explicit_activation_and_timeout(connection, sdk, state, command):
    module, clock = connection
    module._on_state(state)
    await module.handle_motor_command(command)
    module._tick()
    publisher = sdk.B1LowCmdPublisher.return_value
    publisher.Write.assert_not_called()
    sdk.B1LocoClient.return_value.ChangeMode.assert_not_called()
    module.activate()
    module._tick()
    assert module.status()["active"]
    assert publisher.Write.call_count == 2  # prime hold, then the policy target
    clock.return_value = 10.2
    with pytest.raises(RuntimeError, match="timed out"):
        module._tick()
    assert not module.status()["active"]
    sdk.B1LocoClient.return_value.ChangeMode.assert_called_with(sdk.RobotMode.kDamping)
    assert publisher.Write.call_count == 2


async def test_failed_mode_change_does_not_enable_motor_stream(connection, sdk, state, command):
    module, _ = connection
    module._on_state(state)
    await module.handle_motor_command(command)
    sdk.B1LocoClient.return_value.ChangeMode.return_value = 7
    with pytest.raises(RuntimeError, match="failed: 7"):
        module.activate()
    assert not module.status()["active"]


async def test_replayed_command_does_not_refresh_watchdog(connection, state, command):
    module, clock = connection
    module._on_state(state)
    await module.handle_motor_command(command)
    module.activate()
    clock.return_value = 10.2
    module._on_state(state)
    await module.handle_motor_command(command)
    with pytest.raises(RuntimeError, match="timed out"):
        module._tick()
    assert not module.status()["active"]


def test_stop_closes_channels_without_changing_mode_when_passive(connection, sdk):
    module, _ = connection
    module.stop()
    sdk.B1LowStateSubscriber.return_value.CloseChannel.assert_called_once()
    sdk.B1LowCmdPublisher.return_value.CloseChannel.assert_called_once()
    sdk.B1LocoClient.return_value.ChangeMode.assert_not_called()


def test_wrong_robot_feedback_cannot_activate(connection, state):
    module, _ = connection
    state.motor_state_serial.pop()
    module._on_state(state)
    with pytest.raises(RuntimeError, match="timed out"):
        module.activate()
    assert "22 serial motors" in module.status()["error"]
