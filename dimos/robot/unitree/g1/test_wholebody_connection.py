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

from __future__ import annotations

from types import SimpleNamespace

from pydantic import ValidationError
import pytest

from dimos.control.connection.device import ConnectionDevice
from dimos.core.global_config import GlobalConfig
from dimos.core.stream import Transport
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.motor_device import G1_JOINT_NAMES
from dimos.robot.unitree.g1.wholebody_connection import (
    G1WholeBodyConnection,
    G1WholeBodyConnectionConfig,
)


@pytest.fixture
def connection(mocker):
    backend = mocker.Mock(spec=ConnectionDevice)
    module = G1WholeBodyConnection(g=GlobalConfig(simulation=""), deadman_timeout_s=60.0)
    mocker.patch.object(module, "create_hardware_device", return_value=backend)
    transport = mocker.Mock(spec=Transport)
    transport.subscribe.return_value = mocker.Mock()
    module.motor_command.transport = transport
    try:
        module.start()
        yield SimpleNamespace(module=module, backend=backend)
    finally:
        module.stop()


def _command():
    n = len(G1_JOINT_NAMES)
    return MotorCommandArray(q=[1.0] * n, dq=[0.5] * n, kp=[100.0] * n, kd=[5.0] * n, tau=[8.0] * n)


def test_motor_command_preserves_all_pd_values_and_canonical_order(connection):
    connection.module._on_motor_command(_command())

    values = connection.backend.write.call_args.args[0]
    assert len(values) == 29 * 5
    assert {
        key: values[f"g1/left_hip_pitch/{key}"]
        for key in ("position", "velocity", "effort", "kp", "kd")
    } == {
        "position": 1.0,
        "velocity": 0.5,
        "effort": 8.0,
        "kp": 100.0,
        "kd": 5.0,
    }
    assert list(values)[-1] == f"{G1_JOINT_NAMES[-1]}/kd"


def test_virtual_pd_targets_are_not_clamped_to_joint_position_limits(connection):
    message = _command()
    message.q[0] = 100.0

    connection.module._on_motor_command(message)

    assert connection.backend.write.call_args.args[0]["g1/left_hip_pitch/position"] == 100.0


@pytest.mark.parametrize("column", ["q", "dq", "kp", "kd", "tau"])
def test_malformed_motor_vectors_are_dropped_before_device_write(connection, column):
    message = _command()
    getattr(message, column).pop()

    connection.module._on_motor_command(message)

    connection.backend.write.assert_not_called()
    assert connection.module.status().last_rejection is not None


def test_wrong_joint_count_is_dropped(connection):
    connection.module._on_motor_command(MotorCommandArray(q=[0.0] * 5))

    connection.backend.write.assert_not_called()


@pytest.mark.parametrize(
    ("column", "value"),
    [
        ("q", float("nan")),
        ("dq", float("inf")),
        ("tau", float("-inf")),
        ("kp", -1.0),
        ("kd", -1.0),
    ],
)
def test_invalid_pd_values_do_not_reach_the_device(connection, column, value):
    message = _command()
    getattr(message, column)[0] = value

    connection.module._on_motor_command(message)

    connection.backend.write.assert_not_called()
    assert connection.module.status().last_rejection is not None


@pytest.mark.parametrize("value", [float("inf"), float("-inf"), float("nan")])
def test_non_finite_soft_start_is_rejected(value):
    with pytest.raises(ValidationError):
        G1WholeBodyConnectionConfig(soft_start_seconds=value)
