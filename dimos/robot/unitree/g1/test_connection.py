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

import subprocess
import sys
import threading
from types import SimpleNamespace

from pydantic import ValidationError
import pytest

from dimos.control.connection.device import ConnectionDevice
from dimos.core.global_config import GlobalConfig
from dimos.core.stream import Transport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.connection import (
    G1Connection,
    G1ConnectionConfig,
)
from dimos.robot.unitree.g1.effectors.high_level.high_level_spec import G1StockDevice
from dimos.robot.unitree.g1.motor_device import G1_JOINT_NAMES


@pytest.fixture
def connection(mocker):
    backend = mocker.Mock(spec=ConnectionDevice)
    module = G1Connection(
        g=GlobalConfig(simulation=""), control_mode="motors", deadman_timeout_s=60.0
    )
    mocker.patch.object(module, "create_hardware_device", return_value=backend)
    transport = mocker.Mock(spec=Transport)
    transport.subscribe.return_value = mocker.Mock()
    module.motor_command.transport = transport
    module.cmd_vel.transport = transport
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
        G1ConnectionConfig(soft_start_seconds=value)


def test_hardware_connection_import_and_device_creation_need_no_simulation_packages():
    script = """
import importlib.abc
import sys
import threading
from contextlib import ExitStack

class NoSimulationImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.', 1)[0] in {'robosuite', 'mujoco'}:
            raise AssertionError(f'hardware attempted simulation import: {fullname}')

sys.meta_path.insert(0, NoSimulationImports())

from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.g1.motor_device import G1MotorDevice
from dimos.robot.unitree.g1.connection import G1Connection

with ExitStack() as cleanup:
    connection = G1Connection(g=GlobalConfig(simulation=''), control_mode='motors')
    cleanup.callback(connection.stop)
    assert len(connection.limits) == 29 * 5
    assert isinstance(connection.create_hardware_device(), G1MotorDevice)
    assert not connection.status().connected

assert not {'robosuite', 'mujoco'} & sys.modules.keys()
"""
    completed = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=30
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


@pytest.fixture
def stock_connection(mocker):
    backend = mocker.Mock(spec=G1StockDevice)
    module = G1Connection(g=GlobalConfig(simulation=""), source="g1_a", deadman_timeout_s=0.04)
    mocker.patch.object(module, "create_hardware_device", return_value=backend)
    transport = mocker.Mock(spec=Transport)
    transport.subscribe.return_value = mocker.Mock()
    module.cmd_vel.transport = transport
    try:
        module.start()
        yield SimpleNamespace(module=module, backend=backend, transport=transport)
    finally:
        module.stop()


def test_stock_velocity_uses_checked_named_commands(stock_connection):
    assert stock_connection.module.move(
        Twist(linear=Vector3(0.25, -0.1, 0), angular=Vector3(0, 0, 0.3))
    )
    stock_connection.backend.write.assert_called_once_with(
        {
            "g1_a/base/vx": 0.25,
            "g1_a/base/vy": -0.1,
            "g1_a/base/wz": 0.3,
        }
    )


def test_motor_mode_rejects_stock_commands_without_touching_device(connection):
    assert not connection.module.move(Twist(linear=Vector3(0.1, 0, 0)))
    with pytest.raises(RuntimeError, match="stock firmware requests"):
        connection.module.publish_request("rt/api/arm/request", {"api_id": 7106})
    connection.backend.write.assert_not_called()
    assert "control_mode='stock'" in connection.module.status().last_rejection


def test_stock_mode_rejects_motor_commands_without_touching_device(stock_connection):
    stock_connection.module._on_motor_command(_command())
    stock_connection.backend.write.assert_not_called()
    assert "control_mode='motors'" in stock_connection.module.status().last_rejection


@pytest.mark.parametrize("duration", [-1, float("inf"), float("nan")])
def test_invalid_move_duration_does_not_reach_device(stock_connection, duration):
    assert not stock_connection.module.move(Twist(), duration)
    stock_connection.backend.write.assert_not_called()


def test_nonfinite_stock_velocity_is_rejected(stock_connection):
    assert not stock_connection.module.move(Twist(linear=Vector3(float("nan"), 0, 0)))
    stock_connection.backend.write.assert_not_called()


def test_stock_device_failure_is_reported_and_halted(stock_connection):
    stock_connection.backend.write.side_effect = RuntimeError("DDS unavailable")
    assert not stock_connection.module.move(Twist())
    stock_connection.backend.halt.assert_called_once_with()
    assert "DDS unavailable" in stock_connection.module.status().last_error


def test_timed_move_refreshes_past_deadman_then_sends_zero(stock_connection):
    zero = threading.Event()
    stock_connection.backend.write.side_effect = (
        lambda values: zero.set() if values["g1_a/base/vx"] == 0 else None
    )
    assert stock_connection.module.move(Twist(linear=Vector3(0.2, 0, 0)), duration=0.12)
    assert zero.wait(timeout=2)
    writes = [
        call.args[0]["g1_a/base/vx"] for call in stock_connection.backend.write.call_args_list
    ]
    assert len(writes) >= 3
    assert writes[:-1] == [0.2] * (len(writes) - 1)
    assert writes[-1] == 0
    stock_connection.backend.halt.assert_not_called()


def test_a_new_move_invalidates_old_timed_refresh(stock_connection):
    module = stock_connection.module
    assert module.move(Twist(linear=Vector3(0.2, 0, 0)), duration=10)
    stale_generation = module._velocity_generation
    assert module.move(Twist(linear=Vector3(-0.1, 0, 0)))
    before = stock_connection.backend.write.call_count
    module._refresh_velocity(stale_generation, {"g1_a/base/vx": 0.2}, float("inf"))
    assert stock_connection.backend.write.call_count == before
    assert stock_connection.backend.write.call_args.args[0]["g1_a/base/vx"] == -0.1


def test_halt_invalidates_timed_refresh(stock_connection):
    module = stock_connection.module
    assert module.move(Twist(linear=Vector3(0.2, 0, 0)), duration=10)
    stale_generation = module._velocity_generation
    module.stop_movement()
    before = stock_connection.backend.write.call_count
    module._refresh_velocity(stale_generation, {"g1_a/base/vx": 0.2}, float("inf"))
    assert stock_connection.backend.write.call_count == before
    stock_connection.backend.halt.assert_called_once_with()


def test_stock_rpc_uses_selected_device(stock_connection):
    stock_connection.backend.get_state.return_value = "AI_MODE"
    stock_connection.backend.publish_request.return_value = {"code": 0}
    assert stock_connection.module.get_state() == "AI_MODE"
    assert stock_connection.module.publish_request("rt/api/arm/request", {"api_id": 7106}) == {
        "code": 0
    }
    stock_connection.backend.publish_request.assert_called_once_with(
        "rt/api/arm/request", {"api_id": 7106}
    )


def test_velocity_vendor_request_uses_same_validation(stock_connection):
    with pytest.raises(ValueError, match="rejected"):
        stock_connection.module.publish_request(
            "rt/api/sport/request",
            {
                "api_id": 7105,
                "parameter": {"velocity": [float("nan"), 0, 0]},
            },
        )
    stock_connection.backend.publish_request.assert_not_called()
    stock_connection.backend.write.assert_not_called()


def test_simulation_does_not_claim_stock_firmware_gestures(stock_connection):
    stock_connection.module.config.g.simulation = "mujoco"
    with pytest.raises(NotImplementedError, match="firmware gestures"):
        stock_connection.module.stand_up()
    stock_connection.backend.stand_up.assert_not_called()


@pytest.fixture
def rpc_connection(mocker):
    backend = mocker.Mock(spec=G1StockDevice)
    module = G1Connection(g=GlobalConfig(simulation=""))
    factory = mocker.patch.object(module, "create_hardware_device", return_value=backend)
    try:
        yield SimpleNamespace(module=module, backend=backend, factory=factory)
    finally:
        module.stop()


def test_direct_rpc_connection_needs_no_input_transport(rpc_connection):
    rpc_connection.module.start()
    assert rpc_connection.module.move(Twist(linear=Vector3(0.2, 0, 0)))
    rpc_connection.backend.write.assert_called_once_with(
        {
            "g1/base/vx": 0.2,
            "g1/base/vy": 0,
            "g1/base/wz": 0,
        }
    )


def test_subscription_failure_halts_and_releases_started_device(rpc_connection, mocker):
    transport = mocker.Mock(spec=Transport)
    transport.subscribe.side_effect = RuntimeError("subscription failed")
    rpc_connection.module.cmd_vel.transport = transport
    with pytest.raises(RuntimeError, match="subscription failed"):
        rpc_connection.module.start()
    rpc_connection.backend.halt.assert_called_once_with()
    rpc_connection.backend.stop.assert_called_once_with()
    assert not rpc_connection.module.status().connected


def test_replay_never_falls_back_to_live_hardware(rpc_connection):
    rpc_connection.module.config.g.replay = True
    with pytest.raises(ValueError, match="replay is not implemented"):
        rpc_connection.module.start()
    rpc_connection.factory.assert_not_called()
    rpc_connection.backend.start.assert_not_called()
