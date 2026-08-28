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

"""Behavior tests for the guarded M20 command surface."""

from collections.abc import Callable, Iterator
import math

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.deeprobotics.m20.connection import (
    M20Connection,
    M20ConnectionConfig,
    sanitize_twist,
)


@pytest.fixture
def connection_factory(
    mocker: MockerFixture,
) -> Iterator[Callable[..., M20Connection]]:
    """Create connections and close their transport resources after each test."""
    del mocker  # Keep mocked methods installed until connection.stop() completes.
    connections: list[M20Connection] = []

    def create(**kwargs: float) -> M20Connection:
        connection = M20Connection(rpc_transport=LCMRPC, **kwargs)
        connections.append(connection)
        return connection

    yield create

    for connection in connections:
        connection.stop()


def test_sanitize_twist_bounds_planar_command() -> None:
    config = M20ConnectionConfig(
        max_linear_x=0.5,
        max_linear_y=0.25,
        max_angular_z=0.75,
    )
    command = Twist(
        linear=Vector3(2.0, -2.0, 4.0),
        angular=Vector3(1.0, 2.0, -3.0),
    )

    result = sanitize_twist(command, config)

    assert result == Twist(
        linear=Vector3(0.5, -0.25, 0.0),
        angular=Vector3(0.0, 0.0, -0.75),
    )


def test_sanitize_twist_rejects_nonfinite_command() -> None:
    config = M20ConnectionConfig()
    command = Twist(linear=Vector3(math.nan, 0.0, 0.0))

    result = sanitize_twist(command, config)

    assert result == Twist.zero()


def test_connection_blocks_commands_until_explicitly_armed(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    command = Twist(linear=Vector3(0.2, 0.0, 0.0))

    accepted = connection.move(command)

    assert accepted is False
    publish.assert_called_once_with(Twist.zero())


def test_connection_forwards_bounded_command_after_arm(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory(
        max_linear_x=0.4,
        max_linear_y=0.3,
        max_angular_z=0.6,
    )
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")
    command = Twist(linear=Vector3(0.7, -0.4, 2.0), angular=Vector3(1.0, 2.0, 0.9))

    connection._on_lidar_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    connection.arm()
    accepted = connection.move(command)

    assert accepted is True
    armed_publish.assert_called_once()
    assert armed_publish.call_args.args[0].data is True
    safe_publish.assert_called_once_with(
        Twist(linear=Vector3(0.4, -0.3, 0.0), angular=Vector3(0.0, 0.0, 0.6))
    )


def test_disarm_publishes_zero_and_blocks_following_commands(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")

    connection._on_lidar_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    connection.arm()
    safe_publish.reset_mock()
    armed_publish.reset_mock()
    connection.disarm()
    accepted = connection.move(Twist(linear=Vector3(0.2, 0.0, 0.0)))

    assert accepted is False
    assert safe_publish.call_args_list == [mocker.call(Twist.zero()), mocker.call(Twist.zero())]
    armed_publish.assert_called_once()
    assert armed_publish.call_args.args[0].data is False


def test_connection_refuses_arm_until_native_bridge_is_ready(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))

    accepted = connection.arm()

    assert accepted is False
    assert connection.is_armed() is False
    armed_publish.assert_not_called()


def test_connection_disarms_when_native_bridge_loses_readiness(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    connection.arm()
    safe_publish.reset_mock()
    armed_publish.reset_mock()

    connection._on_command_ready(Bool(False))

    assert connection.is_command_ready() is False
    assert connection.is_armed() is False
    safe_publish.assert_called_once_with(Twist.zero())
    armed_publish.assert_called_once()
    assert armed_publish.call_args.args[0].data is False


def test_connection_refuses_arm_without_a_fresh_lidar_stream(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_command_ready(Bool(True))

    accepted = connection.arm()

    assert accepted is False
    assert connection.is_lidar_ready() is False
    assert connection.is_armed() is False
    armed_publish.assert_not_called()


def test_connection_reports_lidar_recovery(
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()

    connection._on_lidar_ready(Bool(True))

    assert connection.is_lidar_ready() is True


def test_connection_disarms_when_lidar_stream_becomes_stale(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    connection.arm()
    safe_publish.reset_mock()
    armed_publish.reset_mock()

    connection._on_lidar_ready(Bool(False))

    assert connection.is_lidar_ready() is False
    assert connection.is_armed() is False
    safe_publish.assert_called_once_with(Twist.zero())
    armed_publish.assert_called_once()
    assert armed_publish.call_args.args[0].data is False
