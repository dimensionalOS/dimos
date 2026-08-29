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
import json
import math
import struct

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.deeprobotics.m20.connection import (
    GAIT_BASIC,
    GAIT_FLAT_AGILE,
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
    connection._on_localization_ready(Bool(True))
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
    connection._on_localization_ready(Bool(True))
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
    connection._on_localization_ready(Bool(True))

    accepted = connection.arm()

    assert accepted is False
    assert connection.is_armed() is False
    armed_publish.assert_not_called()


def test_connection_temporarily_inhibits_output_without_clearing_operator_arm(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))
    connection._on_localization_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    connection.arm()
    safe_publish.reset_mock()
    armed_publish.reset_mock()

    connection._on_command_ready(Bool(False))

    assert connection.is_command_ready() is False
    assert connection.is_armed() is True
    safe_publish.assert_called_once_with(Twist.zero())
    armed_publish.assert_not_called()

    connection._on_command_ready(Bool(True))
    assert connection.move(Twist(linear=Vector3(0.2, 0.0, 0.0))) is True
    assert safe_publish.call_args.args[0].linear.x == 0.2


def test_standup_runs_complete_control_start_sequence(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    enter_navigation_mode = mocker.patch.object(
        connection, "enter_navigation_mode", return_value=True
    )
    ensure_rl_control = mocker.patch.object(connection, "_ensure_rl_control", return_value=True)
    set_gait = mocker.patch.object(connection, "_set_gait_and_wait", return_value=True)
    wait_ready = mocker.patch.object(connection, "_wait_for_control_readiness", return_value=True)
    arm = mocker.patch.object(connection, "arm", return_value=True)

    assert connection.standup() is True

    enter_navigation_mode.assert_called_once_with()
    ensure_rl_control.assert_called_once_with()
    assert set_gait.call_args_list == [mocker.call(GAIT_BASIC), mocker.call(GAIT_FLAT_AGILE)]
    wait_ready.assert_called_once_with(15.0)
    arm.assert_called_once_with()


def test_standup_stops_when_navigation_usage_mode_is_rejected(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    mocker.patch.object(connection, "enter_navigation_mode", return_value=False)
    ensure_rl_control = mocker.patch.object(connection, "_ensure_rl_control")

    assert connection.standup() is False

    ensure_rl_control.assert_not_called()


def test_navigation_mode_uses_documented_basic_server_apdu(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    response_payload = json.dumps(
        {
            "PatrolDevice": {
                "Type": 1101,
                "Command": 5,
                "Items": {"ErrorCode": 0},
            }
        }
    ).encode()
    response_header = struct.pack(
        "<4sHHB7s",
        bytes.fromhex("eb91eb90"),
        len(response_payload),
        0,
        1,
        b"\0" * 7,
    )
    basic_server_socket = mocker.MagicMock()
    basic_server_socket.__enter__.return_value = basic_server_socket
    basic_server_socket.recv.side_effect = [response_header, response_payload]
    create_connection = mocker.patch(
        "dimos.robot.deeprobotics.m20.connection.socket.create_connection",
        return_value=basic_server_socket,
    )

    assert connection.enter_navigation_mode() is True

    create_connection.assert_called_once_with(("10.21.31.103", 30001), timeout=3.0)
    sent = basic_server_socket.sendall.call_args.args[0]
    magic, payload_length, message_id, encoding, reserved = struct.unpack("<4sHHB7s", sent[:16])
    request = json.loads(sent[16:])
    assert (magic, payload_length, message_id, encoding, reserved) == (
        bytes.fromhex("eb91eb90"),
        len(sent) - 16,
        0,
        1,
        b"\0" * 7,
    )
    assert request["PatrolDevice"]["Type"] == 1101
    assert request["PatrolDevice"]["Command"] == 5
    assert request["PatrolDevice"]["Items"] == {"Mode": 1}


def test_low_level_motion_and_gait_endpoints_publish_vendor_commands(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    motion_publish = mocker.patch.object(connection.motion_state_cmd, "publish")
    gait_publish = mocker.patch.object(connection.gait_cmd, "publish")
    wait = mocker.patch.object(connection, "_wait_for_motion_state", return_value=True)

    assert connection.enter_rl_control() is True
    assert connection.set_navigation_gait() is True
    assert connection.liedown() is True

    assert [call.args[0].data for call in motion_publish.call_args_list] == [17, 4]
    assert gait_publish.call_args.args[0].data == GAIT_FLAT_AGILE
    wait.assert_called_once_with(17, 5.0)


def test_rl_control_transition_follows_vendor_stand_sequence(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    motion_publish = mocker.patch.object(connection.motion_state_cmd, "publish")
    wait = mocker.patch.object(connection, "_wait_for_motion_state", return_value=True)

    assert connection._ensure_rl_control() is True

    assert [call.args[0].data for call in motion_publish.call_args_list] == [1, 17]
    assert wait.call_args_list == [mocker.call(1, 12.0), mocker.call(17, 5.0)]


def test_rejects_unknown_gait(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    publish = mocker.patch.object(connection.gait_cmd, "publish")

    assert connection.set_gait(12345) is False
    publish.assert_not_called()


def test_lidar_diagnostics_do_not_gate_robot_control(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_localization_ready(Bool(True))
    connection._on_command_ready(Bool(True))

    accepted = connection.arm()

    assert accepted is True
    assert connection.is_lidar_ready() is False
    assert connection.is_armed() is True
    armed_publish.assert_called_once()


def test_connection_reports_lidar_recovery(
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()

    connection._on_lidar_ready(Bool(True))

    assert connection.is_lidar_ready() is True


def test_lidar_staleness_does_not_clear_operator_arm(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))
    connection._on_localization_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    connection.arm()
    safe_publish.reset_mock()
    armed_publish.reset_mock()

    connection._on_lidar_ready(Bool(False))

    assert connection.is_lidar_ready() is False
    assert connection.is_armed() is True
    safe_publish.assert_not_called()
    armed_publish.assert_not_called()


def test_pointlio_diagnostics_do_not_gate_robot_control(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))
    connection._on_command_ready(Bool(True))

    accepted = connection.arm()

    assert accepted is True
    assert connection.is_localization_ready() is False
    armed_publish.assert_called_once()


def test_pointlio_staleness_does_not_clear_operator_arm(
    mocker: MockerFixture,
    connection_factory: Callable[..., M20Connection],
) -> None:
    connection = connection_factory()
    safe_publish = mocker.patch.object(connection.safe_cmd_vel, "publish")
    armed_publish = mocker.patch.object(connection.armed, "publish")
    connection._on_lidar_ready(Bool(True))
    connection._on_localization_ready(Bool(True))
    connection._on_command_ready(Bool(True))
    assert connection.arm() is True
    safe_publish.reset_mock()
    armed_publish.reset_mock()

    connection._on_localization_ready(Bool(False))

    assert connection.is_localization_ready() is False
    assert connection.is_armed() is True
    safe_publish.assert_not_called()
    armed_publish.assert_not_called()
