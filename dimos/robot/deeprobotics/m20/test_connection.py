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

from collections.abc import Callable, Iterator
import json
import socket
import struct
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.std_msgs.Int32 import Int32
from dimos.robot.deeprobotics.m20 import connection as m20_connection
from dimos.robot.deeprobotics.m20.connection import MOTION_SIT, M20Connection

INPUTS = ("command_ready", "motion_state", "gait_state", "odometry")
HEADER = struct.Struct("<4sHHB7s")


class FakeTransport:
    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], None]] = []

    def subscribe(self, cb: Callable[[Any], None], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def publish(self, msg: Any) -> None:
        for cb in list(self.subscribers):
            cb(msg)

    def stop(self) -> None:
        self.subscribers.clear()


@pytest.fixture
def basic_server() -> Iterator[socket.socket]:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind(("127.0.0.1", 0))
        udp.settimeout(2.0)
        yield udp


@pytest.fixture
def started(
    basic_server: socket.socket, mocker: MockerFixture
) -> Iterator[Callable[..., M20Connection]]:
    """Start M20Connections on fake inputs with standup stubbed, and stop the ones a test left running."""
    built: list[M20Connection] = []

    def start(**config: Any) -> M20Connection:
        conn = M20Connection(
            basic_server_host="127.0.0.1",
            basic_server_udp_port=basic_server.getsockname()[1],
            **config,
        )
        for name in INPUTS:
            getattr(conn, name).transport = FakeTransport()
        mocker.patch.object(conn, "standup", return_value=True)
        conn.start()
        built.append(conn)
        return conn

    yield start
    for conn in built:
        if not conn._heartbeat_stop.is_set():
            conn.stop()


def test_heartbeat_sends_a_type_100_apdu_to_the_udp_port(
    started: Callable[..., M20Connection], basic_server: socket.socket
) -> None:
    started()
    datagram = basic_server.recv(4096)
    magic, length, _message_id, encoding, _reserved = HEADER.unpack_from(datagram)
    payload = datagram[HEADER.size :]
    device = json.loads(payload)["PatrolDevice"]
    assert (magic, length, encoding) == (bytes.fromhex("eb91eb90"), len(payload), 1)
    assert (device["Type"], device["Command"]) == (100, 100)


@pytest.mark.parametrize("robot_sits", [True, False])
def test_stop_lies_the_robot_down_it_stood_up_and_waits_for_sit(
    started: Callable[..., M20Connection], mocker: MockerFixture, robot_sits: bool
) -> None:
    conn = started(auto_stand=True, sit_timeout_s=0.2)
    motion_state = conn.motion_state.transport

    def sit() -> bool:
        if robot_sits:
            motion_state.publish(Int32(MOTION_SIT))
        return True

    liedown = mocker.patch.object(conn, "liedown", side_effect=sit)
    stop_movement = mocker.patch.object(conn, "stop_movement")
    warning = mocker.patch.object(m20_connection.logger, "warning")
    conn.stop()
    liedown.assert_called_once()
    stop_movement.assert_not_called()
    assert warning.called is not robot_sits


def test_stop_only_zeroes_velocity_without_auto_stand(
    started: Callable[..., M20Connection], mocker: MockerFixture
) -> None:
    conn = started()
    liedown = mocker.patch.object(conn, "liedown")
    stop_movement = mocker.patch.object(conn, "stop_movement")
    conn.stop()
    stop_movement.assert_called_once()
    liedown.assert_not_called()
