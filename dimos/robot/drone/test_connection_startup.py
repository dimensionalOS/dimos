# Copyright 2025-2026 Dimensional Inc.
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

from typing import TYPE_CHECKING

import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.robot.drone.mavlink_connection import MavlinkConnection

if TYPE_CHECKING:
    from pytest_mock import MockerFixture


@pytest.mark.parametrize(
    ("invalid", "valid"),
    [
        (" ", "replay"),
        ("tcp:missing-port", "tcp:localhost:5760"),
        ("localhost:badport", "localhost:14550"),
        ("udpout:host:65536", "udpin::14550"),
        ("tcp: :80", "/dev/serial/by-id/example,115200"),
        ("ws:host:bad:/path", "ws:localhost:8080:/path"),
        ("mcast:bad-port", "mcast:"),
        ("mcast:not-a-group:14550", "mcast:239.1.1.1:14550"),
        ("mcast:127.0.0.1:14550", "mcast:14550"),
    ],
)
def test_mavlink_endpoint_preflight(invalid: str, valid: str) -> None:
    parser = BlueprintConfigParser(DroneConnectionModule.blueprint())
    option = "--droneconnectionmodule.connection-string"
    with pytest.raises(BlueprintConfigError, match="connection_string"):
        parser.parse([option, invalid], environ={})

    parsed = parser.parse([option, valid], environ={})
    assert parsed.module_kwargs("droneconnectionmodule")["connection_string"] == valid


def test_start_reports_connection_failure(mocker: MockerFixture) -> None:
    mocker.patch(
        "dimos.robot.drone.mavlink_connection.mavutil.mavlink_connection",
        side_effect=ConnectionRefusedError("connection refused"),
    )
    video = mocker.patch("dimos.robot.drone.connection_module.DJIDroneVideoStream")
    module = DroneConnectionModule(connection_string="tcp:localhost:5760")
    try:
        with pytest.raises(ConnectionError, match="MAVLink"):
            module.start()
        video.assert_not_called()
    finally:
        module.stop()


def test_heartbeat_timeout_closes_connection_and_allows_retry(mocker: MockerFixture) -> None:
    link = mocker.patch("dimos.robot.drone.mavlink_connection.mavutil.mavlink_connection")
    first, second = mocker.Mock(), mocker.Mock()
    first.wait_heartbeat.return_value = None
    second.recv_match.return_value = None
    link.side_effect = [first, second]
    conn = MavlinkConnection("udp:localhost:14550")
    assert not conn.connect()
    assert not conn.connected
    first.close.assert_called_once_with()

    assert conn.connect()
    assert conn.connected
    conn.disconnect()
    second.close.assert_called_once_with()
