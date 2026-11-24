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
from structlog.testing import capture_logs

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.robot.drone.mavlink_connection import MavlinkConnection

if TYPE_CHECKING:
    from pytest_mock import MockerFixture


def test_blank_mavlink_endpoint_is_rejected() -> None:
    parser = BlueprintConfigParser(DroneConnectionModule.blueprint())
    with pytest.raises(BlueprintConfigError, match="MAVLink connection string cannot be blank"):
        parser.parse(["--droneconnectionmodule.connection-string", " "], environ={})


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


def test_failed_close_preserves_connection_error(mocker: MockerFixture) -> None:
    link = mocker.patch(
        "dimos.robot.drone.mavlink_connection.mavutil.mavlink_connection"
    ).return_value
    link.recv_match.side_effect = OSError("initial telemetry unavailable")
    link.close.side_effect = OSError("close failed")
    conn = MavlinkConnection()

    with capture_logs() as logs:
        assert not conn.connect()

    assert any("initial telemetry unavailable" in entry["event"] for entry in logs)
    assert not conn.connected
    assert conn.mavlink is None
