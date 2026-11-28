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

from itertools import count
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
    module = DroneConnectionModule(connection_string="test-link")
    try:
        with pytest.raises(ConnectionError, match="MAVLink") as exc:
            module.start()
        assert isinstance(exc.value.__cause__, ConnectionRefusedError)
        video.assert_not_called()
    finally:
        module.stop()


@pytest.mark.parametrize("input_kind", ["ground-station", "empty"])
def test_heartbeat_timeout_closes_connection_and_allows_retry(
    mocker: MockerFixture, input_kind: str
) -> None:
    clock = mocker.patch("dimos.robot.drone.mavlink_connection.time")
    clock.monotonic.side_effect = count(0, 5)
    factory = mocker.patch("dimos.robot.drone.mavlink_connection.mavutil.mavlink_connection")
    first, second = mocker.Mock(), mocker.Mock(target_system=1)
    station, vehicle = mocker.Mock(), mocker.Mock()
    for msg in (station, vehicle):
        msg.get_type.return_value = "HEARTBEAT"
    vehicle.get_srcSystem.return_value = 1
    first.recv_match.return_value = station if input_kind == "ground-station" else None
    first.probably_vehicle_heartbeat.return_value = False
    second.recv_match.side_effect = [station, vehicle]
    second.probably_vehicle_heartbeat.side_effect = [False, True]
    factory.side_effect = [first, second]
    conn = MavlinkConnection("test-link")
    mocker.patch.object(conn, "update_telemetry")

    assert not conn.connect()
    assert not conn.connected
    first.close.assert_called_once_with()
    assert 1 <= first.recv_match.call_count <= 6

    assert conn.connect()
    assert conn.connected
    assert second.recv_match.call_count == 2
    conn.disconnect()
    second.close.assert_called_once_with()


def test_failed_close_preserves_connection_error(mocker: MockerFixture) -> None:
    link = mocker.patch(
        "dimos.robot.drone.mavlink_connection.mavutil.mavlink_connection"
    ).return_value
    link.recv_match.side_effect = OSError("link read failed")
    link.close.side_effect = OSError("close failed")
    conn = MavlinkConnection("test-link")

    with capture_logs() as logs:
        assert not conn.connect()

    assert any("link read failed" in entry["event"] for entry in logs)
    assert not conn.connected
    assert conn.mavlink is None
