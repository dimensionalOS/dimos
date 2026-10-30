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

"""WebRTC stock firmware device owned by G1Connection."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.control.contract.convert import twist_from_values
from dimos.robot.unitree.connection import UnitreeWebRTCConnection

if TYPE_CHECKING:
    from dimos.robot.unitree.g1.connection import G1ConnectionConfig


class G1WebRtcDevice:
    def __init__(self, config: G1ConnectionConfig) -> None:
        self._config = config
        self._connection: UnitreeWebRTCConnection | None = None

    def start(self) -> None:
        self._connection = UnitreeWebRTCConnection(
            self._config.ip,
            self._config.connection_mode,
            aes_128_key=self._config.aes_128_key,
        )
        self._connection.start()

    def _connected(self) -> UnitreeWebRTCConnection:
        if self._connection is None:
            raise RuntimeError("G1 WebRTC device is not connected")
        return self._connection

    def write(self, values: dict[str, float]) -> None:
        twist = twist_from_values(values, f"{self._config.source}/base")
        if not self._connected().move(twist):
            raise RuntimeError("G1 WebRTC rejected the velocity command")

    def halt(self) -> None:
        if self._connection is not None:
            self._connection.stop_movement()

    def stop(self) -> None:
        connection, self._connection = self._connection, None
        if connection is not None:
            connection.stop()

    def get_state(self) -> str:
        self._connected()
        return "Connected (WebRTC)"

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        response: dict[str, Any] = self._connected().publish_request(topic, data)
        return response

    def stand_up(self) -> bool:
        return self._connected().standup()

    def lie_down(self) -> bool:
        return self._connected().liedown()
