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

"""Go2 Fleet Connection — manage multiple Go2 robots as a fleet.

The primary robot uses the full Go2WebRtcConnection (sensors + RPCs).
Additional robots use a minimal command-only client (no sensor streams),
composing `UnitreeWebRtcSession` for transport.
"""

from __future__ import annotations

from collections.abc import Sequence
import sys
from typing import TYPE_CHECKING, Any

from pydantic import Field, model_validator
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.core.core import rpc
from dimos.robot.unitree.go2.config import ConnectionConfig
from dimos.robot.unitree.go2.connection_webrtc import Go2WebRtcConnection
from dimos.robot.unitree.webrtc_session import UnitreeWebRtcSession
from dimos.utils.logging_config import setup_logger

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Twist import Twist

logger = setup_logger()


class FleetConnectionConfig(ConnectionConfig):
    ips: Sequence[str] = Field(
        default_factory=lambda m: [ip.strip() for ip in m["g"].robot_ips.split(",")]
    )

    @model_validator(mode="after")
    def set_ip_after_validation(self) -> Self:
        if self.ip is None:
            self.ip = self.ips[0]
        return self


class _FleetMemberClient:
    """Command-only WebRTC client for extra fleet robots.

    Wraps a `UnitreeWebRtcSession` and adds the Go2 sport-mode commands
    (standup/liedown/balance_stand/set_obstacle_avoidance). No sensor
    streams — fleet does not subscribe to extras.
    """

    def __init__(self, ip: str) -> None:
        self.session = UnitreeWebRtcSession(ip)

    def start(self) -> None:
        self.session.start()

    def stop(self) -> None:
        self.session.stop()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return self.session.move(twist, duration)

    def standup(self) -> bool:
        return bool(
            self.session.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]})
        )

    def liedown(self) -> bool:
        return bool(
            self.session.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})
        )

    def balance_stand(self) -> bool:
        return bool(
            self.session.publish_request(
                RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]}
            )
        )

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self.session.publish_request(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {"api_id": 1001, "parameter": {"enable": int(enabled)}},
        )

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        return self.session.publish_request(topic, data)  # type: ignore[no-any-return]


class Go2FleetConnection(Go2WebRtcConnection):
    """Inherits all single-robot behaviour from Go2WebRtcConnection for the
    primary (first) robot. Additional robots only receive broadcast commands
    (move, standup, liedown, balance_stand, set_obstacle_avoidance,
    publish_request) via _FleetMemberClient.

    Fleets are real-hardware only — there's no sim/replay equivalent.
    """

    config: FleetConnectionConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._extra_ips = self.config.ips[1:]
        self._extra_connections: list[_FleetMemberClient] = []

    @rpc
    def start(self) -> None:
        self._extra_connections.clear()
        for ip in self._extra_ips:
            client = _FleetMemberClient(ip)
            client.start()
            self._extra_connections.append(client)

        # Parent starts primary robot, subscribes sensors, calls standup() on it.
        super().start()
        for client in self._extra_connections:
            client.balance_stand()
            client.set_obstacle_avoidance(self.config.g.obstacle_avoidance)

    @rpc
    def stop(self) -> None:
        # One robot's error must not prevent others from stopping.
        for client in self._extra_connections:
            try:
                client.liedown()
            except Exception as e:
                logger.error(f"Error lying down fleet Go2: {e}")
            try:
                client.stop()
            except Exception as e:
                logger.error(f"Error stopping fleet Go2: {e}")
        self._extra_connections.clear()
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        results: list[bool] = [super().move(twist, duration)]
        for client in self._extra_connections:
            try:
                results.append(client.move(twist, duration))
            except Exception as e:
                logger.error(f"Fleet move failed: {e}")
                results.append(False)
        return all(results)

    @rpc
    def standup(self) -> bool:
        results: list[bool] = [super().standup()]
        for client in self._extra_connections:
            try:
                results.append(client.standup())
            except Exception as e:
                logger.error(f"Fleet standup failed: {e}")
                results.append(False)
        return all(results)

    @rpc
    def liedown(self) -> bool:
        results: list[bool] = [super().liedown()]
        for client in self._extra_connections:
            try:
                results.append(client.liedown())
            except Exception as e:
                logger.error(f"Fleet liedown failed: {e}")
                results.append(False)
        return all(results)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to all robots, return primary's response."""
        for client in self._extra_connections:
            try:
                client.publish_request(topic, data)
            except Exception as e:
                logger.error(f"Fleet publish_request failed: {e}")
        return super().publish_request(topic, data)
