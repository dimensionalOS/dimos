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

"""Go2 Fleet Connection - manage multiple Go2 robots as a fleet"""

from __future__ import annotations

from collections.abc import Sequence
import sys
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.go2.connection import (
    ConnectionConfig,
    GO2Connection,
    Go2ConnectionProtocol,
    make_connection,
)
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


class Go2FleetConnection(GO2Connection):
    """Inherits all single-robot behaviour from GO2Connection for the primary
    (first) robot (the Rust module on real robots). Additional robots only
    receive broadcast commands (move, standup, liedown, publish_request), over
    Python WebRTC on real robots.
    """

    config: FleetConnectionConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._extra_ips = self.config.ips[1:]
        self._extra_connections: list[Go2ConnectionProtocol] = []

    @rpc
    def start(self) -> None:
        self._extra_connections.clear()
        for ip in self._extra_ips:
            # The AES-128 key is per-device; a heterogeneous fleet would need a
            # per-IP mapping here rather than this shared key.
            conn = make_connection(ip, self.config) or UnitreeWebRTCConnection(
                ip,
                aes_128_key=self.config.aes_128_key,
                velocity_api=self.config.velocity_api,
            )
            conn.start()
            self._extra_connections.append(conn)

        # Parent starts the primary robot. On replay and the sims it calls standup(), which stands every robot; the
        # Rust module stands only its own, so the extras stand up here.
        super().start()
        if self.connection is None:
            for conn in self._extra_connections:
                conn.standup()
            time.sleep(self.config.timing.stand_up_settle)
            # the Rust module drives the primary from cmd_vel; the extras follow it here
            self.register_disposable(
                Disposable(
                    self.cmd_vel.subscribe(
                        lambda twist: self._each_extra("move", lambda c: c.move(twist))
                    )
                )
            )
        for conn in self._extra_connections:
            conn.balance_stand()
            conn.set_obstacle_avoidance(self.config.obstacle_avoidance)

    @rpc
    def stop(self) -> None:
        # one robot's error should not prevent others from stopping
        for conn in self._extra_connections:
            try:
                conn.liedown()
            except Exception as e:
                logger.error(f"Error lying down fleet Go2: {e}")
            try:
                conn.stop()
            except Exception as e:
                logger.error(f"Error stopping fleet Go2: {e}")
        self._extra_connections.clear()
        super().stop()

    def _each_extra(self, what: str, call: Any) -> bool:
        results = []
        for conn in self._extra_connections:
            try:
                results.append(call(conn))
            except Exception as e:
                logger.error(f"Fleet {what} failed: {e}")
                results.append(False)
        return all(results)

    def _broadcast(self, what: str, primary: bool, call: Any) -> bool:
        """The primary's result and every extra robot's, each failure logged and counted as False."""
        extras = self._each_extra(what, call)
        return primary and extras

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        primary = super().move(twist, duration)
        return self._broadcast("move", primary, lambda conn: conn.move(twist, duration))

    @rpc
    def standup(self) -> bool:
        return self._broadcast("standup", super().standup(), lambda conn: conn.standup())

    @rpc
    def liedown(self) -> bool:
        return self._broadcast("liedown", super().liedown(), lambda conn: conn.liedown())

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to all robots, return primary's response."""
        for conn in self._extra_connections:
            try:
                conn.publish_request(topic, data)
            except Exception as e:
                logger.error(f"Fleet publish_request failed: {e}")
        return super().publish_request(topic, data)
