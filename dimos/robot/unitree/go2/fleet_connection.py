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

from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.robot.unitree.go2.connection import (
    GO2Connection,
    Go2ConnectionProtocol,
    make_connection,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Twist

logger = setup_logger()


class Go2FleetConnection(GO2Connection):
    """Inherits all single-robot behaviour from GO2Connection for the primary
    (first) robot. Additional robots only receive broadcast commands
    (move, standup, liedown, publish_request).
    """

    def __init__(
        self,
        ips: list[str] | None = None,
        cfg: GlobalConfig = global_config,
        *args: object,
        **kwargs: object,
    ) -> None:
        if not ips:
            raw = cfg.robot_ips
            if not raw:
                raise ValueError(
                    "No IPs provided. Pass ips= or set ROBOT_IPS (e.g. ROBOT_IPS=10.0.0.102,10.0.0.209)"
                )
            ips = [ip.strip() for ip in raw.split(",") if ip.strip()]
        self._extra_ips = ips[1:]
        self._extra_connections: list[Go2ConnectionProtocol] = []
        super().__init__(ips[0], cfg, *args, **kwargs)

    @rpc
    def start(self) -> None:
        self._extra_connections.clear()
        for ip in self._extra_ips:
            conn = make_connection(ip, self._global_config)
            conn.start()
            self._extra_connections.append(conn)

        # Parent starts primary robot, subscribes sensors, calls standup() on all
        super().start()
        for conn in self._extra_connections:
            conn.balance_stand()
            conn.set_obstacle_avoidance(self._global_config.obstacle_avoidance)

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

    @property
    def _all_connections(self) -> list[Go2ConnectionProtocol]:
        return [self.connection, *self._extra_connections]

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        results: list[bool] = []
        for conn in self._all_connections:
            try:
                results.append(conn.move(twist, duration))
            except Exception as e:
                logger.error(f"Fleet move failed: {e}")
                results.append(False)
        return all(results)

    @rpc
    def standup(self) -> bool:
        results: list[bool] = []
        for conn in self._all_connections:
            try:
                results.append(conn.standup())
            except Exception as e:
                logger.error(f"Fleet standup failed: {e}")
                results.append(False)
        return all(results)

    @rpc
    def liedown(self) -> bool:
        results: list[bool] = []
        for conn in self._all_connections:
            try:
                results.append(conn.liedown())
            except Exception as e:
                logger.error(f"Fleet liedown failed: {e}")
                results.append(False)
        return all(results)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to all robots, return primary's response."""
        for conn in self._extra_connections:
            try:
                conn.publish_request(topic, data)
            except Exception as e:
                logger.error(f"Fleet publish_request failed: {e}")
        return self.connection.publish_request(topic, data)


go2_fleet_connection = Go2FleetConnection.blueprint


__all__ = ["Go2FleetConnection", "go2_fleet_connection"]
