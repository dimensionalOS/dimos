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

"""Test-control client for pimsim's BabylonSceneViewerModule.

Mirrors ``dimos/e2e_tests/dim_sim_client.DimSimClient`` so e2e tests can
swap simulator backends with the same fixture shape. Talks to the
running pimsim WebSocket (defaults to ``ws://localhost:8091/ws``) for
scene mutations; publishes nav goals over LCM exactly like DimSim does.
"""

from __future__ import annotations

import json
from typing import Any

from websockets.exceptions import WebSocketException
from websockets.sync.client import ClientConnection, connect

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_URL = "ws://localhost:8091/ws"
DEFAULT_VEHICLE_HEIGHT = 0.0  # pimsim handler adds the configured vehicle height
DEFAULT_WALL_HEIGHT = 1.5
DEFAULT_WALL_THICKNESS = 0.1


class PimSimClient:
    """Mirror of ``DimSimClient`` against pimsim's WebSocket surface."""

    def __init__(self, url: str = DEFAULT_URL) -> None:
        self._url = url
        self._ws: ClientConnection | None = None
        self._goal_request: LCMTransport[PoseStamped] = LCMTransport("/goal_request", PoseStamped)

    def start(self) -> None:
        # The websocket is opened lazily on first send; the pimsim viewer
        # may not be reachable until the blueprint has fully started.
        self._goal_request.start()

    def stop(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            except WebSocketException:
                pass
            self._ws = None
        self._goal_request.stop()

    def set_agent_position(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> None:
        """Respawn the robot at ``(x, y, z)`` facing ``yaw`` (radians, +Z / CCW).

        ``yaw`` is optional so callers using the bare ``SceneControl`` surface
        (``x, y, z``) are unaffected; the browser keeps the scene's default
        heading when no yaw is sent.
        """
        self._send(
            {"type": "respawn_at", "point": [float(x), float(y), float(z)], "yaw": float(yaw)}
        )

    def add_wall(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        *,
        height: float = DEFAULT_WALL_HEIGHT,
        thickness: float = DEFAULT_WALL_THICKNESS,
    ) -> None:
        """Spawn a static box wall between ``(x1, y1)`` and ``(x2, y2)``."""
        self._send(
            {
                "type": "entity_add_wall",
                "x1": float(x1),
                "y1": float(y1),
                "x2": float(x2),
                "y2": float(y2),
                "height": float(height),
                "thickness": float(thickness),
            }
        )

    def clear_entities(self) -> None:
        """Despawn every entity in the world."""
        self._send({"type": "entity_clear"})

    def publish_goal(self, x: float, y: float) -> None:
        """Publish a nav goal on ``/goal_request`` — identical to DimSimClient."""
        self._goal_request.publish(
            PoseStamped(
                position=(x, y, 0),
                orientation=(0, 0, 0, 1),
                frame_id="world",
            )
        )

    def _send(self, payload: dict[str, Any]) -> None:
        self._connection().send(json.dumps(payload))

    def _connection(self) -> ClientConnection:
        if self._ws is None:
            self._ws = connect(self._url, open_timeout=5.0)
        return self._ws


__all__ = ["DEFAULT_URL", "PimSimClient"]
