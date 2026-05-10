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

"""Tests for RerunWebSocketServer."""

from __future__ import annotations

import asyncio
import json
import threading
import time
from typing import Any

import pytest
import websockets.asyncio.client as ws_client

from dimos.core.global_config import global_config
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_TEST_PORT = 13031


class MockViewerPublisher:
    """Simulates dimos-viewer sending JSON events over WebSocket."""

    def __init__(self, url: str) -> None:
        self._url = url
        self._ws: Any = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def __enter__(self) -> MockViewerPublisher:
        self._loop = asyncio.new_event_loop()
        self._ws = self._loop.run_until_complete(self._connect())
        return self

    def __exit__(self, *_: Any) -> None:
        if self._ws is not None and self._loop is not None:
            self._loop.run_until_complete(self._ws.close())
        if self._loop is not None:
            self._loop.close()

    async def _connect(self) -> Any:
        return await ws_client.connect(self._url)

    def send_click(
        self, x: float, y: float, z: float, entity_path: str = "", timestamp_ms: int = 0
    ) -> None:
        self._send(
            {
                "type": "click",
                "x": x,
                "y": y,
                "z": z,
                "entity_path": entity_path,
                "timestamp_ms": timestamp_ms,
            }
        )

    def send_twist(
        self,
        linear_x: float,
        linear_y: float,
        linear_z: float,
        angular_x: float,
        angular_y: float,
        angular_z: float,
    ) -> None:
        self._send(
            {
                "type": "twist",
                "linear_x": linear_x,
                "linear_y": linear_y,
                "linear_z": linear_z,
                "angular_x": angular_x,
                "angular_y": angular_y,
                "angular_z": angular_z,
            }
        )

    def send_stop(self) -> None:
        self._send({"type": "stop"})

    def flush(self, delay: float = 0.1) -> None:
        time.sleep(delay)

    def _send(self, msg: dict[str, Any]) -> None:
        assert self._loop is not None and self._ws is not None
        self._loop.run_until_complete(self._ws.send(json.dumps(msg)))


@pytest.fixture()
def server(wait_for_server: Any) -> RerunWebSocketServer:
    original_port = global_config.rerun_websocket_server_port
    global_config.update(rerun_websocket_server_port=_TEST_PORT)
    try:
        module = RerunWebSocketServer()
        module.start()
        wait_for_server(_TEST_PORT)
        yield module  # type: ignore[misc]
        module.stop()
    finally:
        global_config.update(rerun_websocket_server_port=original_port)


@pytest.fixture()
def publisher(server: RerunWebSocketServer) -> MockViewerPublisher:
    with MockViewerPublisher(f"ws://127.0.0.1:{_TEST_PORT}/ws") as publisher:
        yield publisher  # type: ignore[misc]


def test_click_publishes_point_stamped(
    server: RerunWebSocketServer, publisher: MockViewerPublisher
) -> None:
    """Click event arrives as PointStamped with correct coords, frame_id, and timestamp."""
    received: list[Any] = []
    done = threading.Event()

    unsub = server.clicked_point.subscribe(lambda point: (received.append(point), done.set()))

    publisher.send_click(1.5, 2.5, 0.0, "/robot/base", timestamp_ms=5000)
    publisher.flush()
    done.wait(timeout=2.0)
    unsub()

    assert len(received) == 1
    point = received[0]
    assert point.x == pytest.approx(1.5)
    assert point.y == pytest.approx(2.5)
    assert point.z == pytest.approx(0.0)
    assert point.frame_id == "/robot/base"
    assert point.ts == pytest.approx(5.0)


def test_twist_publishes_on_tele_cmd_vel(
    server: RerunWebSocketServer, publisher: MockViewerPublisher
) -> None:
    """Twist event arrives as Twist on tele_cmd_vel."""
    received: list[Any] = []
    done = threading.Event()

    unsub = server.tele_cmd_vel.subscribe(lambda twist: (received.append(twist), done.set()))

    publisher.send_twist(0.5, 0.0, 0.0, 0.0, 0.0, 0.8)
    publisher.flush()
    done.wait(timeout=2.0)
    unsub()

    assert len(received) == 1
    assert received[0].linear.x == pytest.approx(0.5)
    assert received[0].angular.z == pytest.approx(0.8)


def test_stop_publishes_zero_twist(
    server: RerunWebSocketServer, publisher: MockViewerPublisher
) -> None:
    """Stop event publishes a zero Twist on tele_cmd_vel."""
    received: list[Any] = []
    done = threading.Event()

    unsub = server.tele_cmd_vel.subscribe(lambda twist: (received.append(twist), done.set()))

    publisher.send_stop()
    publisher.flush()
    done.wait(timeout=2.0)
    unsub()

    assert len(received) == 1
    assert received[0].is_zero()


def test_invalid_json_does_not_crash(server: RerunWebSocketServer) -> None:
    """Malformed JSON is silently dropped; server stays alive for the next message."""

    async def _send_bad() -> None:
        async with ws_client.connect(f"ws://127.0.0.1:{_TEST_PORT}/ws") as ws:
            await ws.send("this is not json {{")
            await asyncio.sleep(0.1)
            await ws.send(json.dumps({"type": "heartbeat", "timestamp_ms": 0}))
            await asyncio.sleep(0.1)

    asyncio.run(_send_bad())


def test_mixed_message_sequence(
    server: RerunWebSocketServer, publisher: MockViewerPublisher
) -> None:
    """Realistic session: heartbeat, click, twist, stop — only the click produces a point."""
    received: list[Any] = []
    done = threading.Event()
    unsub = server.clicked_point.subscribe(lambda point: (received.append(point), done.set()))

    publisher.send_click(7.0, 8.0, 9.0, "/map", timestamp_ms=1100)
    publisher.send_twist(0.3, 0.0, 0.0, 0.0, 0.0, 0.2)
    publisher.send_stop()
    publisher.flush()
    done.wait(timeout=2.0)
    unsub()

    assert len(received) == 1
    assert received[0].x == pytest.approx(7.0)
    assert received[0].y == pytest.approx(8.0)
    assert received[0].z == pytest.approx(9.0)
