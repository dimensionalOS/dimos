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

"""End-to-end tests for dimos-viewer ↔ RerunWebSocketServer protocol."""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import threading
import time
from typing import Any

import pytest
import websockets.asyncio.client as ws_client

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import global_config
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_E2E_PORT = 13032


@pytest.fixture()
def server(wait_for_server: Any) -> RerunWebSocketServer:
    original_port = global_config.rerun_websocket_server_port
    global_config.update(rerun_websocket_server_port=_E2E_PORT)
    try:
        module = RerunWebSocketServer()
        module.start()
        wait_for_server(_E2E_PORT)
        yield module
        module.stop()
    finally:
        global_config.update(rerun_websocket_server_port=original_port)


def _send_messages(port: int, messages: list[dict[str, Any]], *, delay: float = 0.05) -> None:
    async def _run() -> None:
        async with ws_client.connect(f"ws://127.0.0.1:{port}/ws") as ws:
            for msg in messages:
                await ws.send(json.dumps(msg))
            await asyncio.sleep(delay)

    asyncio.run(_run())


class TestViewerProtocolE2E:
    """Verify the Python-server side of the viewer ↔ DimOS protocol."""

    def test_viewer_click_reaches_stream(self, server: RerunWebSocketServer) -> None:
        """A viewer click over WebSocket publishes PointStamped."""
        received: list[Any] = []
        done = threading.Event()
        unsub = server.clicked_point.subscribe(lambda pt: (received.append(pt), done.set()))

        _send_messages(
            _E2E_PORT,
            [
                {
                    "type": "click",
                    "x": 10.0,
                    "y": 20.0,
                    "z": 0.5,
                    "entity_path": "/world/robot",
                    "timestamp_ms": 42000,
                }
            ],
        )

        done.wait(timeout=3.0)
        unsub()

        assert len(received) == 1
        pt = received[0]
        assert pt.x == pytest.approx(10.0)
        assert pt.y == pytest.approx(20.0)
        assert pt.z == pytest.approx(0.5)
        assert pt.frame_id == "/world/robot"
        assert pt.ts == pytest.approx(42.0)

    def test_full_viewer_session_sequence(self, server: RerunWebSocketServer) -> None:
        """Realistic session: heartbeats, click, twist, stop — only the click produces a point."""
        received: list[Any] = []
        done = threading.Event()
        unsub = server.clicked_point.subscribe(lambda pt: (received.append(pt), done.set()))

        _send_messages(
            _E2E_PORT,
            [
                {"type": "heartbeat", "timestamp_ms": 1000},
                {"type": "heartbeat", "timestamp_ms": 2000},
                {
                    "type": "click",
                    "x": 3.14,
                    "y": 2.71,
                    "z": 1.41,
                    "entity_path": "/world",
                    "timestamp_ms": 3000,
                },
                {
                    "type": "twist",
                    "linear_x": 0.5,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                    "angular_x": 0.0,
                    "angular_y": 0.0,
                    "angular_z": 0.0,
                },
                {"type": "stop"},
                {"type": "heartbeat", "timestamp_ms": 4000},
            ],
            delay=0.2,
        )

        done.wait(timeout=3.0)
        unsub()

        assert len(received) == 1, f"Expected exactly 1 click, got {len(received)}"
        assert received[0].x == pytest.approx(3.14)
        assert received[0].y == pytest.approx(2.71)
        assert received[0].z == pytest.approx(1.41)

    def test_reconnect_after_disconnect(self, server: RerunWebSocketServer) -> None:
        """Server keeps accepting new connections after a client disconnects."""
        received: list[Any] = []
        all_done = threading.Event()

        def _on_pt(pt: Any) -> None:
            received.append(pt)
            if len(received) >= 2:
                all_done.set()

        unsub = server.clicked_point.subscribe(_on_pt)

        _send_messages(
            _E2E_PORT,
            [{"type": "click", "x": 1.0, "y": 0.0, "z": 0.0, "entity_path": "", "timestamp_ms": 0}],
        )
        _send_messages(
            _E2E_PORT,
            [{"type": "click", "x": 2.0, "y": 0.0, "z": 0.0, "entity_path": "", "timestamp_ms": 0}],
        )

        all_done.wait(timeout=5.0)
        unsub()

        xs = sorted(pt.x for pt in received)
        assert xs == [1.0, 2.0], f"Unexpected xs: {xs}"


class TestViewerBinaryConnectMode:
    """Smoke test: dimos-viewer binary starts in --connect mode."""

    @pytest.fixture()
    def viewer_process(self, server: RerunWebSocketServer) -> subprocess.Popen[bytes]:
        proc = subprocess.Popen(
            [
                "dimos-viewer",
                "--connect",
                f"--ws-url=ws://127.0.0.1:{_E2E_PORT}/ws",
            ],
            env={**os.environ, "DISPLAY": ""},
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        yield proc  # type: ignore[misc]
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()

    @pytest.mark.skip(
        reason="Incompatible with current winit: fails without DISPLAY (headless CI exits before WS connect) and hangs with DISPLAY (GUI event loop blocks before printing URL).",
    )
    def test_viewer_ws_client_connects(self, viewer_process: subprocess.Popen[bytes]) -> None:
        """dimos-viewer --connect starts and its WS client connects to our server."""
        deadline = time.monotonic() + DEFAULT_THREAD_JOIN_TIMEOUT
        while time.monotonic() < deadline:
            if viewer_process.poll() is not None:
                break
            time.sleep(DEFAULT_THREAD_JOIN_TIMEOUT / 20)

        stdout = (
            viewer_process.stdout.read().decode(errors="replace") if viewer_process.stdout else ""
        )
        stderr = (
            viewer_process.stderr.read().decode(errors="replace") if viewer_process.stderr else ""
        )

        combined = stdout + stderr
        assert f"ws://127.0.0.1:{_E2E_PORT}" in combined, (
            f"Viewer did not attempt WS connection.\nstdout:\n{stdout}\nstderr:\n{stderr}"
        )
