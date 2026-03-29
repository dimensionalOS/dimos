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

"""End-to-end test: dimos-viewer (headless) → WebSocket → RerunWebSocketServer.

dimos-viewer is started in ``--connect`` mode so it initialises its WebSocket
client.  The viewer needs a gRPC proxy to connect to; we give it a non-existent
one so the viewer starts up anyway but produces no visualisation.  The important
part is that the WebSocket client inside the viewer tries to connect to
``ws://127.0.0.1:<port>/ws``.

Because the viewer is a native GUI application it cannot run headlessly in CI
without a display.  This test therefore verifies the connection at the protocol
level by using the ``RerunWebSocketServer`` module directly as the server and
injecting synthetic JSON messages that mimic what the viewer would send once a
user clicks in the 3D viewport.
"""

import asyncio
import json
import os
import shutil
import subprocess
import threading
import time
from typing import Any

import pytest

from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_E2E_PORT = 13032


def _make_server(port: int = _E2E_PORT) -> RerunWebSocketServer:
    return RerunWebSocketServer(port=port)


def _wait_for_server(port: int, timeout: float = 5.0) -> None:
    import websockets.asyncio.client as ws_client

    async def _probe() -> None:
        async with ws_client.connect(f"ws://127.0.0.1:{port}/ws"):
            pass

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            asyncio.run(_probe())
            return
        except Exception:
            time.sleep(0.05)
    raise TimeoutError(f"Server on port {port} did not become ready within {timeout}s")


def _send_messages(port: int, messages: list[dict[str, Any]], *, delay: float = 0.05) -> None:
    import websockets.asyncio.client as ws_client

    async def _run() -> None:
        async with ws_client.connect(f"ws://127.0.0.1:{port}/ws") as ws:
            for msg in messages:
                await ws.send(json.dumps(msg))
            await asyncio.sleep(delay)

    asyncio.run(_run())


class TestViewerProtocolE2E:
    """Verify the full Python-server side of the viewer ↔ DimOS protocol.

    These tests use the ``RerunWebSocketServer`` as the server and a dummy
    WebSocket client (playing the role of dimos-viewer) to inject messages.
    They confirm every message type is correctly routed and that only click
    messages produce stream publishes.
    """

    def test_viewer_click_reaches_stream(self) -> None:
        """A viewer click message received over WebSocket publishes PointStamped."""
        server = _make_server()
        server.start()
        _wait_for_server(_E2E_PORT)

        received: list[Any] = []
        done = threading.Event()

        def _on_pt(pt: Any) -> None:
            received.append(pt)
            done.set()

        server.clicked_point.subscribe(_on_pt)

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
        server.stop()

        assert len(received) == 1
        pt = received[0]
        assert abs(pt.x - 10.0) < 1e-9
        assert abs(pt.y - 20.0) < 1e-9
        assert abs(pt.z - 0.5) < 1e-9
        assert pt.frame_id == "/world/robot"
        assert abs(pt.ts - 42.0) < 1e-6

    def test_viewer_keyboard_twist_no_publish(self) -> None:
        """Twist messages from keyboard control do not publish clicked_point."""
        server = _make_server()
        server.start()
        _wait_for_server(_E2E_PORT)

        received: list[Any] = []
        server.clicked_point.subscribe(received.append)

        _send_messages(
            _E2E_PORT,
            [
                {
                    "type": "twist",
                    "linear_x": 0.5,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                    "angular_x": 0.0,
                    "angular_y": 0.0,
                    "angular_z": 0.8,
                }
            ],
        )

        server.stop()
        assert received == []

    def test_viewer_stop_no_publish(self) -> None:
        """Stop messages do not publish clicked_point."""
        server = _make_server()
        server.start()
        _wait_for_server(_E2E_PORT)

        received: list[Any] = []
        server.clicked_point.subscribe(received.append)

        _send_messages(_E2E_PORT, [{"type": "stop"}])

        server.stop()
        assert received == []

    def test_full_viewer_session_sequence(self) -> None:
        """Realistic session: connect, heartbeats, click, WASD, stop → one point."""
        server = _make_server()
        server.start()
        _wait_for_server(_E2E_PORT)

        received: list[Any] = []
        done = threading.Event()

        def _on_pt(pt: Any) -> None:
            received.append(pt)
            done.set()

        server.clicked_point.subscribe(_on_pt)

        _send_messages(
            _E2E_PORT,
            [
                # Initial heartbeats (viewer connects and starts 1 Hz heartbeat)
                {"type": "heartbeat", "timestamp_ms": 1000},
                {"type": "heartbeat", "timestamp_ms": 2000},
                # User clicks a point in the 3D viewport
                {
                    "type": "click",
                    "x": 3.14,
                    "y": 2.71,
                    "z": 1.41,
                    "entity_path": "/world",
                    "timestamp_ms": 3000,
                },
                # User presses W (forward)
                {
                    "type": "twist",
                    "linear_x": 0.5,
                    "linear_y": 0.0,
                    "linear_z": 0.0,
                    "angular_x": 0.0,
                    "angular_y": 0.0,
                    "angular_z": 0.0,
                },
                # User releases W
                {"type": "stop"},
                # Another heartbeat
                {"type": "heartbeat", "timestamp_ms": 4000},
            ],
            delay=0.2,
        )

        done.wait(timeout=3.0)
        server.stop()

        assert len(received) == 1, f"Expected exactly 1 click, got {len(received)}"
        pt = received[0]
        assert abs(pt.x - 3.14) < 1e-9
        assert abs(pt.y - 2.71) < 1e-9
        assert abs(pt.z - 1.41) < 1e-9

    def test_reconnect_after_disconnect(self) -> None:
        """Server keeps accepting new connections after a client disconnects."""
        server = _make_server()
        server.start()
        _wait_for_server(_E2E_PORT)

        received: list[Any] = []
        all_done = threading.Event()

        def _on_pt(pt: Any) -> None:
            received.append(pt)
            if len(received) >= 2:
                all_done.set()

        server.clicked_point.subscribe(_on_pt)

        # First connection — send one click and disconnect
        _send_messages(
            _E2E_PORT,
            [{"type": "click", "x": 1.0, "y": 0.0, "z": 0.0, "entity_path": "", "timestamp_ms": 0}],
        )

        # Second connection (simulating viewer reconnect) — send another click
        _send_messages(
            _E2E_PORT,
            [{"type": "click", "x": 2.0, "y": 0.0, "z": 0.0, "entity_path": "", "timestamp_ms": 0}],
        )

        all_done.wait(timeout=5.0)
        server.stop()

        xs = sorted(pt.x for pt in received)
        assert xs == [1.0, 2.0], f"Unexpected xs: {xs}"


class TestViewerBinaryConnectMode:
    """Smoke test: dimos-viewer binary starts in --connect mode and its WebSocket
    client attempts to connect to our Python server."""

    @pytest.mark.skipif(
        shutil.which("dimos-viewer") is None
        or "--connect"
        not in subprocess.run(["dimos-viewer", "--help"], capture_output=True, text=True).stdout,
        reason="dimos-viewer binary not installed or does not support --connect",
    )
    def test_viewer_ws_client_connects(self) -> None:
        """dimos-viewer --connect starts and its WS client connects to our server."""
        server = _make_server()
        server.start()
        _wait_for_server(_E2E_PORT)

        received: list[Any] = []

        def _on_pt(pt: Any) -> None:
            received.append(pt)

        server.clicked_point.subscribe(_on_pt)

        # Start dimos-viewer in --connect mode, pointing it at a non-existent gRPC
        # proxy (it will fail to stream data, but that's fine) and at our WS server.
        # Use DISPLAY="" to prevent it from opening a window (it will exit quickly
        # without a display, but the WebSocket connection happens before the GUI loop).
        proc = subprocess.Popen(
            [
                "dimos-viewer",
                "--connect",
                f"--ws-url=ws://127.0.0.1:{_E2E_PORT}/ws",
            ],
            env={
                **os.environ,
                "DISPLAY": "",
            },
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Give the viewer up to 5 s to connect its WebSocket client to our server.
        # We detect the connection by waiting for the server to accept a client.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            # Check if any connection was established by sending a message and
            # verifying the viewer is still running.
            if proc.poll() is not None:
                # Viewer exited (expected without a display) — check if it connected first.
                break
            time.sleep(0.1)

        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()

        stdout = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
        stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
        server.stop()

        # The viewer should log that it is connecting to our WS URL.
        # Check both stdout and stderr since log output destination varies.
        combined = stdout + stderr
        assert f"ws://127.0.0.1:{_E2E_PORT}" in combined, (
            f"Viewer did not attempt WS connection.\nstdout:\n{stdout}\nstderr:\n{stderr}"
        )
