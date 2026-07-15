# Copyright 2026 Dimensional Inc.
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

"""Tests for the LCM <-> WebSocket bridge.

The integration tests run the standalone module against a ``memq://``
LCM provider (in-process queue, no multicast needed on CI) and a real
``websockets`` client, so both directions cross a real socket.
"""

from __future__ import annotations

import asyncio
import socket
import threading
import time
from typing import Any

import pytest
import websockets.asyncio.client as ws_client

from dimos.web.lcm_bridge.bridge import (
    LcmWebSocketBridge,
    decode_packet,
    encode_packet,
)
from dimos.web.lcm_bridge.module import LcmWebSocketBridgeModule

_TEST_PORT = 13041
_SERVER_STARTUP_TIMEOUT_S = 10.0


# ---- wire format ----------------------------------------------------------


def test_packet_roundtrip() -> None:
    packet = encode_packet("/odom#geometry_msgs.PoseStamped", b"\x01\x02\x03", seq=7)
    assert decode_packet(packet) == ("/odom#geometry_msgs.PoseStamped", b"\x01\x02\x03")


def test_decode_rejects_garbage() -> None:
    assert decode_packet(b"") is None
    assert decode_packet(b"\x00" * 8) is None  # wrong magic
    # Right magic but no channel terminator.
    assert decode_packet(encode_packet("x", b"")[:-2]) is None


# ---- filtering ------------------------------------------------------------


def _bridge(**kwargs: Any) -> LcmWebSocketBridge:
    return LcmWebSocketBridge(lcm_url="memq://", **kwargs)


def test_no_filters_forwards_everything() -> None:
    bridge = _bridge()
    assert bridge._forward_allowed("/odom#geometry_msgs.PoseStamped")


def test_blocklist_matches_topic_and_channel() -> None:
    bridge = _bridge(topic_blocklist=["/global_map", "/camera_*"])
    assert not bridge._forward_allowed("/global_map#sensor_msgs.PointCloud2")
    assert not bridge._forward_allowed("/camera_image#sensor_msgs.Image")
    assert bridge._forward_allowed("/odom#geometry_msgs.PoseStamped")


def test_allowlist_restricts_and_blocklist_wins() -> None:
    bridge = _bridge(topic_allowlist=["/odom", "/tf"], topic_blocklist=["/tf"])
    assert bridge._forward_allowed("/odom#geometry_msgs.PoseStamped")
    assert not bridge._forward_allowed("/tf#tf2_msgs.TFMessage")
    assert not bridge._forward_allowed("/joint_state#sensor_msgs.JointState")


def test_rate_pattern_resolves_min_interval() -> None:
    bridge = _bridge(channel_rate_hz={"/global_map": 2.0})
    assert bridge._min_interval_s("/global_map#sensor_msgs.PointCloud2") == pytest.approx(0.5)
    assert bridge._min_interval_s("/odom#geometry_msgs.PoseStamped") == 0.0


# ---- end to end -----------------------------------------------------------


def _wait_for_port(port: int, timeout: float = _SERVER_STARTUP_TIMEOUT_S) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.2)
            if sock.connect_ex(("127.0.0.1", port)) == 0:
                return
        time.sleep(0.05)
    raise TimeoutError(f"server on port {port} did not come up")


@pytest.fixture()
def module() -> Any:
    mod = LcmWebSocketBridgeModule(
        port=_TEST_PORT,
        host="127.0.0.1",
        lcm_url="memq://",
        channel_rate_hz={"/capped": 2.0},
    )
    mod.start()
    _wait_for_port(_TEST_PORT)
    yield mod
    mod.stop()


class _WsClient:
    """Sync facade over a websockets client on a private event loop."""

    def __init__(self, url: str) -> None:
        self._url = url
        self._loop = asyncio.new_event_loop()
        self._ws: Any = None

    def __enter__(self) -> _WsClient:
        self._ws = self._loop.run_until_complete(ws_client.connect(self._url))
        return self

    def __exit__(self, *_: Any) -> None:
        if self._ws is not None:
            self._loop.run_until_complete(self._ws.close())
        self._loop.close()

    def send(self, data: bytes) -> None:
        self._loop.run_until_complete(self._ws.send(data))

    def recv(self, timeout: float = 3.0) -> bytes:
        return self._loop.run_until_complete(asyncio.wait_for(self._ws.recv(), timeout))

    def recv_channel(self, channel: str, timeout: float = 3.0) -> bytes:
        """Receive until a packet for ``channel`` arrives; return its payload."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no packet on {channel}")
            decoded = decode_packet(self.recv(timeout=remaining))
            if decoded is not None and decoded[0] == channel:
                return decoded[1]

    def drain(self, window_s: float) -> list[bytes]:
        """Collect every packet arriving within ``window_s``."""
        got: list[bytes] = []
        deadline = time.monotonic() + window_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return got
            try:
                got.append(self.recv(timeout=remaining))
            except TimeoutError:
                return got


@pytest.fixture()
def client(module: LcmWebSocketBridgeModule) -> Any:
    with _WsClient(f"ws://127.0.0.1:{_TEST_PORT}/lcm-ws") as ws:
        # The bridge registers the client on accept; give the server loop
        # a beat so bus messages published next actually fan out to it.
        deadline = time.monotonic() + 2.0
        while module.bridge.client_count == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert module.bridge.client_count == 1
        yield ws


def test_bus_to_browser(module: LcmWebSocketBridgeModule, client: _WsClient) -> None:
    """A bus publish arrives at the WebSocket client as an LC02 packet."""
    channel = "/test_topic#test_msgs.Blob"
    assert module.bridge.lcm is not None
    module.bridge.lcm.publish(channel, b"payload-bytes")
    assert client.recv_channel(channel) == b"payload-bytes"


def test_browser_to_bus_and_multitab_echo(
    module: LcmWebSocketBridgeModule, client: _WsClient
) -> None:
    """A client publish lands on the bus, and the bridge's subscribe-all
    fans it back out to connected clients (multi-tab sync)."""
    received: list[tuple[str, bytes]] = []
    got_one = threading.Event()
    assert module.bridge.lcm is not None
    module.bridge.lcm.subscribe(
        "/from_browser.*", lambda ch, data: (received.append((ch, data)), got_one.set())
    )

    channel = "/from_browser#test_msgs.Blob"
    client.send(encode_packet(channel, b"hello-from-tab"))

    assert got_one.wait(timeout=3.0), "browser publish never reached the bus"
    assert received[0] == (channel, b"hello-from-tab")
    assert client.recv_channel(channel) == b"hello-from-tab"


def test_rate_cap_drops_bursts(module: LcmWebSocketBridgeModule, client: _WsClient) -> None:
    """A channel capped at 2 Hz forwards ~1 packet from a rapid burst."""
    channel = "/capped#test_msgs.Blob"
    assert module.bridge.lcm is not None
    for i in range(10):
        module.bridge.lcm.publish(channel, bytes([i]))
    packets = client.drain(window_s=0.4)
    assert 1 <= len(packets) <= 2, f"expected the burst capped to 1-2 packets, got {len(packets)}"


def test_status_endpoint(module: LcmWebSocketBridgeModule) -> None:
    import urllib.request

    with urllib.request.urlopen(f"http://127.0.0.1:{_TEST_PORT}/", timeout=3) as response:
        assert response.status == 200
        body = response.read().decode()
    assert "clients" in body and "forwarded" in body


def test_serves_browser_client(module: LcmWebSocketBridgeModule) -> None:
    import urllib.request

    with urllib.request.urlopen(
        f"http://127.0.0.1:{_TEST_PORT}/lcm_client.js", timeout=3
    ) as response:
        assert response.status == 200
        body = response.read().decode()
    assert "dimosLcm" in body
