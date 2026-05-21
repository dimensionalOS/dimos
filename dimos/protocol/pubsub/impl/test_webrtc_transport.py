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

"""Unit tests for WebRTC transport typed mode and BrokerProvider.

These tests use mocks — no network access or CF credentials required.
"""

from __future__ import annotations

from collections.abc import Callable

import pytest

from dimos.protocol.pubsub.impl.webrtcpubsub import DataChannelProvider

# ─── Mock provider ───────────────────────────────────────────────────


class MockProvider(DataChannelProvider):
    """In-memory DataChannelProvider for testing."""

    def __init__(self) -> None:
        self._started = False
        self._subscribers: dict[str, list[Callable[[bytes, str], None]]] = {}

    @property
    def is_connected(self) -> bool:
        return self._started

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    def publish(self, topic: str, data: bytes) -> None:
        # Deliver to local subscribers (loopback for testing)
        for cb in list(self._subscribers.get(topic, [])):
            cb(data, topic)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        self._subscribers.setdefault(topic, []).append(callback)

        def _unsub() -> None:
            try:
                self._subscribers[topic].remove(callback)
            except (ValueError, KeyError):
                pass

        return _unsub


# ─── Test WebRTCTransport raw bytes mode ─────────────────────────────


def test_webrtc_transport_raw_bytes() -> None:
    """Raw bytes mode: messages pass through without encode/decode."""
    from dimos.core.transport import WebRTCTransport

    provider = MockProvider()
    transport: WebRTCTransport = WebRTCTransport("test_topic", provider=provider)
    transport.start()

    received: list[bytes] = []
    transport.subscribe(lambda msg: received.append(msg))

    # Publish raw bytes
    transport.broadcast(None, b"hello")
    assert received == [b"hello"]

    transport.stop()


# ─── Test WebRTCTransport typed LCM mode ─────────────────────────────


class FakeLCMMsg:
    """Minimal LCM-like message for testing fingerprint filtering."""

    _FINGERPRINT = b"\x01\x02\x03\x04\x05\x06\x07\x08"

    def __init__(self, value: float = 0.0):
        self.value = value

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        return cls._FINGERPRINT

    def lcm_encode(self) -> bytes:
        import struct

        return self._FINGERPRINT + struct.pack("<d", self.value)

    @classmethod
    def lcm_decode(cls, data: bytes) -> FakeLCMMsg:
        import struct

        # Skip fingerprint prefix
        value = struct.unpack("<d", data[8:])[0]
        return cls(value)


class OtherLCMMsg:
    """Different message type with different fingerprint."""

    _FINGERPRINT = b"\xaa\xbb\xcc\xdd\xee\xff\x00\x11"

    def __init__(self, text: str = ""):
        self.text = text

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        return cls._FINGERPRINT

    def lcm_encode(self) -> bytes:
        return self._FINGERPRINT + self.text.encode()

    @classmethod
    def lcm_decode(cls, data: bytes) -> OtherLCMMsg:
        return cls(data[8:].decode())


def test_webrtc_transport_typed_encode_decode() -> None:
    """Typed mode: broadcast encodes, subscribe decodes."""
    from dimos.core.transport import WebRTCTransport

    provider = MockProvider()
    transport: WebRTCTransport = WebRTCTransport(
        "cmd_unreliable", msg_type=FakeLCMMsg, provider=provider
    )
    transport.start()

    received: list[FakeLCMMsg] = []
    transport.subscribe(lambda msg: received.append(msg))

    # Publish typed message
    transport.broadcast(None, FakeLCMMsg(3.14))

    assert len(received) == 1
    assert abs(received[0].value - 3.14) < 1e-9

    transport.stop()


def test_webrtc_transport_fingerprint_filter() -> None:
    """Typed mode: only messages matching the type's fingerprint are delivered."""
    from dimos.core.transport import WebRTCTransport

    provider = MockProvider()

    # Transport expects FakeLCMMsg
    transport: WebRTCTransport = WebRTCTransport(
        "cmd_unreliable", msg_type=FakeLCMMsg, provider=provider
    )
    transport.start()

    received: list[FakeLCMMsg] = []
    transport.subscribe(lambda msg: received.append(msg))

    # Publish a different message type on the same topic (multiplexed channel)
    other_msg = OtherLCMMsg("hello")
    provider.publish("cmd_unreliable", other_msg.lcm_encode())

    # Should NOT be delivered (wrong fingerprint)
    assert len(received) == 0

    # Now publish correct type
    correct_msg = FakeLCMMsg(2.71)
    provider.publish("cmd_unreliable", correct_msg.lcm_encode())

    # Should be delivered
    assert len(received) == 1
    assert abs(received[0].value - 2.71) < 1e-9

    transport.stop()


def test_webrtc_transport_multiple_types_multiplexed() -> None:
    """Multiple transports on same provider/topic filter independently."""
    from dimos.core.transport import WebRTCTransport

    provider = MockProvider()

    t1: WebRTCTransport = WebRTCTransport("cmd_unreliable", msg_type=FakeLCMMsg, provider=provider)
    t2: WebRTCTransport = WebRTCTransport("cmd_unreliable", msg_type=OtherLCMMsg, provider=provider)
    t1.start()
    t2.start()

    r1: list[FakeLCMMsg] = []
    r2: list[OtherLCMMsg] = []
    t1.subscribe(lambda msg: r1.append(msg))
    t2.subscribe(lambda msg: r2.append(msg))

    # Send FakeLCMMsg
    provider.publish("cmd_unreliable", FakeLCMMsg(1.0).lcm_encode())
    assert len(r1) == 1
    assert len(r2) == 0

    # Send OtherLCMMsg
    provider.publish("cmd_unreliable", OtherLCMMsg("world").lcm_encode())
    assert len(r1) == 1
    assert len(r2) == 1
    assert r2[0].text == "world"

    t1.stop()
    t2.stop()


# ─── Test BrokerProvider (mocked network) ────────────────────────────


def test_broker_provider_import() -> None:
    """BrokerProvider should be importable."""
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BrokerProvider  # noqa: F401


def test_broker_provider_requires_credentials() -> None:
    """BrokerProvider raises if required env vars / args are missing."""
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BROKER_AVAILABLE

    if not BROKER_AVAILABLE:
        pytest.skip("aiortc not installed")

    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BrokerProvider

    with pytest.raises(RuntimeError, match="TELEOP_BROKER_URL"):
        BrokerProvider(broker_url="", api_key="key", robot_id="r1")

    with pytest.raises(RuntimeError, match="TELEOP_API_KEY"):
        BrokerProvider(broker_url="http://localhost", api_key="", robot_id="r1")

    with pytest.raises(RuntimeError, match="TELEOP_ROBOT_ID"):
        BrokerProvider(broker_url="http://localhost", api_key="key", robot_id="")


def test_broker_provider_subscribe_buffering() -> None:
    """Subscribers registered before operator joins get messages once DC opens."""
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BROKER_AVAILABLE

    if not BROKER_AVAILABLE:
        pytest.skip("aiortc not installed")

    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BrokerProvider

    provider = BrokerProvider(
        broker_url="http://localhost:8000",
        api_key="dtk_live_test",
        robot_id="test-robot",
        robot_name="Test",
    )

    # Subscribe before start (should not raise)
    received: list[bytes] = []
    provider._started = True  # Fake started state for subscription test
    unsub = provider.subscribe("cmd_unreliable", lambda data, topic: received.append(data))

    # Verify callback is registered
    assert "cmd_unreliable" in provider._callbacks
    assert len(provider._callbacks["cmd_unreliable"]) == 1

    unsub()
    assert len(provider._callbacks["cmd_unreliable"]) == 0
