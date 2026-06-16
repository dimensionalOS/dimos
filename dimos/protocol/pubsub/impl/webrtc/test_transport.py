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

"""Unit tests for WebRTCTransport — no network or credentials required."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import pickle
import struct

import pytest

from dimos.core.transport import WebRTCTransport
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.protocol.pubsub.impl.webrtc.providers.spec import ProviderConfig

# ─── Mock provider ───────────────────────────────────────────────────


class MockProvider:
    """In-memory loopback Provider."""

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


@dataclass(frozen=True)
class MockConfig(ProviderConfig):
    name: str = "default"

    def _create(self) -> MockProvider:
        return MockProvider()


class MockTransport(WebRTCTransport):
    _config_cls = MockConfig


# ─── Fake LCM messages ───────────────────────────────────────────────


class FakeLCMMsg:
    msg_name = "test.FakeLCMMsg"
    _FINGERPRINT = b"\x01\x02\x03\x04\x05\x06\x07\x08"

    def __init__(self, value: float = 0.0):
        self.value = value

    @classmethod
    def _get_packed_fingerprint(cls) -> bytes:
        return cls._FINGERPRINT

    def lcm_encode(self) -> bytes:
        return self._FINGERPRINT + struct.pack("<d", self.value)

    @classmethod
    def lcm_decode(cls, data: bytes) -> FakeLCMMsg:
        if data[:8] != cls._FINGERPRINT:  # like real LCM generated code
            raise ValueError("Decode error")
        return cls(struct.unpack("<d", data[8:])[0])


class OtherLCMMsg:
    msg_name = "test.OtherLCMMsg"
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
        if data[:8] != cls._FINGERPRINT:  # like real LCM generated code
            raise ValueError("Decode error")
        return cls(data[8:].decode())


# ─── Transport modes ─────────────────────────────────────────────────


def test_raw_bytes_mode() -> None:
    transport = MockTransport("test_topic", name="raw")
    received: list[bytes] = []
    transport.subscribe(lambda msg: received.append(msg))
    transport.broadcast(None, b"hello")
    assert received == [b"hello"]


def test_typed_encode_decode() -> None:
    transport = MockTransport("cmd_unreliable", FakeLCMMsg, name="typed")
    received: list[FakeLCMMsg] = []
    transport.subscribe(lambda msg: received.append(msg))
    transport.broadcast(None, FakeLCMMsg(3.14))
    assert len(received) == 1
    assert abs(received[0].value - 3.14) < 1e-9


def test_multiple_types_multiplexed() -> None:
    """Typed transports sharing one channel each receive only their own type."""
    t1 = MockTransport("cmd_unreliable", FakeLCMMsg, name="mux")
    t2 = MockTransport("cmd_unreliable", OtherLCMMsg, name="mux")
    r1: list[FakeLCMMsg] = []
    r2: list[OtherLCMMsg] = []
    t1.subscribe(lambda msg: r1.append(msg))
    t2.subscribe(lambda msg: r2.append(msg))

    t1.broadcast(None, FakeLCMMsg(1.0))
    t2.broadcast(None, OtherLCMMsg("world"))
    assert [m.value for m in r1] == [1.0]
    assert [m.text for m in r2] == ["world"]


def test_wire_fingerprint_matches_encoding() -> None:
    """Demux must follow the wire format, not _get_packed_fingerprint().

    TwistStamped inherits Twist's fingerprint but encodes as LCM TwistStamped —
    any filter keyed on the class fingerprint would drop every real message.
    The try-decode demux delegates the check to lcm_decode, which gets it right.
    """
    transport = MockTransport("cmd_unreliable", TwistStamped, name="wire")
    received: list[TwistStamped] = []
    transport.subscribe(lambda msg: received.append(msg))

    wire = TwistStamped(linear=[0.5, 0, 0], angular=[0, 0, 0.1], frame_id="keyboard").lcm_encode()
    assert wire[:8] != TwistStamped._get_packed_fingerprint()
    transport._config.provider().publish("cmd_unreliable", wire)
    assert len(received) == 1
    assert abs(received[0].linear.x - 0.5) < 1e-9
    assert received[0].frame_id == "keyboard"


# ─── Pickling + provider sharing ─────────────────────────────────────


def test_pickle_roundtrip_preserves_everything() -> None:
    """Transports are pickled into module worker processes; topic, type,
    and provider config must all survive."""
    t1 = MockTransport("cmd_unreliable", FakeLCMMsg, name="pickled")
    t2 = pickle.loads(pickle.dumps(t1))

    assert type(t2) is MockTransport
    assert t2.topic == t1.topic
    assert t2._msg_type is FakeLCMMsg
    assert t2._config == t1._config

    # Same config → same per-process provider, so the two halves interoperate.
    received: list[FakeLCMMsg] = []
    t2.subscribe(lambda msg: received.append(msg))
    t1.broadcast(None, FakeLCMMsg(42.0))
    assert len(received) == 1
    assert abs(received[0].value - 42.0) < 1e-9


def test_provider_singleton_per_config() -> None:
    assert MockConfig(name="a").provider() is MockConfig(name="a").provider()
    assert MockConfig(name="a").provider() is not MockConfig(name="b").provider()


# ─── Broker credential validation ────────────────────────────────────


def test_broker_provider_requires_credentials(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig
    from dimos.protocol.pubsub.impl.webrtc.providers.spec import WEBRTC_AVAILABLE

    if not WEBRTC_AVAILABLE:
        pytest.skip("aiortc not installed")
    monkeypatch.delenv("TELEOP_API_KEY", raising=False)
    monkeypatch.delenv("TELEOP_ROBOT_ID", raising=False)

    with pytest.raises(RuntimeError, match="TELEOP_API_KEY"):
        BrokerConfig(robot_id="r1")._create()
    # robot_id is optional — the broker derives it from the API key.
    assert BrokerConfig(api_key="key")._create() is not None


# ─── subscribe_all dedup ─────────────────────────────────────────────


def test_subscribe_all_fires_once_per_message() -> None:
    """N subscriptions on one topic must not duplicate subscribe_all delivery."""
    from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub

    ps = WebRTCPubSub(provider=MockProvider())
    ps.subscribe("t", lambda data, t: None)
    ps.subscribe("t", lambda data, t: None)

    seen: list[tuple[bytes, str]] = []
    ps.subscribe_all(lambda data, t: seen.append((data, t)))

    ps.publish("t", b"x")
    assert seen == [(b"x", "t")]

    # And still exactly once per message after another topic joins.
    ps.subscribe("u", lambda data, t: None)
    ps.publish("u", b"y")
    assert seen == [(b"x", "t"), (b"y", "u")]
