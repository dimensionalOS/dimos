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
import pickle

from dimos_generated.geometry_msgs.msg import Point, TwistStamped, Vector3
from pydantic import ValidationError
import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _materialize_transports
from dimos.core.transport import WebRTCTransport
from dimos.protocol.pubsub.impl.webrtc.providers import spec
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig
from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    AsyncProviderBase,
    ProviderConfig,
)
from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub
from dimos.web.relay_bridge.protocol import FrameHeader, decode_data_frame, encode_data_frame

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


class MockConfig(ProviderConfig):
    name: str = "default"
    count: int = 0

    def _create(self) -> MockProvider:
        return MockProvider()


class MockTransport(WebRTCTransport):
    _config_cls = MockConfig


# ─── Transport modes ─────────────────────────────────────────────────


def test_raw_bytes_mode() -> None:
    transport = MockTransport("test_topic", name="raw")
    received: list[bytes] = []
    transport.subscribe(lambda msg: received.append(msg))
    transport.broadcast(None, b"hello")
    assert received == [b"hello"]


def test_typed_encode_decode() -> None:
    transport = MockTransport("cmd_unreliable", Point, name="typed")
    received: list[Point] = []
    transport.subscribe(lambda msg: received.append(msg))
    transport.broadcast(None, Point(x=3.14))
    assert len(received) == 1
    assert abs(received[0].x - 3.14) < 1e-9


def test_multiple_types_multiplexed() -> None:
    """Typed transports sharing one channel each receive only their own type."""
    t1 = MockTransport("cmd_unreliable", Point, name="mux")
    t2 = MockTransport("cmd_unreliable", Vector3, name="mux")
    r1: list[Point] = []
    r2: list[Vector3] = []
    t1.subscribe(lambda msg: r1.append(msg))
    t2.subscribe(lambda msg: r2.append(msg))

    t1.broadcast(None, Point(x=1.0))
    t2.broadcast(None, Vector3(x=2.0))
    assert [m.x for m in r1] == [1.0]
    assert [m.x for m in r2] == [2.0]


def test_generated_frames_declare_type_and_sequence() -> None:
    publisher = MockTransport("data", Point, name="framed")
    raw = MockTransport("data", name="framed")
    received: list[bytes] = []
    raw.subscribe(received.append)
    publisher.broadcast(None, Point(x=1))
    publisher.broadcast(None, Point(x=2))
    frames = [decode_data_frame(data) for data in received]
    assert [frame.header.seq for frame in frames] == [0, 1]
    assert all(frame.header.ch == "data" for frame in frames)
    assert all(frame.header.meta == {"type": Point.msg_name, "encoding": "cdr"} for frame in frames)
    assert [Point.decode(frame.payload).x for frame in frames] == [1, 2]


@pytest.mark.parametrize("bad", ["type", "topic", "encoding", "truncated", "trailing", "raw"])
def test_invalid_frame_does_not_poison_subscription(bad: str) -> None:
    transport = MockTransport("cmd_unreliable", Point, name=f"invalid-{bad}")
    received: list[Point] = []
    transport.subscribe(received.append)
    header = FrameHeader(
        ch="cmd_unreliable",
        seq=1,
        ts=0,
        delivery="latest",
        meta={"type": Point.msg_name, "encoding": "cdr"},
    )
    if bad == "topic":
        header.ch = "other"
    elif bad in ("type", "encoding"):
        assert header.meta is not None
        header.meta[bad] = "unknown"
    wire = encode_data_frame(header, Point(x=1).encode())
    if bad == "truncated":
        wire = wire[:-1]
    elif bad == "trailing":
        wire += b"extra"
    elif bad == "raw":
        wire = Point(x=1).encode()
    transport._config.provider().publish("cmd_unreliable", wire)
    assert received == []
    transport.broadcast(None, Point(x=2))
    assert received == [Point(x=2)]


# ─── Pickling + provider sharing ─────────────────────────────────────


def test_pickle_roundtrip_preserves_everything() -> None:
    """Transports are pickled into module worker processes; topic, type,
    and provider config must all survive."""
    t1 = MockTransport("cmd_unreliable", Point, name="pickled")
    t2 = pickle.loads(pickle.dumps(t1))

    assert type(t2) is MockTransport
    assert t2.topic == t1.topic
    assert t2._msg_type is Point
    assert t2._config == t1._config

    # Same config → same per-process provider, so the two halves interoperate.
    received: list[Point] = []
    t2.subscribe(lambda msg: received.append(msg))
    t1.broadcast(None, Point(x=42.0))
    assert len(received) == 1
    assert abs(received[0].x - 42.0) < 1e-9


def test_provider_singleton_per_config() -> None:
    assert MockConfig(name="a").provider() is MockConfig(name="a").provider()
    assert MockConfig(name="a").provider() is not MockConfig(name="b").provider()


def test_shutdown_all_providers_stops_and_clears() -> None:
    """Graceful teardown: stop every live provider and empty the registry so the
    worker can exit (see providers/spec.shutdown_all_providers)."""
    p1 = MockConfig(name="sd1").provider()
    p2 = MockConfig(name="sd2").provider()
    p1.start()
    p2.start()

    spec.shutdown_all_providers()

    assert not p1.is_connected and not p2.is_connected
    # Registry emptied, so a later provider() call builds a fresh instance.
    assert MockConfig(name="sd1").provider() is not p1
    # Idempotent — a second call with nothing registered is a no-op.
    spec.shutdown_all_providers()


def test_shutdown_all_providers_continues_past_a_raising_stop() -> None:
    """One provider raising in stop() must not strand the others still running."""
    good = MockConfig(name="ok").provider()
    good.start()
    bad = MockConfig(name="bad").provider()
    bad.start()
    bad.stop = lambda: (_ for _ in ()).throw(RuntimeError("boom"))  # type: ignore[method-assign]

    spec.shutdown_all_providers()  # must not raise

    assert not good.is_connected


# ─── ProviderConfig (pydantic frozen) ────────────────────────────────


def test_provider_config_is_frozen_and_hashable() -> None:
    """ProviderConfig must be hashable and frozen — the `_providers` singleton keys on it."""
    c = MockConfig(name="x")
    assert hash(c) == hash(MockConfig(name="x"))
    assert hash(c) != hash(MockConfig(name="y"))
    with pytest.raises(ValidationError):
        c.name = "mutated"  # type: ignore[misc]
    # Unknown fields are forbidden (extra="forbid").
    with pytest.raises(ValidationError, match="bogus"):
        MockConfig(bogus=1)  # type: ignore[call-arg]


# ─── Blueprint config / transport-override integration ──────────────


def test_blueprint_config_exposes_transport_fields() -> None:
    """Each unique `_config_cls` becomes a `transports.<name>` config section."""
    bp = Blueprint(blueprints=()).transports({("topic", Point): MockTransport.spec("topic")})
    parser = BlueprintConfigParser(bp)
    parsed = parser.parse(environ={}, overrides={"transports": {"mock": {"name": "override"}}})
    assert parsed.transport_overrides() == {"mock": {"name": "override"}}
    # Sub-field name → MockConfig; extra fields and unknown namespaces rejected.
    with pytest.raises(BlueprintConfigError, match="bogus"):
        parser.parse(environ={}, overrides={"transports": {"mock": {"bogus": 1}}})
    with pytest.raises(BlueprintConfigError, match="other"):
        parser.parse(environ={}, overrides={"transports": {"other": {}}})
    # Multiple transports sharing one `_config_cls` collapse to one section.
    bp_shared = Blueprint(blueprints=()).transports(
        {
            ("a", Point): MockTransport.spec("a"),
            ("b", Point): MockTransport.spec("b"),
        }
    )
    shared = BlueprintConfigParser(bp_shared).parse(
        environ={}, overrides={"transports": {"mock": {"name": "both"}}}
    )
    assert shared.transport_overrides() == {"mock": {"name": "both"}}


def test_transport_overrides_apply_and_survive_pickle() -> None:
    """Materialization builds each transport with its resolved config; pickle-safe."""
    bp = Blueprint(blueprints=()).transports({("topic", Point): MockTransport.spec("topic")})

    transport = _materialize_transports(bp, {"mock": {"name": "overridden"}})[("topic", Point)]

    assert transport._config.name == "overridden"
    assert pickle.loads(pickle.dumps(transport))._config.name == "overridden"

    # No override → config built from the spec's defaults.
    default = _materialize_transports(bp, {})[("topic", Point)]
    assert default._config.name == "default"


def test_materialize_uses_resolved_config() -> None:
    """The config reaching the transport is built straight from the overrides,
    not a default instance that is later mutated."""
    bp = Blueprint(blueprints=()).transports({("topic", Point): MockTransport.spec("topic")})

    transport = _materialize_transports(bp, {"mock": {"name": "resolved"}})[("topic", Point)]

    assert transport._config == MockConfig(name="resolved")


def test_transport_overrides_coerce_string_values() -> None:
    """CLI/env overrides arrive as raw strings; non-str fields must coerce, not pass through."""
    bp = Blueprint(blueprints=()).transports({("topic", Point): MockTransport.spec("topic")})
    transport = _materialize_transports(bp, {"mock": {"count": "5"}})[("topic", Point)]
    assert transport._config.count == 5


def test_raw_transport_pins_still_work() -> None:
    """Backwards compat: plain transport instances in .transports() (the pre-spec
    style used across existing blueprints) must survive config parsing and
    materialize unchanged — only spec-declared transports opt into the
    override flow."""
    from dimos.core.transport_factory import make_transport

    # Pinned on the active backend: a plain LCM/Zenoh pin that does not match it
    # is deliberately rebuilt by the backend switch, which is a different path.
    raw = make_transport("/raw_topic", Point)
    bp = Blueprint(blueprints=()).transports(
        {
            ("raw", Point): raw,
            ("speced", Point): MockTransport.spec("topic"),
        }
    )
    # Raw pins contribute no transports.* config sections; the spec still does.
    parsed = BlueprintConfigParser(bp).parse(
        environ={}, overrides={"transports": {"mock": {"name": "x"}}}
    )
    assert parsed.transport_overrides() == {"mock": {"name": "x"}}

    materialized = _materialize_transports(bp, {})
    assert materialized[("raw", Point)] is raw
    assert isinstance(materialized[("speced", Point)], MockTransport)


# ─── Broker credential validation ────────────────────────────────────


def test_broker_provider_requires_credentials() -> None:
    with pytest.raises(RuntimeError, match="api_key required"):
        BrokerConfig(robot_id="r1")._create()
    # robot_id is optional — the broker derives it from the API key.
    assert BrokerConfig(api_key="key")._create() is not None


def test_backend_coercion_leaves_webrtc_untouched() -> None:
    """The global lcm<->zenoh transport switch must never rebuild a webrtc
    transport (deliberate non-default choice, like JpegLcmTransport)."""
    from dimos.core.coordination.module_coordinator import _coerce_transport_to_backend
    from dimos.core.global_config import global_config
    from dimos.core.transport import CloudflareTransport, CloudflareVideoTransport

    dc = CloudflareTransport("cmd_unreliable", TwistStamped, api_key="k")
    video = CloudflareVideoTransport(api_key="k")
    original = global_config.transport
    try:
        for backend in ("lcm", "zenoh"):
            global_config.update(transport=backend)
            assert _coerce_transport_to_backend(dc) is dc
            assert _coerce_transport_to_backend(video) is video
    finally:
        global_config.update(transport=original)


def test_dc_name_no_collisions() -> None:
    """Sanitized OR truncated topics must stay distinct (<=64 char CF limit)."""
    from dimos.protocol.pubsub.impl.webrtc.providers.cloudflare import _dc_name

    assert _dc_name("cmd_unreliable") == "cmd_unreliable"  # safe short names untouched
    assert _dc_name("cmd/vel") != _dc_name("cmd_vel")  # sanitization can't collide
    long_x, long_y = "a" * 64 + "x", "a" * 64 + "y"  # differ only past the cap
    assert _dc_name(long_x) != _dc_name(long_y)
    assert len(_dc_name(long_x)) <= 64


# ─── Provider lifecycle error paths ──────────────────────────────────


def test_failed_connect_runs_disconnect_and_allows_retry() -> None:
    """A failed _connect() must release provider resources (_disconnect) and
    tear the loop thread down so the next start() retries cleanly."""

    class FlakyProvider(AsyncProviderBase):
        def __init__(self) -> None:
            super().__init__()
            self.fail = True
            self.disconnects = 0

        async def _connect(self) -> None:
            if self.fail:
                raise RuntimeError("connect boom")

        async def _disconnect(self) -> None:
            self.disconnects += 1

    p = FlakyProvider()
    with pytest.raises(RuntimeError, match="connect boom"):
        p.start()
    assert p.disconnects == 1, "cleanup must run on failed connect"
    assert p._thread is None and p._loop is None and not p.is_connected

    p.fail = False
    p.start()
    assert p.is_connected
    p.stop()
    assert p.disconnects == 2 and p._thread is None


def test_broker_failed_channel_open_retries_next_heartbeat() -> None:
    """The broker id must be recorded only after a successful open — otherwise
    a createDataChannel failure is never retried (id matches, _dcs empty)."""

    provider = BrokerConfig(api_key="key")._create()
    # _pc is None, so _open_channel's assert fires — a stand-in for any
    # createDataChannel failure inside the heartbeat.
    with pytest.raises(AssertionError):
        provider._reconcile_channels({"cmd_unreliable": 5})
    assert "cmd_unreliable" not in provider._dc_ids, "failed open must not record the id"

    opened: list[tuple[str, int]] = []
    provider._open_channel = lambda name, sctp_id: opened.append((name, sctp_id))  # type: ignore[method-assign]
    provider._reconcile_channels({"cmd_unreliable": 5})
    assert opened == [("cmd_unreliable", 5)], "next heartbeat must retry the open"
    assert provider._dc_ids["cmd_unreliable"] == 5


def test_broker_heartbeat_terminal_notifies_operator_lost() -> None:
    """A revoked session (401/404 streak) may leave the WebRTC room up, so the
    planner would keep driving. The terminal branch must inject operator_lost
    before abandoning the heartbeat loop."""
    import asyncio
    import sys

    provider = BrokerConfig(api_key="key")._create()
    provider._config = provider._config.model_copy(update={"heartbeat_hz": 1000.0})  # fast ticks

    got: list[bytes] = []
    provider._callbacks["state_reliable"] = [lambda data, topic: got.append(data)]

    async def _always_401() -> int:
        return 401

    provider._heartbeat_once = _always_401  # type: ignore[method-assign]
    if sys.version_info >= (3, 12):
        asyncio.run(asyncio.wait_for(provider._heartbeat_loop(), timeout=2.0))
    else:
        asyncio.run(provider._heartbeat_loop())

    assert got and b'"operator_lost"' in got[0], "terminal streak must inject operator_lost"


def test_broker_disconnect_clears_channel_ids() -> None:
    """Stale _dc_ids after stop() would make the reconnect heartbeat skip
    _open_channel when the broker hands out the same SCTP ids."""
    import asyncio

    provider = BrokerConfig(api_key="key")._create()
    provider._dc_ids = {"cmd_unreliable": 5, "state_reliable": 6}
    asyncio.run(provider._disconnect())
    assert provider._dc_ids == {}


def test_cloudflare_failed_subscribe_deregisters_callback() -> None:
    """If _ensure_sub fails, the callback must not stay registered (the caller
    has no unsub handle, and a later subscribe would revive it)."""
    from dimos.protocol.pubsub.impl.webrtc.providers.cloudflare import (
        CloudflareConfig,
        CloudflareProvider,
    )

    provider = CloudflareProvider(CloudflareConfig(app_id="a", app_secret="s"))
    provider._started = True  # skip auto-start; no network in this test

    def _boom(coro, timeout=30.0):  # type: ignore[no-untyped-def]
        coro.close()
        raise RuntimeError("ensure_sub boom")

    provider._run_sync = _boom  # type: ignore[method-assign]

    def _cb(data: bytes, topic: str) -> None:
        pass

    with pytest.raises(RuntimeError, match="ensure_sub boom"):
        provider.subscribe("some_topic", _cb)
    assert _cb not in provider._callbacks["some_topic"]


# ─── subscribe_all dedup ─────────────────────────────────────────────


def test_subscribe_all_fires_once_per_message() -> None:
    """N subscriptions on one topic must not duplicate subscribe_all delivery."""
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
