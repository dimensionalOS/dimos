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

"""Contract tests for `dimos spy` (TASK.md on branch agent/spy-architect).

These encode the acceptance criteria and FAIL until the task is implemented:
- TopicStats: deterministic windowed stats from injected timestamps.
- subscribe_all: delivers EVERY message on LCM and zenoh (non-conflating).
- subscribe_latest: conflates to the newest message per topic.
- SpySources: count every message without ever decoding a payload.
- TransportSpy: merges sources into (transport, topic)-keyed stats + totals.
- subscribe_decoded: spec'd hook, must stay NotImplementedError in v1.
"""

from collections.abc import Callable
from contextlib import contextmanager
import threading
import time
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import ZenohPubSubBase
from dimos.protocol.pubsub.spec import AllPubSub
from dimos.protocol.pubsub.spy import (
    LCMSpySource,
    SpyKey,
    TopicStats,
    TransportSpy,
    ZenohSpySource,
    split_type_suffix,
)
from dimos.protocol.service.zenohservice import ZenohSessionPool

VEC = Vector3(1.0, 2.0, 3.0)
VEC_BYTES = VEC.lcm_encode()


# TopicStats: pure, deterministic (injected timestamps, no sleeps)


def test_topic_stats_counts_and_totals():
    s = TopicStats(history_window=60.0)
    t0 = 1000.0
    for i in range(10):
        s.record(100, t0 + i * 0.1)
    assert s.total_msgs == 10
    assert s.total_bytes == 1000
    assert s.last_seen == pytest.approx(t0 + 0.9)


def test_topic_stats_windowed_rates():
    s = TopicStats(history_window=60.0)
    t0 = 1000.0
    for i in range(10):  # 1 Hz, 50 bytes each
        s.record(50, t0 + i)
    now = t0 + 9.0
    assert s.freq(5.0, now) == pytest.approx(1.0, rel=0.25)
    assert s.bytes_per_sec(5.0, now) == pytest.approx(50.0, rel=0.25)
    assert s.avg_size(5.0, now) == pytest.approx(50.0)


def test_topic_stats_empty():
    s = TopicStats()
    assert s.total_msgs == 0
    assert s.total_bytes == 0
    assert s.last_seen is None
    assert s.freq(5.0, now=123.0) == 0.0
    assert s.bytes_per_sec(5.0, now=123.0) == 0.0
    assert s.avg_size(5.0, now=123.0) == 0.0


def test_topic_stats_history_eviction_keeps_totals():
    s = TopicStats(history_window=60.0)
    s.record(100, 0.0)
    s.record(100, 1000.0)  # first record is now far outside the history window
    assert s.freq(60.0, now=1000.0) == pytest.approx(1 / 60.0)
    assert s.total_msgs == 2  # totals survive eviction
    assert s.total_bytes == 200


# split_type_suffix: render-time topic parsing


def test_split_type_suffix():
    assert split_type_suffix("/cmd_vel#geometry_msgs.Twist") == ("/cmd_vel", "geometry_msgs.Twist")
    assert split_type_suffix("dimos/cmd_vel#geometry_msgs.Twist") == (
        "dimos/cmd_vel",
        "geometry_msgs.Twist",
    )
    assert split_type_suffix("/plain") == ("/plain", None)


# subscribe_all contract: EVERY message, no conflation (spec-level fix)


@contextmanager
def lcm_base_ctx():
    bus = LCMPubSubBase()
    bus.start()
    try:
        yield bus, Topic("/spy_contract", Vector3)
    finally:
        bus.stop()


@contextmanager
def zenoh_base_ctx():
    pool = ZenohSessionPool()
    bus = ZenohPubSubBase(session_pool=pool)
    bus.start()
    try:
        yield bus, Topic("dimos/spy_contract", Vector3)
    finally:
        bus.stop()
        pool.close_all()


@pytest.mark.parametrize("bus_ctx", [lcm_base_ctx, zenoh_base_ctx], ids=["lcm", "zenoh"])
def test_subscribe_all_delivers_every_message(bus_ctx):
    """A gated (slow) consumer must still receive every message eventually.

    This is the sharp edge of the non-conflation contract: zenoh's original
    subscribe_all conflated to latest-per-topic and fails this with ~2 of 50.
    """
    n = 50
    with bus_ctx() as (bus, topic):
        gate = threading.Event()
        got = []
        done = threading.Event()

        def cb(msg, t):
            gate.wait(15.0)  # simulate a consumer slower than the burst
            got.append((str(t), len(msg)))
            if len(got) >= n:
                done.set()

        unsub = bus.subscribe_all(cb)
        time.sleep(0.5)  # let the wildcard subscription establish

        for _ in range(n):
            bus.publish(topic, VEC_BYTES)
        gate.set()

        done.wait(10.0)
        ours = [g for g in got if g[0] == str(topic)]
        assert len(ours) == n
        assert all(size == len(VEC_BYTES) for _, size in ours)
        unsub()


# subscribe_latest contract: conflation is the explicit opt-in


class FakeAllPubSub(AllPubSub[str, bytes]):
    """In-memory AllPubSub: publish() synchronously fans out to subscribers."""

    def __init__(self):
        self._subs: list[Callable[[bytes, str], None]] = []

    def publish(self, topic: str, message: bytes) -> None:
        for cb in list(self._subs):
            cb(message, topic)

    def subscribe(self, topic, callback):
        return self.subscribe_all(callback)

    def subscribe_all(self, callback):
        self._subs.append(callback)

        def unsub():
            if callback in self._subs:
                self._subs.remove(callback)

        return unsub


def test_subscribe_latest_conflates_to_newest():
    bus = FakeAllPubSub()
    delivered = []
    first = threading.Event()
    gate = threading.Event()
    quiet = threading.Event()

    def cb(msg, topic):
        delivered.append((topic, msg))
        first.set()
        gate.wait(10.0)  # block the drain thread while the burst happens
        if msg == b"99":
            quiet.set()

    unsub = bus.subscribe_latest(cb)
    bus.publish("t", b"0")
    assert first.wait(5.0)
    for i in range(1, 100):
        bus.publish("t", str(i).encode())
    gate.set()

    assert quiet.wait(5.0)
    # Newest message must arrive; the 98 intermediate ones conflate away.
    assert delivered[-1] == ("t", b"99")
    assert len(delivered) <= 5
    unsub()


def test_subscribe_latest_keeps_latest_per_topic():
    bus = FakeAllPubSub()
    delivered = []
    first = threading.Event()
    gate = threading.Event()
    done = threading.Event()

    def cb(msg, topic):
        delivered.append((topic, msg))
        first.set()
        gate.wait(10.0)
        if {t for t, _ in delivered} >= {"a", "b"} and len(delivered) >= 3:
            done.set()

    unsub = bus.subscribe_latest(cb)
    bus.publish("a", b"a0")
    assert first.wait(5.0)
    for i in range(1, 50):
        bus.publish("a", f"a{i}".encode())
        bus.publish("b", f"b{i}".encode())
    gate.set()

    assert done.wait(5.0)
    latest = dict(delivered)  # last write per topic wins
    assert latest["a"] == b"a49"
    assert latest["b"] == b"b49"
    unsub()


def test_subscribe_latest_unsubscribe_stops_delivery():
    bus = FakeAllPubSub()
    delivered = []
    got_one = threading.Event()

    def cb(msg, topic):
        delivered.append(msg)
        got_one.set()

    unsub = bus.subscribe_latest(cb)
    bus.publish("t", b"1")
    assert got_one.wait(5.0)
    unsub()
    count = len(delivered)
    bus.publish("t", b"2")
    time.sleep(0.2)
    assert len(delivered) == count


def test_subscribe_latest_failed_subscribe_stops_drain_thread():
    class FailingBus(FakeAllPubSub):
        def subscribe_all(self, callback):
            raise RuntimeError("subscriber declaration failed")

    bus = FailingBus()
    before = set(threading.enumerate())
    with pytest.raises(RuntimeError, match="subscriber declaration failed"):
        bus.subscribe_latest(lambda msg, topic: None)
    leaked = [
        t
        for t in threading.enumerate()
        if t not in before and t.name == "subscribe-latest-drain" and t.is_alive()
    ]
    assert not leaked  # drain thread must be stopped and joined on subscribe failure


# SpySources: every message counted, payloads never decoded


class _TapCollector:
    def __init__(self, topic_str: str, n: int):
        self.topic_str = topic_str
        self.n = n
        self.events: list[tuple[str, int]] = []
        self.done = threading.Event()

    def __call__(self, topic: str, nbytes: int) -> None:
        self.events.append((topic, nbytes))
        if len(self.ours()) >= self.n:
            self.done.set()

    def ours(self) -> list[tuple[str, int]]:
        return [e for e in self.events if e[0] == self.topic_str]


def _assert_no_decode(monkeypatch):
    """Make any Vector3 decode explode — the spy must never trigger one."""

    def boom(*a, **k):
        raise AssertionError("spy decoded a payload on the hot path")

    monkeypatch.setattr(Vector3, "lcm_decode", staticmethod(boom))


def test_lcm_source_counts_all_without_decoding(monkeypatch):
    _assert_no_decode(monkeypatch)
    topic = Topic("/spy_e2e", Vector3)
    collector = _TapCollector(str(topic), 20)

    src = LCMSpySource()
    src.start()
    untap = src.tap(collector)
    pub = LCMPubSubBase()
    pub.start()
    try:
        time.sleep(0.3)
        for _ in range(20):
            pub.publish(topic, VEC_BYTES)
            time.sleep(0.01)
        assert collector.done.wait(10.0)
        assert [n for _, n in collector.ours()] == [len(VEC_BYTES)] * 20
    finally:
        untap()
        src.stop()
        pub.stop()


def test_lcm_source_counts_undecodable_garbage(monkeypatch):
    """Payloads that would crash a decoder must still be counted (proves raw tap)."""
    _assert_no_decode(monkeypatch)
    topic = Topic("/spy_garbage", Vector3)
    collector = _TapCollector(str(topic), 5)

    src = LCMSpySource()
    src.start()
    untap = src.tap(collector)
    pub = LCMPubSubBase()
    pub.start()
    try:
        time.sleep(0.3)
        for _ in range(5):
            pub.publish(topic, b"\x00garbage-not-lcm")
            time.sleep(0.01)
        assert collector.done.wait(10.0)
        assert [n for _, n in collector.ours()] == [len(b"\x00garbage-not-lcm")] * 5
    finally:
        untap()
        src.stop()
        pub.stop()


def test_zenoh_source_counts_all_without_decoding(monkeypatch):
    _assert_no_decode(monkeypatch)
    topic = Topic("dimos/spy_e2e", Vector3)
    collector = _TapCollector(str(topic), 20)

    pool = ZenohSessionPool()
    src = ZenohSpySource(session_pool=pool)
    src.start()
    untap = src.tap(collector)
    pub = ZenohPubSubBase(session_pool=pool)
    pub.start()
    try:
        time.sleep(0.5)
        for _ in range(20):
            pub.publish(topic, VEC_BYTES)
            time.sleep(0.01)
        assert collector.done.wait(10.0)
        assert [n for _, n in collector.ours()] == [len(VEC_BYTES)] * 20
    finally:
        untap()
        src.stop()
        pub.stop()
        pool.close_all()


# TransportSpy: merging + totals + lifecycle (fake sources, deterministic)


class FakeSource:
    def __init__(self, name: str):
        self.name = name
        self.started = False
        self.taps: list[Callable[[str, int], None]] = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def tap(self, callback):
        self.taps.append(callback)

        def untap():
            if callback in self.taps:
                self.taps.remove(callback)

        return untap

    def subscribe_decoded(self, topic: str, callback: Callable[[Any], None]):
        raise NotImplementedError

    def emit(self, topic: str, nbytes: int) -> None:
        for cb in list(self.taps):
            cb(topic, nbytes)


def test_transport_spy_merges_sources_and_totals():
    a, b = FakeSource("lcm"), FakeSource("zenoh")
    spy = TransportSpy(sources=[a, b])
    spy.start()
    assert a.started and b.started

    a.emit("/t#geometry_msgs.Twist", 10)
    a.emit("/t#geometry_msgs.Twist", 10)
    b.emit("dimos/t#geometry_msgs.Twist", 20)
    a.emit("/u", 5)

    snap = spy.snapshot()
    assert snap[SpyKey("lcm", "/t#geometry_msgs.Twist")].total_msgs == 2
    assert snap[SpyKey("lcm", "/t#geometry_msgs.Twist")].total_bytes == 20
    assert snap[SpyKey("zenoh", "dimos/t#geometry_msgs.Twist")].total_msgs == 1
    assert snap[SpyKey("lcm", "/u")].total_bytes == 5
    assert spy.totals.total_msgs == 4
    assert spy.totals.total_bytes == 45

    spy.stop()
    assert not a.started and not b.started
    assert not a.taps and not b.taps  # untapped on stop


def test_transport_spy_snapshot_is_stable_copy():
    a = FakeSource("lcm")
    spy = TransportSpy(sources=[a])
    spy.start()
    a.emit("/t", 1)
    snap = spy.snapshot()
    a.emit("/new_topic_after_snapshot", 1)
    assert SpyKey("lcm", "/new_topic_after_snapshot") not in snap  # snapshot doesn't mutate
    assert SpyKey("lcm", "/new_topic_after_snapshot") in spy.snapshot()
    spy.stop()


def test_transport_spy_start_failure_stops_started_sources():
    class FailingStartSource(FakeSource):
        def start(self) -> None:
            raise RuntimeError("no zenoh here")

    a, b = FakeSource("lcm"), FailingStartSource("zenoh")
    spy = TransportSpy(sources=[a, b])
    with pytest.raises(RuntimeError, match="no zenoh here"):
        spy.start()
    assert not a.started  # rolled back, not left running
    assert not a.taps


def test_transport_spy_tap_failure_stops_started_sources():
    class FailingTapSource(FakeSource):
        def tap(self, callback):
            raise RuntimeError("tap exploded")

    a, b = FakeSource("lcm"), FailingTapSource("zenoh")
    spy = TransportSpy(sources=[a, b])
    with pytest.raises(RuntimeError, match="tap exploded"):
        spy.start()
    assert not a.started and not b.started
    assert not a.taps


def test_fake_source_satisfies_protocol():
    from dimos.protocol.pubsub.spy import SpySource

    assert isinstance(FakeSource("x"), SpySource)


# Lazy decode hook: spec'd now, implemented in a follow-up (stays off hot path)


def test_subscribe_decoded_is_not_implemented_in_v1():
    src = LCMSpySource()
    with pytest.raises(NotImplementedError):
        src.subscribe_decoded("/x#geometry_msgs.Vector3", lambda m: None)
