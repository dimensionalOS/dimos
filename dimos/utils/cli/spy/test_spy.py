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

"""Contract tests for the spy core (`dimos spy`).

- TopicStats: deterministic windowed stats from injected timestamps.
- subscribe_all: LCM delivers every message (non-conflating regex '.*').
- SpySources: count every message without ever decoding a payload.
- TransportSpy: merges sources into (transport, topic)-keyed stats + totals.
- subscribe_decoded: spec'd hook, must stay NotImplementedError in v1.
"""

from collections.abc import Callable
import threading
import time
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase
from dimos.protocol.pubsub.impl.zenohpubsub import ZenohPubSubBase
from dimos.protocol.pubsub.topic import Topic
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.cli.spy.core import (
    SOURCE_FACTORIES,
    LCMSpySource,
    SpyKey,
    SpySource,
    TopicStats,
    TransportSpy,
    ZenohSpySource,
    default_sources,
    split_type_suffix,
)

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


def test_topic_stats_window_stats_single_reading():
    s = TopicStats(history_window=60.0)
    t0 = 1000.0
    for i in range(10):  # 1 Hz, 50 bytes each
        s.record(50, t0 + i)
    view = s.window_stats(5.0, now=t0 + 9.0)
    assert view.freq == pytest.approx(1.0, rel=0.25)
    assert view.bytes_per_sec == pytest.approx(50.0, rel=0.25)
    assert view.total_bytes == 500
    assert view.total_msgs == 10
    assert view.last_seen == pytest.approx(t0 + 9.0)


def test_topic_stats_empty():
    s = TopicStats()
    assert s.total_msgs == 0
    assert s.total_bytes == 0
    assert s.last_seen is None
    assert s.freq(5.0, now=123.0) == 0.0
    assert s.bytes_per_sec(5.0, now=123.0) == 0.0


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


@pytest.fixture
def lcm_bus():
    bus = LCMPubSubBase()
    bus.start()
    yield bus, Topic("/spy_contract", Vector3)
    bus.stop()


def _publish_probes_until(pub, probe_topic: Topic, established: Callable[[], bool]) -> None:
    """Publish probes until the subscription delivers one, instead of a fixed sleep."""
    deadline = time.time() + 10.0
    while not established():
        assert time.time() < deadline, f"subscription never saw a probe on {probe_topic}"
        pub.publish(probe_topic, VEC_BYTES)
        time.sleep(0.01)


def test_subscribe_all_delivers_every_message(lcm_bus):
    """A gated (slow) consumer must still receive every message eventually.

    LCM's subscribe_all is non-conflating (regex '.*', 10k queue), so all 50
    messages are delivered even when the consumer lags the burst.
    """
    n = 50
    bus, topic = lcm_bus
    probe_topic = Topic("/spy_contract_probe", Vector3)
    probe_seen = threading.Event()
    gate = threading.Event()
    got = []
    done = threading.Event()

    def cb(msg, t):
        if str(t) == str(probe_topic):
            probe_seen.set()
            return
        gate.wait(15.0)  # simulate a consumer slower than the burst
        got.append((str(t), len(msg)))
        if len(got) >= n:
            done.set()

    bus.subscribe_all(cb)
    _publish_probes_until(bus, probe_topic, probe_seen.is_set)

    for _ in range(n):
        bus.publish(topic, VEC_BYTES)
    gate.set()

    done.wait(10.0)
    ours = [g for g in got if g[0] == str(topic)]
    assert len(ours) == n
    assert all(size == len(VEC_BYTES) for _, size in ours)


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

    def saw(self, topic_str: str) -> bool:
        return any(t == topic_str for t, _ in self.events)


def _assert_no_decode(monkeypatch):
    """Make any Vector3 decode explode — the spy must never trigger one."""

    def boom(*a, **k):
        raise AssertionError("spy decoded a payload on the hot path")

    monkeypatch.setattr(Vector3, "lcm_decode", staticmethod(boom))


@pytest.fixture
def lcm_spy_source():
    src = LCMSpySource()
    src.start()
    yield src
    src.stop()


@pytest.fixture
def lcm_pub():
    pub = LCMPubSubBase()
    pub.start()
    yield pub
    pub.stop()


def test_lcm_source_counts_all_without_decoding(monkeypatch, lcm_spy_source, lcm_pub):
    _assert_no_decode(monkeypatch)
    topic = Topic("/spy_e2e", Vector3)
    probe = Topic("/spy_e2e_probe", Vector3)
    collector = _TapCollector(str(topic), 20)

    lcm_spy_source.tap(collector)
    _publish_probes_until(lcm_pub, probe, lambda: collector.saw(str(probe)))
    for _ in range(20):
        lcm_pub.publish(topic, VEC_BYTES)
        time.sleep(0.01)
    assert collector.done.wait(10.0)
    assert [n for _, n in collector.ours()] == [len(VEC_BYTES)] * 20


def test_lcm_source_counts_undecodable_garbage(monkeypatch, lcm_spy_source, lcm_pub):
    """Payloads that would crash a decoder must still be counted (proves raw tap)."""
    _assert_no_decode(monkeypatch)
    topic = Topic("/spy_garbage", Vector3)
    probe = Topic("/spy_garbage_probe", Vector3)
    collector = _TapCollector(str(topic), 5)

    lcm_spy_source.tap(collector)
    _publish_probes_until(lcm_pub, probe, lambda: collector.saw(str(probe)))
    for _ in range(5):
        lcm_pub.publish(topic, b"\x00garbage-not-lcm")
        time.sleep(0.01)
    assert collector.done.wait(10.0)
    assert [n for _, n in collector.ours()] == [len(b"\x00garbage-not-lcm")] * 5


@pytest.fixture
def zenoh_pool():
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


@pytest.fixture
def zenoh_spy_source(zenoh_pool):
    src = ZenohSpySource(session_pool=zenoh_pool)
    src.start()
    yield src
    src.stop()


@pytest.fixture
def zenoh_pub(zenoh_pool):
    pub = ZenohPubSubBase(session_pool=zenoh_pool)
    pub.start()
    yield pub
    pub.stop()


def test_zenoh_source_counts_all_without_decoding(monkeypatch, zenoh_spy_source, zenoh_pub):
    _assert_no_decode(monkeypatch)
    topic = Topic("dimos/spy_e2e", Vector3)
    probe = Topic("dimos/spy_e2e_probe", Vector3)
    collector = _TapCollector(str(topic), 20)

    zenoh_spy_source.tap(collector)
    _publish_probes_until(zenoh_pub, probe, lambda: collector.saw(str(probe)))
    for _ in range(20):
        zenoh_pub.publish(topic, VEC_BYTES)
        time.sleep(0.01)
    assert collector.done.wait(10.0)
    assert [n for _, n in collector.ours()] == [len(VEC_BYTES)] * 20


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


def test_transport_spy_stop_continues_past_failing_source(spy_warnings):
    class FailingStopSource(FakeSource):
        def stop(self) -> None:
            raise RuntimeError("stop exploded")

    a, b = FailingStopSource("lcm"), FakeSource("zenoh")
    spy = TransportSpy(sources=[a, b])
    spy.start()
    spy.stop()  # must not raise, and must still stop the healthy source
    assert not b.started
    assert not a.taps and not b.taps
    assert "stop exploded" in spy_warnings.text


class _FailingStartSource(FakeSource):
    def start(self) -> None:
        raise RuntimeError("backend down")


def test_transport_spy_best_effort_skips_failed_source(request, spy_warnings):
    good, bad = FakeSource("lcm"), _FailingStartSource("zenoh")
    spy = TransportSpy(sources=[good, bad])
    spy.start(best_effort=True)  # degrade to the survivor instead of dying
    request.addfinalizer(spy.stop)
    assert good.started and good.taps  # survivor is live and tapped
    assert not bad.started
    good.emit("/t", 5)
    assert spy.snapshot()[SpyKey("lcm", "/t")].total_bytes == 5
    assert "zenoh" in spy_warnings.text  # skipped source is warned about


def test_transport_spy_best_effort_raises_when_all_fail():
    spy = TransportSpy(sources=[_FailingStartSource("lcm"), _FailingStartSource("zenoh")])
    with pytest.raises(RuntimeError, match="no spy transports could start"):
        spy.start(best_effort=True)


def test_default_sources_skips_unavailable_backend(monkeypatch, spy_warnings):
    def unavailable():
        raise ImportError("zenoh backend missing")

    monkeypatch.setitem(SOURCE_FACTORIES, "lcm", lambda: FakeSource("lcm"))
    monkeypatch.setitem(SOURCE_FACTORIES, "zenoh", unavailable)
    sources = default_sources()
    assert [s.name for s in sources] == ["lcm"]  # degrades instead of crashing
    assert "zenoh" in spy_warnings.text


def test_default_sources_errors_when_no_backend_available(monkeypatch):
    def unavailable():
        raise ImportError("backend missing")

    for name in SOURCE_FACTORIES:
        monkeypatch.setitem(SOURCE_FACTORIES, name, unavailable)
    with pytest.raises(RuntimeError, match="no spy transports available"):
        default_sources()


def test_fake_source_satisfies_protocol():
    assert isinstance(FakeSource("x"), SpySource)


# Lazy decode hook: spec'd now, implemented in a follow-up (stays off hot path)


def test_subscribe_decoded_is_not_implemented_in_v1():
    src = LCMSpySource()
    with pytest.raises(NotImplementedError):
        src.subscribe_decoded("/x#geometry_msgs.Vector3", lambda m: None)
