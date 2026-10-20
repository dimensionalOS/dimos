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

"""Tests for ZMQPubSubBase: raw bytes pub/sub over a ZeroMQ XSUB/XPUB proxy."""

from __future__ import annotations

import multiprocessing
import pickle
import re
import threading
import time

import numpy as np
import pytest

pytest.importorskip("zmq")

import zmq

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.zmqpubsub import (
    ZMQ,
    PickleZMQ,
    Topic,
    ZMQPubSubBase,
    _wire_key,
)
from dimos.protocol.pubsub.patterns import Glob
from dimos.protocol.service.zmqservice import ZMQConfig, close_proxies, hosted_proxies
from dimos.utils.testing.collector import CallbackCollector
from dimos.utils.testing.ports import free_zmq_endpoints
from dimos.utils.testing.waiting import retry_until, wait_until


@pytest.fixture()
def endpoints():
    """A private bus on two free loopback ports; its proxy is closed after."""
    yield free_zmq_endpoints()
    close_proxies()


@pytest.fixture()
def make_pubsub(endpoints):
    """Build started pub/sub instances on the private bus, stop them after."""
    created = []

    def _make(cls=ZMQPubSubBase, **kwargs):
        ps = cls(**endpoints, **kwargs)
        ps.start()
        created.append(ps)
        return ps

    yield _make
    for ps in created:
        ps.stop()


@pytest.fixture()
def pubsub(make_pubsub):
    return make_pubsub()


def test_wire_key_is_channel_plus_nul() -> None:
    assert _wire_key(Topic("/foo", Vector3)) == b"/foo#geometry_msgs.Vector3\x00"
    assert _wire_key(Topic("/foo")) == b"/foo\x00"
    assert _wire_key("/foo") == b"/foo\x00"


def test_wire_key_rejects_nul_in_topic() -> None:
    with pytest.raises(ValueError, match="NUL"):
        _wire_key("/fo\x00o")


def test_config_defaults_come_from_global_config() -> None:
    from dimos.core.global_config import global_config

    config = ZMQConfig()
    assert config.pub_endpoint == global_config.zmq_pub_endpoint
    assert config.sub_endpoint == global_config.zmq_sub_endpoint
    assert config.proxy == "auto"


def test_first_message_after_subscribe_is_delivered(pubsub) -> None:
    """subscribe() returns only once the subscription reached the publisher."""
    collector = CallbackCollector(1)
    topic = Topic("/first")
    pubsub.subscribe(topic, collector)
    pubsub.publish(topic, b"one")
    collector.wait()
    assert collector.results == [(b"one", topic)]


def test_exact_topic_never_matches_a_longer_name(pubsub) -> None:
    """zmq filters by prefix; the NUL terminator makes the match exact."""
    collector = CallbackCollector(1)
    pubsub.subscribe(Topic("/foo"), collector)
    pubsub.publish(Topic("/foobar"), b"longer")
    pubsub.publish(Topic("/foo", Vector3), b"typed")
    pubsub.publish(Topic("/foo"), b"exact")
    collector.wait()
    time.sleep(0.1)
    assert [msg for msg, _ in collector.results] == [b"exact"]


def test_multiple_callbacks_share_one_zmq_subscription(pubsub) -> None:
    topic = Topic("/shared")
    key = _wire_key(topic)
    unsub_a = pubsub.subscribe(topic, lambda m, t: None)
    unsub_b = pubsub.subscribe(topic, lambda m, t: None)
    assert pubsub._refs[key] == 2
    unsub_a()
    assert pubsub._refs[key] == 1
    unsub_b()
    assert key not in pubsub._refs
    unsub_b()  # idempotent


def test_unsubscribe_stops_delivery_at_once(pubsub) -> None:
    topic = Topic("/once")
    received = []
    unsubscribe = pubsub.subscribe(topic, lambda m, t: received.append(m))
    pubsub.publish(topic, b"before")
    wait_until(lambda: received == [b"before"], timeout=2.0)
    unsubscribe()
    pubsub.publish(topic, b"after")
    time.sleep(0.1)
    assert received == [b"before"]


def test_glob_pattern_matches_one_level(pubsub) -> None:
    collector = CallbackCollector(1)
    pubsub.subscribe(Topic(Glob("/sensor/*")), collector)
    pubsub.publish(Topic("/sensor/a/b"), b"deep")
    pubsub.publish(Topic("/other"), b"other")
    pubsub.publish(Topic("/sensor/temp", Vector3), b"temp")
    collector.wait()
    time.sleep(0.1)
    assert len(collector.results) == 1
    msg, topic = collector.results[0]
    assert msg == b"temp"
    # The delivered topic is rebuilt from the wire key, type included.
    assert topic.topic == "/sensor/temp"
    assert topic.lcm_type is Vector3


def test_regex_pattern_is_anchored(pubsub) -> None:
    collector = CallbackCollector(2)
    pubsub.subscribe(Topic(re.compile("/robot/.*")), collector)
    pubsub.publish(Topic("/xrobot/arm"), b"no")
    pubsub.publish(Topic("/robot/arm"), b"arm")
    pubsub.publish(Topic("/robot/leg/motor"), b"motor")
    collector.wait()
    time.sleep(0.1)
    assert [msg for msg, _ in collector.results] == [b"arm", b"motor"]


def test_subscribe_all_sees_every_topic_but_no_probes(pubsub) -> None:
    collector = CallbackCollector(2)
    pubsub.subscribe_all(collector)
    # Another subscription probes the bus; subscribe_all must not see the probe.
    pubsub.subscribe(Topic("/unrelated"), lambda m, t: None)
    pubsub.publish(Topic("/a", Vector3), b"a")
    pubsub.publish(Topic("/b/c"), b"bc")
    collector.wait()
    time.sleep(0.1)
    assert [(msg, str(topic)) for msg, topic in collector.results] == [
        (b"a", "/a#geometry_msgs.Vector3"),
        (b"bc", "/b/c"),
    ]


def test_callback_error_does_not_stop_delivery(pubsub) -> None:
    topic = Topic("/raise")
    received = []

    def bad(msg, t):
        received.append(msg)
        raise RuntimeError("boom")

    pubsub.subscribe(topic, bad)
    pubsub.publish(topic, b"1")
    pubsub.publish(topic, b"2")
    wait_until(lambda: received == [b"1", b"2"], timeout=2.0)


def test_large_typed_message_roundtrip(make_pubsub) -> None:
    """Frames above the zero-copy threshold take the no-copy send path."""
    typed = make_pubsub(ZMQ)
    topic = Topic("/image", Image)
    collector = CallbackCollector(1)
    typed.subscribe(topic, collector)
    image = Image(np.full((480, 640, 3), 7, dtype=np.uint8))
    typed.publish(topic, image)
    collector.wait()
    received = collector.results[0][0]
    assert isinstance(received, Image)
    assert received.data.shape == (480, 640, 3)
    assert int(received.data[0, 0, 0]) == 7


def test_pickle_encoding_roundtrip(make_pubsub) -> None:
    pickled = make_pubsub(PickleZMQ)
    topic = Topic("/obj")
    collector = CallbackCollector(1)
    pickled.subscribe(topic, collector)
    pickled.publish(topic, {"key": [1, 2, 3]})
    collector.wait()
    assert collector.results[0][0] == {"key": [1, 2, 3]}


def test_second_instance_connects_to_the_hosted_proxy(make_pubsub, endpoints) -> None:
    first = make_pubsub()
    second = make_pubsub()
    # Hosting is per process: both instances share the one proxy it runs.
    assert first.hosts_proxy
    assert second.hosts_proxy
    assert hosted_proxies() == [(endpoints["pub_endpoint"], endpoints["sub_endpoint"])]

    # subscribe() settles against the subscriber's own publisher; another
    # instance's publisher learns of the subscription asynchronously.
    to_second = threading.Event()
    to_first = threading.Event()
    second.subscribe(Topic("/ping"), lambda m, t: to_second.set())
    first.subscribe(Topic("/pong"), lambda m, t: to_first.set())
    retry_until(to_second, lambda: first.publish(Topic("/ping"), b"ping"))
    retry_until(to_first, lambda: second.publish(Topic("/pong"), b"pong"))


def test_connect_mode_never_hosts(endpoints) -> None:
    ps = ZMQPubSubBase(**endpoints, proxy="connect", settle_timeout=0)
    ps.start()
    try:
        assert not ps.hosts_proxy
        assert hosted_proxies() == []
    finally:
        ps.stop()


def test_host_mode_fails_when_another_process_holds_the_endpoints(endpoints) -> None:
    # A foreign socket on the subscriber endpoint stands in for another process.
    foreign = zmq.Context.instance().socket(zmq.XPUB)
    foreign.linger = 0
    foreign.bind(endpoints["sub_endpoint"])
    try:
        with pytest.raises(RuntimeError, match="cannot host the ZeroMQ proxy"):
            ZMQPubSubBase(**endpoints, proxy="host").start()
        # auto mode takes the taken port as a proxy run elsewhere.
        auto = ZMQPubSubBase(**endpoints, settle_timeout=0)
        auto.start()
        try:
            assert not auto.hosts_proxy
            assert hosted_proxies() == []
        finally:
            auto.stop()
    finally:
        foreign.close()


def test_auto_mode_fails_on_a_half_taken_bus(endpoints) -> None:
    """Only the publisher endpoint taken means no consistent proxy exists."""
    foreign = zmq.Context.instance().socket(zmq.XSUB)
    foreign.linger = 0
    foreign.bind(endpoints["pub_endpoint"])
    try:
        with pytest.raises(RuntimeError, match="no proxy to join"):
            ZMQPubSubBase(**endpoints).start()
    finally:
        foreign.close()


def test_settle_timeout_zero_returns_at_once(pubsub, endpoints) -> None:
    """Without settling, delivery still starts once the subscription propagates."""
    eager = ZMQPubSubBase(**endpoints, settle_timeout=0)
    eager.start()
    try:
        topic = Topic("/eager")
        got = threading.Event()
        started = time.monotonic()
        eager.subscribe(topic, lambda m, t: got.set())
        assert time.monotonic() - started < 0.5
        retry_until(got, lambda: eager.publish(topic, b"x"))
    finally:
        eager.stop()


def test_subscribe_from_callback_does_not_deadlock(pubsub) -> None:
    """A callback subscribing cannot wait on a probe only its own thread receives."""
    inner_got = threading.Event()
    outer_done = threading.Event()

    def outer(msg, topic):
        pubsub.subscribe(Topic("/inner"), lambda m, t: inner_got.set())
        outer_done.set()

    pubsub.subscribe(Topic("/outer"), outer)
    pubsub.publish(Topic("/outer"), b"go")
    assert outer_done.wait(timeout=2.0)
    retry_until(inner_got, lambda: pubsub.publish(Topic("/inner"), b"in"))


def test_restart_keeps_subscriptions(pubsub) -> None:
    topic = Topic("/restart")
    got = threading.Event()
    pubsub.subscribe(topic, lambda m, t: got.set())
    pubsub.stop()
    pubsub.start()
    retry_until(got, lambda: pubsub.publish(topic, b"again"))


def test_pickle_roundtrip_preserves_config_and_works(endpoints) -> None:
    original = ZMQPubSubBase(**endpoints, hwm=5, settle_timeout=0.5)
    original.start()
    try:
        restored = pickle.loads(pickle.dumps(original))
        assert restored.config == original.config
        restored.start()
        try:
            collector = CallbackCollector(1)
            restored.subscribe(Topic("/restored"), collector)
            original.publish(Topic("/restored"), b"hi")
            collector.wait()
        finally:
            restored.stop()
    finally:
        original.stop()


def test_publish_before_start_raises(endpoints) -> None:
    with pytest.raises(RuntimeError, match="not started"):
        ZMQPubSubBase(**endpoints).publish(Topic("/x"), b"x")
    with pytest.raises(RuntimeError, match="not started"):
        ZMQPubSubBase(**endpoints).subscribe(Topic("/x"), lambda m, t: None)


def test_publish_after_stop_is_logged_not_raised(endpoints) -> None:
    ps = ZMQPubSubBase(**endpoints)
    ps.start()
    ps.stop()
    ps.publish(Topic("/x"), b"x")
    ps.stop()  # idempotent


def test_stop_from_callback_completes_cleanup(run_in_subprocess, endpoints) -> None:
    run_in_subprocess(_check_stop_from_callback, endpoints)


def _check_stop_from_callback(endpoints) -> None:
    ps = ZMQPubSubBase(**endpoints)
    ps.start()
    thread = ps._thread
    done = threading.Event()

    def callback(msg, topic):
        ps.stop()
        done.set()

    ps.subscribe(Topic("/stop"), callback)
    ps.publish(Topic("/stop"), b"stop")
    assert done.wait(timeout=2.0)
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert ps._thread is None
    close_proxies()


def _publish_from_child(endpoints, stop_after: float) -> None:
    ps = ZMQPubSubBase(**endpoints, proxy="connect", settle_timeout=0)
    ps.start()
    deadline = time.monotonic() + stop_after
    while time.monotonic() < deadline:
        ps.publish(Topic("/xproc", Vector3), Vector3(1, 2, 3).lcm_encode())
        time.sleep(0.02)
    ps.stop()


def test_cross_process_delivery(pubsub, endpoints) -> None:
    """This process hosts the proxy; a spawned process connects and publishes."""
    collector = CallbackCollector(1)
    pubsub.subscribe(Topic("/xproc", Vector3), collector)
    child = multiprocessing.get_context("spawn").Process(
        target=_publish_from_child, args=(endpoints, 5.0)
    )
    child.start()
    try:
        collector.wait()
    finally:
        child.join(timeout=10.0)
        if child.is_alive():
            child.kill()
            child.join(timeout=2.0)
        child.close()
    assert collector.results[0][0] == Vector3(1, 2, 3).lcm_encode()
