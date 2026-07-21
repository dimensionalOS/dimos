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

"""Tests for ZenohPubSubBase — raw bytes pub/sub over Zenoh."""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Any

import numpy as np
import pytest

pytest.importorskip("zenoh")

import zenoh

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic
from dimos.protocol.pubsub.impl.zenohpubsub import (
    Topic,
    Zenoh,
    ZenohPubSubBase,
    ZenohQoS,
    _key_expr_to_topic,
    _topic_to_key_expr,
)
from dimos.protocol.pubsub.patterns import Glob
from dimos.protocol.service.zenohservice import ZenohSessionPool


@pytest.fixture()
def make_pubsub():
    """Build started ZenohPubSubBase instances on isolated pools, clean up after."""
    created = []

    def _make(pubsub_cls=ZenohPubSubBase, **kwargs):
        pool = ZenohSessionPool()
        ps = pubsub_cls(session_pool=pool, **kwargs)
        ps.start()
        created.append((ps, pool))
        return ps

    yield _make
    for ps, pool in created:
        ps.stop()
        # Close the pool so Zenoh's internal threads are joined
        pool.close_all()


@pytest.fixture()
def pubsub(make_pubsub):
    """Create and start a ZenohPubSubBase instance on an isolated pool, clean up after."""
    return make_pubsub()


def _image_message() -> Image:
    return Image(np.array([[[1, 2, 3]]], dtype=np.uint8), frame_id="camera", ts=1.0)


def _pointcloud_message() -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32), frame_id="lidar", timestamp=1.0
    )


class TestZenohPubSubBase:
    def test_publish_and_subscribe(self, pubsub, retry_until) -> None:
        received = []
        event = threading.Event()
        topic = Topic("dimos/test/basic")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)
            event.set()

        pubsub.subscribe(topic, callback)
        retry_until(event, lambda: pubsub.publish(topic, b"hello zenoh"))
        assert received[0] == b"hello zenoh"

    def test_multiple_subscribers(self, pubsub, retry_until) -> None:
        received_a: list[bytes] = []
        received_b: list[bytes] = []
        both_received = threading.Event()
        countdown = threading.Barrier(2, action=both_received.set)
        topic = Topic("dimos/test/multi")

        def callback_a(msg: bytes, t: Topic) -> None:
            received_a.append(msg)
            countdown.wait()

        def callback_b(msg: bytes, t: Topic) -> None:
            received_b.append(msg)
            countdown.wait()

        pubsub.subscribe(topic, callback_a)
        pubsub.subscribe(topic, callback_b)
        retry_until(both_received, lambda: pubsub.publish(topic, b"broadcast"))
        assert received_a[-1:] == [b"broadcast"]
        assert received_b[-1:] == [b"broadcast"]

    def test_unsubscribe(self, pubsub, retry_until) -> None:
        received: list[bytes] = []
        event = threading.Event()
        topic = Topic("dimos/test/unsub")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)
            event.set()

        unsub = pubsub.subscribe(topic, callback)
        retry_until(event, lambda: pubsub.publish(topic, b"before"))
        assert received == [b"before"]

        # Unsubscribe and verify no more messages arrive
        unsub()
        received.clear()
        event.clear()
        pubsub.publish(topic, b"after")

        # We can't prove a negative with an event, so use a short timeout
        assert not event.wait(timeout=0.2), "Received message after unsubscribe"
        assert received == []

    def test_unsubscribe_is_idempotent(self, pubsub) -> None:
        topic = Topic("dimos/test/idempotent")
        unsub = pubsub.subscribe(topic, lambda msg, t: None)
        unsub()
        unsub()  # should not raise

    def test_concurrent_unsubscribe(self, pubsub) -> None:
        topic = Topic("dimos/test/concurrent_unsub")
        unsub = pubsub.subscribe(topic, lambda msg, t: None)

        n_threads = 8
        barrier = threading.Barrier(n_threads)
        errors: list[Exception] = []

        def race() -> None:
            barrier.wait()
            try:
                unsub()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=race) for _ in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert errors == []
        assert len(pubsub._subscribers) == 0

    def test_concrete_subscription_passes_topic_through(self, pubsub, retry_until) -> None:
        received: list[Topic] = []
        event = threading.Event()
        topic = Topic("dimos/test/passthrough", lcm_type=Twist)

        def callback(msg: bytes, t: Topic) -> None:
            received.append(t)
            event.set()

        pubsub.subscribe(topic, callback)
        retry_until(event, lambda: pubsub.publish(topic, b"data"))
        assert received[0] is topic

    def test_untyped_topic_with_dotted_segment_round_trips(self, pubsub, retry_until) -> None:
        # The last segment resolves as a type name, but a concrete subscription
        # must not re-parse it into base+type on receive.
        received: list[Topic] = []
        event = threading.Event()
        topic = Topic("dimos/test/geometry_msgs.Twist")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(t)
            event.set()

        pubsub.subscribe(topic, callback)
        retry_until(event, lambda: pubsub.publish(topic, b"data"))
        assert received[0] is topic

    def test_publish_before_subscriber_does_not_error(self, pubsub) -> None:
        topic = Topic("dimos/test/no_sub")
        pubsub.publish(topic, b"orphan message")  # should not raise

    def test_stop_cleans_up_publishers_and_subscribers(self, pubsub) -> None:
        topic = Topic("dimos/test/cleanup")
        pubsub.subscribe(topic, lambda msg, t: None)
        pubsub.publish(topic, b"test")
        pubsub.stop()
        assert len(pubsub._publishers) == 0
        assert len(pubsub._subscribers) == 0

    def test_subscribe_all(self, pubsub, retry_until) -> None:
        received: list[bytes] = []
        event = threading.Event()
        topic = Topic("dimos/test/any/topic")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)
            if msg == b"wildcard":
                event.set()

        pubsub.subscribe_all(callback)
        retry_until(event, lambda: pubsub.publish(topic, b"wildcard"))
        assert b"wildcard" in received

    def test_subscribe_all_accept_filter_drops_rejected_topic(self, pubsub, retry_until) -> None:
        """DIM-1125"""
        received: list[Topic] = []
        delivered = threading.Event()
        rejected_seen = threading.Event()
        accepted = Topic("dimos/test/accept/yes")
        rejected = Topic("dimos/test/accept/no")

        def accept(topic: Topic) -> bool:
            if topic.topic == rejected.topic:
                rejected_seen.set()
                return False
            return True

        def callback(msg: bytes, t: Topic) -> None:
            received.append(t)
            delivered.set()

        pubsub.subscribe_all(callback, accept)
        retry_until(rejected_seen, lambda: pubsub.publish(rejected, b"drop"))
        retry_until(delivered, lambda: pubsub.publish(accepted, b"keep"))
        assert {t.topic for t in received} == {accepted.topic}

    def test_subscribe_all_heavy_false_excludes_heavy_topics(self, pubsub, retry_until) -> None:
        """DIM-1125"""
        seen: set[str] = set()
        event = threading.Event()

        def callback(msg: bytes, t: Topic) -> None:
            seen.add(t.topic)
            event.set()

        pubsub.subscribe_all(callback, heavy=False)
        retry_until(
            event,
            lambda: (
                pubsub.publish(Topic("dimos/test/hf", Image), b"heavy"),
                pubsub.publish(Topic("dimos/test/light"), b"light"),
            ),
        )
        assert seen == {"dimos/test/light"}

    def test_subscribe_all_heavy_allowlist_selects_heavy_topics(self, pubsub, retry_until) -> None:
        """DIM-1125"""
        seen: set[str] = set()
        event = threading.Event()
        expected = {"dimos/test/himg", "dimos/test/hglob/a", "dimos/test/hlight"}

        def callback(msg: bytes, t: Topic) -> None:
            seen.add(t.topic)
            if seen >= expected:
                event.set()

        pubsub.subscribe_all(callback, heavy=["/test/himg", Glob("/test/hglob/*")])
        retry_until(
            event,
            lambda: (
                pubsub.publish(Topic("dimos/test/himg", Image), b"h1"),
                pubsub.publish(Topic("dimos/test/hglob/a", Image), b"h2"),
                pubsub.publish(Topic("dimos/test/hother", Image), b"h3"),
                pubsub.publish(Topic("dimos/test/hlight"), b"l1"),
            ),
        )
        assert seen == expected

    def test_subscribe_after_stop_does_not_track(self, pubsub) -> None:
        # Models the declare/stop race: once stopped, a newly declared subscriber
        # must undeclare itself rather than be tracked (and leak past shutdown).
        pubsub.stop()
        unsub = pubsub.subscribe(Topic("dimos/test/after_stop"), lambda msg, t: None)
        assert pubsub._subscribers == []
        unsub()  # no-op, must not raise

    def test_subscribe_all_after_stop_is_noop(self, pubsub) -> None:
        pubsub.stop()
        unsub = pubsub.subscribe_all(lambda msg, t: None)
        assert pubsub._drain_stops == []
        unsub()  # no-op, must not raise


@pytest.mark.parametrize("make_message", (_image_message, _pointcloud_message))
@pytest.mark.parametrize("subscription", ("fixed", "pattern", "all"))
def test_clause_2_typed_zenoh_heavy_delivery_enters_callback_with_wire_bytes(
    make_pubsub,
    make_message: Callable[[], Image | PointCloud2],
    mocker,
    subscription: str,
    retry_until,
) -> None:
    """Clause 2 DIM-1125: typed Zenoh heavy subscriptions receive bytes before decode."""
    pubsub = make_pubsub(Zenoh)
    message = make_message()
    publisher_topic = Topic("dimos/clause2/heavy", type(message))
    callback_decode_counts: list[int] = []
    payloads: list[Any] = []
    received = threading.Event()
    decode = mocker.spy(type(message), "lcm_decode")

    def callback(payload: Any, _: Topic) -> None:
        callback_decode_counts.append(decode.call_count)
        payloads.append(payload)
        received.set()

    if subscription == "all":
        pubsub.subscribe_all(
            callback, lambda received_topic: received_topic.topic == publisher_topic.topic
        )
    else:
        subscriber_topic = (
            Topic("dimos/clause2/heavy/**") if subscription == "pattern" else publisher_topic
        )
        pubsub.subscribe(subscriber_topic, callback)
    retry_until(received, lambda: pubsub.publish(publisher_topic, message))

    assert callback_decode_counts == [0]
    assert isinstance(payloads[0], bytes)
    assert type(message).lcm_decode(payloads[0]).lcm_encode() == message.lcm_encode()
    assert decode.call_count == 1


class TestPublisherQoS:
    """Publisher QoS comes from the Topic and is applied at declare time."""

    def test_publisher_qos_from_topic(self, pubsub) -> None:
        topic = Topic(
            "dimos/test/qos/stream",
            qos=ZenohQoS(reliability="best_effort", congestion_control="drop"),
        )
        pubsub.publish(topic, b"x")
        pub = pubsub._publishers["dimos/test/qos/stream"]
        assert pub.reliability == zenoh.Reliability.BEST_EFFORT
        assert pub.congestion_control == zenoh.CongestionControl.DROP

    def test_publisher_default_qos_without_topic_qos(self, pubsub) -> None:
        # Pins zenoh's publisher defaults: reliable, drop under congestion.
        pubsub.publish(Topic("dimos/test/qos/defaults"), b"x")
        pub = pubsub._publishers["dimos/test/qos/defaults"]
        assert pub.reliability == zenoh.Reliability.RELIABLE
        assert pub.congestion_control == zenoh.CongestionControl.DROP

    def test_plain_lcm_topic_gets_default_qos(self, pubsub) -> None:
        # Shared code (TF, encoders) still passes lcmpubsub Topics.
        pubsub.publish(LCMTopic("dimos/test/qos/plain"), b"x")
        pub = pubsub._publishers["dimos/test/qos/plain"]
        assert pub.reliability == zenoh.Reliability.RELIABLE

    def test_partial_qos_omits_unset_fields(self, pubsub) -> None:
        topic = Topic("dimos/test/qos/partial", qos=ZenohQoS(congestion_control="block"))
        pubsub.publish(topic, b"x")
        pub = pubsub._publishers["dimos/test/qos/partial"]
        assert pub.reliability == zenoh.Reliability.RELIABLE  # zenoh default kept
        assert pub.congestion_control == zenoh.CongestionControl.BLOCK


class TestZenohQoSToWire:
    """QoS serialization sent to native modules over stdin."""

    def test_full_qos(self) -> None:
        qos = ZenohQoS(reliability="best_effort", congestion_control="drop")
        assert qos.to_wire() == {"reliability": "best_effort", "congestion_control": "drop"}

    def test_partial_qos_omits_unset(self) -> None:
        assert ZenohQoS(congestion_control="block").to_wire() == {"congestion_control": "block"}

    def test_empty_qos(self) -> None:
        assert ZenohQoS().to_wire() == {}


class TestTopicKeyExprConversion:
    """Tests for _topic_to_key_expr and _key_expr_to_topic round-trip."""

    def test_typed_topic_to_key_expr(self) -> None:
        topic = Topic("dimos/cmd_vel", lcm_type=Twist)
        key = _topic_to_key_expr(topic)
        assert key == "dimos/cmd_vel/geometry_msgs.Twist"

    def test_untyped_topic_to_key_expr(self) -> None:
        topic = Topic("dimos/data")
        key = _topic_to_key_expr(topic)
        assert key == "dimos/data"

    def test_key_expr_to_topic_with_known_type(self) -> None:
        topic = _key_expr_to_topic("dimos/cmd_vel/geometry_msgs.Twist")
        assert topic.topic == "dimos/cmd_vel"
        assert topic.lcm_type is Twist

    def test_key_expr_to_topic_with_unknown_type(self) -> None:
        topic = _key_expr_to_topic("dimos/data/unknown.FooBar")
        # Last segment doesn't resolve — entire string becomes the topic
        assert topic.topic == "dimos/data/unknown.FooBar"
        assert topic.lcm_type is None

    def test_key_expr_to_topic_with_no_slash(self) -> None:
        topic = _key_expr_to_topic("simple_topic")
        assert topic.topic == "simple_topic"
        assert topic.lcm_type is None

    def test_key_expr_to_topic_uses_default_type(self) -> None:
        topic = _key_expr_to_topic("dimos/data", default_lcm_type=Twist)
        assert topic.topic == "dimos/data"
        assert topic.lcm_type is Twist

    def test_round_trip_typed(self) -> None:
        original = Topic("dimos/color_image", lcm_type=Image)
        key = _topic_to_key_expr(original)
        reconstructed = _key_expr_to_topic(key)
        assert reconstructed.topic == original.topic
        assert reconstructed.lcm_type is original.lcm_type

    def test_round_trip_untyped(self) -> None:
        original = Topic("dimos/gps_location")
        key = _topic_to_key_expr(original)
        reconstructed = _key_expr_to_topic(key)
        assert reconstructed.topic == original.topic
        assert reconstructed.lcm_type is None
