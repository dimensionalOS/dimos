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

import threading

import pytest

from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.impl.zenohpubsub import ZenohPubSubBase
from dimos.protocol.service.zenohservice import _sessions


@pytest.fixture()
def pubsub():
    """Create and start a ZenohPubSubBase instance, clean up after."""
    # Each test gets a fresh session to avoid thread leak detection
    for session in _sessions.values():
        session.close()
    _sessions.clear()

    ps = ZenohPubSubBase()
    ps.start()
    yield ps
    ps.stop()
    # Close sessions so Zenoh's internal threads are joined
    for session in _sessions.values():
        session.close()
    _sessions.clear()


def wait_for_delivery() -> None:
    """Wait for published messages to be delivered to subscribers."""
    import time

    time.sleep(0.1)


def wait_for_subscribers() -> None:
    """Wait for Zenoh subscriber declarations to propagate.

    Zenoh's Python API does not expose a "subscriber ready" signal or
    callback. After declare_subscriber(), there is a brief window where
    messages published to the same key expression may not be delivered.
    This is a known limitation of the peer discovery protocol — peers
    need time to exchange interest declarations over the network.
    """
    import time

    time.sleep(0.05)


class TestZenohPubSubBase:
    def test_publish_and_subscribe(self, pubsub) -> None:
        received = []
        event = threading.Event()
        topic = Topic("dimos/test/basic")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)
            event.set()

        pubsub.subscribe(topic, callback)
        wait_for_subscribers()
        pubsub.publish(topic, b"hello zenoh")

        assert event.wait(timeout=2.0), f"Timed out waiting for message (got {len(received)})"
        assert received[0] == b"hello zenoh"

    def test_multiple_subscribers(self, pubsub) -> None:
        received_a: list[bytes] = []
        received_b: list[bytes] = []
        countdown = threading.Barrier(2, action=lambda: event.set())
        event = threading.Event()
        topic = Topic("dimos/test/multi")

        def callback_a(msg: bytes, t: Topic) -> None:
            received_a.append(msg)
            countdown.wait()

        def callback_b(msg: bytes, t: Topic) -> None:
            received_b.append(msg)
            countdown.wait()

        pubsub.subscribe(topic, callback_a)
        pubsub.subscribe(topic, callback_b)
        wait_for_subscribers()
        pubsub.publish(topic, b"broadcast")

        assert event.wait(timeout=2.0), "Timed out waiting for both subscribers"
        assert received_a == [b"broadcast"]
        assert received_b == [b"broadcast"]

    def test_unsubscribe(self, pubsub) -> None:
        received: list[bytes] = []
        topic = Topic("dimos/test/unsub")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)

        unsub = pubsub.subscribe(topic, callback)
        wait_for_subscribers()
        pubsub.publish(topic, b"before")
        wait_for_delivery()
        unsub()
        wait_for_subscribers()
        pubsub.publish(topic, b"after")
        wait_for_delivery()

        assert received == [b"before"]

    def test_unsubscribe_is_idempotent(self, pubsub) -> None:
        topic = Topic("dimos/test/idempotent")
        unsub = pubsub.subscribe(topic, lambda msg, t: None)
        unsub()
        unsub()  # should not raise

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

    def test_subscribe_all(self, pubsub) -> None:
        received: list[bytes] = []
        event = threading.Event()

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)
            event.set()

        pubsub.subscribe_all(callback)
        wait_for_subscribers()
        pubsub.publish(Topic("dimos/test/any/topic"), b"wildcard")

        assert event.wait(timeout=2.0), "Timed out waiting for wildcard message"
        assert received[0] == b"wildcard"


class TestTopicKeyExprConversion:
    """Tests for _topic_to_key_expr and _key_expr_to_topic round-trip."""

    def test_typed_topic_to_key_expr(self) -> None:
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.protocol.pubsub.impl.zenohpubsub import _topic_to_key_expr

        topic = Topic("dimos/cmd_vel", lcm_type=Twist)
        key = _topic_to_key_expr(topic)
        assert key == "dimos/cmd_vel/geometry_msgs.Twist"

    def test_untyped_topic_to_key_expr(self) -> None:
        from dimos.protocol.pubsub.impl.zenohpubsub import _topic_to_key_expr

        topic = Topic("dimos/data")
        key = _topic_to_key_expr(topic)
        assert key == "dimos/data"

    def test_key_expr_to_topic_with_known_type(self) -> None:
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.protocol.pubsub.impl.zenohpubsub import _key_expr_to_topic

        topic = _key_expr_to_topic("dimos/cmd_vel/geometry_msgs.Twist")
        assert topic.topic == "dimos/cmd_vel"
        assert topic.lcm_type is Twist

    def test_key_expr_to_topic_with_unknown_type(self) -> None:
        from dimos.protocol.pubsub.impl.zenohpubsub import _key_expr_to_topic

        topic = _key_expr_to_topic("dimos/data/unknown.FooBar")
        # Last segment doesn't resolve — entire string becomes the topic
        assert topic.topic == "dimos/data/unknown.FooBar"
        assert topic.lcm_type is None

    def test_key_expr_to_topic_with_no_slash(self) -> None:
        from dimos.protocol.pubsub.impl.zenohpubsub import _key_expr_to_topic

        topic = _key_expr_to_topic("simple_topic")
        assert topic.topic == "simple_topic"
        assert topic.lcm_type is None

    def test_key_expr_to_topic_uses_default_type(self) -> None:
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.protocol.pubsub.impl.zenohpubsub import _key_expr_to_topic

        topic = _key_expr_to_topic("dimos/data", default_lcm_type=Twist)
        assert topic.topic == "dimos/data"
        assert topic.lcm_type is Twist

    def test_round_trip_typed(self) -> None:
        from dimos.msgs.sensor_msgs.Image import Image
        from dimos.protocol.pubsub.impl.zenohpubsub import (
            _key_expr_to_topic,
            _topic_to_key_expr,
        )

        original = Topic("dimos/color_image", lcm_type=Image)
        key = _topic_to_key_expr(original)
        reconstructed = _key_expr_to_topic(key)
        assert reconstructed.topic == original.topic
        assert reconstructed.lcm_type is original.lcm_type

    def test_round_trip_untyped(self) -> None:
        from dimos.protocol.pubsub.impl.zenohpubsub import (
            _key_expr_to_topic,
            _topic_to_key_expr,
        )

        original = Topic("dimos/gps_location")
        key = _topic_to_key_expr(original)
        reconstructed = _key_expr_to_topic(key)
        assert reconstructed.topic == original.topic
        assert reconstructed.lcm_type is None
