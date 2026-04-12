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
import time

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


class TestZenohPubSubBase:
    def test_publish_and_subscribe(self, pubsub) -> None:
        received = []
        event = threading.Event()
        topic = Topic("dimos/test/basic")

        def callback(msg: bytes, t: Topic) -> None:
            received.append(msg)
            event.set()

        pubsub.subscribe(topic, callback)
        time.sleep(0.05)  # let subscriber register
        pubsub.publish(topic, b"hello zenoh")

        assert event.wait(timeout=2.0), f"Timed out waiting for message (got {len(received)})"
        assert received[0] == b"hello zenoh"

    def test_multiple_subscribers(self, pubsub) -> None:
        received_a: list[bytes] = []
        received_b: list[bytes] = []
        event = threading.Event()
        topic = Topic("dimos/test/multi")

        def callback_a(msg: bytes, t: Topic) -> None:
            received_a.append(msg)
            if received_a and received_b:
                event.set()

        def callback_b(msg: bytes, t: Topic) -> None:
            received_b.append(msg)
            if received_a and received_b:
                event.set()

        pubsub.subscribe(topic, callback_a)
        pubsub.subscribe(topic, callback_b)
        time.sleep(0.05)
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
        time.sleep(0.05)
        pubsub.publish(topic, b"before")
        time.sleep(0.1)
        unsub()
        time.sleep(0.05)
        pubsub.publish(topic, b"after")
        time.sleep(0.1)

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
        time.sleep(0.05)
        pubsub.publish(Topic("dimos/test/any/topic"), b"wildcard")

        assert event.wait(timeout=2.0), "Timed out waiting for wildcard message"
        assert received[0] == b"wildcard"
