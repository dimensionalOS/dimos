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

from collections.abc import Callable
from dataclasses import dataclass
import threading
from typing import Any, ClassVar, Literal

from dimos.msgs.visualization_msgs.SonicPoseReference import SonicPoseReference
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.visualization.rerun.bridge import (
    _LatestOnlyDispatcher,
    _subscribe_topics,
)


@dataclass(frozen=True)
class _Frame:
    index: int


class _FakePubSub:
    class Config:
        transport: ClassVar[Literal["lcm"]] = "lcm"

    config = Config()

    def __init__(self) -> None:
        self.subscribed: list[Topic] = []
        self.subscribe_all_calls = 0

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        self.subscribed.append(topic)
        return lambda: None

    def subscribe_all(self, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        self.subscribe_all_calls += 1
        return lambda: None


class _FakeZenohPubSub(_FakePubSub):
    class Config:
        transport: ClassVar[Literal["zenoh"]] = "zenoh"

    config = Config()


def test_live_dispatcher_keeps_only_newest_pending_message() -> None:
    first_started = threading.Event()
    release_first = threading.Event()
    second_finished = threading.Event()
    received: list[int] = []

    def consume(frame: _Frame, topic: str) -> None:
        received.append(frame.index)
        if frame.index == 0:
            first_started.set()
            assert release_first.wait(timeout=1.0)
        else:
            second_finished.set()

    dispatcher = _LatestOnlyDispatcher(consume, min_interval=lambda topic: 0.02)
    dispatcher.start()
    try:
        dispatcher.submit(_Frame(0), "/pose")
        assert first_started.wait(timeout=1.0)

        for index in range(1, 100):
            dispatcher.submit(_Frame(index), "/pose")

        release_first.set()
        assert second_finished.wait(timeout=1.0)
    finally:
        dispatcher.stop()

    assert received == [0, 99]


def test_live_topics_use_one_exact_bounded_subscription() -> None:
    pubsub = _FakePubSub()

    unsubscribes = _subscribe_topics(
        pubsub,
        {"sonic_pose_reference": SonicPoseReference.msg_name},
        lambda msg, topic: None,
        latest_only=True,
    )

    assert len(unsubscribes) == 1
    assert pubsub.subscribe_all_calls == 0
    assert pubsub.subscribed == [
        Topic(
            topic="/sonic_pose_reference",
            lcm_type=SonicPoseReference,
            queue_capacity=1,
        )
    ]


def test_default_topics_retain_wildcard_subscription() -> None:
    pubsub = _FakePubSub()

    unsubscribes = _subscribe_topics(
        pubsub,
        None,
        lambda msg, topic: None,
        latest_only=False,
    )

    assert len(unsubscribes) == 1
    assert pubsub.subscribe_all_calls == 1
    assert pubsub.subscribed == []


def test_live_topics_use_active_zenoh_namespace() -> None:
    pubsub = _FakeZenohPubSub()

    _subscribe_topics(
        pubsub,
        {"sonic_pose_reference": SonicPoseReference.msg_name},
        lambda msg, topic: None,
        latest_only=True,
    )

    assert len(pubsub.subscribed) == 1
    topic = pubsub.subscribed[0]
    assert topic.topic == "dimos/sonic_pose_reference"
    assert topic.lcm_type is SonicPoseReference
    assert topic.queue_capacity == 1
