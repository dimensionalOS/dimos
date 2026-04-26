#!/usr/bin/env python3
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

"""Grid tests for subscribe_all pattern subscriptions."""

from collections.abc import Callable, Generator
from contextlib import AbstractContextManager, contextmanager
from dataclasses import dataclass, field
import re
import time
from typing import Any, Generic, TypeVar

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, LCMPubSubBase, Topic
from dimos.protocol.pubsub.patterns import Glob
from dimos.protocol.pubsub.spec import AllPubSub, PubSub
from dimos.utils.testing.collector import CallbackCollector

TopicT = TypeVar("TopicT")
MsgT = TypeVar("MsgT")

# Type alias for (publisher, subscriber) tuple
PubSubPair = tuple[PubSub[TopicT, MsgT], AllPubSub[TopicT, MsgT]]


@dataclass
class Case(Generic[TopicT, MsgT]):
    """Test case for grid testing pubsub implementations."""

    name: str
    pubsub_context: Callable[[], AbstractContextManager[PubSubPair[TopicT, MsgT]]]
    topic_values: list[tuple[TopicT, MsgT]]
    tags: set[str] = field(default_factory=set)
    # Pattern tests: (pattern_topic, {indices of topic_values that should match})
    glob_patterns: list[tuple[TopicT, set[int]]] = field(default_factory=list)
    regex_patterns: list[tuple[TopicT, set[int]]] = field(default_factory=list)


# Use an isolated multicast group to avoid cross-test LCM contamination.
_ISOLATED_LCM_URL = "udpm://239.255.76.99:7699?ttl=0"


@contextmanager
def lcm_typed_context() -> Generator[tuple[LCM, LCM], None, None]:
    pub = LCM(url=_ISOLATED_LCM_URL)
    sub = LCM(url=_ISOLATED_LCM_URL)
    pub.start()
    sub.start()
    try:
        yield pub, sub
    finally:
        pub.stop()
        sub.stop()


@contextmanager
def lcm_bytes_context() -> Generator[tuple[LCMPubSubBase, LCMPubSubBase], None, None]:
    pub = LCMPubSubBase(url=_ISOLATED_LCM_URL)
    sub = LCMPubSubBase(url=_ISOLATED_LCM_URL)
    pub.start()
    sub.start()
    try:
        yield pub, sub
    finally:
        pub.stop()
        sub.stop()


testcases: list[Case[Any, Any]] = [
    Case(
        name="lcm_typed",
        pubsub_context=lcm_typed_context,
        topic_values=[
            (Topic("/sensor/position", Vector3), Vector3(1, 2, 3)),
            (Topic("/sensor/orientation", Quaternion), Quaternion(0, 0, 0, 1)),
            (Topic("/robot/arm", Pose), Pose(Vector3(4, 5, 6), Quaternion(0, 0, 0, 1))),
        ],
        tags={"all", "glob", "regex"},
        glob_patterns=[
            (Topic(topic=Glob("/sensor/*")), {0, 1}),
            (Topic(topic=Glob("/**/arm")), {2}),
            (Topic(topic=Glob("/**")), {0, 1, 2}),
        ],
        regex_patterns=[
            (Topic(re.compile(r"/sensor/.*")), {0, 1}),
            (Topic(re.compile(r".*/arm"), Pose), {2}),
            (Topic(re.compile(r".*/arm")), {2}),
            (Topic(re.compile(r".*/arm#geometry.*")), {2}),
        ],
    ),
    Case(
        name="lcm_bytes",
        pubsub_context=lcm_bytes_context,
        topic_values=[
            (Topic("/sensor/temp"), b"temp"),
            (Topic("/sensor/humidity"), b"humidity"),
            (Topic("/robot/arm"), b"arm"),
        ],
        tags={"all", "glob", "regex"},
        glob_patterns=[
            (Topic(topic=Glob("/sensor/*")), {0, 1}),
            (Topic(topic=Glob("/**/arm")), {2}),
            (Topic(topic=Glob("/**")), {0, 1, 2}),
        ],
        regex_patterns=[
            (Topic(re.compile(r"/sensor/.*")), {0, 1}),
            (Topic(re.compile(r".*/arm")), {2}),
        ],
    ),
]

# Build filtered lists for parametrize
all_cases = [c for c in testcases if "all" in c.tags]
glob_cases = [c for c in testcases if "glob" in c.tags]
regex_cases = [c for c in testcases if "regex" in c.tags]


def _topic_matches_prefix(topic: Any, prefix: str = "/") -> bool:
    """Check if topic string starts with prefix.

    LCM uses UDP multicast, so messages from other tests running in parallel
    can leak into subscribe_all callbacks. We filter to only our test topics.
    """
    topic_str = str(topic.topic if hasattr(topic, "topic") else topic)
    return topic_str.startswith(prefix)


@pytest.mark.parametrize("tc", all_cases, ids=lambda c: c.name)
def test_subscribe_all_receives_all_topics(tc: Case[Any, Any]) -> None:
    """Test that subscribe_all receives messages from all topics."""
    collector = CallbackCollector(len(tc.topic_values))

    with tc.pubsub_context() as (pub, sub):
        sub.subscribe_all(collector)
        time.sleep(0.01)  # Allow subscription to register

        for topic, value in tc.topic_values:
            pub.publish(topic, value)

        collector.wait()

        assert len(collector.results) == len(tc.topic_values)

        received_msgs = [r[0] for r in collector.results]
        expected_msgs = [v for _, v in tc.topic_values]
        for expected in expected_msgs:
            assert expected in received_msgs


@pytest.mark.parametrize("tc", all_cases, ids=lambda c: c.name)
def test_subscribe_all_unsubscribe(tc: Case[Any, Any]) -> None:
    """Test that unsubscribe stops receiving messages."""
    collector = CallbackCollector(1)
    topic, value = tc.topic_values[0]

    with tc.pubsub_context() as (pub, sub):
        unsub = sub.subscribe_all(collector)
        time.sleep(0.01)  # Allow subscription to register

        pub.publish(topic, value)
        collector.wait()
        assert len(collector.results) == 1

        unsub()

        pub.publish(topic, value)
        time.sleep(0.1)  # Wait to confirm no new messages arrive
        assert len(collector.results) == 1  # No new messages


@pytest.mark.parametrize("tc", all_cases, ids=lambda c: c.name)
def test_subscribe_all_with_regular_subscribe(tc: Case[Any, Any]) -> None:
    """Test that subscribe_all coexists with regular subscriptions."""
    all_collector = CallbackCollector(2)
    specific_received: list[tuple[Any, Any]] = []
    topic1, value1 = tc.topic_values[0]
    topic2, value2 = tc.topic_values[1]

    with tc.pubsub_context() as (pub, sub):
        sub.subscribe_all(
            lambda msg, topic: all_collector(msg, topic) if _topic_matches_prefix(topic) else None
        )
        sub.subscribe(topic1, lambda msg, topic: specific_received.append((msg, topic)))
        time.sleep(0.01)  # Allow subscriptions to register

        pub.publish(topic1, value1)
        pub.publish(topic2, value2)
        all_collector.wait()

        # subscribe_all gets both
        assert len(all_collector.results) == 2

        # specific subscription gets only topic1
        assert len(specific_received) == 1
        assert specific_received[0][0] == value1


@pytest.mark.parametrize("tc", glob_cases, ids=lambda c: c.name)
def test_subscribe_glob(tc: Case[Any, Any]) -> None:
    """Test that glob pattern subscriptions receive only matching topics."""
    for pattern_topic, expected_indices in tc.glob_patterns:
        collector = CallbackCollector(len(expected_indices))

        with tc.pubsub_context() as (pub, sub):
            sub.subscribe(pattern_topic, collector)
            time.sleep(0.01)  # Allow subscription to register

            for topic, value in tc.topic_values:
                pub.publish(topic, value)

            collector.wait()

            assert len(collector.results) == len(expected_indices), (
                f"Expected {len(expected_indices)} messages for pattern {pattern_topic}, "
                f"got {len(collector.results)}"
            )

            expected_msgs = [tc.topic_values[i][1] for i in expected_indices]
            received_msgs = [r[0] for r in collector.results]
            for expected in expected_msgs:
                assert expected in received_msgs


@pytest.mark.parametrize("tc", regex_cases, ids=lambda c: c.name)
def test_subscribe_regex(tc: Case[Any, Any]) -> None:
    """Test that regex pattern subscriptions receive only matching topics."""
    for pattern_topic, expected_indices in tc.regex_patterns:
        collector = CallbackCollector(len(expected_indices))

        with tc.pubsub_context() as (pub, sub):
            sub.subscribe(pattern_topic, collector)
            time.sleep(0.01)  # Allow subscription to register

            for topic, value in tc.topic_values:
                pub.publish(topic, value)

            collector.wait()

            assert len(collector.results) == len(expected_indices), (
                f"Expected {len(expected_indices)} messages for pattern {pattern_topic}, "
                f"got {len(collector.results)}"
            )

            expected_msgs = [tc.topic_values[i][1] for i in expected_indices]
            received_msgs = [r[0] for r in collector.results]
            for expected in expected_msgs:
                assert expected in received_msgs
