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

from collections.abc import Iterator
from typing import Any

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from dimos_generated.std_msgs.msg import String
import pytest

from dimos.protocol.pubsub.impl.lcmpubsub import (
    LCM,
    LCMPubSubBase,
    PickleLCM,
    Topic,
)
from dimos.utils.testing.collector import CallbackCollector


@pytest.fixture
def lcm_pub_sub_base(lcm_url: str) -> Iterator[LCMPubSubBase]:
    lcm = LCMPubSubBase(url=lcm_url)
    lcm.start()
    yield lcm
    lcm.stop()


@pytest.fixture
def pickle_lcm(lcm_url: str) -> Iterator[PickleLCM]:
    lcm = PickleLCM(url=lcm_url)
    lcm.start()
    yield lcm
    lcm.stop()


@pytest.fixture
def lcm(lcm_url: str) -> Iterator[LCM]:
    lcm = LCM(url=lcm_url)
    lcm.start()
    yield lcm
    lcm.stop()


def test_LCMPubSubBase_pubsub(lcm_pub_sub_base: LCMPubSubBase) -> None:
    lcm = lcm_pub_sub_base
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic", msg_type=String)
    test_message = String(data="test_data")

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message.encode())
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, bytes)
    assert String.decode(received_data).data == "test_data"

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


def test_lcm_autodecoder_pubsub(lcm: LCM) -> None:
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic", msg_type=String)
    test_message = String(data="test_data")

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message)
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, String)
    assert received_data.encode() == test_message.encode()

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


def test_invalid_cdr_does_not_stop_the_subscriber(lcm: LCM) -> None:
    topic = Topic("/test_corrupt", String)
    collector = CallbackCollector(1)
    lcm.subscribe(topic, collector)

    lcm.publish(topic, b"invalid CDR")
    lcm.publish(topic, String(data="after malformed payload"))
    collector.wait()

    assert [message.data for message, _ in collector.results] == ["after malformed payload"]


def test_different_message_types_cannot_be_published_on_a_typed_topic(lcm: LCM) -> None:
    with pytest.raises(ValueError, match="does not match"):
        lcm.publish(Topic("/point", Vector3), Quaternion(w=1))


def test_explicit_unknown_type_does_not_use_default_decoder() -> None:
    topic = Topic.from_channel_str("/point#unknown_msgs/msg/Point", Vector3)

    assert topic.topic == "/point"
    assert topic.msg_type is None


@pytest.mark.parametrize("channel", ["x" * 64, "invalid\0channel", "é" * 32])
def test_invalid_lcm_channel_fails_before_sending(lcm: LCM, channel: str) -> None:
    with pytest.raises(ValueError, match="63 bytes"):
        lcm.publish(Topic(channel), b"payload")


test_msgs = [
    (Vector3(x=1, y=2, z=3)),
    (Quaternion(x=1, y=2, z=3, w=4)),
    (Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(w=1))),
]


# passes some geometry types through LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_pubsub(test_message: Any, lcm: LCM) -> None:
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic", msg_type=test_message.__class__)

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message)
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, test_message.__class__)
    assert received_data.encode() == test_message.encode()

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


# passes some geometry types through pickle LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_autopickle_pubsub(test_message: Any, pickle_lcm: PickleLCM) -> None:
    lcm = pickle_lcm
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic")

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message)
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, test_message.__class__)
    assert received_data.encode() == test_message.encode()

    assert isinstance(received_topic, Topic)
    assert received_topic == topic
