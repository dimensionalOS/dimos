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

from collections.abc import Callable, Iterator
import re
import threading
from typing import Any

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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


class MockLCMMessage:
    """Mock LCM message for testing"""

    msg_name = "geometry_msgs.Mock"

    def __init__(self, data: Any) -> None:
        self.data = data

    def lcm_encode(self) -> bytes:
        return str(self.data).encode("utf-8")

    @classmethod
    def lcm_decode(cls, data: bytes) -> "MockLCMMessage":
        return cls(data.decode("utf-8"))

    def __eq__(self, other: object) -> bool:
        return isinstance(other, MockLCMMessage) and self.data == other.data


def _image_message() -> Image:
    return Image(np.array([[[1, 2, 3]]], dtype=np.uint8), frame_id="camera", ts=1.0)


def _pointcloud_message() -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32), frame_id="lidar", timestamp=1.0
    )


def test_LCMPubSubBase_pubsub(lcm_pub_sub_base: LCMPubSubBase) -> None:
    lcm = lcm_pub_sub_base
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("test_data")

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message.lcm_encode())
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, bytes)
    assert received_data.decode() == "test_data"

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


def test_lcm_autodecoder_pubsub(lcm: LCM) -> None:
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("test_data")

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message)
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, MockLCMMessage)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


test_msgs = [
    (Vector3(1, 2, 3)),
    (Quaternion(1, 2, 3, 4)),
    (Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))),
]


# passes some geometry types through LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_pubsub(test_message: Any, lcm: LCM) -> None:
    collector = CallbackCollector(1)

    topic = Topic(topic="/test_topic", lcm_type=test_message.__class__)

    lcm.subscribe(topic, collector)
    lcm.publish(topic, test_message)
    collector.wait()

    assert len(collector.results) == 1

    received_data = collector.results[0][0]
    received_topic = collector.results[0][1]

    assert isinstance(received_data, test_message.__class__)
    assert received_data == test_message

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
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


@pytest.mark.parametrize("make_message", (_image_message, _pointcloud_message))
@pytest.mark.parametrize("subscription", ("fixed", "pattern", "all"))
def test_clause_2_typed_lcm_heavy_delivery_enters_callback_with_wire_bytes(
    lcm: LCM,
    make_message: Callable[[], Image | PointCloud2],
    mocker,
    subscription: str,
    retry_until,
) -> None:
    """Clause 2 DIM-1125: typed LCM heavy subscriptions receive bytes before decode."""
    message = make_message()
    publisher_topic = Topic("/clause2/heavy", type(message))
    callback_decode_counts: list[int] = []
    payloads: list[Any] = []
    received = threading.Event()
    decode = mocker.spy(type(message), "lcm_decode")

    def callback(payload: Any, _: Topic) -> None:
        callback_decode_counts.append(decode.call_count)
        payloads.append(payload)
        received.set()

    if subscription == "all":
        lcm.subscribe_all(
            callback, lambda received_topic: received_topic.topic == publisher_topic.topic
        )
    else:
        subscriber_topic = (
            Topic(re.compile("/clause2/heavy")) if subscription == "pattern" else publisher_topic
        )
        lcm.subscribe(subscriber_topic, callback)
    retry_until(received, lambda: lcm.publish(publisher_topic, message))

    assert callback_decode_counts == [0]
    assert isinstance(payloads[0], bytes)
    assert type(message).lcm_decode(payloads[0]).lcm_encode() == message.lcm_encode()
    assert decode.call_count == 1
