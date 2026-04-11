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

from collections.abc import Generator
import time
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.lcmpubsub import (
    LCM,
    LCMPubSubBase,
    PickleLCM,
    Topic,
)
from dimos.utils.testing.collector import CallbackCollector

# Isolated multicast group so stale messages from other tests
# (which use the default 239.255.76.67:7667) don't leak in.
_ISOLATED_LCM_URL = "udpm://239.255.76.98:7698?ttl=0"


@pytest.fixture
def lcm_pub_sub_base() -> Generator[LCMPubSubBase, None, None]:
    lcm = LCMPubSubBase(url=_ISOLATED_LCM_URL)
    lcm.start()
    time.sleep(0.05)  # let the handler thread enter the LCM loop
    yield lcm
    lcm.stop()


@pytest.fixture
def pickle_lcm() -> Generator[PickleLCM, None, None]:
    lcm = PickleLCM(url=_ISOLATED_LCM_URL)
    lcm.start()
    time.sleep(0.05)  # let the handler thread enter the LCM loop
    yield lcm
    lcm.stop()


@pytest.fixture
def lcm() -> Generator[LCM, None, None]:
    lcm = LCM(url=_ISOLATED_LCM_URL)
    lcm.start()
    time.sleep(0.05)  # let the handler thread enter the LCM loop
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
