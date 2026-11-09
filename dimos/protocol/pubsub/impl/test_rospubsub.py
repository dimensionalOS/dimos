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

from collections.abc import Generator

import pytest

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    Vector3,
)
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.pubsub.impl.rospubsub import DimosROS, ROSTopic
from dimos.utils.testing.collector import CallbackCollector


@pytest.fixture()
def publisher() -> Generator[DimosROS, None, None]:
    ros = DimosROS()
    ros.start()
    try:
        yield ros
    finally:
        ros.stop()


@pytest.fixture()
def subscriber() -> Generator[DimosROS, None, None]:
    ros = DimosROS()
    ros.start()
    try:
        yield ros
    finally:
        ros.stop()


@pytest.mark.skipif_no_ros
@pytest.mark.parametrize(
    "original",
    [
        Vector3(1.0, 2.0, 3.0),
        PoseStamped(
            header=Header(stamp=Time(sec=1234567890, nanosec=123456000), frame_id="base_link"),
            pose=Pose(
                position=Point(1.0, 2.0, 3.0),
                orientation=Quaternion(0.0, 0.0, 0.7071068, 0.7071068),
            ),
        ),
        PointStamped(
            header=Header(stamp=Time(sec=1234567890, nanosec=123456000), frame_id="map"),
            point=Point(1.5, 2.5, 3.5),
        ),
        Twist(linear=Vector3(1.0, 2.0, 3.0), angular=Vector3(0.1, 0.2, 0.3)),
    ],
)
def test_generated_message_pubsub(publisher, subscriber, original):
    from dimos_cdrgen.registry import encode

    topic = ROSTopic("/test_ros_topic", type(original))
    collector = CallbackCollector(1)
    subscriber.subscribe(topic, collector)
    publisher.publish(topic, original)
    collector.wait()
    assert len(collector.results) == 1
    converted = collector.results[0][0]
    assert type(converted) is type(original)
    assert encode(converted) == encode(original)
