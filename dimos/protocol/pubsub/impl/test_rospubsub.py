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

"""Generated CDR streams through the optional ROS2 pub/sub bridge."""

from collections.abc import Generator
import threading
import uuid

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist, Vector3
from dimos_generated.sensor_msgs.msg import Image, PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.protocol.pubsub.impl.rospubsub import ROS_AVAILABLE, DimosROS, ROSTopic

pytestmark = pytest.mark.skipif(not ROS_AVAILABLE, reason="requires ROS 2")


@pytest.fixture()
def nodes() -> Generator[tuple[DimosROS, DimosROS], None, None]:
    publisher, subscriber = DimosROS(), DimosROS()
    try:
        publisher.start()
        subscriber.start()
        yield publisher, subscriber
    finally:
        subscriber.stop()
        publisher.stop()


@pytest.mark.parametrize(
    "original",
    [
        Vector3(x=1.0, y=2.0, z=3.0),
        Twist(linear=Vector3(x=1.0), angular=Vector3(z=-0.5)),
        PoseStamped(header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")),
        PointStamped(header=Header(stamp=Time(sec=-1, nanosec=999999999)), point=Point(x=1.5)),
        PointCloud2(),
        PointCloud2(
            height=1,
            width=1,
            fields=[PointField(name="x", datatype=7, count=1)],
            point_step=4,
            row_step=4,
            data=bytes([0, 0, 128, 63]),
        ),
        Image(height=1, width=2, encoding="rgb8", step=6, data=bytes([0, 1, 2, 253, 254, 255])),
    ],
    ids=lambda msg: msg.msg_name,
)
def test_generated_pubsub(nodes, original):
    publisher, subscriber = nodes
    topic = ROSTopic(f"/dimos_cdr_test_{uuid.uuid4().hex}", type(original))
    received = threading.Event()
    values = []

    def collect(message, _topic):
        values.append(message)
        received.set()

    unsubscribe = subscriber.subscribe(topic, collect)
    node = publisher._raw._node
    assert node is not None
    # A live periodic source tolerates asynchronous DDS discovery without a fixed sleep.
    timer = node.create_timer(0.05, lambda: publisher.publish(topic, original))
    try:
        assert received.wait(10), "ROS subscription did not receive the generated message"
        assert values[0] == original
    finally:
        node.destroy_timer(timer)
        unsubscribe()
