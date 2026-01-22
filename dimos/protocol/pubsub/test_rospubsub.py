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
import threading
import time

from dimos_lcm.geometry_msgs import PointStamped
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.rospubsub import DimosROS, ROSTopic

# Add msg_name to LCM PointStamped for testing nested message conversion
PointStamped.msg_name = "geometry_msgs.PointStamped"
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


def ros_node():
    ros = DimosROS()
    ros.start()
    try:
        yield ros
    finally:
        ros.stop()


@pytest.fixture()
def publisher() -> Generator[DimosROS, None, None]:
    yield from ros_node()


@pytest.fixture()
def subscriber() -> Generator[DimosROS, None, None]:
    yield from ros_node()


@pytest.mark.ros
def test_basic_conversion(publisher, subscriber):
    """Test Vector3 publish/subscribe through ROS.

    Simple flat dimos.msgs type with no nesting (just x/y/z floats).
    """
    topic = ROSTopic("/test_ros_topic", Vector3)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    publisher.publish(topic, Vector3(1.0, 2.0, 3.0))

    assert event.wait(timeout=2.0), "No message received"
    assert len(received) == 1
    msg = received[0]
    assert msg.x == 1.0
    assert msg.y == 2.0
    assert msg.z == 3.0


@pytest.mark.ros
def test_pointcloud2_pubsub(publisher, subscriber):
    """Test PointCloud2 publish/subscribe through ROS.

    COMPLEX_TYPE - has non-standard attributes (numpy arrays, custom accessors)
    that can't be treated like a standard message with direct field copy.
    Uses LCM encode/decode roundtrip to properly convert internal representation.
    """
    dir_name = get_data("unitree_go2_bigoffice")

    # Load real lidar data from replay (5 seconds in)
    replay = TimedSensorReplay(f"{dir_name}/lidar")
    original = replay.find_closest_seek(5.0)

    assert original is not None, "Failed to load lidar data from replay"
    assert len(original) > 0, "Loaded empty pointcloud"

    topic = ROSTopic("/test_pointcloud2", PointCloud2)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    publisher.publish(topic, original)

    assert event.wait(timeout=5.0), "No PointCloud2 message received"
    assert len(received) == 1

    converted = received[0]

    # Verify point cloud data is preserved
    original_points, _ = original.as_numpy()
    converted_points, _ = converted.as_numpy()

    assert len(original_points) == len(converted_points), (
        f"Point count mismatch: {len(original_points)} vs {len(converted_points)}"
    )

    np.testing.assert_allclose(
        original_points,
        converted_points,
        rtol=1e-5,
        atol=1e-5,
        err_msg="Points don't match after ROS pubsub roundtrip",
    )

    # Verify frame_id is preserved
    assert converted.frame_id == original.frame_id

    # Verify timestamp is preserved (within 1ms tolerance)
    assert abs(original.ts - converted.ts) < 0.001


@pytest.mark.ros
def test_pointcloud2_empty_pubsub(publisher, subscriber):
    """Test empty PointCloud2 publish/subscribe.

    Edge case for COMPLEX_TYPE with zero points.
    """
    original = PointCloud2.from_numpy(
        np.array([]).reshape(0, 3),
        frame_id="empty_frame",
        timestamp=1234567890.0,
    )

    topic = ROSTopic("/test_empty_pointcloud", PointCloud2)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    publisher.publish(topic, original)

    assert event.wait(timeout=2.0), "No empty PointCloud2 message received"
    assert len(received) == 1
    assert len(received[0]) == 0


@pytest.mark.ros
def test_posestamped_pubsub(publisher, subscriber):
    """Test PoseStamped publish/subscribe through ROS.

    COMPLEX_TYPE with custom dimos.msgs implementation and nested messages
    (Header, Pose containing Point and Quaternion). Uses LCM roundtrip.
    """
    original = PoseStamped(
        ts=1234567890.123456,
        frame_id="base_link",
        position=[1.0, 2.0, 3.0],
        orientation=[0.0, 0.0, 0.7071068, 0.7071068],  # 90 degree yaw
    )

    topic = ROSTopic("/test_posestamped", PoseStamped)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    publisher.publish(topic, original)

    assert event.wait(timeout=2.0), "No PoseStamped message received"
    assert len(received) == 1

    converted = received[0]

    # Verify all fields preserved
    assert converted.frame_id == original.frame_id
    assert abs(converted.ts - original.ts) < 0.001  # 1ms tolerance
    assert converted.x == original.x
    assert converted.y == original.y
    assert converted.z == original.z
    np.testing.assert_allclose(converted.orientation.z, original.orientation.z, rtol=1e-5)
    np.testing.assert_allclose(converted.orientation.w, original.orientation.w, rtol=1e-5)


@pytest.mark.ros
def test_pointstamped_pubsub(publisher, subscriber):
    """Test PointStamped publish/subscribe through ROS.

    Raw LCM type with nested messages (Header, Point) but NO custom dimos.msgs
    implementation. Tests recursive field copy for non-COMPLEX_TYPES.
    """
    original = PointStamped()
    original.header.stamp.sec = 1234567890
    original.header.stamp.nsec = 123456000
    original.header.frame_id = "map"
    original.point.x = 1.5
    original.point.y = 2.5
    original.point.z = 3.5

    topic = ROSTopic("/test_pointstamped", PointStamped)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    publisher.publish(topic, original)

    assert event.wait(timeout=2.0), "No PointStamped message received"
    assert len(received) == 1

    converted = received[0]

    # Verify nested header fields are preserved
    assert converted.header.frame_id == original.header.frame_id
    assert converted.header.stamp.sec == original.header.stamp.sec
    assert converted.header.stamp.nsec == original.header.stamp.nsec

    # Verify point coordinates are preserved
    assert converted.point.x == original.point.x
    assert converted.point.y == original.point.y
    assert converted.point.z == original.point.z


@pytest.mark.ros
def test_twist_pubsub(publisher, subscriber):
    """Test Twist publish/subscribe through ROS.

    dimos.msgs type with nested Vector3 messages (linear, angular).
    Tests recursive field copy with custom dimos.msgs nested types.
    """
    original = Twist(
        linear=[1.0, 2.0, 3.0],
        angular=[0.1, 0.2, 0.3],
    )

    topic = ROSTopic("/test_twist", Twist)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    publisher.publish(topic, original)

    assert event.wait(timeout=2.0), "No Twist message received"
    assert len(received) == 1

    converted = received[0]

    # Verify linear velocity preserved
    assert converted.linear.x == original.linear.x
    assert converted.linear.y == original.linear.y
    assert converted.linear.z == original.linear.z

    # Verify angular velocity preserved
    assert converted.angular.x == original.angular.x
    assert converted.angular.y == original.angular.y
    assert converted.angular.z == original.angular.z
