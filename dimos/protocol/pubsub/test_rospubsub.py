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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.rospubsub import DimosROS, ROSTopic
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
    topic = ROSTopic("/test_ros_topic", Vector3)

    received = []
    event = threading.Event()

    def callback(msg, t):
        received.append(msg)
        event.set()

    subscriber.subscribe(topic, callback)
    time.sleep(0.1)  # let subscription establish
    publisher.publish(topic, Vector3(1.0, 2.0, 3.0))

    assert event.wait(timeout=2.0), "No message received"
    assert len(received) == 1
    msg = received[0]
    assert msg.x == 1.0
    assert msg.y == 2.0
    assert msg.z == 3.0


@pytest.mark.ros
def test_pointcloud2_pubsub(publisher, subscriber):
    """Test PointCloud2 publish/subscribe through ROS."""
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
    time.sleep(0.1)  # let subscription establish
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
    """Test empty PointCloud2 publish/subscribe."""
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
    time.sleep(0.1)
    publisher.publish(topic, original)

    assert event.wait(timeout=2.0), "No empty PointCloud2 message received"
    assert len(received) == 1
    assert len(received[0]) == 0


@pytest.mark.ros
def test_posestamped_pubsub(publisher, subscriber):
    """Test PoseStamped publish/subscribe through ROS."""
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
    time.sleep(0.1)
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
