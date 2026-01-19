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

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.rospubsub import DimosROS, ROSTopic
from dimos.protocol.pubsub.rospubsub_conversion import (
    derive_ros_type,
    dimos_to_ros,
    ros_to_dimos,
)
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


def test_pointcloud2_ros_conversion():
    """Test PointCloud2 conversion using ros_to_dimos and dimos_to_ros."""

    dir_name = get_data("unitree_go2_bigoffice")

    # Load real lidar data from replay (5 seconds in)
    replay = TimedSensorReplay(f"{dir_name}/lidar")
    original = replay.find_closest_seek(5.0)

    print(original)
    assert original is not None, "Failed to load lidar data from replay"
    assert len(original) > 0, "Loaded empty pointcloud"

    # Get the ROS type
    ros_type = derive_ros_type(PointCloud2)

    # Convert dimos -> ROS
    ros_msg = dimos_to_ros(original, ros_type)

    # Verify ROS message properties
    assert ros_msg.header.frame_id == original.frame_id
    assert ros_msg.width == len(original)
    assert ros_msg.height == 1
    assert ros_msg.point_step >= 12  # At least x, y, z (3 floats)
    assert len(ros_msg.data) > 0

    # Convert ROS -> dimos
    converted = ros_to_dimos(ros_msg, PointCloud2)

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
        err_msg="Points don't match after ROS roundtrip conversion",
    )

    # Verify frame_id is preserved
    assert converted.frame_id == original.frame_id, (
        f"Frame ID mismatch: '{original.frame_id}' vs '{converted.frame_id}'"
    )

    # Verify timestamp is preserved (within 1ms tolerance for nanosecond precision)
    assert abs(original.ts - converted.ts) < 0.001, (
        f"Timestamp mismatch: {original.ts} vs {converted.ts}"
    )


def test_pointcloud2_empty_cloud():
    """Test empty PointCloud2 conversion."""
    # Create empty point cloud
    original = PointCloud2.from_numpy(
        np.array([]).reshape(0, 3),
        frame_id="empty_frame",
        timestamp=1234567890.0,
    )

    # Get ROS type and convert
    ros_type = derive_ros_type(PointCloud2)
    ros_msg = dimos_to_ros(original, ros_type)

    # Verify empty cloud properties
    assert ros_msg.width == 0 or ros_msg.height == 0

    # Convert back
    converted = ros_to_dimos(ros_msg, PointCloud2)
    assert len(converted) == 0


def test_vector3_simple_conversion():
    """Test Vector3 simple type conversion (no LCM roundtrip)."""
    # Create dimos Vector3
    original = Vector3(1.5, 2.5, 3.5)

    # Get ROS type and convert
    ros_type = derive_ros_type(Vector3)
    ros_msg = dimos_to_ros(original, ros_type)

    # Verify ROS message
    assert ros_msg.x == 1.5
    assert ros_msg.y == 2.5
    assert ros_msg.z == 3.5

    # Convert back
    converted = ros_to_dimos(ros_msg, Vector3)
    assert converted.x == 1.5
    assert converted.y == 2.5
    assert converted.z == 3.5
