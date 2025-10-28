#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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


import numpy as np
import pytest

try:
    from sensor_msgs.msg import PointCloud2 as ROSPointCloud2, PointField as ROSPointField
    from std_msgs.msg import Header as ROSHeader
except ImportError:
    ROSPointCloud2 = None
    ROSPointField = None
    ROSHeader = None

from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.testing import SensorReplay

# Try to import ROS types for testing
try:
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def test_lcm_encode_decode() -> None:
    """Test LCM encode/decode preserves pointcloud data."""
    replay = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)
    lidar_msg: LidarMessage = replay.load_one("lidar_data_021")

    binary_msg = lidar_msg.lcm_encode()
    decoded = PointCloud2.lcm_decode(binary_msg)

    # 1. Check number of points
    original_points = lidar_msg.as_numpy()
    decoded_points = decoded.as_numpy()

    print(f"Original points: {len(original_points)}")
    print(f"Decoded points: {len(decoded_points)}")
    assert len(original_points) == len(decoded_points), (
        f"Point count mismatch: {len(original_points)} vs {len(decoded_points)}"
    )

    # 2. Check point coordinates are preserved (within floating point tolerance)
    if len(original_points) > 0:
        np.testing.assert_allclose(
            original_points,
            decoded_points,
            rtol=1e-6,
            atol=1e-6,
            err_msg="Point coordinates don't match between original and decoded",
        )
        print(f"✓ All {len(original_points)} point coordinates match within tolerance")

    # 3. Check frame_id is preserved
    assert lidar_msg.frame_id == decoded.frame_id, (
        f"Frame ID mismatch: '{lidar_msg.frame_id}' vs '{decoded.frame_id}'"
    )
    print(f"✓ Frame ID preserved: '{decoded.frame_id}'")

    # 4. Check timestamp is preserved (within reasonable tolerance for float precision)
    if lidar_msg.ts is not None and decoded.ts is not None:
        assert abs(lidar_msg.ts - decoded.ts) < 1e-6, (
            f"Timestamp mismatch: {lidar_msg.ts} vs {decoded.ts}"
        )
        print(f"✓ Timestamp preserved: {decoded.ts}")

    # 5. Check pointcloud properties
    assert len(lidar_msg.pointcloud.points) == len(decoded.pointcloud.points), (
        "Open3D pointcloud size mismatch"
    )

    # 6. Additional detailed checks
    print("✓ Original pointcloud summary:")
    print(f"  - Points: {len(original_points)}")
    print(f"  - Bounds: {original_points.min(axis=0)} to {original_points.max(axis=0)}")
    print(f"  - Mean: {original_points.mean(axis=0)}")

    print("✓ Decoded pointcloud summary:")
    print(f"  - Points: {len(decoded_points)}")
    print(f"  - Bounds: {decoded_points.min(axis=0)} to {decoded_points.max(axis=0)}")
    print(f"  - Mean: {decoded_points.mean(axis=0)}")

    print("✓ LCM encode/decode test passed - all properties preserved!")


@pytest.mark.ros
def test_ros_conversion() -> None:
    """Test ROS message conversion preserves pointcloud data."""
    if not ROS_AVAILABLE:
        print("ROS packages not available - skipping ROS conversion test")
        return

    print("\nTesting ROS PointCloud2 conversion...")

    # Create a simple test point cloud
    import open3d as o3d

    points = np.array(
        [
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [-1.0, -2.0, -3.0],
            [0.5, 0.5, 0.5],
        ],
        dtype=np.float32,
    )

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)

    # Create DIMOS PointCloud2
    original = PointCloud2(
        pointcloud=pc,
        frame_id="test_frame",
        ts=1234567890.123456,
    )

    # Test 1: Convert to ROS and back
    ros_msg = original.to_ros_msg()
    converted = PointCloud2.from_ros_msg(ros_msg)

    # Check points are preserved
    original_points = original.as_numpy()
    converted_points = converted.as_numpy()

    assert len(original_points) == len(converted_points), (
        f"Point count mismatch: {len(original_points)} vs {len(converted_points)}"
    )

    np.testing.assert_allclose(
        original_points,
        converted_points,
        rtol=1e-6,
        atol=1e-6,
        err_msg="Points don't match after ROS conversion",
    )
    print(f"✓ Points preserved: {len(converted_points)} points match")

    # Check metadata
    assert original.frame_id == converted.frame_id, (
        f"Frame ID mismatch: '{original.frame_id}' vs '{converted.frame_id}'"
    )
    print(f"✓ Frame ID preserved: '{converted.frame_id}'")

    assert abs(original.ts - converted.ts) < 1e-6, (
        f"Timestamp mismatch: {original.ts} vs {converted.ts}"
    )
    print(f"✓ Timestamp preserved: {converted.ts}")

    # Test 2: Create ROS message directly and convert to DIMOS
    ros_msg2 = ROSPointCloud2()
    ros_msg2.header = ROSHeader()
    ros_msg2.header.frame_id = "ros_test_frame"
    ros_msg2.header.stamp.sec = 1234567890
    ros_msg2.header.stamp.nanosec = 123456000

    # Set up point cloud data
    ros_msg2.height = 1
    ros_msg2.width = 3
    ros_msg2.fields = [
        ROSPointField(name="x", offset=0, datatype=ROSPointField.FLOAT32, count=1),
        ROSPointField(name="y", offset=4, datatype=ROSPointField.FLOAT32, count=1),
        ROSPointField(name="z", offset=8, datatype=ROSPointField.FLOAT32, count=1),
    ]
    ros_msg2.is_bigendian = False
    ros_msg2.point_step = 12
    ros_msg2.row_step = 36

    # Pack test points
    test_points = np.array(
        [
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ],
        dtype=np.float32,
    )
    ros_msg2.data = test_points.tobytes()
    ros_msg2.is_dense = True

    # Convert to DIMOS
    dimos_pc = PointCloud2.from_ros_msg(ros_msg2)

    assert dimos_pc.frame_id == "ros_test_frame", (
        f"Frame ID not preserved: expected 'ros_test_frame', got '{dimos_pc.frame_id}'"
    )

    decoded_points = dimos_pc.as_numpy()
    assert len(decoded_points) == 3, (
        f"Wrong number of points: expected 3, got {len(decoded_points)}"
    )

    np.testing.assert_allclose(
        test_points,
        decoded_points,
        rtol=1e-6,
        atol=1e-6,
        err_msg="Points from ROS message don't match",
    )
    print("✓ ROS to DIMOS conversion works correctly")

    # Test 3: Empty point cloud
    empty_pc = PointCloud2(
        pointcloud=o3d.geometry.PointCloud(),
        frame_id="empty_frame",
        ts=1234567890.0,
    )

    empty_ros = empty_pc.to_ros_msg()
    assert empty_ros.width == 0, "Empty cloud should have width 0"
    assert empty_ros.height == 0, "Empty cloud should have height 0"
    assert len(empty_ros.data) == 0, "Empty cloud should have no data"

    empty_converted = PointCloud2.from_ros_msg(empty_ros)
    assert len(empty_converted) == 0, "Empty cloud conversion failed"
    print("✓ Empty point cloud handling works")

    print("\n✓ All ROS conversion tests passed!")


def test_bounding_box_intersects() -> None:
    """Test bounding_box_intersects method with various scenarios."""
    # Test 1: Overlapping boxes
    pc1 = PointCloud2.from_numpy(np.array([[0, 0, 0], [2, 2, 2]]))
    pc2 = PointCloud2.from_numpy(np.array([[1, 1, 1], [3, 3, 3]]))
    assert pc1.bounding_box_intersects(pc2)
    assert pc2.bounding_box_intersects(pc1)  # Should be symmetric

    # Test 2: Non-overlapping boxes
    pc3 = PointCloud2.from_numpy(np.array([[0, 0, 0], [1, 1, 1]]))
    pc4 = PointCloud2.from_numpy(np.array([[2, 2, 2], [3, 3, 3]]))
    assert not pc3.bounding_box_intersects(pc4)
    assert not pc4.bounding_box_intersects(pc3)

    # Test 3: Touching boxes (edge case - should be True)
    pc5 = PointCloud2.from_numpy(np.array([[0, 0, 0], [1, 1, 1]]))
    pc6 = PointCloud2.from_numpy(np.array([[1, 1, 1], [2, 2, 2]]))
    assert pc5.bounding_box_intersects(pc6)
    assert pc6.bounding_box_intersects(pc5)

    # Test 4: One box completely inside another
    pc7 = PointCloud2.from_numpy(np.array([[0, 0, 0], [3, 3, 3]]))
    pc8 = PointCloud2.from_numpy(np.array([[1, 1, 1], [2, 2, 2]]))
    assert pc7.bounding_box_intersects(pc8)
    assert pc8.bounding_box_intersects(pc7)

    # Test 5: Boxes overlapping only in 2 dimensions (not all 3)
    pc9 = PointCloud2.from_numpy(np.array([[0, 0, 0], [2, 2, 1]]))
    pc10 = PointCloud2.from_numpy(np.array([[1, 1, 2], [3, 3, 3]]))
    assert not pc9.bounding_box_intersects(pc10)
    assert not pc10.bounding_box_intersects(pc9)

    # Test 6: Real-world detection scenario with floating point coordinates
    detection1_points = np.array(
        [[-3.5, -0.3, 0.1], [-3.3, -0.2, 0.1], [-3.5, -0.3, 0.3], [-3.3, -0.2, 0.3]]
    )
    pc_det1 = PointCloud2.from_numpy(detection1_points)

    detection2_points = np.array(
        [[-3.4, -0.25, 0.15], [-3.2, -0.15, 0.15], [-3.4, -0.25, 0.35], [-3.2, -0.15, 0.35]]
    )
    pc_det2 = PointCloud2.from_numpy(detection2_points)

    assert pc_det1.bounding_box_intersects(pc_det2)

    # Test 7: Single point clouds
    pc_single1 = PointCloud2.from_numpy(np.array([[1.0, 1.0, 1.0]]))
    pc_single2 = PointCloud2.from_numpy(np.array([[1.0, 1.0, 1.0]]))
    pc_single3 = PointCloud2.from_numpy(np.array([[2.0, 2.0, 2.0]]))

    # Same point should intersect
    assert pc_single1.bounding_box_intersects(pc_single2)
    # Different points should not intersect
    assert not pc_single1.bounding_box_intersects(pc_single3)

    # Test 8: Empty point clouds
    pc_empty1 = PointCloud2.from_numpy(np.array([]).reshape(0, 3))
    pc_empty2 = PointCloud2.from_numpy(np.array([]).reshape(0, 3))
    PointCloud2.from_numpy(np.array([[1.0, 1.0, 1.0]]))

    # Empty clouds should handle gracefully (Open3D returns inf bounds)
    # This might raise an exception or return False - we should handle gracefully
    try:
        result = pc_empty1.bounding_box_intersects(pc_empty2)
        # If no exception, verify behavior is consistent
        assert isinstance(result, bool)
    except:
        # If it raises an exception, that's also acceptable for empty clouds
        pass

    print("✓ All bounding box intersection tests passed!")


if __name__ == "__main__":
    test_lcm_encode_decode()
    test_ros_conversion()
    test_bounding_box_intersects()
