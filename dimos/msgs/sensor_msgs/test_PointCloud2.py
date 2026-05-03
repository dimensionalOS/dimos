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


from dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2 as LCMPointCloud2
from dimos_lcm.sensor_msgs.PointField import PointField
from dimos_lcm.std_msgs.Header import Header
import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.lidar import pointcloud2_from_webrtc_lidar
from dimos.utils.testing.replay import SensorReplay


def _field(name: str, offset: int) -> PointField:
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = PointField.FLOAT32
    field.count = 1
    return field


def _lcm_xyz_intensity(points: np.ndarray, intensity: np.ndarray) -> LCMPointCloud2:
    msg = LCMPointCloud2()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = 123
    msg.header.stamp.nsec = 456_000_000
    msg.height = 1
    msg.width = len(points)
    msg.fields = [_field("x", 0), _field("y", 4), _field("z", 8), _field("intensity", 12)]
    msg.fields_length = len(msg.fields)
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.data = np.column_stack([points.astype(np.float32), intensity.astype(np.float32)]).tobytes()
    msg.data_length = len(msg.data)
    msg.is_dense = True
    return msg


def test_lcm_decode_preserves_intensity_field() -> None:
    points = np.array([[0.0, 0.0, -0.45], [1.0, 0.0, 0.10]], dtype=np.float32)
    intensity = np.array([0.0, 0.31], dtype=np.float32)

    decoded = PointCloud2.lcm_decode(_lcm_xyz_intensity(points, intensity).lcm_encode())

    np.testing.assert_allclose(decoded.points_f32(), points)
    np.testing.assert_allclose(decoded.intensity_f32(), intensity)


def test_from_numpy_intensity_roundtrips_through_lcm() -> None:
    points = np.array([[0.0, 0.0, -0.45], [1.0, 0.0, 0.10]], dtype=np.float32)
    intensity = np.array([0.0, 0.31], dtype=np.float32)

    decoded = PointCloud2.lcm_decode(
        PointCloud2.from_numpy(
            points, frame_id="map", timestamp=123.456, intensity=intensity
        ).lcm_encode()
    )

    np.testing.assert_allclose(decoded.points_f32(), points)
    np.testing.assert_allclose(decoded.intensity_f32(), intensity)


def test_lcm_encode_decode() -> None:
    """Test LCM encode/decode preserves pointcloud data."""
    replay = SensorReplay("office_lidar", autocast=pointcloud2_from_webrtc_lidar)
    lidar_msg: PointCloud2 = replay.load_one("lidar_data_021")

    binary_msg = lidar_msg.lcm_encode()
    decoded = PointCloud2.lcm_decode(binary_msg)

    # 1. Check number of points
    original_points, _ = lidar_msg.as_numpy()
    decoded_points, _ = decoded.as_numpy()

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
