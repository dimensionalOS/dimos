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

from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.testing import SensorReplay


def test_lcm_encode_decode():
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
