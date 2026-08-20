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


import json

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.lidar import pointcloud2_from_webrtc_lidar
from dimos.utils.testing.replay import SensorReplay


@pytest.mark.self_hosted
def test_lcm_encode_decode() -> None:
    """Test LCM encode/decode preserves pointcloud data."""
    replay = SensorReplay("office_lidar", autocast=pointcloud2_from_webrtc_lidar)
    lidar_msg: PointCloud2 = replay.load_one("lidar_data_021")

    binary_msg = lidar_msg.lcm_encode()
    decoded = PointCloud2.lcm_decode(binary_msg)

    # 1. Check number of points
    original_points, _ = lidar_msg.as_numpy()
    decoded_points, _ = decoded.as_numpy()

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

    # 3. Check frame_id is preserved
    assert lidar_msg.frame_id == decoded.frame_id, (
        f"Frame ID mismatch: '{lidar_msg.frame_id}' vs '{decoded.frame_id}'"
    )

    # 4. Check timestamp is preserved (within reasonable tolerance for float precision)
    if lidar_msg.ts is not None and decoded.ts is not None:
        assert abs(lidar_msg.ts - decoded.ts) < 1e-6, (
            f"Timestamp mismatch: {lidar_msg.ts} vs {decoded.ts}"
        )

    # 5. Check pointcloud properties
    assert len(lidar_msg.pointcloud.points) == len(decoded.pointcloud.points), (
        "Open3D pointcloud size mismatch"
    )


def test_lcm_intensity_round_trip() -> None:
    """Test that intensity values survive an lcm_encode → lcm_decode round trip."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], dtype=np.float32)
    intensities = np.array([0.25, 1.1, 0.0], dtype=np.float32)

    original = PointCloud2.from_numpy(
        points, frame_id="map", timestamp=42.0, intensities=intensities
    )

    # Verify getter before encoding
    got = original.intensities_f32()
    assert got is not None, "intensities_f32() returned None on source cloud"
    np.testing.assert_allclose(got, intensities, atol=1e-6)

    # Round-trip through LCM
    binary = original.lcm_encode()
    decoded = PointCloud2.lcm_decode(binary)

    # Positions preserved
    decoded_pts, _ = decoded.as_numpy()
    np.testing.assert_allclose(decoded_pts.astype(np.float32), points, atol=1e-6)

    # Intensities preserved
    decoded_intensities = decoded.intensities_f32()
    assert decoded_intensities is not None, "intensities lost after lcm_decode"
    np.testing.assert_allclose(decoded_intensities, intensities, atol=1e-6)


def test_lcm_no_intensity_round_trip() -> None:
    """Clouds without intensity should round-trip without creating spurious intensities."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    original = PointCloud2.from_numpy(points, frame_id="map", timestamp=1.0)

    assert original.intensities_f32() is None

    binary = original.lcm_encode()
    decoded = PointCloud2.lcm_decode(binary)

    # No intensities should appear (all-zero wire data is ignored)
    assert decoded.intensities_f32() is None, "Spurious intensities created from zero wire data"

    decoded_pts, _ = decoded.as_numpy()
    np.testing.assert_allclose(decoded_pts.astype(np.float32), points, atol=1e-6)


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
    except Exception:
        # If it raises an exception, that's also acceptable for empty clouds
        pass


def test_agent_encode_scalars_are_exact() -> None:
    """The scalars the eval suite quizzes must match numpy, not approximate it —
    a slab wall at x=2, plus floor points that must not enter the body band."""
    wall = np.stack(
        [
            np.full(200, 2.0),
            np.linspace(-1.0, 1.0, 200),
            np.linspace(0.2, 0.9, 200),
        ],
        axis=1,
    )
    floor = np.stack([np.linspace(-3, 3, 100), np.linspace(-3, 3, 100), np.zeros(100)], axis=1)
    encoded = PointCloud2.from_numpy(np.vstack([wall, floor])).agent_encode()

    stats = encoded["exact_stats"]
    assert isinstance(stats, dict)
    assert encoded["num_points"] == 300
    assert stats["x_range"] == [-3.0, 3.0]
    assert stats["horizontal_extent_m"] == 6.0
    assert stats["vertical_span_m"] == 0.9
    occupancy = encoded["body_height_occupancy"]
    assert isinstance(occupancy, dict)
    # body-height boxes see the wall only: x pinned at 2.0, floor (z=0) excluded
    assert all(b.startswith("2.00@") for b in str(occupancy["boxes"]).split(","))
    assert encoded["centroid_xy_m"] == [1.33, 0.0]  # 200 wall pts at x=2, 100 floor at mean 0


def test_agent_encode_empty_cloud() -> None:
    encoded = PointCloud2.from_numpy(np.zeros((0, 3))).agent_encode()
    assert encoded["num_points"] == 0
    assert "exact_stats" not in encoded


def test_agent_encode_ground_relief_flat_floor_is_level() -> None:
    """A floor at one level must report no relief areas at all.

    This is the negative half of the floor-level question, and it is the half
    that regresses first: an encoder that hints at steps everywhere makes the
    model invent them on flat frames.
    """
    grid = np.stack(np.meshgrid(np.arange(-3, 3, 0.05), np.arange(-3, 3, 0.05)), -1).reshape(-1, 2)
    flat = np.column_stack([grid, np.zeros(len(grid))])

    relief = PointCloud2.from_numpy(flat).agent_encode()["ground_relief"]

    assert isinstance(relief, dict)
    assert relief["floor_z"] == 0.0
    assert relief["areas"] == []


def test_agent_encode_ground_relief_detects_raised_platform() -> None:
    """A 2 m x 2 m platform 0.30 m up must surface as one area, located and measured.

    The body-height band cannot see the surface underneath returns, so without
    this channel a platform reads as ordinary floor.
    """
    grid = np.stack(np.meshgrid(np.arange(-3, 3, 0.05), np.arange(-3, 3, 0.05)), -1).reshape(-1, 2)
    raised = (grid[:, 0] > 0.0) & (grid[:, 0] < 2.0) & (grid[:, 1] > 0.0) & (grid[:, 1] < 2.0)
    stepped = np.column_stack([grid, np.where(raised, 0.30, 0.0)])

    relief = PointCloud2.from_numpy(stepped).agent_encode()["ground_relief"]

    assert isinstance(relief, dict)
    assert relief["floor_z"] == 0.0
    areas = relief["areas"]
    assert isinstance(areas, list)
    assert len(areas) == 1
    assert areas[0] == {"x": 1.0, "y": 1.0, "area_m2": 4.0, "dz_m": 0.3, "far_dz_m": 0.3}


def test_agent_encode_stays_within_prompt_budget() -> None:
    """The encoding is prompt text, so its size is a hard product constraint.

    A busy room -- floor, four walls, scattered clutter -- is the verbose case.
    The encoder trims its own tail rather than letting a dense frame run over.
    """
    rng = np.random.default_rng(7)
    grid = np.stack(np.meshgrid(np.arange(-6, 6, 0.04), np.arange(-6, 6, 0.04)), -1).reshape(-1, 2)
    parts = [np.column_stack([grid, np.zeros(len(grid))])]
    for edge in (-6.0, 6.0):
        span = np.arange(-6, 6, 0.02)
        height = rng.uniform(0.15, 1.0, span.size)
        parts.append(np.column_stack([np.full(span.size, edge), span, height]))
        parts.append(np.column_stack([span, np.full(span.size, edge), height]))
    parts.append(
        np.column_stack(
            [rng.uniform(-6, 6, 4000), rng.uniform(-6, 6, 4000), rng.uniform(0.15, 1.0, 4000)]
        )
    )

    encoded = PointCloud2.from_numpy(np.vstack(parts)).agent_encode()

    assert len(json.dumps(encoded)) < 6000
