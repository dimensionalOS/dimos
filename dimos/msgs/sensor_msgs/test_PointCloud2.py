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


from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.pointcloud import (
    cloud_bounds_intersect,
    pointcloud_from_xyz,
    pointcloud_from_xyz_rgb,
    pointcloud_to_open3d,
    pointcloud_view,
    pointcloud_xyz,
)
from dimos.msgs.time import time_from_seconds, to_nanoseconds
from dimos.robot.unitree.type.lidar import pointcloud2_from_webrtc_lidar
from dimos.visualization.rerun.message_helpers import cloud_archetype


def _cloud(
    points,
    *,
    frame_id="",
    timestamp=0.0,
    intensities=None,
    offset_times=None,
    tags=None,
    lines=None,
):
    extras = [
        (name, values, dtype, datatype)
        for name, values, dtype, datatype in [
            ("intensity", intensities, "<f4", PointField.FLOAT32),
            ("offset_time", offset_times, "<u4", PointField.UINT32),
            ("tag", tags, "u1", PointField.UINT8),
            ("line", lines, "u1", PointField.UINT8),
        ]
        if values is not None
    ]
    if not extras:
        return pointcloud_from_xyz(
            points, header=Header(frame_id=frame_id, stamp=time_from_seconds(timestamp))
        )
    dtype = np.dtype(
        [(name, "<f4") for name in ("x", "y", "z")] + [(name, kind) for name, _, kind, _ in extras]
    )
    packed = np.zeros(len(points), dtype=dtype)
    fields = []
    for axis, name in enumerate(("x", "y", "z")):
        packed[name] = points[:, axis]
        fields.append(
            PointField(
                name=name, offset=dtype.fields[name][1], datatype=PointField.FLOAT32, count=1
            )
        )
    for name, values, _, datatype in extras:
        packed[name] = values
        fields.append(
            PointField(name=name, offset=dtype.fields[name][1], datatype=datatype, count=1)
        )
    return PointCloud2(
        header=Header(frame_id=frame_id, stamp=time_from_seconds(timestamp)),
        height=1,
        width=len(points),
        fields=fields,
        point_step=dtype.itemsize,
        row_step=len(points) * dtype.itemsize,
        data=packed.view(np.uint8),
        is_dense=True,
    )


def _field(message, name):
    view = pointcloud_view(message)
    return view[name].ravel() if name in view.dtype.names else None


def test_cdr_encode_decode() -> None:
    points = np.arange(300, dtype=np.float32).reshape(100, 3) / 10
    lidar_msg = pointcloud2_from_webrtc_lidar({"data": {"stamp": 12.5, "data": {"points": points}}})

    binary_msg = lidar_msg.encode()
    decoded = PointCloud2.decode(binary_msg)

    # 1. Check number of points
    original_points = pointcloud_xyz(lidar_msg)
    decoded_points = pointcloud_xyz(decoded)

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
    assert lidar_msg.header.frame_id == decoded.header.frame_id, (
        f"Frame ID mismatch: '{lidar_msg.header.frame_id}' vs '{decoded.header.frame_id}'"
    )

    # 4. Check timestamp is preserved (within reasonable tolerance for float precision)
    if (
        to_nanoseconds(lidar_msg.header.stamp) is not None
        and to_nanoseconds(decoded.header.stamp) is not None
    ):
        assert (
            abs(to_nanoseconds(lidar_msg.header.stamp) - to_nanoseconds(decoded.header.stamp))
            < 1e-6
        ), (
            f"Timestamp mismatch: {to_nanoseconds(lidar_msg.header.stamp)} vs {to_nanoseconds(decoded.header.stamp)}"
        )

    # 5. Check pointcloud properties
    assert len(pointcloud_to_open3d(lidar_msg).points) == len(
        pointcloud_to_open3d(decoded).points
    ), "Open3D pointcloud size mismatch"


def test_cdr_intensity_round_trip() -> None:
    """Test that intensity values survive an lcm_encode → lcm_decode round trip."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], dtype=np.float32)
    intensities = np.array([0.25, 1.1, 0.0], dtype=np.float32)

    original = _cloud(points, frame_id="map", timestamp=42.0, intensities=intensities)

    # Verify getter before encoding
    got = _field(original, "intensity")
    assert got is not None, "intensities_f32() returned None on source cloud"
    np.testing.assert_allclose(got, intensities, atol=1e-6)

    # Round-trip through LCM
    binary = original.encode()
    decoded = PointCloud2.decode(binary)

    # Positions preserved
    decoded_pts = pointcloud_xyz(decoded)
    np.testing.assert_allclose(decoded_pts.astype(np.float32), points, atol=1e-6)

    # Intensities preserved
    decoded_intensities = _field(decoded, "intensity")
    assert decoded_intensities is not None, "intensities lost after lcm_decode"
    np.testing.assert_allclose(decoded_intensities, intensities, atol=1e-6)


def test_cdr_no_intensity_round_trip() -> None:
    """Clouds without intensity should round-trip without creating spurious intensities."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    original = _cloud(points, frame_id="map", timestamp=1.0)

    assert _field(original, "intensity") is None

    binary = original.encode()
    decoded = PointCloud2.decode(binary)

    # No intensities should appear (all-zero wire data is ignored)
    assert _field(decoded, "intensity") is None, "Spurious intensities created from zero wire data"

    decoded_pts = pointcloud_xyz(decoded)
    np.testing.assert_allclose(decoded_pts.astype(np.float32), points, atol=1e-6)


def test_cdr_per_point_timing_round_trip() -> None:
    """offset_time/tag/line survive an lcm_encode → lcm_decode round trip."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], dtype=np.float32)
    intensities = np.array([10.0, 20.0, 30.0], dtype=np.float32)
    # First point offset 0 is meaningful and must survive (no nonzero filtering).
    offset_times = np.array([0, 41_666, 83_332], dtype=np.uint32)
    tags = np.array([0, 16, 32], dtype=np.uint8)
    lines = np.array([0, 1, 3], dtype=np.uint8)

    original = _cloud(
        points,
        frame_id="mid360_link",
        timestamp=100.5,
        intensities=intensities,
        offset_times=offset_times,
        tags=tags,
        lines=lines,
    )

    got_offsets = _field(original, "offset_time")
    assert got_offsets is not None
    np.testing.assert_array_equal(got_offsets, offset_times)

    decoded = PointCloud2.decode(original.encode())

    decoded_pts = pointcloud_xyz(decoded)
    np.testing.assert_allclose(decoded_pts.astype(np.float32), points, atol=1e-6)
    decoded_intensities = _field(decoded, "intensity")
    assert decoded_intensities is not None
    np.testing.assert_allclose(decoded_intensities, intensities, atol=1e-6)

    decoded_offsets = _field(decoded, "offset_time")
    assert decoded_offsets is not None, "offset_time lost after lcm_decode"
    assert decoded_offsets.dtype == np.uint32
    np.testing.assert_array_equal(decoded_offsets, offset_times)

    decoded_tags = _field(decoded, "tag")
    assert decoded_tags is not None, "tag lost after lcm_decode"
    np.testing.assert_array_equal(decoded_tags, tags)

    decoded_lines = _field(decoded, "line")
    assert decoded_lines is not None, "line lost after lcm_decode"
    np.testing.assert_array_equal(decoded_lines, lines)


def test_bounding_box_intersects() -> None:
    """Test bounding_box_intersects method with various scenarios."""
    # Test 1: Overlapping boxes
    pc1 = _cloud(np.array([[0, 0, 0], [2, 2, 2]]))
    pc2 = _cloud(np.array([[1, 1, 1], [3, 3, 3]]))
    assert cloud_bounds_intersect(pc1, pc2)
    assert cloud_bounds_intersect(pc2, pc1)  # Should be symmetric

    # Test 2: Non-overlapping boxes
    pc3 = _cloud(np.array([[0, 0, 0], [1, 1, 1]]))
    pc4 = _cloud(np.array([[2, 2, 2], [3, 3, 3]]))
    assert not cloud_bounds_intersect(pc3, pc4)
    assert not cloud_bounds_intersect(pc4, pc3)

    # Test 3: Touching boxes (edge case - should be True)
    pc5 = _cloud(np.array([[0, 0, 0], [1, 1, 1]]))
    pc6 = _cloud(np.array([[1, 1, 1], [2, 2, 2]]))
    assert cloud_bounds_intersect(pc5, pc6)
    assert cloud_bounds_intersect(pc6, pc5)

    # Test 4: One box completely inside another
    pc7 = _cloud(np.array([[0, 0, 0], [3, 3, 3]]))
    pc8 = _cloud(np.array([[1, 1, 1], [2, 2, 2]]))
    assert cloud_bounds_intersect(pc7, pc8)
    assert cloud_bounds_intersect(pc8, pc7)

    # Test 5: Boxes overlapping only in 2 dimensions (not all 3)
    pc9 = _cloud(np.array([[0, 0, 0], [2, 2, 1]]))
    pc10 = _cloud(np.array([[1, 1, 2], [3, 3, 3]]))
    assert not cloud_bounds_intersect(pc9, pc10)
    assert not cloud_bounds_intersect(pc10, pc9)

    # Test 6: Real-world detection scenario with floating point coordinates
    detection1_points = np.array(
        [[-3.5, -0.3, 0.1], [-3.3, -0.2, 0.1], [-3.5, -0.3, 0.3], [-3.3, -0.2, 0.3]]
    )
    pc_det1 = _cloud(detection1_points)

    detection2_points = np.array(
        [[-3.4, -0.25, 0.15], [-3.2, -0.15, 0.15], [-3.4, -0.25, 0.35], [-3.2, -0.15, 0.35]]
    )
    pc_det2 = _cloud(detection2_points)

    assert cloud_bounds_intersect(pc_det1, pc_det2)

    # Test 7: Single point clouds
    pc_single1 = _cloud(np.array([[1.0, 1.0, 1.0]]))
    pc_single2 = _cloud(np.array([[1.0, 1.0, 1.0]]))
    pc_single3 = _cloud(np.array([[2.0, 2.0, 2.0]]))

    # Same point should intersect
    assert cloud_bounds_intersect(pc_single1, pc_single2)
    # Different points should not intersect
    assert not cloud_bounds_intersect(pc_single1, pc_single3)

    # Test 8: Empty point clouds
    pc_empty1 = _cloud(np.array([]).reshape(0, 3))
    pc_empty2 = _cloud(np.array([]).reshape(0, 3))
    _cloud(np.array([[1.0, 1.0, 1.0]]))

    assert not cloud_bounds_intersect(pc_empty1, pc_empty2)


def test_to_rerun_points_mode_is_screen_space() -> None:
    """ "points" must be flat screen-space dots, not the world-space spheres branch."""
    cloud = _cloud(np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]))

    points = cloud_archetype(cloud, mode="points", voxel_size=0.05, ui_radius=1.5)
    spheres = cloud_archetype(cloud, mode="spheres", voxel_size=0.05)

    # Negative radii are UI points in rerun; positive ones are world-space.
    assert points.radii.as_arrow_array().to_pylist() == pytest.approx([-1.5])
    assert spheres.radii.as_arrow_array().to_pylist() == pytest.approx([0.025])


def test_to_rerun_keeps_the_clouds_own_rgb() -> None:
    """An RGBD cloud renders in its own colors; rgb=False falls back to the height ramp."""
    cloud = pointcloud_from_xyz_rgb(
        np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
        np.array([[255, 0, 0], [0, 0, 255]], dtype=np.uint8),
        header=Header(),
    )

    colored = cloud_archetype(cloud, mode="points")
    assert colored.colors is not None
    assert colored.class_ids is None

    ramp = cloud_archetype(cloud, mode="points", rgb=False)
    assert ramp.colors is not None
    assert ramp.class_ids is None
    assert ramp.colors.as_arrow_array().to_pylist() != colored.colors.as_arrow_array().to_pylist()
