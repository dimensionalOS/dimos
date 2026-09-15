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
from typing import Any

import numpy as np
import open3d.core as o3c
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


def test_lcm_per_point_timing_round_trip() -> None:
    """offset_time/tag/line survive an lcm_encode → lcm_decode round trip."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], dtype=np.float32)
    intensities = np.array([10.0, 20.0, 30.0], dtype=np.float32)
    # First point offset 0 is meaningful and must survive (no nonzero filtering).
    offset_times = np.array([0, 41_666, 83_332], dtype=np.uint32)
    tags = np.array([0, 16, 32], dtype=np.uint8)
    lines = np.array([0, 1, 3], dtype=np.uint8)

    original = PointCloud2.from_numpy(
        points,
        frame_id="mid360_link",
        timestamp=100.5,
        intensities=intensities,
        offset_times=offset_times,
        tags=tags,
        lines=lines,
    )

    got_offsets = original.offset_times_u32()
    assert got_offsets is not None
    np.testing.assert_array_equal(got_offsets, offset_times)

    decoded = PointCloud2.lcm_decode(original.lcm_encode())

    decoded_pts, _ = decoded.as_numpy()
    np.testing.assert_allclose(decoded_pts.astype(np.float32), points, atol=1e-6)
    decoded_intensities = decoded.intensities_f32()
    assert decoded_intensities is not None
    np.testing.assert_allclose(decoded_intensities, intensities, atol=1e-6)

    decoded_offsets = decoded.offset_times_u32()
    assert decoded_offsets is not None, "offset_time lost after lcm_decode"
    assert decoded_offsets.dtype == np.uint32
    np.testing.assert_array_equal(decoded_offsets, offset_times)

    decoded_tags = decoded.tags_u8()
    assert decoded_tags is not None, "tag lost after lcm_decode"
    np.testing.assert_array_equal(decoded_tags, tags)

    decoded_lines = decoded.lines_u8()
    assert decoded_lines is not None, "line lost after lcm_decode"
    np.testing.assert_array_equal(decoded_lines, lines)


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


def test_to_rerun_points_mode_is_screen_space() -> None:
    """ "points" must be flat screen-space dots, not the world-space spheres branch."""
    cloud = PointCloud2.from_numpy(np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]))

    points = cloud.to_rerun(mode="points", voxel_size=0.05, ui_radius=1.5)
    spheres = cloud.to_rerun(mode="spheres", voxel_size=0.05)

    # Negative radii are UI points in rerun; positive ones are world-space.
    assert points.radii.as_arrow_array().to_pylist() == pytest.approx([-1.5])
    assert spheres.radii.as_arrow_array().to_pylist() == pytest.approx([0.025])


def test_to_rerun_keeps_the_clouds_own_rgb() -> None:
    """An RGBD cloud renders in its own colors; rgb=False falls back to the height ramp."""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]))
    pcd.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]))
    cloud = PointCloud2(pointcloud=pcd)

    colored = cloud.to_rerun(mode="points")
    assert colored.colors is not None
    assert colored.class_ids is None

    ramp = cloud.to_rerun(mode="points", rgb=False)
    assert ramp.colors is None
    assert ramp.class_ids is not None


def _cloud(points: Any, **kwargs: Any) -> PointCloud2:
    return PointCloud2.from_numpy(np.asarray(points, dtype=np.float64), **kwargs)


def _box(
    lower: tuple[float, float, float], upper: tuple[float, float, float], n: int = 9
) -> np.ndarray:
    axes = [np.linspace(lower[i], upper[i], n) for i in range(3)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)


def test_agent_encode_reports_exact_native_extrema_gaps_and_map() -> None:
    points = np.array([[-3, -3, 0], [3, 3, 0], [2, -1, 0.5], [2, 1, 1]])
    cloud = _cloud(points, frame_id="sensor_native", timestamp=12.345)

    encoded = cloud.agent_encode(cells=8)

    assert encoded["frame_id"] == "sensor_native"
    assert encoded["ts"] == 12.345
    assert encoded["num_points"] == encoded["points_selected"] == 4
    assert encoded["selection"] is None
    assert encoded["bounds_m"] == {"x": [-3, 3], "y": [-3, 3], "z": [0, 1]}
    assert encoded["distinct_coordinates"] == {"x": 3, "y": 4, "z": 3}
    assert encoded["largest_gaps_m"]["x"] == [[-3, 2, 5], [2, 3, 1]]
    assert encoded["centroid_xy_m"] == [1, 0]
    assert encoded["footprint_m2"] == 0.16
    grid = encoded["height_map"]
    assert grid["cell_m"] == 1.0
    assert grid["x_centers_m"] == [-2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]
    assert grid["y_centers_m"] == [3.5, 2.5, 1.5, 0.5, -0.5, -1.5, -2.5]
    assert (grid["z0_m"], grid["z_step_m"]) == (0.0, 0.05)
    # First row is the largest y; z=1.0 is glyph 20 ("K"), z=0.5 is glyph 10 ("A").
    assert grid["zmax_rows"] == grid["zmin_rows"]
    assert grid["zmax_rows"] == [
        "......0",
        ".......",
        ".....K.",
        ".......",
        ".....A.",
        ".......",
        "0......",
    ]
    json.dumps(encoded, allow_nan=False)


def test_agent_encode_legend_documents_every_field() -> None:
    encoded = _cloud([[0, 0, 0], [1, 1, 1]]).agent_encode()
    legend = PointCloud2.AGENT_ENCODE_LEGEND

    for key in [*encoded, *encoded["height_map"]]:
        assert key in legend, key


def test_agent_encode_empty_and_nonfinite_geometry() -> None:
    empty = _cloud(np.zeros((0, 3))).agent_encode()
    invalid = _cloud([[np.nan, 1, 2]]).agent_encode()

    for encoded in [empty, invalid]:
        assert encoded["bounds_m"] == {"x": [], "y": [], "z": []}
        assert encoded["largest_gaps_m"] == {"x": [], "y": [], "z": []}
        assert encoded["height_map"] is None
        assert encoded["points_selected"] == 0
        json.dumps(encoded, allow_nan=False)
    assert empty["num_points"] == 0
    assert invalid["nonfinite_points"] == invalid["num_points"] == 1


def test_agent_encode_counts_nonfinite_returns_without_using_their_geometry() -> None:
    encoded = _cloud([[1, 2, 3], [np.nan, 0, 0], [0, np.inf, 0]]).agent_encode()

    assert encoded["num_points"] == 3
    assert encoded["nonfinite_points"] == 2
    assert encoded["bounds_m"] == {"x": [1, 1], "y": [2, 2], "z": [3, 3]}
    assert encoded["height_map"]["zmax_rows"] == ["0"]


@pytest.mark.parametrize("scale", [0.001, 1.0, 100.0])
def test_agent_encode_map_is_the_same_shape_at_every_scale(scale: float) -> None:
    points = np.concatenate([_box((-5, -2, 0), (-1, 2, 3)), _box((1, -1, 0), (7, 3, 5))])
    reference = _cloud(points).agent_encode()
    scaled = _cloud(points * scale).agent_encode()

    assert scaled["height_map"]["cell_m"] == pytest.approx(0.5 * scale)
    assert scaled["height_map"]["z_step_m"] == pytest.approx(0.2 * scale)
    assert len(scaled["height_map"]["x_centers_m"]) == 25
    assert abs(len(scaled["height_map"]["y_centers_m"]) - 11) <= (scale < 1)
    assert scaled["bounds_m"]["x"] == pytest.approx([-5 * scale, 7 * scale], rel=1e-6)
    assert scaled["largest_gaps_m"]["x"][0][2] == pytest.approx(2 * scale, rel=1e-6)
    if scale >= 1:  # float32 storage moves tiny returns across cell edges
        assert scaled["height_map"]["zmax_rows"] == reference["height_map"]["zmax_rows"]
        assert scaled["height_map"]["zmin_rows"] == reference["height_map"]["zmin_rows"]


def test_agent_encode_translation_keeps_native_coordinates() -> None:
    points = np.concatenate([_box((-5, -2, 0), (-1, 2, 3)), _box((1, -1, 0), (7, 3, 5))])
    offset = np.array([11.0, -7.0, 40.0])
    reference = _cloud(points).agent_encode()
    moved = _cloud(points + offset).agent_encode()

    assert moved["height_map"]["zmax_rows"] == reference["height_map"]["zmax_rows"]
    assert moved["height_map"]["z0_m"] == reference["height_map"]["z0_m"] + 40
    assert moved["height_map"]["x_centers_m"] == pytest.approx(
        [x + 11 for x in reference["height_map"]["x_centers_m"]]
    )
    assert moved["bounds_m"]["z"] == [40, 45]
    assert moved["centroid_xy_m"] == pytest.approx(
        [reference["centroid_xy_m"][0] + 11, reference["centroid_xy_m"][1] - 7]
    )


def test_agent_encode_large_offsets_keep_stored_precision() -> None:
    points = _box((-4, -3, -1), (-1, 1, 2)) + np.array([1e6, -2e6, 3e6])
    encoded = _cloud(points).agent_encode()
    stored = np.asarray(points, dtype=np.float32).astype(np.float64)

    assert encoded["bounds_m"]["x"] == [stored[:, 0].min(), stored[:, 0].max()]
    assert encoded["bounds_m"]["y"] == [stored[:, 1].min(), stored[:, 1].max()]
    grid = encoded["height_map"]
    assert grid["cell_m"] == 0.1
    assert grid["x_centers_m"][0] - 0.05 <= 999996 <= grid["x_centers_m"][0] + 0.05
    assert grid["y_centers_m"][0] - 0.05 <= -1999999 <= grid["y_centers_m"][0] + 0.05
    assert grid["z0_m"] == 2999999.0
    assert any(row.strip(".") for row in grid["zmax_rows"])


def test_agent_encode_axis_permutation_transposes_the_map() -> None:
    points = np.array([[0, 0, 0], [3, 0, 0], [3, 1, 2]])
    encoded = _cloud(points).agent_encode(cells=4)
    swapped = _cloud(points[:, [1, 0, 2]]).agent_encode(cells=4)

    assert encoded["height_map"]["x_centers_m"] == swapped["height_map"]["y_centers_m"][::-1]
    rows = encoded["height_map"]["zmax_rows"]
    columns = ["".join(row[i] for row in reversed(rows)) for i in range(len(rows[0]))]
    assert swapped["height_map"]["zmax_rows"] == list(reversed(columns))


def test_agent_encode_height_glyphs_quantize_from_z0_in_z_step_units() -> None:
    points = [[0, 0, -0.38], [1, 0, 0.15], [2, 0, 0.199], [3, 0, 1.2]]
    grid = _cloud(points).agent_encode(cell=1.0)["height_map"]

    assert (grid["z0_m"], grid["z_step_m"]) == (-0.4, 0.05)
    assert grid["glyph_z_m"][:3] == [-0.4, -0.35, -0.3]
    assert grid["glyph_z_m"][32] == 1.2
    assert len(grid["glyph_z_m"]) == 33
    assert grid["zmax_rows"] == ["0BBW"]
    assert grid["zmax_rows"] == grid["zmin_rows"]


def test_agent_encode_separates_highest_and_lowest_return_per_cell() -> None:
    encoded = _cloud([[0, 0, 0], [0.1, 0.1, 1.0], [2, 0, 0.5]]).agent_encode(cell=1.0, z_step=0.5)

    assert encoded["height_map"]["zmax_rows"] == ["2.1"]
    assert encoded["height_map"]["zmin_rows"] == ["0.1"]


def test_agent_encode_selection_grids_the_whole_window_and_filters_heights() -> None:
    points = np.array([[0, 0, 0], [1, 1, 1], [5, 5, 0], [0.5, 0.5, 3]])
    encoded = _cloud(points).agent_encode(center=(1, 1), radius=1, z_range=(0, 2), cell=0.5)

    assert encoded["selection"] == {
        "center_xy": [1, 1],
        "radius_m": 1,
        "shape": "square",
        "z_range_m": [0, 2],
    }
    assert encoded["num_points"] == 4
    assert encoded["points_selected"] == 2
    assert encoded["bounds_m"]["x"] == [0, 1]
    grid = encoded["height_map"]
    assert grid["x_centers_m"] == [0.25, 0.75, 1.25, 1.75, 2.25]
    assert grid["y_centers_m"] == [2.25, 1.75, 1.25, 0.75, 0.25]
    assert grid["zmax_rows"] == [".....", ".....", "..K..", ".....", "0...."]


def test_agent_encode_range_profile_measures_bearing_sectors_from_the_center() -> None:
    points = [[1.5, 0, 0], [0, 2, 0.5], [-0.4, 0, 1.0], [0.7, 0.7, 2.0], [0, -3, 0], [2.5, 0, 0]]
    encoded = _cloud(points).agent_encode(center=(0, 0), radius=3)
    profile = encoded["range_profile_m"]

    assert len(profile) == 36
    assert profile[0] == 1.5  # east, nearest of two returns
    assert profile[9] == 2.0  # north
    assert profile[18] == 0.4  # west
    assert profile[27] == 3.0  # south
    assert profile[5] == pytest.approx(0.99, abs=0.005)  # 45 degrees opens sector 5
    assert profile.count(None) == 31

    banded = _cloud(points).agent_encode(center=(0, 0), radius=3, z_range=(0.4, 1.5))
    assert banded["range_profile_m"][9] == 2.0
    assert banded["range_profile_m"][0] is None
    assert _cloud(points).agent_encode()["range_profile_m"] is None
    whole = _cloud(points).agent_encode(center=(0, 0))
    assert whole["selection"] == {"center_xy": [0, 0], "radius_m": None}
    assert whole["points_selected"] == 6
    assert whole["range_profile_m"] == profile


def test_agent_encode_selection_without_returns_still_describes_the_window() -> None:
    encoded = _cloud([[10, 10, 0]]).agent_encode(center=(0, 0), radius=1, cell=1.0)

    assert encoded["points_selected"] == 0
    assert encoded["bounds_m"]["x"] == []
    assert encoded["height_map"]["zmax_rows"] == ["...", "...", "..."]
    assert encoded["range_profile_m"] == [None] * 36


@pytest.mark.parametrize(
    "kwargs",
    [
        {"radius": 1.0},
        {"center": (0, 0), "radius": 0},
        {"z_range": (1, 0)},
        {"cell": 0},
        {"z_step": -1},
    ],
)
def test_agent_encode_rejects_invalid_options(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        _cloud([[0, 0, 0], [5, 5, 1]]).agent_encode(**kwargs)  # type: ignore[arg-type]


def test_agent_encode_explicit_cell_may_exceed_default_cells_but_not_the_limit() -> None:
    cloud = _cloud([[0, 0, 0], [5, 5, 1]])

    fine = cloud.agent_encode(cell=0.05)["height_map"]
    assert fine["cell_m"] == 0.05
    assert len(fine["x_centers_m"]) == 101

    coarsened = cloud.agent_encode(cell=0.01)["height_map"]
    assert coarsened["cell_m"] == 0.05
    assert len(cloud.agent_encode(cells=1000)["height_map"]["x_centers_m"]) <= 120
    assert len(cloud.agent_encode(cells=1)["height_map"]["x_centers_m"]) <= 2


def test_agent_encode_default_cell_fits_the_requested_grid() -> None:
    points = _box((0, 0, 0), (10, 3, 1), n=41)

    for cells in (10, 48, 120):
        grid = _cloud(points).agent_encode(cells=cells)["height_map"]
        assert max(len(grid["x_centers_m"]), len(grid["y_centers_m"])) <= cells
        assert grid["cell_m"] in {0.1, 0.2, 0.25, 0.5, 1.0, 2.0}


def test_agent_encode_is_independent_of_return_order() -> None:
    rng = np.random.default_rng(3)
    points = rng.uniform(-4, 4, size=(2000, 3))
    ordered = _cloud(points).agent_encode()
    shuffled = _cloud(points[rng.permutation(len(points))]).agent_encode()

    assert shuffled == ordered


def test_agent_encode_payload_stays_compact_for_dense_maps() -> None:
    rng = np.random.default_rng(0)
    points = rng.uniform((-5, -5, -0.5), (5, 5, 2.5), size=(200_000, 3))

    encoded = _cloud(points).agent_encode()

    assert len(json.dumps(encoded)) < 8000
    assert len(encoded["height_map"]["x_centers_m"]) <= 48


@pytest.mark.parametrize("points", [[[1e300, 0, 0], [-1e300, 0, 0]], [[1e300, 0, 0]]])
def test_agent_encode_rejects_unrepresentable_numeric_ranges(points: list[list[float]]) -> None:
    import open3d as o3d

    pcd = o3d.t.geometry.PointCloud()
    pcd.point["positions"] = o3c.Tensor(np.array(points, dtype=np.float64), dtype=o3c.float64)

    with pytest.raises(ValueError):
        PointCloud2(pointcloud=pcd).agent_encode()


def test_nice_step_picks_round_sizes() -> None:
    from dimos.msgs.sensor_msgs.pointcloud_height_map import nice_step

    assert [nice_step(v) for v in (0.011, 0.02, 0.21, 0.3, 0.6, 7, 0.0004)] == [
        0.02,
        0.02,
        0.25,
        0.5,
        1.0,
        10.0,
        0.0005,
    ]
