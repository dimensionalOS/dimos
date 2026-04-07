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

"""Integration tests: PGO global map functionality.

Tests the PGO (Pose Graph Optimization) module's global map capabilities:
- Global map accumulation from keyframes
- Global map point cloud contains points from ALL keyframes
- Loop closure updates the global map positions
- Global map can be exported as a valid PointCloud2

Uses the Python reference implementation for algorithm-level testing.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

try:
    from dimos.navigation.smart_nav.modules.pgo.pgo_reference import PGOConfig, SimplePGOReference

    _HAS_PGO_DEPS = True
except ImportError:
    _HAS_PGO_DEPS = False

pytestmark = pytest.mark.skipif(not _HAS_PGO_DEPS, reason="gtsam not installed")

# ─── Helpers ─────────────────────────────────────────────────────────────────


def make_rotation(yaw_deg: float) -> np.ndarray:
    return Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()


def make_structured_cloud(center: np.ndarray, n_points: int = 500, seed: int = 42) -> np.ndarray:
    """Create a sphere-surface point cloud around a center."""
    rng = np.random.default_rng(seed)
    phi = rng.uniform(0, 2 * np.pi, n_points)
    theta = rng.uniform(0, np.pi, n_points)
    r = 2.0
    x = r * np.sin(theta) * np.cos(phi) + center[0]
    y = r * np.sin(theta) * np.sin(phi) + center[1]
    z = r * np.cos(theta) + center[2]
    return np.column_stack([x, y, z])


def make_random_cloud(
    center: np.ndarray, n_points: int = 200, spread: float = 1.0, seed: int | None = None
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return center + rng.normal(0, spread, (n_points, 3))


def drive_trajectory(
    pgo: SimplePGOReference,
    waypoints: list[np.ndarray],
    step: float = 0.4,
    time_per_step: float = 1.0,
    cloud_seed_base: int = 0,
) -> None:
    """Drive a trajectory through a list of waypoints, adding keyframes."""
    t = 0.0
    pos = waypoints[0].copy()
    for i in range(1, len(waypoints)):
        direction = waypoints[i] - waypoints[i - 1]
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        direction_norm = direction / dist
        yaw = math.degrees(math.atan2(direction_norm[1], direction_norm[0]))
        r = make_rotation(yaw)
        n_steps = int(dist / step)

        for s in range(n_steps):
            pos = waypoints[i - 1] + direction_norm * step * (s + 1)
            cloud = make_structured_cloud(
                np.zeros(3), n_points=200, seed=(cloud_seed_base + int(t)) % 10000
            )
            added = pgo.add_key_pose(r, pos, t, cloud)
            if added:
                pgo.search_for_loop_pairs()
                pgo.smooth_and_update()
            t += time_per_step


# ─── Global Map Accumulation Tests ───────────────────────────────────────────


class TestGlobalMapAccumulation:
    """Test that PGO produces a valid global map from keyframes."""

    def test_global_map_contains_all_keyframes(self):
        """Global map should contain transformed points from every keyframe."""
        config = PGOConfig(
            key_pose_delta_trans=0.3,
            global_map_voxel_size=0.0,  # No downsampling
        )
        pgo = SimplePGOReference(config)

        n_keyframes = 10
        pts_per_frame = 100
        for i in range(n_keyframes):
            pos = np.array([i * 1.0, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=pts_per_frame, seed=i)
            pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            pgo.smooth_and_update()

        assert len(pgo.key_poses) == n_keyframes
        global_map = pgo.build_global_map(voxel_size=0.0)
        assert len(global_map) == n_keyframes * pts_per_frame, (
            f"Expected {n_keyframes * pts_per_frame} points, got {len(global_map)}"
        )

    def test_global_map_points_are_in_world_frame(self):
        """Points in the global map should be transformed to world coordinates."""
        config = PGOConfig(
            key_pose_delta_trans=0.3,
            global_map_voxel_size=0.0,
        )
        pgo = SimplePGOReference(config)

        # Add keyframe at origin with cloud centered at body origin
        cloud_body = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        pgo.add_key_pose(np.eye(3), np.array([10.0, 20.0, 0.0]), 0.0, cloud_body)
        pgo.smooth_and_update()

        global_map = pgo.build_global_map(voxel_size=0.0)

        # Points should be shifted by the keyframe position (10, 20, 0)
        expected = cloud_body + np.array([10.0, 20.0, 0.0])
        np.testing.assert_allclose(global_map, expected, atol=1e-6)

    def test_global_map_with_rotation(self):
        """Global map should correctly rotate body-frame points."""
        config = PGOConfig(
            key_pose_delta_trans=0.3,
            global_map_voxel_size=0.0,
        )
        pgo = SimplePGOReference(config)

        # 90 degree yaw rotation
        r_90 = make_rotation(90.0)
        cloud_body = np.array([[1.0, 0.0, 0.0]])  # Point along body x-axis
        pgo.add_key_pose(r_90, np.zeros(3), 0.0, cloud_body)
        pgo.smooth_and_update()

        global_map = pgo.build_global_map(voxel_size=0.0)

        # After 90 deg yaw, body x-axis → world y-axis
        np.testing.assert_allclose(global_map[0, 0], 0.0, atol=1e-6)
        np.testing.assert_allclose(global_map[0, 1], 1.0, atol=1e-6)
        np.testing.assert_allclose(global_map[0, 2], 0.0, atol=1e-6)

    def test_global_map_grows_with_trajectory(self):
        """Global map should grow as more keyframes are added."""
        config = PGOConfig(key_pose_delta_trans=0.3, global_map_voxel_size=0.0)
        pgo = SimplePGOReference(config)

        sizes = []
        for i in range(20):
            pos = np.array([i * 0.5, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=50, seed=i)
            pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            pgo.smooth_and_update()
            sizes.append(len(pgo.build_global_map(voxel_size=0.0)))

        # Map should be monotonically growing
        for j in range(1, len(sizes)):
            assert sizes[j] >= sizes[j - 1], f"Map shrunk: {sizes[j]} < {sizes[j - 1]} at step {j}"

    def test_global_map_voxel_downsampling(self):
        """Downsampled global map should have fewer points."""
        config = PGOConfig(key_pose_delta_trans=0.3)
        pgo = SimplePGOReference(config)

        for i in range(10):
            pos = np.array([i * 1.0, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=200, seed=i)
            pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            pgo.smooth_and_update()

        map_full = pgo.build_global_map(voxel_size=0.0)
        map_ds = pgo.build_global_map(voxel_size=0.5)

        assert len(map_ds) < len(map_full), (
            f"Downsampled map ({len(map_ds)}) should be smaller than full ({len(map_full)})"
        )
        assert len(map_ds) > 0


# ─── Loop Closure Global Map Tests ──────────────────────────────────────────


class TestLoopClosureGlobalMap:
    """Test that loop closure correctly updates the global map."""

    def test_global_map_updates_after_loop_closure(self):
        """After loop closure, global map positions should be corrected."""
        config = PGOConfig(
            key_pose_delta_trans=0.4,
            key_pose_delta_deg=10.0,
            loop_search_radius=15.0,
            loop_time_thresh=30.0,
            loop_score_thresh=2.0,  # Very relaxed for synthetic data
            loop_submap_half_range=3,
            submap_resolution=0.2,
            min_loop_detect_duration=0.0,
            global_map_voxel_size=0.0,
            max_icp_iterations=30,
            max_icp_correspondence_dist=20.0,
        )
        pgo = SimplePGOReference(config)

        # Drive a square trajectory
        side = 20.0
        waypoints = [
            np.array([0.0, 0.0, 0.0]),
            np.array([side, 0.0, 0.0]),
            np.array([side, side, 0.0]),
            np.array([0.0, side, 0.0]),
            np.array([0.0, 0.0, 0.0]),  # Return to start
        ]
        drive_trajectory(pgo, waypoints, step=0.4, time_per_step=1.0)

        # Should have accumulated keyframes
        assert len(pgo.key_poses) > 20

        # Build global map
        global_map = pgo.build_global_map(voxel_size=0.0)
        assert len(global_map) > 0

        # If loop closure detected, verify map is consistent
        if len(pgo.history_pairs) > 0:
            # The start and end keyframe positions should be close
            start_pos = pgo.key_poses[0].t_global
            end_pos = pgo.key_poses[-1].t_global
            # After loop closure correction
            dist = np.linalg.norm(end_pos - start_pos)
            assert dist < 15.0, f"After loop closure, start-end distance {dist:.2f}m is too large"

    def test_global_map_all_keyframes_present_after_loop(self):
        """After loop closure, ALL keyframes should still be in the map."""
        config = PGOConfig(
            key_pose_delta_trans=0.3,
            loop_search_radius=15.0,
            loop_time_thresh=20.0,
            loop_score_thresh=2.0,
            min_loop_detect_duration=0.0,
            global_map_voxel_size=0.0,
            max_icp_correspondence_dist=20.0,
        )
        pgo = SimplePGOReference(config)

        pts_per_frame = 50
        n_poses = 0
        for i in range(40):
            pos = np.array([i * 0.5, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=pts_per_frame, seed=i % 5)
            added = pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            if added:
                pgo.smooth_and_update()
                n_poses += 1

        global_map = pgo.build_global_map(voxel_size=0.0)
        expected_points = n_poses * pts_per_frame
        assert len(global_map) == expected_points, (
            f"Expected {expected_points} points from {n_poses} keyframes, got {len(global_map)}"
        )


# ─── PointCloud2 Export Tests ────────────────────────────────────────────────


class TestGlobalMapExport:
    """Test that global map can be exported as valid PointCloud2."""

    def test_export_as_pointcloud2(self):
        """Global map numpy array should convert to valid PointCloud2."""
        config = PGOConfig(key_pose_delta_trans=0.3, global_map_voxel_size=0.0)
        pgo = SimplePGOReference(config)

        for i in range(5):
            pos = np.array([i * 1.0, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=100, seed=i)
            pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            pgo.smooth_and_update()

        global_map = pgo.build_global_map(voxel_size=0.1)
        assert len(global_map) > 0

        # Convert to PointCloud2
        pc2 = PointCloud2.from_numpy(
            global_map.astype(np.float32),
            frame_id="map",
            timestamp=time.time(),
        )

        # Verify round-trip
        points_back, _ = pc2.as_numpy()
        assert points_back.shape[0] > 0
        assert points_back.shape[1] >= 3

    def test_export_empty_map(self):
        """Exporting an empty global map should not crash."""
        pgo = SimplePGOReference()
        global_map = pgo.build_global_map()
        assert len(global_map) == 0

    def test_export_large_map(self):
        """Test export with a larger accumulated map (many keyframes)."""
        config = PGOConfig(
            key_pose_delta_trans=0.3,
            global_map_voxel_size=0.2,
        )
        pgo = SimplePGOReference(config)

        for i in range(50):
            pos = np.array([i * 0.5, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=200, seed=i)
            pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            pgo.smooth_and_update()

        global_map = pgo.build_global_map()
        assert len(global_map) > 0

        # Should be downsampled (less than 50 * 200 = 10000)
        assert len(global_map) < 10000

        # Convert to PointCloud2
        pc2 = PointCloud2.from_numpy(
            global_map.astype(np.float32),
            frame_id="map",
            timestamp=time.time(),
        )
        points_back, _ = pc2.as_numpy()
        assert len(points_back) == len(global_map)

    def test_global_map_spatial_extent(self):
        """Global map should span the spatial extent of the trajectory."""
        config = PGOConfig(
            key_pose_delta_trans=0.3,
            global_map_voxel_size=0.0,
        )
        pgo = SimplePGOReference(config)

        # Drive 10 meters in x direction
        for i in range(30):
            pos = np.array([i * 0.5, 0.0, 0.0])
            cloud = make_random_cloud(np.zeros(3), n_points=50, spread=0.5, seed=i)
            pgo.add_key_pose(np.eye(3), pos, float(i), cloud)
            pgo.smooth_and_update()

        global_map = pgo.build_global_map(voxel_size=0.0)

        # Map x-range should roughly span trajectory
        x_min = global_map[:, 0].min()
        x_max = global_map[:, 0].max()
        x_span = x_max - x_min

        # Should span close to the trajectory length (15m) +/- cloud spread
        assert x_span > 10.0, f"X-span {x_span:.1f}m too narrow for 15m trajectory"
        assert x_span < 25.0, f"X-span {x_span:.1f}m too wide"
