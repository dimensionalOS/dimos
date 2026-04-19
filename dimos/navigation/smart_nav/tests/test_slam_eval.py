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

"""Tests for the SLAM evaluation harness."""

from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.navigation.smart_nav.modules.slam_eval.slam_eval import (
    EvalDataset,
    NoiseProfile,
    SensorFrame,
    TrajectoryPose,
    _align_trajectories,
    apply_noise,
    compute_ate,
    compute_drift,
    compute_rpe,
    load_dataset,
    slam_eval,
)

try:
    from dimos.navigation.smart_nav.modules.pgo.pgo import _SimplePGO
    from dimos.navigation.smart_nav.modules.slam_eval.slam_eval import PGOBackend

    _HAS_PGO_DEPS = True
    del _SimplePGO  # only needed to check import availability
except ImportError:
    _HAS_PGO_DEPS = False

_requires_gtsam = pytest.mark.skipif(not _HAS_PGO_DEPS, reason="gtsam not installed")


# ─── Helpers ────────────────────────────────────────────────────────────────


def _make_pose(
    x: float, y: float, z: float, yaw_deg: float = 0.0, ts: float = 0.0
) -> TrajectoryPose:
    r = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    return TrajectoryPose(timestamp=ts, position=np.array([x, y, z]), rotation=r)


# ─── Unit tests: metrics computation ───────────────────────────────────────


class TestATE:
    def test_perfect_trajectory(self) -> None:
        """Zero error when estimated == ground truth."""
        poses = [_make_pose(i * 0.5, 0, 0, ts=i * 0.1) for i in range(10)]
        pairs = list(zip(poses, poses, strict=True))
        ate = compute_ate(pairs)
        assert ate["rmse"] == pytest.approx(0.0, abs=1e-10)
        assert ate["n_pairs"] == 10

    def test_constant_offset(self) -> None:
        """ATE equals the constant offset magnitude."""
        gt = [_make_pose(i * 0.5, 0, 0, ts=i * 0.1) for i in range(10)]
        est = [_make_pose(i * 0.5 + 1.0, 0, 0, ts=i * 0.1) for i in range(10)]
        pairs = list(zip(est, gt, strict=True))
        ate = compute_ate(pairs)
        assert ate["rmse"] == pytest.approx(1.0, abs=1e-6)
        assert ate["mean"] == pytest.approx(1.0, abs=1e-6)

    def test_empty_pairs(self) -> None:
        ate = compute_ate([])
        assert math.isnan(ate["rmse"])
        assert ate["n_pairs"] == 0


class TestRPE:
    def test_perfect_trajectory(self) -> None:
        poses = [_make_pose(i * 0.5, 0, 0, ts=i * 0.1) for i in range(10)]
        pairs = list(zip(poses, poses, strict=True))
        rpe = compute_rpe(pairs)
        assert rpe["trans_rmse"] == pytest.approx(0.0, abs=1e-10)
        assert rpe["rot_rmse"] == pytest.approx(0.0, abs=1e-10)

    def test_scale_error(self) -> None:
        """RPE detects consistent scale error in translations."""
        gt = [_make_pose(i * 1.0, 0, 0, ts=i * 0.1) for i in range(10)]
        est = [_make_pose(i * 1.1, 0, 0, ts=i * 0.1) for i in range(10)]
        pairs = list(zip(est, gt, strict=True))
        rpe = compute_rpe(pairs)
        # Each relative step: estimated=1.1, gt=1.0, error=0.1
        assert rpe["trans_rmse"] == pytest.approx(0.1, abs=1e-6)

    def test_too_few_pairs(self) -> None:
        pose = _make_pose(0, 0, 0)
        rpe = compute_rpe([(pose, pose)])
        assert math.isnan(rpe["trans_rmse"])


class TestDrift:
    def test_zero_drift(self) -> None:
        poses = [_make_pose(i * 0.5, 0, 0, ts=i * 0.1) for i in range(10)]
        pairs = list(zip(poses, poses, strict=True))
        drift = compute_drift(pairs)
        assert drift["drift_pct"] == pytest.approx(0.0, abs=1e-10)

    def test_known_drift(self) -> None:
        gt = [_make_pose(i * 1.0, 0, 0, ts=i * 0.1) for i in range(11)]
        # Estimated drifts by 1m over 10m total distance
        est = [_make_pose(i * 1.0 + (i / 10.0), 0, 0, ts=i * 0.1) for i in range(11)]
        pairs = list(zip(est, gt, strict=True))
        drift = compute_drift(pairs)
        assert drift["total_distance"] == pytest.approx(10.0, abs=1e-6)
        assert drift["final_error"] == pytest.approx(1.0, abs=1e-6)
        assert drift["drift_pct"] == pytest.approx(10.0, abs=1e-4)


class TestAlignment:
    def test_exact_timestamps(self) -> None:
        est = [_make_pose(0, 0, 0, ts=1.0), _make_pose(1, 0, 0, ts=2.0)]
        gt = [_make_pose(0, 0, 0, ts=1.0), _make_pose(1, 0, 0, ts=2.0)]
        pairs = _align_trajectories(est, gt)
        assert len(pairs) == 2

    def test_close_timestamps(self) -> None:
        est = [_make_pose(0, 0, 0, ts=1.01)]
        gt = [_make_pose(0, 0, 0, ts=1.0)]
        pairs = _align_trajectories(est, gt)
        assert len(pairs) == 1

    def test_far_timestamps_rejected(self) -> None:
        est = [_make_pose(0, 0, 0, ts=5.0)]
        gt = [_make_pose(0, 0, 0, ts=1.0)]
        pairs = _align_trajectories(est, gt)
        assert len(pairs) == 0


# ─── Dataset loading ──────────────────────────────────────────────────────


class TestDatasets:
    def test_synthetic_circle(self) -> None:
        ds = load_dataset("synthetic_circle", max_frames=50)
        assert ds.name == "synthetic_circle"
        assert len(ds.frames) == 50
        assert ds.frames[0].point_cloud.shape[1] == 3

    def test_synthetic_figure8(self) -> None:
        ds = load_dataset("synthetic_figure8", max_frames=50)
        assert ds.name == "synthetic_figure8"
        assert len(ds.frames) == 50

    def test_unknown_dataset(self) -> None:
        with pytest.raises(ValueError, match="Unknown dataset"):
            load_dataset("nonexistent_dataset_xyz")

    @pytest.mark.slow
    def test_tum_fr1_desk_download_and_load(self) -> None:
        """Download TUM fr1/desk (360MB) and verify real sensor data loads."""
        ds = load_dataset("tum_fr1_desk", max_frames=10)
        assert ds.name == "tum_fr1_desk"
        assert len(ds.frames) == 10
        # Real point clouds from depth images should have many points
        assert ds.frames[0].point_cloud.shape[0] > 100
        assert ds.frames[0].point_cloud.shape[1] == 3
        # Should have actual RGB and depth images
        assert ds.frames[0].rgb_image is not None
        assert ds.frames[0].rgb_image.shape == (480, 640, 3)
        assert ds.frames[0].depth_image is not None
        assert ds.frames[0].depth_image.shape == (480, 640)

    def test_tum_sequences_list(self) -> None:
        """All TUM sequence names are valid identifiers."""
        from dimos.navigation.smart_nav.modules.slam_eval.slam_eval import _TUM_SEQUENCES

        assert len(_TUM_SEQUENCES) > 10
        for name in _TUM_SEQUENCES:
            assert name.startswith("tum_")


# ─── Integration: PGO backend ─────────────────────────────────────────────


@_requires_gtsam
class TestPGOBackend:
    def test_process_and_trajectory(self) -> None:
        """PGOBackend produces a trajectory from synthetic data."""
        backend = PGOBackend(key_pose_delta_trans=0.3)
        ds = load_dataset("synthetic_circle", max_frames=100, cloud_points=100)

        for frame in ds.frames:
            backend.process_frame(
                rotation=frame.rotation,
                translation=frame.position,
                timestamp=frame.timestamp,
                point_cloud=frame.point_cloud,
            )

        traj = backend.get_trajectory()
        assert len(traj) > 0

    def test_reset(self) -> None:
        backend = PGOBackend()
        ds = load_dataset("synthetic_circle", max_frames=20, cloud_points=50)
        for frame in ds.frames:
            backend.process_frame(
                frame.rotation, frame.position, frame.timestamp, frame.point_cloud
            )

        backend.reset()
        assert len(backend.get_trajectory()) == 0


# ─── Integration: full slam_eval ──────────────────────────────────────────


@_requires_gtsam
class TestSlamEval:
    def test_full_eval_synthetic(self) -> None:
        """Full eval pipeline produces valid metrics JSON."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "metrics.json"
            backend = PGOBackend(key_pose_delta_trans=0.3)
            results = slam_eval(
                backend,
                dataset="synthetic_circle",
                output_path=output,
                max_frames=100,
                cloud_points=100,
            )

            # Check result structure
            assert "ate" in results
            assert "rpe" in results
            assert "drift" in results
            assert "timing" in results
            assert results["n_frames"] == 100
            assert results["n_keyframes"] > 0
            assert results["backend"] == "PGOBackend"

            # ATE should be finite
            assert math.isfinite(results["ate"]["rmse"])

            # Check JSON was written
            assert output.exists()
            with open(output) as f:
                loaded = json.load(f)
            assert loaded["ate"]["rmse"] == results["ate"]["rmse"]

    def test_eval_figure8(self) -> None:
        """Figure-8 dataset works end-to-end."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "metrics.json"
            backend = PGOBackend(key_pose_delta_trans=0.3)
            results = slam_eval(
                backend,
                dataset="synthetic_figure8",
                output_path=output,
                max_frames=150,
                cloud_points=100,
            )
            assert results["n_keyframes"] > 0
            assert math.isfinite(results["ate"]["rmse"])

    def test_custom_dataset(self) -> None:
        """slam_eval accepts a pre-built EvalDataset."""
        frames = []
        for i in range(20):
            r = Rotation.from_euler("z", i * 10.0, degrees=True).as_matrix()
            frames.append(
                SensorFrame(
                    timestamp=float(i) * 0.5,
                    position=np.array([float(i) * 0.6, 0.0, 0.0]),
                    rotation=r,
                    point_cloud=np.random.default_rng(i).normal(0, 1, (50, 3)).astype(np.float32),
                )
            )
        ds = EvalDataset(name="custom_test", frames=frames)

        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "metrics.json"
            backend = PGOBackend(key_pose_delta_trans=0.3)
            results = slam_eval(backend, dataset=ds, output_path=output)
            assert results["dataset"] == "custom_test"
            assert math.isfinite(results["ate"]["rmse"])


# ─── Noise injection ─────────────────────────────────────────────────────


class TestNoiseProfile:
    def test_presets_exist(self) -> None:
        """All named presets create valid NoiseProfile instances."""
        for name in (
            "none",
            "lidar_mild",
            "lidar_harsh",
            "visual_mild",
            "visual_harsh",
            "wheel_odom",
        ):
            p = getattr(NoiseProfile, name)()
            assert p.label == name

    def test_none_is_zero(self) -> None:
        p = NoiseProfile.none()
        assert p.translation_noise_std == 0.0
        assert p.rotation_noise_std_deg == 0.0
        assert p.translation_drift_rate == 0.0
        assert p.point_cloud_noise_std == 0.0

    def test_lidar_mild_is_small(self) -> None:
        p = NoiseProfile.lidar_mild()
        assert 0 < p.translation_noise_std < 0.05
        assert 0 < p.rotation_noise_std_deg < 1.0

    def test_harsh_noisier_than_mild(self) -> None:
        mild = NoiseProfile.lidar_mild()
        harsh = NoiseProfile.lidar_harsh()
        assert harsh.translation_noise_std > mild.translation_noise_std
        assert harsh.rotation_noise_std_deg > mild.rotation_noise_std_deg
        assert harsh.point_cloud_noise_std > mild.point_cloud_noise_std


class TestApplyNoise:
    def _make_straight_line(self, n: int = 20) -> list[SensorFrame]:
        frames = []
        for i in range(n):
            r = Rotation.from_euler("z", 0.0, degrees=True).as_matrix()
            frames.append(
                SensorFrame(
                    timestamp=float(i) * 0.5,
                    position=np.array([float(i) * 0.5, 0.0, 0.0]),
                    rotation=r,
                    point_cloud=np.zeros((10, 3), dtype=np.float32),
                )
            )
        return frames

    def test_none_noise_unchanged(self) -> None:
        """NoiseProfile.none() should not change frames."""
        frames = self._make_straight_line()
        noisy = apply_noise(frames, NoiseProfile.none())
        for orig, nf in zip(frames, noisy, strict=True):
            np.testing.assert_array_equal(orig.position, nf.position)
            np.testing.assert_array_equal(orig.rotation, nf.rotation)

    def test_noise_changes_positions(self) -> None:
        """Non-zero noise should perturb positions."""
        frames = self._make_straight_line()
        noisy = apply_noise(frames, NoiseProfile.lidar_mild())
        diffs = [
            np.linalg.norm(f.position - n.position) for f, n in zip(frames, noisy, strict=True)
        ]
        # At least some frames should be perturbed
        assert max(diffs) > 0.001

    def test_noise_is_reproducible(self) -> None:
        """Same seed produces same noise."""
        frames = self._make_straight_line()
        noisy1 = apply_noise(frames, NoiseProfile.lidar_mild())
        noisy2 = apply_noise(frames, NoiseProfile.lidar_mild())
        for n1, n2 in zip(noisy1, noisy2, strict=True):
            np.testing.assert_array_equal(n1.position, n2.position)

    def test_drift_accumulates(self) -> None:
        """Later frames should have more drift than earlier ones."""
        frames = self._make_straight_line(50)
        noise = NoiseProfile(
            translation_drift_rate=0.01,
            rotation_drift_rate_deg=0.0,
            translation_noise_std=0.0,
            rotation_noise_std_deg=0.0,
            point_cloud_noise_std=0.0,
            seed=42,
            label="drift_only",
        )
        noisy = apply_noise(frames, noise)
        early_error = np.linalg.norm(frames[5].position - noisy[5].position)
        late_error = np.linalg.norm(frames[-1].position - noisy[-1].position)
        assert late_error > early_error

    def test_point_cloud_noise(self) -> None:
        """Point cloud noise should change cloud values."""
        frames = self._make_straight_line()
        # Give non-zero clouds
        for f in frames:
            f.point_cloud = np.ones((20, 3), dtype=np.float32)
        noise = NoiseProfile(
            point_cloud_noise_std=0.1,
            translation_noise_std=0.0,
            rotation_noise_std_deg=0.0,
            seed=42,
            label="cloud_only",
        )
        noisy = apply_noise(frames, noise)
        assert not np.array_equal(frames[5].point_cloud, noisy[5].point_cloud)


@_requires_gtsam
class TestSlamEvalWithNoise:
    def test_noise_increases_ate(self) -> None:
        """Adding noise should increase ATE compared to no noise."""
        with tempfile.TemporaryDirectory() as tmpdir:
            backend_clean = PGOBackend(key_pose_delta_trans=0.3)
            results_clean = slam_eval(
                backend_clean,
                dataset="synthetic_circle",
                output_path=Path(tmpdir) / "clean.json",
                max_frames=200,
                cloud_points=100,
            )

            backend_noisy = PGOBackend(key_pose_delta_trans=0.3)
            results_noisy = slam_eval(
                backend_noisy,
                dataset="synthetic_circle",
                noise="lidar_harsh",
                output_path=Path(tmpdir) / "noisy.json",
                max_frames=200,
                cloud_points=100,
            )

            assert results_noisy["ate"]["rmse"] > results_clean["ate"]["rmse"]
            assert results_noisy["noise"]["label"] == "lidar_harsh"
            assert results_clean["noise"]["label"] == "none"

    def test_noise_preset_string(self) -> None:
        """Can pass noise as a string preset name."""
        with tempfile.TemporaryDirectory() as tmpdir:
            backend = PGOBackend(key_pose_delta_trans=0.3)
            results = slam_eval(
                backend,
                dataset="synthetic_circle",
                noise="visual_mild",
                output_path=Path(tmpdir) / "metrics.json",
                max_frames=100,
                cloud_points=50,
            )
            assert results["noise"]["label"] == "visual_mild"
            assert math.isfinite(results["ate"]["rmse"])
