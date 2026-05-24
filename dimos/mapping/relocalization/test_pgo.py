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

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.mapping.relocalization.pgo import (
    Keyframe,
    PGOConfig,
    _obs_to_pose3,
    _pose3_to_transform,
    _r_t_from_transform,
    _transform_from_r_t,
    apply_corrections,
    keyframes_to_corrections,
    make_interpolator,
    pgo_keyframes,
)
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _random_R(rng: np.random.Generator) -> np.ndarray:
    """Random uniform rotation matrix via random quaternion."""
    q = rng.standard_normal(4)
    q /= np.linalg.norm(q)
    return np.asarray(Rotation.from_quat(q).as_matrix())


class TestPGOConfig:
    def test_accepts_known_fields(self) -> None:
        cfg = PGOConfig(key_pose_delta_trans=0.7, max_icp_iterations=42)
        assert cfg.key_pose_delta_trans == 0.7
        assert cfg.max_icp_iterations == 42

    def test_rejects_unknown_fields(self) -> None:
        # The plan deleted these; BaseConfig has extra="forbid" so they raise.
        for dead in (
            "world_frame",
            "publish_global_map",
            "global_map_publish_rate",
            "global_map_voxel_size",
            "unregister_input",
        ):
            with pytest.raises(Exception):
                PGOConfig(**{dead: True})


class TestTransformHelpers:
    def test_r_t_transform_roundtrip(self) -> None:
        rng = np.random.default_rng(0)
        R_in = _random_R(rng)
        t_in = rng.uniform(-5, 5, size=3)
        tf = _transform_from_r_t(R_in, t_in, ts=1.23)
        R_out, t_out = _r_t_from_transform(tf)
        np.testing.assert_allclose(R_out, R_in, atol=1e-10)
        np.testing.assert_allclose(t_out, t_in, atol=1e-10)
        assert tf.ts == 1.23

    def test_observation_normalizes_transform_pose(self) -> None:
        """Constructing/deriving with pose=Transform should coerce to 7-tuple."""
        from dimos.memory2.type.observation import Observation

        tf = Transform(
            translation=Vector3(1.5, -2.0, 0.7),
            rotation=Quaternion(0.1, 0.2, 0.3, 0.927),
            ts=1.0,
        )
        obs: Observation[int] = Observation(id=0, ts=1.0, pose=tf, _data=0)
        assert isinstance(obs.pose, tuple)
        assert obs.pose[0] == pytest.approx(1.5)
        assert obs.pose[6] == pytest.approx(0.927)

        # derive() also runs the normalization via __post_init__.
        derived = obs.derive(data=0, pose=tf)
        assert isinstance(derived.pose, tuple)
        assert derived.pose == obs.pose

    def test_observation_normalizes_posestamped(self) -> None:
        from dimos.memory2.type.observation import Observation
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        ps = PoseStamped(ts=1.0, position=(1.0, 2.0, 3.0), orientation=(0.0, 0.0, 0.0, 1.0))
        obs: Observation[int] = Observation(id=0, ts=1.0, pose=ps, _data=0)
        assert isinstance(obs.pose, tuple)
        assert obs.pose == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)

    def test_obs_to_pose3_roundtrip(self) -> None:
        from dimos.memory2.type.observation import Observation

        rng = np.random.default_rng(4)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        tf = _transform_from_r_t(R, t, ts=1.0)
        obs: Observation[int] = Observation(id=0, ts=1.0, pose=tf, _data=0)
        p = _obs_to_pose3(obs)
        np.testing.assert_allclose(p.rotation().matrix(), R, atol=1e-9)
        np.testing.assert_allclose(np.asarray(p.translation()), t, atol=1e-9)

    def test_pose3_to_transform(self) -> None:
        import gtsam  # type: ignore[import-not-found,import-untyped]

        rng = np.random.default_rng(2)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        p = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t))
        tf = _pose3_to_transform(p, ts=7.89)
        R_out, t_out = _r_t_from_transform(tf)
        np.testing.assert_allclose(R_out, R, atol=1e-10)
        np.testing.assert_allclose(t_out, t, atol=1e-10)

    def test_pose3_to_transform_with_frames(self) -> None:
        import gtsam

        rng = np.random.default_rng(3)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        p = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t))
        tf = _pose3_to_transform(p, ts=1.0, frame_id="world_corrected", child_frame_id="body")
        assert tf.frame_id == "world_corrected"
        assert tf.child_frame_id == "body"
        R_out, t_out = _r_t_from_transform(tf)
        np.testing.assert_allclose(R_out, R, atol=1e-10)
        np.testing.assert_allclose(t_out, t, atol=1e-10)


def _make_lidar_stream(n_frames: int = 12, points_per_frame: int = 500) -> Stream[PointCloud2]:
    """Straight-line trajectory along +x with small yaw, random body points.

    Note: `pgo_keyframes` skips poses with zero translation OR identity
    rotation as placeholders, so we use a constant non-identity yaw.
    """
    rng = np.random.default_rng(0)
    mem = MemoryStore()
    lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
    # Small yaw (~6 deg) -> non-identity quaternion that survives the
    # placeholder filter.
    q = Rotation.from_euler("z", 0.1).as_quat()  # xyzw
    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    R_world = Rotation.from_euler("z", 0.1).as_matrix()
    for i in range(1, n_frames + 1):
        body = rng.uniform(-1, 1, size=(points_per_frame, 3)).astype(np.float32)
        world = (R_world @ body.T).T + np.array([i, 0, 0], dtype=np.float32)
        lidar.append(
            PointCloud2.from_numpy(world.astype(np.float32)),
            ts=float(i),
            pose=(float(i), 0.0, 0.0, qx, qy, qz, qw),
        )
    return lidar


class TestPipelineEndToEnd:
    def test_straight_line_produces_keyframes(self) -> None:
        lidar = _make_lidar_stream(n_frames=12)
        kfs = pgo_keyframes(lidar)
        # 12 frames spaced 1m apart with key_pose_delta_trans=0.5 -> every frame
        # after the first triggers a keyframe; some may dedupe but ~11 emitted.
        n = kfs.count()
        assert 10 <= n <= 12

    def test_keyframes_to_corrections_size(self) -> None:
        lidar = _make_lidar_stream(n_frames=12)
        kfs = pgo_keyframes(lidar)
        corr = keyframes_to_corrections(kfs)
        assert corr.count() == kfs.count()

    def test_apply_identity_corrections_preserves_poses(self) -> None:
        # With no loop closures the optimization is a no-op -> drift = identity ->
        # apply_corrections is a no-op on input poses.
        lidar = _make_lidar_stream(n_frames=12)
        kfs = pgo_keyframes(lidar)
        corr = keyframes_to_corrections(kfs)
        corrected = apply_corrections(lidar, corr)
        in_poses = [o.pose for o in lidar if o.pose is not None]
        out_poses = [o.pose for o in corrected if o.pose is not None]
        assert len(in_poses) == len(out_poses)
        for p_in, p_out in zip(in_poses, out_poses, strict=True):
            for a, b in zip(p_in, p_out, strict=True):
                assert a == pytest.approx(b, abs=1e-6)


def _make_corrections(transforms: list[Transform]) -> Stream[Transform]:
    mem = MemoryStore()
    stream: Stream[Transform] = mem.stream("corrections", Transform)
    for tf in transforms:
        stream.append(tf, ts=tf.ts)
    return stream


class TestInterpolator:
    def test_empty_stream_raises(self) -> None:
        empty = _make_corrections([])
        with pytest.raises(ValueError):
            make_interpolator(empty)

    def test_single_keyframe_returns_constant(self) -> None:
        R = Rotation.from_euler("z", np.pi / 4).as_matrix()
        t = np.array([1.0, 2.0, 3.0])
        only = _transform_from_r_t(R, t, ts=10.0)
        interp = make_interpolator(_make_corrections([only]))
        for query_ts in (0.0, 10.0, 100.0):
            out = interp(query_ts)
            assert out.translation.x == pytest.approx(1.0, abs=1e-10)
            assert out.translation.y == pytest.approx(2.0, abs=1e-10)
            assert out.translation.z == pytest.approx(3.0, abs=1e-10)

    def test_out_of_range_clips_to_endpoints(self) -> None:
        # Note: Transform's constructor maps ts=0.0 -> time.time(); use ts>0
        # so test timestamps are deterministic.
        R = np.eye(3)
        a = _transform_from_r_t(R, np.array([0.0, 0.0, 0.0]), ts=1.0)
        b = _transform_from_r_t(R, np.array([10.0, 0.0, 0.0]), ts=11.0)
        # _make_corrections uses tf.ts; obs.ts ends up the same.
        mem = MemoryStore()
        stream: Stream[Transform] = mem.stream("corrections", Transform)
        stream.append(a, ts=1.0)
        stream.append(b, ts=11.0)
        interp = make_interpolator(stream)
        # Below range -> clipped to a
        assert interp(-5.0).translation.x == pytest.approx(0.0, abs=1e-10)
        # Above range -> clipped to b
        assert interp(100.0).translation.x == pytest.approx(10.0, abs=1e-10)
        # In-range midpoint
        assert interp(6.0).translation.x == pytest.approx(5.0, abs=1e-10)


class TestApplyCorrections:
    def test_pure_translation_shifts_poses(self) -> None:
        # Build a stream of 3 frames at the origin (identity pose) with a known
        # correction that shifts everything by +5 in x. Expected: corrected
        # poses sit at x=5.
        mem = MemoryStore()
        lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
        for i in range(3):
            lidar.append(
                PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32)),
                ts=float(i + 1),
                pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            )

        # Constant correction: translate by (5, 0, 0), identity rotation.
        c_a = _transform_from_r_t(np.eye(3), np.array([5.0, 0.0, 0.0]), ts=1.0)
        c_b = _transform_from_r_t(np.eye(3), np.array([5.0, 0.0, 0.0]), ts=3.0)
        mem2 = MemoryStore()
        corr: Stream[Transform] = mem2.stream("corrections", Transform)
        corr.append(c_a, ts=1.0)
        corr.append(c_b, ts=3.0)

        corrected = apply_corrections(lidar, corr)
        for obs in corrected:
            assert obs.pose is not None
            assert obs.pose[0] == pytest.approx(5.0, abs=1e-9)
            assert obs.pose[1] == pytest.approx(0.0, abs=1e-9)
            assert obs.pose[2] == pytest.approx(0.0, abs=1e-9)

    def test_passes_through_pose_none(self) -> None:
        mem = MemoryStore()
        lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
        lidar.append(
            PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32)),
            ts=1.0,
            pose=None,
        )
        c_a = _transform_from_r_t(np.eye(3), np.array([5.0, 0.0, 0.0]), ts=1.0)
        c_b = _transform_from_r_t(np.eye(3), np.array([5.0, 0.0, 0.0]), ts=2.0)
        mem2 = MemoryStore()
        corr: Stream[Transform] = mem2.stream("corrections", Transform)
        corr.append(c_a, ts=1.0)
        corr.append(c_b, ts=2.0)
        corrected = apply_corrections(lidar, corr)
        for obs in corrected:
            assert obs.pose is None


class TestKeyframeType:
    def test_keyframe_is_frozen(self) -> None:
        identity = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=1.0,
        )
        kf = Keyframe(ts=1.0, local=identity, optimized=identity)
        with pytest.raises(Exception):
            kf.ts = 2.0  # type: ignore[misc]
        assert isinstance(kf.local, Transform)
        assert isinstance(kf.optimized, Transform)


# Real-recording smoke test. ~45-60s on go2_short.db. Skipped when the LFS
# file isn't present (CI without LFS pulled, fresh clone, etc.).
_DATA_DB = Path(__file__).resolve().parents[3] / "data" / "go2_short.db"


@pytest.mark.skipif(not _DATA_DB.exists(), reason=f"requires LFS file {_DATA_DB}")
class TestRealRecording:
    def test_pgo_pipeline_against_go2_short(self) -> None:
        """Run the full PGO pipeline on a real 60-second go2 recording.

        Asserts: keyframes produced, drift correction actually corrects (some
        optimized poses differ from local), correction stream length matches
        keyframes, apply_corrections preserves input frame count.
        """
        from dimos.memory2.store.sqlite import SqliteStore

        store = SqliteStore(path=_DATA_DB)
        lidar = store.streams.lidar
        in_count = lidar.count()
        assert in_count > 0, "recording is empty"

        kfs = pgo_keyframes(lidar)
        n_kf = kfs.count()
        assert n_kf > 0, "PGO emitted no keyframes"
        # 60s recording at ~0.5m keyframe spacing -> at least a handful.
        assert n_kf >= 5

        # Loop closure detection: at least one optimized pose should differ
        # from its odom-frame counterpart. Without loops, the optimization is
        # a no-op and local == optimized for every keyframe.
        drifted = sum(
            1
            for obs in kfs
            if obs.data.local.translation != obs.data.optimized.translation
            or obs.data.local.rotation != obs.data.optimized.rotation
        )
        assert drifted > 0, "expected loop closures to drift at least one keyframe"

        corr = keyframes_to_corrections(kfs)
        assert corr.count() == n_kf

        # apply_corrections preserves frame count, including pose=None rows.
        corrected = apply_corrections(lidar, corr)
        out_count = sum(1 for _ in corrected)
        assert out_count == in_count
