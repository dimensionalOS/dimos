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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.mapping.loop_closure.pgo import (
    PGO,
    Keyframe,
    PGOConfig,
    PoseGraph,
    _obs_to_pose3,
    _PGOState,
    _pose3_to_transform,
    _transform_to_pose3,
)
from dimos.memory.store.memory import MemoryStore
from dimos.memory.stream import Stream
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# TODO(PY311): drop — the mapping extra excludes gtsam-extended where it has no
# wheels (py3.10 Linux), see pyproject.
pytest.importorskip("gtsam")


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

    def test_kwargs_typed_dict_matches_config(self) -> None:
        """`PGOKwargs` must mirror every `PGOConfig` field 1:1."""
        from dimos.mapping.loop_closure.pgo import PGOKwargs

        assert set(PGOConfig.model_fields.keys()) == set(PGOKwargs.__annotations__.keys())


class TestTransformHelpers:
    def test_observation_normalizes_transform_pose(self) -> None:
        """Constructing/deriving with pose=Transform should coerce to 7-tuple."""
        from dimos.memory.type.observation import Observation

        tf = Transform(
            translation=Vector3(1.5, -2.0, 0.7),
            rotation=Quaternion(0.1, 0.2, 0.3, 0.927),
            ts=1.0,
        )
        obs: Observation[int] = Observation(id=0, ts=1.0, pose=tf, _data=0)
        assert obs.pose_tuple is not None
        assert obs.pose_tuple[0] == pytest.approx(1.5)
        assert obs.pose_tuple[6] == pytest.approx(0.927)

        # derive() also re-runs the normalization.
        derived = obs.derive(data=0, pose=tf)
        assert derived.pose_tuple == obs.pose_tuple

    def test_observation_normalizes_posestamped(self) -> None:
        from dimos.memory.type.observation import Observation
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        ps = PoseStamped(ts=1.0, position=(1.0, 2.0, 3.0), orientation=(0.0, 0.0, 0.0, 1.0))
        obs: Observation[int] = Observation(id=0, ts=1.0, pose=ps, _data=0)
        assert obs.pose_tuple == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)

    def test_obs_to_pose3_roundtrip(self) -> None:
        from dimos.memory.type.observation import Observation

        rng = np.random.default_rng(4)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        tf = Transform(translation=Vector3(t), rotation=Quaternion.from_rotation_matrix(R), ts=1.0)
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
        tf = _pose3_to_transform(p, ts=7.89, frame_id="world", child_frame_id="body")
        np.testing.assert_allclose(tf.rotation.to_rotation_matrix(), R, atol=1e-10)
        np.testing.assert_allclose(tf.translation.to_numpy(), t, atol=1e-10)

    def test_pose3_to_transform_with_frames(self) -> None:
        import gtsam

        rng = np.random.default_rng(3)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        p = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t))
        tf = _pose3_to_transform(p, ts=1.0, frame_id="world_corrected", child_frame_id="body")
        assert tf.frame_id == "world_corrected"
        assert tf.child_frame_id == "body"
        np.testing.assert_allclose(tf.rotation.to_rotation_matrix(), R, atol=1e-10)
        np.testing.assert_allclose(tf.translation.to_numpy(), t, atol=1e-10)


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


class TestSubmapExclusion:
    @pytest.mark.parametrize(
        "excluded, expected",
        [
            (1, [21.0, 31.0]),
            (2, [11.0, 31.0]),
            (3, [11.0, 21.0]),
            (0, [11.0, 21.0, 31.0]),
            (4, [11.0, 21.0, 31.0]),
            (None, [11.0, 21.0, 31.0]),
        ],
    )
    def test_members(self, excluded: int | None, expected: list[float]) -> None:
        state = _PGOState(PGOConfig())
        for i in range(5):
            ts = 100.0 + i
            tf = Transform(translation=Vector3(10.0 * i + 1.0, 0.0, 0.0), ts=ts)
            pose = _transform_to_pose3(tf)
            cloud = PointCloud2.from_numpy(np.array([[0.0, 0.0, 0.0]]), timestamp=ts)
            state.process(pose, ts, cloud.transform(tf))
        points, _ = state._get_submap(2, 1, exclude_idx=excluded).as_numpy()
        np.testing.assert_allclose(np.sort(points[:, 0]), expected)
        assert len(state._get_submap(4, 0, exclude_idx=4)) == 0
        singleton, _ = state._get_submap(4, 0).as_numpy()
        np.testing.assert_allclose(singleton, [[41.0, 0.0, 0.0]])


class TestPipelineEndToEnd:
    def test_closed_trajectory_still_improves_position(self) -> None:
        u, v = np.meshgrid(np.arange(-3.0, 3.01, 0.15), np.arange(-3.0, 3.01, 0.15))
        u, v = u.ravel(), v.ravel()
        room = np.concatenate(
            [
                np.column_stack([u, v, np.full_like(u, -3.0)]),
                np.column_stack([np.full_like(u, 3.0), u, v]),
                np.column_stack([u, np.full_like(u, 3.0), v]),
            ]
        )
        angles = np.linspace(0.0, 2.0 * np.pi, 33)
        truth = np.column_stack([4.0 * np.cos(angles) + 1.0, 4.0 * np.sin(angles), np.ones(33)])
        raw = truth.copy()
        raw[:, 0] += np.linspace(0.0, 0.12, 33)
        mem = MemoryStore()
        lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
        for i in range(33):
            ts = 100.0 + 3.0 * i
            lidar.append(
                PointCloud2.from_numpy(room + raw[i] - truth[i], timestamp=ts),
                ts=ts,
                pose=(*raw[i].tolist(), 0.0, 0.0, 0.0, 1.0),
            )
        graph = lidar.transform(PGO()).last().data
        assert len(graph.keyframes) == 33
        assert len(graph.loops) > 0
        corrected = np.array([k.optimized.translation.to_numpy() for k in graph.keyframes])
        assert np.isfinite(corrected).all()
        assert np.mean(np.sum((corrected - truth) ** 2, axis=1)) < np.mean(
            np.sum((raw - truth) ** 2, axis=1)
        )
        assert np.max(np.linalg.norm(corrected - truth, axis=1)) < 0.15

    def test_no_loop_without_historical_cloud_overlap(self) -> None:
        # Three perpendicular planes constrain ICP. All historical scans are
        # far from the current scan, despite poses passing the candidate gates.
        u, v = np.meshgrid(np.arange(-3.0, 3.01, 0.15), np.arange(-3.0, 3.01, 0.15))
        u, v = u.ravel(), v.ravel()
        room = np.concatenate(
            [
                np.column_stack([u, v, np.full_like(u, -3.0)]),
                np.column_stack([np.full_like(u, 3.0), u, v]),
                np.column_stack([u, np.full_like(u, 3.0), v]),
            ]
        )
        mem = MemoryStore()
        lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
        for i in range(10):
            ts = 100.0 + 3.0 * i
            world = room + 100.0 if i < 9 else room
            x = float(i + 1) if i < 9 else 1.0
            lidar.append(
                PointCloud2.from_numpy(world.astype(np.float32), timestamp=ts),
                ts=ts,
                pose=(x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            )

        graph = lidar.transform(PGO()).last().data
        assert len(graph.keyframes) == 10
        assert len(graph.loops) == 0

    def test_straight_line_produces_keyframes(self) -> None:
        lidar = _make_lidar_stream(n_frames=12)
        graph = lidar.transform(PGO()).last().data
        # 12 frames spaced 1m apart with key_pose_delta_trans=0.5 -> every frame
        # after the first triggers a keyframe; some may dedupe but ~11 emitted.
        n = len(graph.keyframes)
        assert 10 <= n <= 12

    def test_apply_identity_corrections_preserves_poses(self) -> None:
        # With no loop closures the optimization is a no-op -> drift = identity ->
        # stream.transform(graph) is a no-op on input poses.
        lidar = _make_lidar_stream(n_frames=12)
        graph = lidar.transform(PGO()).last().data
        corrected = lidar.transform(graph)
        in_poses = [o.pose_tuple for o in lidar if o.pose_tuple is not None]
        out_poses = [o.pose_tuple for o in corrected if o.pose_tuple is not None]
        assert len(in_poses) == len(out_poses)
        for p_in, p_out in zip(in_poses, out_poses, strict=True):
            for a, b in zip(p_in, p_out, strict=True):
                assert a == pytest.approx(b, abs=1e-6)


def _graph_with_drift_at(drifts: list[Transform]) -> PoseGraph:
    """PoseGraph whose drift correction equals each ``drifts[i]`` at ``drifts[i].ts``.

    Trick: drift = optimized + local^-1. With local=identity, drift==optimized.
    """
    identity = Vector3(0.0, 0.0, 0.0)
    identity_rot = Quaternion(0.0, 0.0, 0.0, 1.0)
    return PoseGraph(
        keyframes=tuple(
            Keyframe(
                ts=d.ts,
                local=Transform(translation=identity, rotation=identity_rot, ts=d.ts),
                optimized=Transform(translation=d.translation, rotation=d.rotation, ts=d.ts),
            )
            for d in drifts
        )
    )


class TestPoseGraphCorrection:
    def test_empty_raises(self) -> None:
        with pytest.raises(ValueError):
            PoseGraph().correction_at(0.0)

    def test_single_keyframe_returns_constant(self) -> None:
        R = Rotation.from_euler("z", np.pi / 4).as_matrix()
        only = Transform(
            translation=Vector3(1.0, 2.0, 3.0),
            rotation=Quaternion.from_rotation_matrix(R),
            ts=10.0,
        )
        graph = _graph_with_drift_at([only])
        for query_ts in (0.0, 10.0, 100.0):
            out = graph.correction_at(query_ts)
            assert out.translation.x == pytest.approx(1.0, abs=1e-10)
            assert out.translation.y == pytest.approx(2.0, abs=1e-10)
            assert out.translation.z == pytest.approx(3.0, abs=1e-10)

    def test_out_of_range_clips_to_endpoints(self) -> None:
        # Transform's ctor maps ts=0.0 -> time.time(); use ts>0 for determinism.
        a = Transform(translation=Vector3(0.0, 0.0, 0.0), ts=1.0)
        b = Transform(translation=Vector3(10.0, 0.0, 0.0), ts=11.0)
        graph = _graph_with_drift_at([a, b])
        # Below range -> clipped to a
        assert graph.correction_at(-5.0).translation.x == pytest.approx(0.0, abs=1e-10)
        # Above range -> clipped to b
        assert graph.correction_at(100.0).translation.x == pytest.approx(10.0, abs=1e-10)
        # In-range midpoint
        assert graph.correction_at(6.0).translation.x == pytest.approx(5.0, abs=1e-10)

    def test_frozen(self) -> None:
        graph = PoseGraph()
        with pytest.raises(Exception):
            graph.keyframes = (Keyframe(ts=0, local=Transform(), optimized=Transform()),)  # type: ignore[misc]


class TestApplyAsTransformer:
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
        graph = _graph_with_drift_at(
            [
                Transform(translation=Vector3(5.0, 0.0, 0.0), ts=1.0),
                Transform(translation=Vector3(5.0, 0.0, 0.0), ts=3.0),
            ]
        )
        for obs in lidar.transform(graph):
            p = obs.pose_tuple
            assert p is not None
            assert p[0] == pytest.approx(5.0, abs=1e-9)
            assert p[1] == pytest.approx(0.0, abs=1e-9)
            assert p[2] == pytest.approx(0.0, abs=1e-9)

    def test_passes_through_pose_none(self) -> None:
        mem = MemoryStore()
        lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
        lidar.append(
            PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32)),
            ts=1.0,
            pose=None,
        )
        graph = _graph_with_drift_at(
            [
                Transform(translation=Vector3(5.0, 0.0, 0.0), ts=1.0),
                Transform(translation=Vector3(5.0, 0.0, 0.0), ts=2.0),
            ]
        )
        for obs in lidar.transform(graph):
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
