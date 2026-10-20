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

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.memory.store.memory import MemoryStore
from dimos.memory.stream import Stream
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry import quaternion_from_matrix, transform_matrix, vector_array
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz, transform_cloud
from dimos.msgs.time import time_from_nanoseconds, time_from_seconds, to_seconds
from dimos.navigation.go2.loop_closure.pgo import (
    PGO,
    Keyframe,
    PGOConfig,
    PoseGraph,
    _icp,
    _obs_to_pose3,
    _PGOState,
    _pose3_to_transform,
)

# TODO(PY311): drop — the mapping extra excludes gtsam-extended where it has no
# wheels (py3.10 Linux), see pyproject.
pytest.importorskip("gtsam")


def _transform(*, translation=None, rotation=None, ts=0.0):
    return TransformStamped(
        header=Header(stamp=time_from_seconds(ts), frame_id="world_raw"),
        child_frame_id="body",
        transform=Transform(
            translation=translation if translation is not None else Vector3(0, 0, 0),
            rotation=rotation if rotation is not None else Quaternion(0, 0, 0, 1),
        ),
    )


def _cloud(points, *, timestamp=0.0, stamps=None):
    return pointcloud_from_xyz(
        points,
        header=Header(stamp=time_from_seconds(timestamp), frame_id="world_raw"),
        stamps=stamps,
    )


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
        from dimos.navigation.go2.loop_closure.pgo import PGOKwargs

        assert set(PGOConfig.model_fields.keys()) == set(PGOKwargs.__annotations__.keys())


class TestTransformHelpers:
    def test_observation_normalizes_transform_pose(self) -> None:
        """Constructing/deriving with pose=Transform should coerce to 7-tuple."""
        from dimos.memory.type.observation import Observation

        tf = _transform(
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

        ps = PoseStamped(
            header=Header(stamp=time_from_seconds(1.0), frame_id="world_raw"),
            pose=Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        )
        obs: Observation[int] = Observation(id=0, ts=1.0, pose=ps, _data=0)
        assert obs.pose_tuple == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)

    def test_obs_to_pose3_roundtrip(self) -> None:
        from dimos.memory.type.observation import Observation

        rng = np.random.default_rng(4)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        tf = _transform(translation=Vector3(*t), rotation=quaternion_from_matrix(R), ts=1.0)
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
        np.testing.assert_allclose(transform_matrix(tf.transform)[:3, :3], R, atol=1e-10)
        np.testing.assert_allclose(vector_array(tf.transform.translation), t, atol=1e-10)

    def test_pose3_to_transform_with_frames(self) -> None:
        import gtsam

        rng = np.random.default_rng(3)
        R = _random_R(rng)
        t = rng.uniform(-3, 3, size=3)
        p = gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(t))
        tf = _pose3_to_transform(p, ts=1.0, frame_id="world_corrected", child_frame_id="body")
        assert tf.header.frame_id == "world_corrected"
        assert tf.child_frame_id == "body"
        np.testing.assert_allclose(transform_matrix(tf.transform)[:3, :3], R, atol=1e-10)
        np.testing.assert_allclose(vector_array(tf.transform.translation), t, atol=1e-10)


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
            _cloud(world.astype(np.float32)),
            ts=float(i),
            pose=(float(i), 0.0, 0.0, qx, qy, qz, qw),
        )
    return lidar


def test_state_reports_keyframes_anchored_at_the_latest() -> None:
    state = _PGOState(PGOConfig())
    for obs in _make_lidar_stream(n_frames=12):
        state.process(_obs_to_pose3(obs), obs.ts, obs.data)

    positions, quats = state.keyframe_poses()
    assert positions.shape == (state.n_keyframes, 3)
    assert quats.shape == (state.n_keyframes, 4)
    # no loop closed, so nothing moved: keyframes sit on the odometry line
    np.testing.assert_allclose(positions[-1], [12.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(positions[:, 1:], 0.0, atol=1e-6)

    segments, scores = state.loop_segments()
    assert segments.shape == (0, 2, 3)
    assert scores.shape == (0,)


class TestPipelineEndToEnd:
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
                ts=to_seconds(d.header.stamp),
                local=_transform(
                    translation=identity, rotation=identity_rot, ts=to_seconds(d.header.stamp)
                ),
                optimized=_transform(
                    translation=d.transform.translation,
                    rotation=d.transform.rotation,
                    ts=to_seconds(d.header.stamp),
                ),
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
        only = _transform(
            translation=Vector3(1.0, 2.0, 3.0),
            rotation=quaternion_from_matrix(R),
            ts=10.0,
        )
        graph = _graph_with_drift_at([only])
        for query_ts in (0.0, 10.0, 100.0):
            out = graph.correction_at(query_ts)
            assert out.transform.translation.x == pytest.approx(1.0, abs=1e-10)
            assert out.transform.translation.y == pytest.approx(2.0, abs=1e-10)
            assert out.transform.translation.z == pytest.approx(3.0, abs=1e-10)

    def test_out_of_range_clips_to_endpoints(self) -> None:
        a = _transform(translation=Vector3(0.0, 0.0, 0.0), ts=1.0)
        b = _transform(translation=Vector3(10.0, 0.0, 0.0), ts=11.0)
        graph = _graph_with_drift_at([a, b])
        # Below range -> clipped to a
        assert graph.correction_at(-5.0).transform.translation.x == pytest.approx(0.0, abs=1e-10)
        # Above range -> clipped to b
        assert graph.correction_at(100.0).transform.translation.x == pytest.approx(10.0, abs=1e-10)
        # In-range midpoint
        assert graph.correction_at(6.0).transform.translation.x == pytest.approx(5.0, abs=1e-10)

    def test_frozen(self) -> None:
        graph = PoseGraph()
        with pytest.raises(Exception):
            graph.keyframes = (Keyframe(ts=0, local=_transform(), optimized=_transform()),)  # type: ignore[misc]


class TestPlace:
    def _graph(self) -> PoseGraph:
        turn = quaternion_from_matrix(Rotation.from_euler("z", np.pi / 2).as_matrix())
        return _graph_with_drift_at(
            [
                _transform(translation=Vector3(0.0, 0.0, 0.0), ts=1.0),
                _transform(translation=Vector3(10.0, 0.0, 0.0), rotation=turn, ts=11.0),
            ]
        )

    def test_latest_data_stays_put(self) -> None:
        cloud = _cloud(np.array([[1.0, 2.0, 3.0]]), timestamp=11.0)
        np.testing.assert_allclose(
            pointcloud_xyz(self._graph().place(cloud)), [[1.0, 2.0, 3.0]], atol=1e-5
        )

    def test_old_data_moves_relative_to_the_latest_keyframe(self) -> None:
        # C(now)^-1 . C(1): undo the latest correction (turn 90 deg, +10 x).
        cloud = _cloud(np.array([[10.0, 1.0, 0.0]]), timestamp=1.0)
        np.testing.assert_allclose(
            pointcloud_xyz(self._graph().place(cloud)), [[1.0, 0.0, 0.0]], atol=1e-5
        )

    def test_per_point_stamps_override_the_cloud_ts(self) -> None:
        cloud = _cloud(
            np.array([[10.0, 1.0, 0.0], [1.0, 2.0, 3.0]]),
            timestamp=1.0,
            stamps=np.array([1.0, 11.0]),
        )
        np.testing.assert_allclose(
            pointcloud_xyz(self._graph().place(cloud)),
            [[1.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
            atol=1e-5,
        )


class TestApplyAsTransformer:
    def test_pure_translation_shifts_poses(self) -> None:
        # Build a stream of 3 frames at the origin (identity pose) with a known
        # correction that shifts everything by +5 in x. Expected: corrected
        # poses sit at x=5.
        mem = MemoryStore()
        lidar: Stream[PointCloud2] = mem.stream("lidar", PointCloud2)
        for i in range(3):
            lidar.append(
                _cloud(np.zeros((1, 3), dtype=np.float32)),
                ts=float(i + 1),
                pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            )
        graph = _graph_with_drift_at(
            [
                _transform(translation=Vector3(5.0, 0.0, 0.0), ts=1.0),
                _transform(translation=Vector3(5.0, 0.0, 0.0), ts=3.0),
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
            _cloud(np.zeros((1, 3), dtype=np.float32)),
            ts=1.0,
            pose=None,
        )
        graph = _graph_with_drift_at(
            [
                _transform(translation=Vector3(5.0, 0.0, 0.0), ts=1.0),
                _transform(translation=Vector3(5.0, 0.0, 0.0), ts=2.0),
            ]
        )
        for obs in lidar.transform(graph):
            assert obs.pose is None


class TestKeyframeType:
    def test_keyframe_is_frozen(self) -> None:
        identity = _transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ts=1.0,
        )
        kf = Keyframe(ts=1.0, local=identity, optimized=identity)
        with pytest.raises(Exception):
            kf.ts = 2.0  # type: ignore[misc]
        assert isinstance(kf.local, TransformStamped)
        assert isinstance(kf.optimized, TransformStamped)


def test_place_retains_capture_fields_header_and_input_bytes():
    graph = _graph_with_drift_at(
        [_transform(translation=Vector3(5, 0, 0), ts=1), _transform(ts=11)]
    )
    cloud = _cloud(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]), stamps=np.array([1.0, 11.0]))
    cloud.header.stamp = time_from_nanoseconds(1700000000123456789)
    before = encode(cloud)
    output = decode(encode(graph.place(cloud)), PointCloud2)
    np.testing.assert_allclose(pointcloud_xyz(output), [[6, 2, 3], [4, 5, 6]])
    assert output.header == cloud.header and output.fields == cloud.fields
    assert encode(cloud) == before


def test_correct_preserves_exact_input_stamp_and_child_frame():
    graph = _graph_with_drift_at([_transform(translation=Vector3(5, 0, 0), ts=1)])
    pose = _transform(translation=Vector3(1, 2, 3), ts=1)
    pose.header.stamp = time_from_nanoseconds(1700000000123456789)
    before = encode(pose)
    corrected = decode(encode(graph.correct(pose)), TransformStamped)
    assert corrected.header.stamp == pose.header.stamp
    assert corrected.header.frame_id == "world_corrected" and corrected.child_frame_id == "body"
    np.testing.assert_allclose(vector_array(corrected.transform.translation), [6, 2, 3])
    assert encode(pose) == before


def test_actual_point_to_plane_icp_recovers_rigid_motion():
    axis = np.linspace(-1, 1, 17)
    u, v = np.meshgrid(axis, axis)
    walls = np.concatenate(
        [
            np.column_stack([np.full(u.size, -1), u.ravel(), v.ravel()]),
            np.column_stack([u.ravel(), np.full(u.size, -1), v.ravel()]),
            np.column_stack([u.ravel(), v.ravel(), np.full(u.size, -1)]),
        ]
    )
    target = _cloud(walls, timestamp=7.25)
    motion = _transform(
        translation=Vector3(0.03, -0.02, 0.01),
        rotation=quaternion_from_matrix(
            Rotation.from_euler("xyz", [0.01, -0.015, 0.02]).as_matrix()
        ),
        ts=7.25,
    )
    motion.header.frame_id = motion.child_frame_id = target.header.frame_id
    source = transform_cloud(target, motion)
    before = encode(source)
    correction, fitness = _icp(source, target, max_dist=0.2)
    np.testing.assert_allclose(
        transform_matrix(correction.transform),
        np.linalg.inv(transform_matrix(motion.transform)),
        atol=1e-4,
    )
    assert fitness < 1e-8
    assert encode(source) == before
