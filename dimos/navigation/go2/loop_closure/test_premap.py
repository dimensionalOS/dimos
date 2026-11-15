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

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

# TODO(PY311): drop. Skip before importing test_pgo, which skips mid-import without gtsam.
gtsam = pytest.importorskip("gtsam")

from dimos.navigation.go2.loop_closure.pgo_map import PGOMap
from dimos.navigation.go2.loop_closure.premap import ROS_GTSAM_SWAP, Premap
from dimos.navigation.go2.loop_closure.test_pgo import _make_lidar_stream


def _to_gtsam(premap: Premap):
    """The premap as a gtsam graph: keyframe i as key i, keyframe 0 pinned."""
    graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
    for i, m in enumerate(premap.poses):
        values.insert(i, gtsam.Pose3(m))
    pinned = gtsam.noiseModel.Diagonal.Variances(np.full(6, 1e-12))
    graph.add(gtsam.PriorFactorPose3(0, values.atPose3(0), pinned))
    for (a, b), m, cov in zip(
        premap.edges, premap.edge_poses, premap.edge_covariances, strict=True
    ):
        noise = gtsam.noiseModel.Gaussian.Covariance(cov[ROS_GTSAM_SWAP][:, ROS_GTSAM_SWAP])
        graph.add(gtsam.BetweenFactorPose3(int(a), int(b), gtsam.Pose3(m), noise))
    return graph, values


def _premap() -> tuple[Premap, np.ndarray]:
    lidar = _make_lidar_stream(n_frames=12)
    world_map = PGOMap()
    clouds = []
    for obs in lidar:
        obs.data.ts = obs.ts
        clouds.append(obs.data)
        world_map.add(obs.data, obs.pose)
    premap = world_map.joint_premap()
    world_map.dispose()
    assert premap is not None
    return premap, np.concatenate([c.points_f32() for c in clouds])


def test_a_map_built_by_the_live_mapper_keeps_what_its_frames_saw() -> None:
    premap, world = _premap()
    assert len(premap) >= 1
    assert premap.edges.max(initial=0) < len(premap)
    # no loop closed, so nothing moved: every mapped point is on something a frame saw
    d, _ = cKDTree(world).query(premap.cloud().points_f32())
    assert d.max() < 0.05 * np.sqrt(3)


def test_save_load_roundtrip(tmp_path: Path) -> None:
    premap, _ = _premap()
    premap.save(tmp_path / "office.map.db")
    back = Premap.load(tmp_path / "office")
    np.testing.assert_allclose(back.stamps, premap.stamps)
    np.testing.assert_allclose(back.poses, premap.poses, atol=1e-9)
    np.testing.assert_array_equal(back.edges, premap.edges)
    np.testing.assert_allclose(back.edge_poses, premap.edge_poses, atol=1e-9)
    np.testing.assert_allclose(back.edge_covariances, premap.edge_covariances)
    for a, b in zip(back.scans, premap.scans, strict=True):
        np.testing.assert_array_equal(a, b)


def test_to_gtsam_reproduces_the_poses() -> None:
    premap, _ = _premap()
    graph, values = _to_gtsam(premap)
    assert graph.size() == 1 + len(premap.edges)
    result = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    for i, m in enumerate(premap.poses):
        np.testing.assert_allclose(result.atPose3(i).matrix(), m, atol=1e-6)


def _graph_with_empty_keyframes() -> Premap:
    """A loop of 10 keyframes, every third one empty, with two loop edges through empty ones."""
    rng = np.random.default_rng(1)
    angles = np.linspace(0, 2 * np.pi, 10, endpoint=False)
    poses = []
    for t in angles:
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler("z", t + np.pi / 2).as_matrix()
        m[:2, 3] = 5 * np.cos(t), 5 * np.sin(t)
        poses.append(m)
    poses = np.array(poses)
    pairs = [(i, i + 1) for i in range(9)] + [(0, 6), (3, 9)]
    measured = np.array([np.linalg.inv(poses[a]) @ poses[b] for a, b in pairs])
    scans = tuple(
        np.empty((0, 3), np.float32)
        if i % 3 == 0 and i
        else rng.normal(size=(5, 3)).astype(np.float32)
        for i in range(10)
    )
    return Premap(
        stamps=np.arange(10.0),
        poses=poses,
        scans=scans,
        edges=np.array(pairs),
        edge_poses=measured,
        edge_covariances=np.repeat(np.diag([1e-4] * 3 + [1e-6] * 3)[None], len(pairs), axis=0),
    )


def test_pruned_splices_out_only_what_comes_out_exactly() -> None:
    premap = _graph_with_empty_keyframes()
    pruned = premap.pruned()
    # 3 and 6 hold loop edges, so they stay without points; 9 sits on a plain chain
    kept = [k for k in range(10) if k != 9]
    assert len(pruned) == len(kept)
    np.testing.assert_allclose(pruned.poses, premap.poses[kept])
    assert not len(pruned.scans[kept.index(3)]) and not len(pruned.scans[kept.index(6)])
    assert pruned.edges.max() < len(pruned)
    # measurements were consistent, so the spliced graph solves back to the same poses
    graph, values = _to_gtsam(pruned)
    jittered = gtsam.Values()
    for i in range(len(pruned)):
        jittered.insert(
            i, values.atPose3(i).compose(gtsam.Pose3(gtsam.Rot3.Yaw(0.05), [0.2, -0.1, 0.0]))
        )
    result = gtsam.LevenbergMarquardtOptimizer(graph, jittered).optimize()
    for i, m in enumerate(pruned.poses):
        np.testing.assert_allclose(result.atPose3(i).matrix(), m, atol=1e-5)


def test_pruned_keeps_keyframe_zero_and_leaves_a_full_graph_alone() -> None:
    premap = _graph_with_empty_keyframes()
    points = np.ones((1, 3), np.float32)
    full = Premap(
        stamps=premap.stamps,
        poses=premap.poses,
        scans=tuple(points for _ in premap.scans),
        edges=premap.edges,
        edge_poses=premap.edge_poses,
        edge_covariances=premap.edge_covariances,
    )
    same = full.pruned()
    assert len(same) == len(full)
    np.testing.assert_array_equal(same.edges, full.edges)

    empty_first = Premap(
        stamps=premap.stamps,
        poses=premap.poses,
        scans=(np.empty((0, 3), np.float32), *full.scans[1:]),
        edges=premap.edges,
        edge_poses=premap.edge_poses,
        edge_covariances=premap.edge_covariances,
    )
    assert len(empty_first.pruned()) == len(full)


def test_pruned_drops_an_old_keyframe_a_much_younger_one_stands_on() -> None:
    n = 25
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, 0, 3] = np.r_[np.arange(n - 1), 1.0]  # the last returns to keyframe 1
    scans = tuple(np.array([[0.0, 0.0, float(k)]], np.float32) for k in range(n))
    edges = np.array([(i, i + 1) for i in range(n - 1)])
    premap = Premap(
        stamps=np.arange(float(n)),
        poses=poses,
        scans=scans,
        edges=edges,
        edge_poses=np.array([np.linalg.inv(poses[a]) @ poses[b] for a, b in edges]),
        edge_covariances=np.repeat(np.eye(6)[None] * 1e-4, len(edges), axis=0),
    )
    pruned = premap.pruned()
    assert len(pruned) == n - 1
    np.testing.assert_allclose(pruned.poses[:, 0, 3], np.r_[0.0, np.arange(2, n - 1), 1.0])
    # keyframe 1's point now belongs to the keyframe that replaced it, at the same place
    np.testing.assert_allclose(np.sort(pruned.cloud().points_f32()[:, 2]), np.arange(n))


def test_a_saved_map_keeps_its_tension_through_pruning() -> None:
    """A loop that disagrees with odometry stays a measurement, not a shape baked into the map."""
    n = 12
    truth = np.tile(np.eye(4), (n, 1, 1))
    truth[:, 0, 3] = np.arange(n)
    pairs = [(i, i + 1) for i in range(n - 1)]
    measured = [np.linalg.inv(truth[a]) @ truth[b] for a, b in pairs]
    bad = np.linalg.inv(truth[0]) @ truth[n - 1]
    bad[1, 3] += 0.5  # a loop closure half a metre off
    pairs.append((0, n - 1))
    measured.append(bad)
    cov = np.diag([1e-4] * 3 + [1e-6] * 3)
    scans = tuple(
        np.ones((1, 3), np.float32) if k % 2 == 0 else np.empty((0, 3), np.float32)
        for k in range(n)
    )
    raw = Premap(
        stamps=np.arange(float(n)),
        poses=truth,
        scans=scans,
        edges=np.array(pairs),
        edge_poses=np.array(measured),
        edge_covariances=np.repeat(cov[None], len(pairs), axis=0),
    )
    graph, values = _to_gtsam(raw)
    solved = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    at_optimum = Premap(
        stamps=raw.stamps,
        poses=np.array([solved.atPose3(i).matrix() for i in range(n)]),
        scans=scans,
        edges=raw.edges,
        edge_poses=raw.edge_poses,
        edge_covariances=raw.edge_covariances,
    )
    pruned = at_optimum.pruned()
    assert len(pruned) < n
    graph, values = _to_gtsam(pruned)
    again = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    for i, m in enumerate(pruned.poses):
        np.testing.assert_allclose(again.atPose3(i).matrix(), m, atol=1e-5)
    # the disagreement is still there to be outvoted, not flattened into the saved shape
    assert graph.error(values) == pytest.approx(
        _to_gtsam(at_optimum)[0].error(_to_gtsam(at_optimum)[1]), rel=1e-3
    )
    assert graph.error(values) > 1.0
