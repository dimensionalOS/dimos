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

"""A live graph joining a premap graph: attaching, moving the attachment, premap loops."""

from itertools import pairwise
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

gtsam = pytest.importorskip("gtsam")

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.pgo import PGOConfig, PGOState
from dimos.navigation.go2.loop_closure.pgo_map import PGOMap
from dimos.navigation.go2.loop_closure.premap import Premap, apply, matrix_pose

# The room is the map; the live odom frame sees it through TRUE.
ROOM = (12.0, 6.0, 2.5)
PATH_Y = 4.0


def _tf(x: float = 0.0, y: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    m[:2, 3] = x, y
    return m


TRUE = _tf(3.0, -2.0, 30.0)  # world <- map
WRONG = _tf(-4.0, 5.0, 120.0)


def _room() -> np.ndarray:
    lx, ly, lz = ROOM
    step = 0.1
    xs, ys, zs = (np.arange(0, n + 1e-9, step) for n in (lx, ly, lz))
    walls = []
    for x in (0.0, lx):
        y, z = np.meshgrid(ys, zs)
        walls.append(np.stack([np.full(y.size, x), y.ravel(), z.ravel()], 1))
    for y in (0.0, ly):
        x, z = np.meshgrid(xs, zs)
        walls.append(np.stack([x.ravel(), np.full(x.size, y), z.ravel()], 1))
    x, y = np.meshgrid(xs, ys)
    walls.append(np.stack([x.ravel(), y.ravel(), np.zeros(x.size)], 1))
    # a few pillars so the room is not a corridor along x
    for px in (3.0, 6.5, 9.0):
        z, a = np.meshgrid(zs, np.linspace(0, 2 * np.pi, 24))
        ring = [px + 0.3 * np.cos(a.ravel()), 1.5 + 0.3 * np.sin(a.ravel()), z.ravel()]
        walls.append(np.stack(ring, 1))
    # off the 5 cm voxel grid, as real surfaces are, so carving is not decided by float noise
    return np.concatenate(walls) + 0.0123


def _premap(room: np.ndarray, bend_m: float = 0.0, edge_sigma_m: float = 0.01) -> Premap:
    """Keyframes along the path, each owning its nearest room points; the last few stored bent."""
    truth = np.array([_tf(x, PATH_Y) for x in np.arange(0.5, 12.0, 1.0)])
    owner = np.argmin(np.linalg.norm(room[:, None, :2] - truth[None, :, :2, 3], axis=2), axis=1)
    scans = tuple(
        apply(np.linalg.inv(m), room[owner == i]).astype(np.float32) for i, m in enumerate(truth)
    )
    stored = truth.copy()
    stored[-4:, 1, 3] += bend_m * np.arange(1, 5) / 4
    edges = np.array([(i, i + 1) for i in range(len(stored) - 1)])
    between = np.array([np.linalg.inv(stored[a]) @ stored[b] for a, b in edges])
    # the default is as stiff as the live odometry (LIVE_PGO); ROS order x, y, z, rx, ry, rz
    var = edge_sigma_m**2
    cov = np.diag([var, var, var / 100, var / 100, var / 100, var / 100])
    return Premap(
        stamps=np.arange(len(stored), dtype=float),
        poses=stored,
        scans=scans,
        edges=edges,
        edge_poses=between,
        edge_covariances=np.repeat(cov[None], len(edges), axis=0),
    )


def _walk(state: PGOState, room: np.ndarray, steps: range, drift_per_step: float = 0.0) -> None:
    """Walk along x through the room; odom drifts sideways by drift_per_step each step."""
    for i in steps:
        in_map = _tf(1.0 + 0.5 * i, PATH_Y + 0.02 * (i % 2))
        odom = TRUE @ _tf(0.0, drift_per_step * i) @ in_map
        near = room[np.linalg.norm(room[:, :2] - in_map[:2, 3], axis=1) < 8.0]
        body = apply(np.linalg.inv(in_map), near)
        world = PointCloud2.from_numpy(apply(odom, body).astype(np.float32), timestamp=float(i))
        state.process(gtsam.Pose3(odom), float(i), world)


def _fix(m: np.ndarray, ts: float = 0.0) -> Transform:
    return Transform.from_matrix(m, frame_id="world", child_frame_id="map", ts=ts)


def _assert_near(tf: Transform | None, m: np.ndarray, tol_m: float = 0.05) -> None:
    assert tf is not None
    np.testing.assert_allclose(tf.to_matrix()[:3, 3], m[:3, 3], atol=tol_m)
    np.testing.assert_allclose(tf.to_matrix()[:3, :3], m[:3, :3], atol=0.02)


def _no_premap_loops() -> PGOConfig:
    return PGOConfig(premap_crop_m=0.0, loop_search_radius=0.0)


def _relative(state: PGOState) -> np.ndarray:
    m = [np.asarray(kp.optimized.matrix()) for kp in state._key_poses]
    return np.array([np.linalg.inv(a) @ b for a, b in pairwise(m)])


def test_the_first_fix_attaches() -> None:
    room = _room()
    state = PGOState(_no_premap_loops(), _premap(room))
    _walk(state, room, range(5))
    assert not state.attached and state.world_to_map() is None
    state.relocalize(_fix(TRUE, 1.0))
    assert state.attached
    _assert_near(state.world_to_map(), TRUE)


def test_agreeing_fixes_settle_and_only_a_better_fix_moves_a_settled_attachment() -> None:
    room = _room()
    state = PGOState(_no_premap_loops(), _premap(room))
    _walk(state, room, range(5))
    state.relocalize(_fix(TRUE, 1.0))
    state.relocalize(_fix(TRUE @ _tf(0.1), 2.0))
    assert not state.settled
    state.relocalize(_fix(TRUE, 3.0))
    assert state.settled and state.n_attaches == 1
    # worse than the current attachment on the same window: kept
    state.relocalize(_fix(WRONG, 4.0), fitness=0.5, current_fitness=0.8)
    assert state.settled and state.n_attaches == 1
    _assert_near(state.world_to_map(), TRUE)
    # better: moved
    state.relocalize(_fix(WRONG, 4.0), fitness=0.9, current_fitness=0.2)
    assert state.attached and not state.settled and state.n_attaches == 2
    _assert_near(state.world_to_map(), WRONG)


def test_moving_the_attachment_leaves_both_graphs_their_own_shape() -> None:
    room = _room()
    premap = _premap(room)
    state = PGOState(_no_premap_loops(), premap)
    _walk(state, room, range(8))
    before = _relative(state)
    state.relocalize(_fix(WRONG, 1.0))
    state.relocalize(_fix(TRUE, 2.0))
    assert state.n_attaches == 2
    _assert_near(state.world_to_map(), TRUE)
    np.testing.assert_allclose(_relative(state), before, atol=1e-6)
    np.testing.assert_allclose(state.premap_poses(), premap.poses, atol=1e-6)


def test_premap_loops_pull_a_drifting_walk_onto_the_premap() -> None:
    room = _room()
    state = PGOState(PGOConfig(), _premap(room))
    drift = 0.04  # 0.6 m sideways by the end
    _walk(state, room, range(3), drift)
    state.relocalize(_fix(TRUE, 0.0))
    assert state.attached
    _walk(state, room, range(3, 16), drift)
    assert state.n_premap_loops >= 2
    # attached, the solver works in map
    last = np.asarray(state._key_poses[-1].optimized.translation())
    truth = _tf(1.0 + 0.5 * 15, PATH_Y + 0.02 * (15 % 2))[:3, 3]
    # drift grows between premap loops, so a quarter of the uncorrected 0.6 m
    assert np.linalg.norm(last - truth) < 0.15


def test_a_loosely_held_premap_gives_way_to_a_stiff_live_run() -> None:
    room = _room()
    bend = 0.4
    # a stretch of premap held together by loose edges, stored bent
    premap = _premap(room, bend_m=bend, edge_sigma_m=0.1)
    state = PGOState(PGOConfig(), premap)
    _walk(state, room, range(3))
    state.relocalize(_fix(TRUE, 0.0))
    _walk(state, room, range(3, 22))
    stored = premap.poses[:, 1, 3] - PATH_Y
    now = state.premap_poses()[:, 1, 3] - PATH_Y
    _, _, edges, _, _ = state.joint_graph()
    joined = edges[(edges[:, 0] < len(premap)) & (edges[:, 1] >= len(premap)), 0]
    bent = sorted({int(j) for j in joined if stored[j] > 0.05})
    assert bent, "the walk closed onto the bent stretch"
    for j in bent:
        assert now[j] < 0.75 * stored[j]


def _walk_map(world_map: PGOMap, room: np.ndarray, steps: range, t0: float = 0.0) -> None:
    for i in steps:
        in_map = _tf(1.0 + 0.5 * i, PATH_Y + 0.02 * (i % 2))
        odom = TRUE @ in_map
        near = room[np.linalg.norm(room[:, :2] - in_map[:2, 3], axis=1) < 8.0]
        world = apply(TRUE, near).astype(np.float32)
        world_map.add(PointCloud2.from_numpy(world, timestamp=t0 + i), matrix_pose(odom))


def test_the_joint_premap_holds_both_runs_and_lies_on_the_room(tmp_path: Path) -> None:
    room = _room()
    premap = _premap(room)
    world_map = PGOMap(premap=premap)
    _walk_map(world_map, room, range(3))
    world_map.pgo.relocalize(_fix(TRUE, 1.0))
    _walk_map(world_map, room, range(3, 16))
    assert world_map.joint_premap() is None, "attached but not settled yet"
    world_map.pgo.relocalize(_fix(TRUE, 14.0))
    world_map.pgo.relocalize(_fix(TRUE, 15.0))
    joint = world_map.joint_premap()
    assert joint is not None
    n_live = world_map.pgo.n_keyframes
    # the walk saw the whole room again: the saved map is about the walk's points alone
    stored = sum(len(scan) for scan in premap.scans)
    live = len(world_map.live_map().points_f32())
    assert sum(len(scan) for scan in joint.scans) < live + 0.05 * stored
    assert len(joint) < len(premap) + n_live
    assert joint.edges.max() < len(joint) and (joint.edges[:, 0] == 0).any()

    joint.save(tmp_path / "house.map.db")
    back = Premap.load(tmp_path / "house")
    np.testing.assert_allclose(back.poses, joint.poses, atol=1e-9)
    d, _ = cKDTree(room).query(back.cloud().points_f32())
    assert np.mean(d < 0.1) > 0.95
    world_map.dispose()


def test_a_first_run_saves_as_a_new_map() -> None:
    room = _room()
    world_map = PGOMap()
    _walk_map(world_map, room, range(8))
    joint = world_map.joint_premap()
    assert joint is not None and 1 <= len(joint) <= world_map.pgo.n_keyframes
    # pruned to the keyframes that own points, still one connected chain
    assert all(len(scan) for scan in joint.scans[1:])
    assert len(joint.edges) >= len(joint) - 1 and joint.edges.max() < len(joint)
    world_map.dispose()


def test_an_unsettled_run_has_nothing_to_save() -> None:
    room = _room()
    world_map = PGOMap(premap=_premap(room))
    _walk_map(world_map, room, range(5))
    assert world_map.joint_premap() is None
    world_map.dispose()


def test_walking_a_path_again_hands_the_old_keyframes_over_and_keeps_the_map() -> None:
    room = _room()
    world_map = PGOMap(rebuild_cooldown_s=0.0)
    path = [1.0 + 0.5 * i for i in range(21)]
    path += path[::-1][1:]  # out along the room and back the same way
    for t, x in enumerate(path):
        in_map = _tf(x, PATH_Y + 0.02 * (t % 2))
        near = room[np.linalg.norm(room[:, :2] - in_map[:2, 3], axis=1) < 8.0]
        world = apply(TRUE, near).astype(np.float32)
        world_map.add(PointCloud2.from_numpy(world, timestamp=float(t)), matrix_pose(TRUE @ in_map))
    # the graph keeps every node, so a change of mind finds it as it was
    assert world_map.pgo.n_keyframes == len(path)
    # the last ten steps back stood where steps at least 20 s older did
    saved = world_map.joint_premap()
    assert saved is not None and len(saved) <= len(path) - 10
    d, _ = cKDTree(world_map.global_map().points_f32()).query(apply(TRUE, room))
    assert np.mean(d < 0.1) > 0.9
    world_map.dispose()


def test_a_later_run_over_the_same_ground_replaces_the_premap_keyframes_there() -> None:
    room = _room()
    world_map = PGOMap(premap=_premap(room))
    later = 1000.0  # a second session, long after the premap's
    _walk_map(world_map, room, range(3), t0=later)
    for i in range(3):
        world_map.pgo.relocalize(_fix(TRUE, later + i))
    _walk_map(world_map, room, range(3, 16), t0=later)
    joint = world_map.joint_premap()
    assert joint is not None
    xy, stamps = joint.poses[:, :2, 3], joint.stamps
    for k in range(1, len(joint)):
        later_near = (np.linalg.norm(xy - xy[k], axis=1) <= 0.5) & (stamps - stamps[k] >= 20.0)
        # it may stay as a node holding its edges, but its points went to the younger one
        assert not (later_near.any() and len(joint.scans[k])), f"keyframe {k} kept its points"
    world_map.dispose()


def _joins(state: PGOState) -> list:
    return [e for e in state._edges if e.kind == "join"]


def test_changing_its_mind_after_premap_loops_drops_every_join() -> None:
    room = _room()
    premap = _premap(room)
    state = PGOState(PGOConfig(), premap)
    _walk(state, room, range(3))
    state.relocalize(_fix(TRUE, 0.0))
    _walk(state, room, range(3, 12))
    assert state.n_premap_loops >= 1 and len(_joins(state)) >= 2
    state.relocalize(_fix(WRONG, 11.0), fitness=0.9, current_fitness=0.1)
    # only the fix that moved it joins the sessions now; the loops went with the old attachment
    assert len(_joins(state)) == 1
    _assert_near(state.world_to_map(), WRONG)
    np.testing.assert_allclose(state.premap_poses(), premap.poses, atol=1e-6)


def test_a_fix_joins_the_keyframe_at_its_stamp() -> None:
    room = _room()
    state = PGOState(_no_premap_loops(), _premap(room))
    _walk(state, room, range(8))
    state.relocalize(_fix(TRUE, 3.5))
    (join,) = _joins(state)
    assert join.b == len(state._premap) + 3  # the keyframe at 3.0, the last one before 3.5


def test_a_fix_against_the_stored_premap_agrees_after_the_premap_gave_way() -> None:
    room = _room()
    premap = _premap(room, bend_m=1.2, edge_sigma_m=3.0)  # loose, so it gives way far
    state = PGOState(PGOConfig(), premap)
    _walk(state, room, range(3))
    state.relocalize(_fix(TRUE, 0.0))
    _walk(state, room, range(3, 22))
    # the fix a matcher would give now: it sees the premap as stored, the graph has it moved
    local, optimized = state.keyframe_matrices()
    k = state.n_keyframes - 1
    current, stored = state.premap_poses(), premap.poses
    j = int(np.argmin(np.linalg.norm(current[:, :3, 3] - optimized[k, :3, 3], axis=1)))
    moved = current[j] @ np.linalg.inv(stored[j])
    assert np.linalg.norm(moved[:3, 3]) > 0.25, "the premap gave way by more than agreement allows"
    fix = local[k] @ np.linalg.inv(optimized[k]) @ moved
    state.relocalize(_fix(fix, float(k) + 0.1))
    assert state.n_attaches == 1
    assert len(_joins(state)) >= 2
