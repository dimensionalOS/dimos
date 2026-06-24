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

"""Pure-Python PGO ported from Ivan's `ivan/feat/go2loopclosure` branch
(`dimos/mapping/pgo.py`), adapted to the jnav LoopClosure spec.

GTSAM iSAM2 pose graph + Open3D point-to-plane ICP loop verification +
KD-tree loop candidate search. Differences from the original:
  * input renamed `registered_scan` -> `lidar` (LoopClosure spec)
  * publishes `pose_graph: Out[Graph3D]` (optimized keyframes, odometry +
    loop edges) and `loop_closure_event: Out[GraphDelta3D]` so the eval
    harness can capture the corrected trajectory and count closures
  * offline experiment helpers (transformers, two-pass voxel pipelines)
    were left behind — this is just the runtime module
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

import gtsam  # type: ignore[import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from reactivex.disposable import Disposable
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.spec import LoopClosure
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.msgs.GraphDelta3D import GraphDelta3D
from dimos.utils.logging_config import setup_logger

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BODY = "base_link"

logger = setup_logger()


class PGOConfig(ModuleConfig):
    world_frame: str = FRAME_MAP

    # Keyframe detection
    key_pose_delta_trans: float = 0.5
    key_pose_delta_deg: float = 10.0

    # Loop closure
    loop_search_radius: float = 2.0
    loop_time_thresh: float = 20.0
    loop_score_thresh: float = 0.3
    loop_submap_half_range: int = 10
    min_icp_inliers: int = 10
    min_keyframes_for_loop_search: int = 10
    loop_closure_extra_iterations: int = 4
    submap_resolution: float = 0.2
    min_loop_detect_duration: float = 5.0

    # Input mode
    unregister_input: bool = True  # Transform world-frame scans to body-frame using odom

    # Global map
    publish_global_map: bool = True
    global_map_publish_rate: float = 0.5
    global_map_voxel_size: float = 0.15

    # ICP
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 1.0


@dataclass
class _KeyPose:
    r_local: np.ndarray  # 3x3 rotation in local/odom frame
    t_local: np.ndarray  # 3-vec translation in local/odom frame
    r_global: np.ndarray  # 3x3 corrected rotation
    t_global: np.ndarray  # 3-vec corrected translation
    timestamp: float
    body_cloud: np.ndarray  # Nx3 points in body frame


def _icp(
    source: np.ndarray,
    target: np.ndarray,
    max_iter: int = 50,
    max_dist: float = 1.0,
    tol: float = 1e-6,
    min_inliers: int = 10,
    init: np.ndarray | None = None,
) -> tuple[np.ndarray, float]:
    """Point-to-plane ICP using Open3D's tensor pipeline.

    Returns ``(T, fitness)`` where ``fitness`` is mean squared inlier
    distance (m²)."""
    if len(source) < min_inliers or len(target) < min_inliers:
        return np.eye(4), float("inf")

    cpu = o3c.Device("CPU:0")
    src_pcd = o3d.t.geometry.PointCloud(o3c.Tensor(source.astype(np.float32), device=cpu))
    tgt_pcd = o3d.t.geometry.PointCloud(o3c.Tensor(target.astype(np.float32), device=cpu))

    # Normals on the target enable point-to-plane ICP, which converges
    # tighter than point-to-point on indoor scenes (walls give unambiguous
    # normals that resolve the slide-along-wall ambiguity).
    tgt_pcd.estimate_normals(max_nn=30, radius=0.3)

    init_T = (
        o3c.Tensor(init.astype(np.float64), dtype=o3c.float64, device=cpu)
        if init is not None
        else o3c.Tensor.eye(4, dtype=o3c.float64, device=cpu)
    )

    # Silence Open3D's "0 correspondence" warning — we deliberately use a
    # tight max_correspondence_distance and reject loops with poor fitness.
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
        result = o3d.t.pipelines.registration.icp(
            source=src_pcd,
            target=tgt_pcd,
            max_correspondence_distance=max_dist,
            init_source_to_target=init_T,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=tol,
                relative_rmse=tol,
                max_iteration=max_iter,
            ),
        )

    fitness_inlier_frac = float(result.fitness)
    if fitness_inlier_frac == 0.0:
        return np.eye(4), float("inf")

    rmse = float(result.inlier_rmse)
    T = result.transformation.numpy()
    return T, rmse * rmse


def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    if len(pts) == 0 or voxel_size <= 0:
        return pts
    keys = np.floor(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


class _SimplePGO:
    def __init__(self, config: PGOConfig) -> None:
        self._cfg = config
        self._key_poses: list[_KeyPose] = []
        self._history_pairs: list[tuple[int, int]] = []
        self._cache_pairs: list[dict[str, Any]] = []
        self._r_offset = np.eye(3)
        self._t_offset = np.zeros(3)

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.01)
        params.relinearizeSkip = 1
        self._isam2 = gtsam.ISAM2(params)
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

    def is_key_pose(self, r: np.ndarray, t: np.ndarray) -> bool:
        if not self._key_poses:
            return True
        last = self._key_poses[-1]
        delta_trans = np.linalg.norm(t - last.t_local)
        # Angular distance via quaternion dot product
        q_cur = Rotation.from_matrix(r).as_quat()  # [x,y,z,w]
        q_last = Rotation.from_matrix(last.r_local).as_quat()
        dot = abs(np.dot(q_cur, q_last))
        delta_deg = np.degrees(2.0 * np.arccos(min(dot, 1.0)))
        return bool(
            delta_trans > self._cfg.key_pose_delta_trans or delta_deg > self._cfg.key_pose_delta_deg
        )

    def add_key_pose(
        self, r_local: np.ndarray, t_local: np.ndarray, timestamp: float, body_cloud: np.ndarray
    ) -> bool:
        if not self.is_key_pose(r_local, t_local):
            return False

        idx = len(self._key_poses)
        init_r = self._r_offset @ r_local
        init_t = self._r_offset @ t_local + self._t_offset

        pose = gtsam.Pose3(gtsam.Rot3(init_r), gtsam.Point3(init_t))
        self._values.insert(idx, pose)

        if idx == 0:
            noise = gtsam.noiseModel.Diagonal.Variances(np.full(6, 1e-12))
            self._graph.add(gtsam.PriorFactorPose3(idx, pose, noise))
        else:
            last = self._key_poses[-1]
            r_between = last.r_local.T @ r_local
            t_between = last.r_local.T @ (t_local - last.t_local)
            noise = gtsam.noiseModel.Diagonal.Variances(
                np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6])
            )
            self._graph.add(
                gtsam.BetweenFactorPose3(
                    idx - 1, idx, gtsam.Pose3(gtsam.Rot3(r_between), gtsam.Point3(t_between)), noise
                )
            )

        kp = _KeyPose(
            r_local=r_local.copy(),
            t_local=t_local.copy(),
            r_global=init_r.copy(),
            t_global=init_t.copy(),
            timestamp=timestamp,
            body_cloud=_voxel_downsample(body_cloud, self._cfg.submap_resolution),
        )
        self._key_poses.append(kp)
        return True

    def _get_submap(self, idx: int, half_range: int) -> np.ndarray:
        lo = max(0, idx - half_range)
        hi = min(len(self._key_poses) - 1, idx + half_range)
        parts = []
        for i in range(lo, hi + 1):
            kp = self._key_poses[i]
            world = (kp.r_global @ kp.body_cloud.T).T + kp.t_global
            parts.append(world)
        if not parts:
            return np.empty((0, 3))
        cloud = np.vstack(parts)
        return _voxel_downsample(cloud, self._cfg.submap_resolution)

    def search_for_loops(self) -> None:
        if len(self._key_poses) < self._cfg.min_keyframes_for_loop_search:
            return

        # Rate limit
        if self._history_pairs:
            cur_time = self._key_poses[-1].timestamp
            last_time = self._key_poses[self._history_pairs[-1][1]].timestamp
            if cur_time - last_time < self._cfg.min_loop_detect_duration:
                return

        cur_idx = len(self._key_poses) - 1
        cur_kp = self._key_poses[-1]

        # Build KD-tree of previous keyframe positions
        positions = np.array([kp.t_global for kp in self._key_poses[:-1]])
        tree = KDTree(positions)

        idxs = tree.query_ball_point(cur_kp.t_global, self._cfg.loop_search_radius)
        if not idxs:
            return

        # Pick the spatially closest keyframe that's also old enough in time.
        # query_ball_point doesn't sort, so we sort by distance ourselves.
        candidates = [
            (float(np.linalg.norm(self._key_poses[i].t_global - cur_kp.t_global)), i)
            for i in idxs
            if abs(cur_kp.timestamp - self._key_poses[i].timestamp) > self._cfg.loop_time_thresh
        ]
        if not candidates:
            return
        candidates.sort()
        loop_idx = candidates[0][1]

        # ICP verification
        target = self._get_submap(loop_idx, self._cfg.loop_submap_half_range)
        source = self._get_submap(cur_idx, 0)

        transform, fitness = _icp(
            source,
            target,
            max_iter=self._cfg.max_icp_iterations,
            max_dist=self._cfg.max_icp_correspondence_dist,
            min_inliers=self._cfg.min_icp_inliers,
        )
        if fitness > self._cfg.loop_score_thresh:
            return

        # Compute relative pose
        R_icp = transform[:3, :3]
        t_icp = transform[:3, 3]
        r_refined = R_icp @ cur_kp.r_global
        t_refined = R_icp @ cur_kp.t_global + t_icp
        r_offset = self._key_poses[loop_idx].r_global.T @ r_refined
        t_offset = self._key_poses[loop_idx].r_global.T @ (
            t_refined - self._key_poses[loop_idx].t_global
        )

        self._cache_pairs.append(
            {
                "source": cur_idx,
                "target": loop_idx,
                "r_offset": r_offset,
                "t_offset": t_offset,
                "score": fitness,
            }
        )
        self._history_pairs.append((loop_idx, cur_idx))
        logger.info(
            "Loop closure detected",
            source=cur_idx,
            target=loop_idx,
            score=round(fitness, 4),
        )

    def smooth_and_update(self) -> None:
        has_loop = bool(self._cache_pairs)

        for pair in self._cache_pairs:
            # Pose3 noise model is [rx, ry, rz, x, y, z]. Use ICP fitness as
            # the translation variance and a generous fixed rotation variance —
            # loops shouldn't be trusted to fix rotation tightly.
            trans_var = max(0.01, float(pair["score"]))  # >= sigma_trans = 10 cm
            rot_var = 0.05  # sigma_rot ~= 13 deg
            noise = gtsam.noiseModel.Diagonal.Variances(
                np.array([rot_var, rot_var, rot_var, trans_var, trans_var, trans_var])
            )
            self._graph.add(
                gtsam.BetweenFactorPose3(
                    pair["target"],
                    pair["source"],
                    gtsam.Pose3(gtsam.Rot3(pair["r_offset"]), gtsam.Point3(pair["t_offset"])),
                    noise,
                )
            )
        self._cache_pairs.clear()

        self._isam2.update(self._graph, self._values)
        self._isam2.update()
        if has_loop:
            for _ in range(self._cfg.loop_closure_extra_iterations):
                self._isam2.update()
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

        estimates = self._isam2.calculateBestEstimate()
        for i in range(len(self._key_poses)):
            pose = estimates.atPose3(i)
            self._key_poses[i].r_global = pose.rotation().matrix()
            self._key_poses[i].t_global = pose.translation()

        last = self._key_poses[-1]
        self._r_offset = last.r_global @ last.r_local.T
        self._t_offset = last.t_global - self._r_offset @ last.t_local

    def get_corrected_pose(
        self, r_local: np.ndarray, t_local: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        return self._r_offset @ r_local, self._r_offset @ t_local + self._t_offset

    def build_global_map(self, voxel_size: float) -> np.ndarray:
        if not self._key_poses:
            return np.empty((0, 3), dtype=np.float32)
        parts = []
        for kp in self._key_poses:
            world = (kp.r_global @ kp.body_cloud.T).T + kp.t_global
            parts.append(world)
        cloud = np.vstack(parts).astype(np.float32)
        return _voxel_downsample(cloud, voxel_size)

    @property
    def num_key_poses(self) -> int:
        return len(self._key_poses)


def process_scan(
    pgo: _SimplePGO,
    cloud: PointCloud2,
    r_local: np.ndarray,
    t_local: np.ndarray,
    ts: float,
    unregister_input: bool,
) -> tuple[Odometry, Transform, bool] | None:
    """Add a keyframe, run loop closure, return (corrected odom, map->odom tf,
    keyframe_added) — or None on empty cloud.

    Caller must hold ``pgo``'s lock during this call."""
    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    if unregister_input:
        # registered_scan is world-frame; transform back to body-frame.
        body_pts = (r_local.T @ (points[:, :3].T - t_local[:, None])).T
    else:
        body_pts = points[:, :3]

    added = pgo.add_key_pose(r_local, t_local, ts, body_pts)
    if added:
        pgo.search_for_loops()
        pgo.smooth_and_update()

    r_corr, t_corr = pgo.get_corrected_pose(r_local, t_local)
    return (
        build_corrected_odometry(r_corr, t_corr, ts),
        build_map_odom_tf(pgo._r_offset.copy(), pgo._t_offset.copy(), ts),
        added,
    )


def build_corrected_odometry(
    r: np.ndarray,
    t: np.ndarray,
    ts: float,
    world_frame: str = FRAME_MAP,
) -> Odometry:
    q = Rotation.from_matrix(r).as_quat()  # [x,y,z,w]
    return Odometry(
        ts=ts,
        frame_id=world_frame,
        child_frame_id=FRAME_BODY,
        pose=Pose(
            position=[float(t[0]), float(t[1]), float(t[2])],
            orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        ),
    )


def build_map_odom_tf(
    r_offset: np.ndarray,
    t_offset: np.ndarray,
    ts: float,
    world_frame: str = FRAME_MAP,
    odom_frame: str = FRAME_ODOM,
) -> Transform:
    q = Rotation.from_matrix(r_offset).as_quat()  # [x,y,z,w]
    return Transform(
        frame_id=world_frame,
        child_frame_id=odom_frame,
        translation=Vector3(float(t_offset[0]), float(t_offset[1]), float(t_offset[2])),
        rotation=Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
        ts=ts,
    )


def _keyframe_node(index: int, key_pose: _KeyPose, world_frame: str) -> Graph3D.Node3D:
    q = Rotation.from_matrix(key_pose.r_global).as_quat()  # [x,y,z,w]
    return Graph3D.Node3D(
        pose=PoseStamped(
            ts=key_pose.timestamp,
            frame_id=world_frame,
            position=[float(v) for v in key_pose.t_global],
            orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        ),
        id=index,
    )


class PGO(Module, LoopClosure):
    """Pose graph optimization with loop closure (pure Python).

    Detects keyframes, performs loop closure via ICP + KD-tree search, and
    optimizes the pose graph with GTSAM iSAM2. Publishes corrected odometry,
    the optimized pose graph, loop-closure events, and an accumulated
    global map."""

    config: PGOConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
    pose_graph: Out[Graph3D]
    loop_closure_event: Out[GraphDelta3D]
    global_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = False
        self._thread: threading.Thread | None = None
        self._pgo: _SimplePGO | None = None
        self._latest_r = np.eye(3)
        self._latest_t = np.zeros(3)
        self._latest_time = 0.0
        self._has_odom = False
        self._last_global_map_time = 0.0
        self._published_loops = 0
        self._lock = threading.Lock()
        # Protects _pgo mutations (add_key_pose, search_for_loops,
        # smooth_and_update, build_global_map) against concurrent access
        # from _on_scan and _publish_loop threads.
        self._pgo_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self._pgo = _SimplePGO(self.config)
        # Identity map -> odom so consumers querying map -> body get a result
        # before any loop-closure correction exists.
        self.tf.publish(build_map_odom_tf(np.eye(3), np.zeros(3), time.time()))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_scan)))
        self._running = True
        if self.config.publish_global_map:
            self._thread = threading.Thread(target=self._publish_global_map_loop, daemon=True)
            self._thread.start()
        logger.info(
            "PGO module started (gtsam iSAM2, pure python)",
            publish_global_map=self.config.publish_global_map,
        )

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        r = Rotation.from_quat(q).as_matrix()
        t = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        with self._lock:
            self._latest_r = r
            self._latest_t = t
            self._latest_time = msg.ts if msg.ts else time.time()
            self._has_odom = True

    def _on_scan(self, cloud: PointCloud2) -> None:
        with self._lock:
            if not self._has_odom:
                return
            r_local = self._latest_r.copy()
            t_local = self._latest_t.copy()
            ts = self._latest_time

        pgo = self._pgo
        assert pgo is not None

        with self._pgo_lock:
            result = process_scan(pgo, cloud, r_local, t_local, ts, self.config.unregister_input)
            if result is None:
                return
            corrected_odom, tf_msg, keyframe_added = result
            if keyframe_added:
                graph_msg, loop_events = self._snapshot_graph(pgo, ts)
            else:
                graph_msg, loop_events = None, []

        self.corrected_odometry.publish(corrected_odom)
        self.tf.publish(tf_msg)
        if graph_msg is not None:
            self.pose_graph.publish(graph_msg)
        for event in loop_events:
            self.loop_closure_event.publish(event)

    def _snapshot_graph(self, pgo: _SimplePGO, ts: float) -> tuple[Graph3D, list[GraphDelta3D]]:
        """The optimized graph (odometry chain + loop edges) and one
        GraphDelta3D per loop pair not yet published.

        Caller must hold ``_pgo_lock``."""
        world_frame = self.config.world_frame
        nodes = [
            _keyframe_node(index, key_pose, world_frame)
            for index, key_pose in enumerate(pgo._key_poses)
        ]
        edges = [
            Graph3D.Edge(
                start_id=index - 1, end_id=index, timestamp=pgo._key_poses[index].timestamp
            )
            for index in range(1, len(pgo._key_poses))
        ]
        edges += [
            Graph3D.Edge(start_id=target, end_id=source, metadata_id=1)
            for target, source in pgo._history_pairs
        ]
        graph_msg = Graph3D(ts=ts, nodes=nodes, edges=edges)

        loop_events: list[GraphDelta3D] = []
        identity = GraphDelta3D.Transform(
            translation=Vector3(0.0, 0.0, 0.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )
        for target, source in pgo._history_pairs[self._published_loops :]:
            loop_events.append(
                GraphDelta3D(
                    ts=ts,
                    nodes=[
                        _keyframe_node(target, pgo._key_poses[target], world_frame),
                        _keyframe_node(source, pgo._key_poses[source], world_frame),
                    ],
                    transforms=[identity, identity],
                )
            )
        self._published_loops = len(pgo._history_pairs)
        return graph_msg, loop_events

    def _publish_global_map_loop(self) -> None:
        pgo = self._pgo
        assert pgo is not None
        rate = self.config.global_map_publish_rate
        interval = 1.0 / rate if rate > 0 else 2.0

        while self._running:
            t0 = time.monotonic()

            if t0 - self._last_global_map_time > interval and pgo.num_key_poses > 0:
                with self._pgo_lock:
                    cloud_np = pgo.build_global_map(self.config.global_map_voxel_size)
                if len(cloud_np) > 0:
                    now = time.time()
                    self.global_map.publish(
                        PointCloud2.from_numpy(
                            cloud_np, frame_id=self.config.world_frame, timestamp=now
                        )
                    )
                self._last_global_map_time = t0

            elapsed = time.monotonic() - t0
            sleep_time = max(DEFAULT_THREAD_JOIN_TIMEOUT, interval - elapsed)
            time.sleep(sleep_time)
