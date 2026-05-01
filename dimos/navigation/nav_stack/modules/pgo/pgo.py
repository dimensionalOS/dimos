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

from dataclasses import dataclass
import threading
import time
from typing import Any

import gtsam  # type: ignore[import-untyped]
import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Below this many ICP point-to-point correspondences, the alignment is
# under-constrained — bail out with infinite cost rather than reporting a
# spurious match.
_MIN_ICP_INLIERS = 10

# Loop-closure detection needs enough keyframes to have a meaningful
# distance-vs-time history. Below this, skip the search entirely.
_MIN_KEYFRAMES_FOR_LOOP_SEARCH = 10


class PGOConfig(ModuleConfig):
    # Keyframe detection
    key_pose_delta_trans: float = 0.5
    key_pose_delta_deg: float = 10.0

    # Loop closure
    loop_search_radius: float = 15.0
    loop_time_thresh: float = 60.0
    loop_score_thresh: float = 0.3
    loop_submap_half_range: int = 5
    submap_resolution: float = 0.1
    min_loop_detect_duration: float = 5.0

    # Input mode
    unregister_input: bool = True  # Transform world-frame scans to body-frame using odom

    # Global map
    global_map_publish_rate: float = 0.5
    global_map_voxel_size: float = 0.15

    # ICP
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 10.0


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
    max_dist: float = 10.0,
    tol: float = 1e-6,
) -> tuple[np.ndarray, float]:
    if len(source) == 0 or len(target) == 0:
        return np.eye(4), float("inf")

    tree = KDTree(target)
    T = np.eye(4)
    src = source.copy()

    for _ in range(max_iter):
        dists, idxs = tree.query(src)
        mask = np.asarray(dists < max_dist)
        idxs = np.asarray(idxs)
        if int(mask.sum()) < _MIN_ICP_INLIERS:
            return T, float("inf")

        p = src[mask]
        q = target[idxs[mask]]

        cp = p.mean(axis=0)
        cq = q.mean(axis=0)
        H = (p - cp).T @ (q - cq)

        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = cq - R @ cp

        dT = np.eye(4)
        dT[:3, :3] = R
        dT[:3, 3] = t
        T = dT @ T
        src = (R @ src.T).T + t

        if np.linalg.norm(t) < tol:
            break

    # Fitness: mean squared distance of inliers
    dists_final, _ = tree.query(src)
    mask_final = np.asarray(dists_final < max_dist)
    dists_final = np.asarray(dists_final)
    fitness = (
        float(np.mean(dists_final[mask_final] ** 2)) if int(mask_final.sum()) > 0 else float("inf")
    )
    return T, fitness


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
        if len(self._key_poses) < _MIN_KEYFRAMES_FOR_LOOP_SEARCH:
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

        # Find candidate far enough in time
        loop_idx = -1
        for i in idxs:
            if abs(cur_kp.timestamp - self._key_poses[i].timestamp) > self._cfg.loop_time_thresh:
                loop_idx = i
                break
        if loop_idx == -1:
            return

        # ICP verification
        target = self._get_submap(loop_idx, self._cfg.loop_submap_half_range)
        source = self._get_submap(cur_idx, 0)

        transform, fitness = _icp(
            source,
            target,
            max_iter=self._cfg.max_icp_iterations,
            max_dist=self._cfg.max_icp_correspondence_dist,
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
            noise = gtsam.noiseModel.Diagonal.Variances(np.full(6, pair["score"]))
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
            for _ in range(4):
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
) -> tuple[Odometry, Transform] | None:
    """Add a keyframe (if it qualifies), run loop closure, and return the
    messages to publish. Returns None if the cloud is empty.

    Caller is responsible for holding ``pgo``'s lock during this call.
    """
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
        logger.info(
            "Keyframe added",
            keyframe=pgo.num_key_poses,
            position=f"({t_local[0]:.1f}, {t_local[1]:.1f}, {t_local[2]:.1f})",
        )

    r_corr, t_corr = pgo.get_corrected_pose(r_local, t_local)
    return (
        build_corrected_odometry(r_corr, t_corr, ts),
        build_map_odom_tf(pgo._r_offset.copy(), pgo._t_offset.copy(), ts),
    )


def build_corrected_odometry(r: np.ndarray, t: np.ndarray, ts: float) -> Odometry:
    q = Rotation.from_matrix(r).as_quat()  # [x,y,z,w]
    return Odometry(
        ts=ts,
        frame_id=FRAME_MAP,
        child_frame_id=FRAME_BODY,
        pose=Pose(
            position=[float(t[0]), float(t[1]), float(t[2])],
            orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        ),
    )


def build_map_odom_tf(r_offset: np.ndarray, t_offset: np.ndarray, ts: float) -> Transform:
    q = Rotation.from_matrix(r_offset).as_quat()  # [x,y,z,w]
    return Transform(
        frame_id=FRAME_MAP,
        child_frame_id=FRAME_ODOM,
        translation=Vector3(float(t_offset[0]), float(t_offset[1]), float(t_offset[2])),
        rotation=Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
        ts=ts,
    )


class PGO(Module):
    """Pose graph optimization with loop closure.

    Detects keyframes, performs loop closure via ICP + KD-tree search, and optimizes the pose graph with GTSAM iSAM2.
    Publishes corrected odometry and accumulated global map.
    """

    config: PGOConfig

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    corrected_odometry: Out[Odometry]
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
        self._lock = threading.Lock()
        # Protects _pgo mutations (add_key_pose, search_for_loops,
        # smooth_and_update, build_global_map) against concurrent access
        # from _on_scan and _publish_loop threads.
        self._pgo_lock = threading.Lock()

    def _seed_initial_tf(self, ts: float) -> None:
        """Publish an identity ``map → odom`` so consumers querying
        ``map → body`` get a result immediately, before any loop closure
        correction has been computed."""
        self._publish_map_odom_tf(np.eye(3), np.zeros(3), ts)

    @rpc
    def start(self) -> None:
        super().start()
        self._pgo = _SimplePGO(self.config)
        self._seed_initial_tf(time.time())
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.registered_scan.subscribe(self._on_scan)))
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info("PGO module started (gtsam iSAM2)")

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
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
        corrected_odom, tf_msg = result
        self.corrected_odometry.publish(corrected_odom)
        self.tf.publish(tf_msg)

    def _publish_corrected_odom(self, r: np.ndarray, t: np.ndarray, ts: float) -> None:
        self.corrected_odometry.publish(build_corrected_odometry(r, t, ts))

    def _publish_map_odom_tf(self, r_offset: np.ndarray, t_offset: np.ndarray, ts: float) -> None:
        """Publish the ``map → odom`` correction transform to the TF tree.

        Composed with FastLio2's ``odom → body``, this gives any
        consumer ``map → body`` via BFS chain lookup.
        """
        self.tf.publish(build_map_odom_tf(r_offset, t_offset, ts))

    def _publish_loop(self) -> None:
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
                        PointCloud2.from_numpy(cloud_np, frame_id=FRAME_MAP, timestamp=now)
                    )
                self._last_global_map_time = t0

            elapsed = time.monotonic() - t0
            sleep_time = max(0.1, interval - elapsed)
            time.sleep(sleep_time)
