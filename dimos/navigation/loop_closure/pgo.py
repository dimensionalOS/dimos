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

"""PGO Module: Python pose graph optimization with loop closure.

Ported from FASTLIO2_ROS2/pgo. Detects keyframes, performs loop closure
via ICP + KD-tree search, and optimizes the pose graph with GTSAM iSAM2.
Publishes corrected odometry and accumulated global map.

Falls back from native C++ to pure Python when the native binary cannot
be built (e.g. missing GTSAM in nixpkgs).
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import gtsam
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PGOConfig(ModuleConfig):
    """Config for the PGO Python module."""

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
    global_static_map_publish_rate: float = 0.5
    global_static_map_voxel_size: float = 0.15

    # ICP
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 10.0


# ─── Keyframe storage ────────────────────────────────────────────────────────


@dataclass
class _KeyPose:
    r_local: np.ndarray  # 3x3 rotation in local/odom frame
    t_local: np.ndarray  # 3-vec translation in local/odom frame
    r_global: np.ndarray  # 3x3 corrected rotation
    t_global: np.ndarray  # 3-vec corrected translation
    timestamp: float
    body_cloud: np.ndarray  # Nx3 points in body frame


# ─── Simple ICP (point-to-point, no PCL dependency) ─────────────────────────


def _icp(
    source: np.ndarray,
    target: np.ndarray,
    max_iter: int = 50,
    max_dist: float = 10.0,
    tol: float = 1e-6,
) -> tuple[np.ndarray, float]:
    """Simple point-to-point ICP. Returns (4x4 transform, fitness score)."""
    if len(source) == 0 or len(target) == 0:
        return np.eye(4), float("inf")

    tree = KDTree(target)
    T = np.eye(4)
    src = source.copy()

    for _ in range(max_iter):
        dists, idxs = tree.query(src)
        mask = dists < max_dist
        if mask.sum() < 10:
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
    mask = dists_final < max_dist
    fitness = float(np.mean(dists_final[mask] ** 2)) if mask.sum() > 0 else float("inf")
    return T, fitness


def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel grid downsampling."""
    if len(pts) == 0 or voxel_size <= 0:
        return pts
    keys = np.floor(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


# ─── SimplePGO core algorithm ────────────────────────────────────────────────


class _SimplePGO:
    """Python port of the C++ SimplePGO class."""

    def __init__(self, config: PGOConfig) -> None:
        self._cfg = config
        self._key_poses: list[_KeyPose] = []
        self._history_pairs: list[tuple[int, int]] = []
        self._cache_pairs: list[dict] = []
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
        return (
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
        if len(self._key_poses) < 10:
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
        logger.info(f"[PGO] Loop closure detected: {loop_idx} <-> {cur_idx} (score={fitness:.4f})")

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

    def build_global_static_map(self, voxel_size: float) -> np.ndarray:
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


# ─── PGO Module ──────────────────────────────────────────────────────────────


class PGO(Module):
    """Pose graph optimization with loop closure detection.

    Pure-Python implementation using GTSAM iSAM2 and scipy KDTree.
    Detects keyframes from odometry, searches for loop closures,
    optimizes with iSAM2, and publishes corrected poses + global map.

    Ports:
        registered_scan (In[PointCloud2]): World-frame registered point cloud.
        raw_odom (In[PoseStamped]): Raw pose from robot (PoseStamped).
        corrected_odometry (Out[Odometry]): Loop-closure-corrected pose (Odometry).
        odom (Out[PoseStamped]): Loop-closure-corrected pose (PoseStamped, for downstream).
        global_static_map (Out[PointCloud2]): Accumulated keyframe map.
    """

    config: PGOConfig
    default_config = PGOConfig

    registered_scan: In[PointCloud2]
    raw_odom: In[PoseStamped]
    corrected_odometry: Out[Odometry]
    odom: Out[PoseStamped]
    global_static_map: Out[PointCloud2]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._pgo_lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._pgo: _SimplePGO | None = None
        # Latest odom
        self._latest_r = np.eye(3)
        self._latest_t = np.zeros(3)
        self._latest_time = 0.0
        self._has_odom = False
        self._last_global_static_map_time = 0.0

    def __getstate__(self) -> dict:
        state = super().__getstate__()
        for k in ("_lock", "_pgo_lock", "_thread", "_pgo"):
            state.pop(k, None)
        return state

    def __setstate__(self, state: dict) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._pgo_lock = threading.Lock()
        self._thread = None
        self._pgo = None

    def start(self) -> None:
        super().start()
        self._pgo = _SimplePGO(self.config)
        self.raw_odom._transport.subscribe(self._on_raw_odom)
        self.registered_scan._transport.subscribe(self._on_scan)
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info("[PGO] Python PGO module started (gtsam iSAM2)")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        super().stop()

    def _on_raw_odom(self, msg: PoseStamped) -> None:
        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        r = Rotation.from_quat(q).as_matrix()
        t = np.array([msg.x, msg.y, msg.z])
        with self._lock:
            self._latest_r = r
            self._latest_t = t
            self._latest_time = msg.ts if msg.ts is not None else time.time()
            self._has_odom = True

    def _on_scan(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return

        with self._lock:
            if not self._has_odom:
                return
            r_local = self._latest_r.copy()
            t_local = self._latest_t.copy()
            ts = self._latest_time

        pgo = self._pgo
        if pgo is None:
            return

        # Body-frame points
        if self.config.unregister_input:
            # registered_scan is world-frame, transform back to body-frame
            body_pts = (r_local.T @ (points[:, :3].T - t_local[:, None])).T
        else:
            body_pts = points[:, :3]

        with self._pgo_lock:
            added = pgo.add_key_pose(r_local, t_local, ts, body_pts)
            if added:
                pgo.search_for_loops()
                pgo.smooth_and_update()
                logger.info(
                    f"[PGO] Keyframe {pgo.num_key_poses} added "
                    f"({t_local[0]:.1f}, {t_local[1]:.1f}, {t_local[2]:.1f})"
                )

            # Publish corrected odometry
            r_corr, t_corr = pgo.get_corrected_pose(r_local, t_local)

        self._publish_corrected_odom(r_corr, t_corr, ts)

    def _publish_corrected_odom(self, r: np.ndarray, t: np.ndarray, ts: float) -> None:
        from dimos.msgs.geometry_msgs.Pose import Pose

        q = Rotation.from_matrix(r).as_quat()  # [x,y,z,w]

        odom = Odometry(
            ts=ts,
            frame_id="map",
            child_frame_id="sensor",
            pose=Pose(
                position=[float(t[0]), float(t[1]), float(t[2])],
                orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            ),
        )
        self.corrected_odometry._transport.publish(odom)

        ps = PoseStamped(
            ts=ts,
            frame_id="map",
            position=[float(t[0]), float(t[1]), float(t[2])],
            orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        )
        self.odom._transport.publish(ps)

        # Publish corrected TF so the robot box tracks corrected pose
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=ts,
        )
        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=ts,
        )
        self.tf.publish(Transform.from_pose("base_link", ps), camera_link, camera_optical)

    def _publish_loop(self) -> None:
        """Periodically publish global map."""
        pgo = self._pgo
        if pgo is None:
            return
        rate = self.config.global_static_map_publish_rate
        interval = 1.0 / rate if rate > 0 else 2.0

        while self._running:
            t0 = time.monotonic()
            now = time.time()

            if now - self._last_global_static_map_time > interval and pgo.num_key_poses > 0:
                with self._pgo_lock:
                    cloud_np = pgo.build_global_static_map(self.config.global_static_map_voxel_size)
                    n_keyframes = pgo.num_key_poses
                if len(cloud_np) > 0:
                    self.global_static_map._transport.publish(
                        PointCloud2.from_numpy(cloud_np, frame_id="map", timestamp=now)
                    )
                    logger.info(
                        f"[PGO] Global map published: {len(cloud_np)} points, "
                        f"{n_keyframes} keyframes"
                    )
                self._last_global_static_map_time = now

            elapsed = time.monotonic() - t0
            sleep_time = max(0.1, interval - elapsed)
            time.sleep(sleep_time)
