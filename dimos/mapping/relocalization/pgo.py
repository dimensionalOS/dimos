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

# NOTE: This lives under mapping/relocalization/ for now because the only
# consumer is the premap export pipeline (`dimos map --export`). It is
# temporary and can be moved/split out later when PGO grows other consumers.

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import gtsam  # type: ignore[import-not-found,import-untyped]
import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation

from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

FRAME_MAP = "world"

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
class KeyPose:
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
    """Point-to-point ICP using Open3D's tensor pipeline.

    Returns ``(T, fitness)`` where ``fitness`` is mean squared inlier
    distance (m²) — same semantic as the previous SVD implementation, so
    the GTSAM noise model in ``smooth_and_update`` keeps working.
    """
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
    # tight max_correspondence_distance and reject loops with poor fitness;
    # the warning is informational, not an error.
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
    # Return mean squared inlier distance (m²) to match prior _icp contract.
    return T, rmse * rmse


def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    if len(pts) == 0 or voxel_size <= 0:
        return pts
    keys = np.floor(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


class _SimplePGO:
    key_poses: list[KeyPose]

    def __init__(self, config: PGOConfig) -> None:
        self.key_poses = []
        self._cfg = config
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
        if not self.key_poses:
            return True
        last = self.key_poses[-1]
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

        idx = len(self.key_poses)
        init_r = self._r_offset @ r_local
        init_t = self._r_offset @ t_local + self._t_offset

        pose = gtsam.Pose3(gtsam.Rot3(init_r), gtsam.Point3(init_t))
        self._values.insert(idx, pose)

        if idx == 0:
            noise = gtsam.noiseModel.Diagonal.Variances(np.full(6, 1e-12))
            self._graph.add(gtsam.PriorFactorPose3(idx, pose, noise))
        else:
            last = self.key_poses[-1]
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

        kp = KeyPose(
            r_local=r_local.copy(),
            t_local=t_local.copy(),
            r_global=init_r.copy(),
            t_global=init_t.copy(),
            timestamp=timestamp,
            body_cloud=_voxel_downsample(body_cloud, self._cfg.submap_resolution),
        )
        self.key_poses.append(kp)
        return True

    def _get_submap(self, idx: int, half_range: int) -> np.ndarray:
        lo = max(0, idx - half_range)
        hi = min(len(self.key_poses) - 1, idx + half_range)
        parts = []
        for i in range(lo, hi + 1):
            kp = self.key_poses[i]
            world = (kp.r_global @ kp.body_cloud.T).T + kp.t_global
            parts.append(world)
        if not parts:
            return np.empty((0, 3))
        cloud = np.vstack(parts)
        return _voxel_downsample(cloud, self._cfg.submap_resolution)

    def search_for_loops(self) -> None:
        if len(self.key_poses) < self._cfg.min_keyframes_for_loop_search:
            return

        # Rate limit
        if self._history_pairs:
            cur_time = self.key_poses[-1].timestamp
            last_time = self.key_poses[self._history_pairs[-1][1]].timestamp
            if cur_time - last_time < self._cfg.min_loop_detect_duration:
                return

        cur_idx = len(self.key_poses) - 1
        cur_kp = self.key_poses[-1]

        # Build KD-tree of previous keyframe positions
        positions = np.array([kp.t_global for kp in self.key_poses[:-1]])
        tree = KDTree(positions)

        idxs = tree.query_ball_point(cur_kp.t_global, self._cfg.loop_search_radius)
        if not idxs:
            return

        # Pick the spatially closest keyframe that's also old enough in time.
        # query_ball_point doesn't sort, so we sort by distance ourselves.
        candidates = [
            (float(np.linalg.norm(self.key_poses[i].t_global - cur_kp.t_global)), i)
            for i in idxs
            if abs(cur_kp.timestamp - self.key_poses[i].timestamp) > self._cfg.loop_time_thresh
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
        r_offset = self.key_poses[loop_idx].r_global.T @ r_refined
        t_offset = self.key_poses[loop_idx].r_global.T @ (
            t_refined - self.key_poses[loop_idx].t_global
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
            # Pose3 noise model is [rx, ry, rz, x, y, z]. The two halves
            # have different units (rad² vs m²), so a uniform variance —
            # the original behaviour — silently makes one half pathological
            # (e.g. score=0.07 → σ_rot ≈ 15° AND σ_trans ≈ 26 cm; one of
            # those is too tight, one is too loose, depending on the loop).
            # Use ICP fitness as the *translation* variance and a
            # generous fixed rotation variance — loops shouldn't be
            # trusted to fix rotation tightly without normals + p2plane.
            trans_var = max(0.01, float(pair["score"]))  # ≥ σ_trans = 10 cm
            rot_var = 0.05  # σ_rot ≈ 13°
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
        for i in range(len(self.key_poses)):
            pose = estimates.atPose3(i)
            self.key_poses[i].r_global = pose.rotation().matrix()
            self.key_poses[i].t_global = pose.translation()

        last = self.key_poses[-1]
        self._r_offset = last.r_global @ last.r_local.T
        self._t_offset = last.t_global - self._r_offset @ last.t_local

    def get_corrected_pose(
        self, r_local: np.ndarray, t_local: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        return self._r_offset @ r_local, self._r_offset @ t_local + self._t_offset

    def build_global_map(self, voxel_size: float) -> np.ndarray:
        if not self.key_poses:
            return np.empty((0, 3), dtype=np.float32)
        parts = []
        for kp in self.key_poses:
            world = (kp.r_global @ kp.body_cloud.T).T + kp.t_global
            parts.append(world)
        cloud = np.vstack(parts).astype(np.float32)
        return _voxel_downsample(cloud, voxel_size)

    @property
    def num_key_poses(self) -> int:
        return len(self.key_poses)


def pgo_then_voxels(
    stream: Any,
    *,
    voxel_size: float = 0.05,
    block_count: int = 2_000_000,
    device: str = "CUDA:0",
    corrected_path_out: list[tuple[float, float, float]] | None = None,
    pass1_on_frame: Callable[[Any], None] | None = None,
    pass2_on_frame: Callable[[Any], None] | None = None,
    **pgo_cfg: Any,
) -> PointCloud2:
    """Two-pass PGO mapping (eliminates duplicate-wall artifacts).

    Pass 1 runs PGO over the lidar stream to build its corrected
    keyframe trajectory.

    Pass 2 re-streams every lidar frame through ``VoxelGrid``, but each
    frame's world-frame cloud is first transformed by the rigid drift
    correction interpolated (SLERP for rotation, linear for translation)
    from the keyframe corrections at the frame's timestamp.

    Each frame is therefore inserted exactly once at its converged
    corrected pose, so walls collapse to a single layer instead of the
    "smear of slightly-offset re-projections" that the single-pass
    ``_SimplePGO.build_global_map`` produces.
    """
    from scipy.spatial.transform import Slerp

    from dimos.mapping.voxels import VoxelGrid

    cfg = PGOConfig(**pgo_cfg)
    pgo = _SimplePGO(cfg)

    n_frames = 0
    for obs in stream:
        if pass1_on_frame is not None:
            pass1_on_frame(obs)
        if obs.pose is None:
            continue
        x, y, z, qx, qy, qz, qw = obs.pose
        # Skip placeholder poses: zero translation, or invalid (zero-norm) quaternion.
        if x == 0 and y == 0 and z == 0:
            continue
        if qx == 0 and qy == 0 and qz == 0 and qw == 0:
            continue
        r = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        t = np.array([x, y, z])
        points, _ = obs.data.as_numpy()
        if len(points) == 0:
            continue
        body_pts = (
            (r.T @ (points[:, :3].T - t[:, None])).T if cfg.unregister_input else points[:, :3]
        )
        if pgo.add_key_pose(r, t, obs.ts, body_pts):
            pgo.search_for_loops()
            pgo.smooth_and_update()
        n_frames += 1

    n_kf = pgo.num_key_poses
    print(f"  Pass 1: {n_frames} frames, {n_kf} keyframes")

    if corrected_path_out is not None:
        corrected_path_out.extend(
            (float(kp.t_global[0]), float(kp.t_global[1]), float(kp.t_global[2]))
            for kp in pgo.key_poses
        )

    grid = VoxelGrid(voxel_size=voxel_size, block_count=block_count, device=device)
    try:
        if n_kf < 2:
            for obs in stream:
                if pass2_on_frame is not None:
                    pass2_on_frame(obs)
                grid.add_frame(obs.data)
            return grid.get_global_pointcloud2()

        # Sort + dedupe by timestamp: Slerp requires strictly increasing ts.
        kps = sorted(pgo.key_poses, key=lambda kp: kp.timestamp)
        kps = [kp for i, kp in enumerate(kps) if i == 0 or kp.timestamp > kps[i - 1].timestamp]
        kf_ts = np.array([kp.timestamp for kp in kps])
        # Per-keyframe rigid drift correction: T_corr = T_global @ T_local.inv()
        R_corr_list = [kp.r_global @ kp.r_local.T for kp in kps]
        t_corr_list = [kp.t_global - (kp.r_global @ kp.r_local.T) @ kp.t_local for kp in kps]
        t_corrs = np.stack(t_corr_list)
        rot_slerp = Slerp(kf_ts, Rotation.from_matrix(np.stack(R_corr_list)))

        n_inserted = 0
        for obs in stream:
            if pass2_on_frame is not None:
                pass2_on_frame(obs)
            if obs.pose is None:
                continue
            ts = float(np.clip(obs.ts, kf_ts[0], kf_ts[-1]))
            r_correction = rot_slerp([ts])[0].as_matrix()
            idx = int(np.searchsorted(kf_ts, ts))
            if idx == 0:
                t_correction = t_corrs[0]
            elif idx >= len(kf_ts):
                t_correction = t_corrs[-1]
            else:
                t_lo, t_hi = kf_ts[idx - 1], kf_ts[idx]
                alpha = (ts - t_lo) / (t_hi - t_lo) if t_hi > t_lo else 0.0
                t_correction = (1 - alpha) * t_corrs[idx - 1] + alpha * t_corrs[idx]

            points, _ = obs.data.as_numpy()
            if len(points) == 0:
                continue
            corrected_pts = (r_correction @ points[:, :3].T).T + t_correction
            grid.add_frame(PointCloud2.from_numpy(corrected_pts.astype(np.float32)))
            n_inserted += 1

        print(f"  Pass 2: {n_inserted} frames inserted with PGO-corrected poses")
        return grid.get_global_pointcloud2()
    finally:
        grid.dispose()
