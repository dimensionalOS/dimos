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
# consumer is the premap export pipeline (`dimos export-premap`). It is
# temporary and can be moved/split out later when PGO grows other consumers.

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING, Any

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
from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation

FRAME_MAP = "world"
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


def map_quality(cloud: PointCloud2, voxel_size: float = 0.05) -> dict[str, float]:
    """Geometric quality metrics for an accumulated voxel map.

    Maps are first downsampled to a common ``voxel_size`` so the same
    metric is comparable across maps that came out of different
    pipelines (e.g. PGO's 0.15 m global map vs the voxel grid's 0.05 m).

    Lower ``knn_mean_cm`` = sharper walls (a well-aligned multi-pass map
    has thin walls so neighbors are close). ``n_points`` after the
    common-grid downsample reflects the spatial extent of the map.
    """
    pts, _ = cloud.as_numpy()
    pts = _voxel_downsample(pts[:, :3], voxel_size)
    n = pts.shape[0]
    if n < 6:
        return {"n_points": float(n), "knn_mean_cm": 0.0, "bbox_m3": 0.0}
    tree = KDTree(pts)
    dists, _ = tree.query(pts, k=6)
    dists_arr = np.asarray(dists)
    extent = pts.max(axis=0) - pts.min(axis=0)
    return {
        "n_points": float(n),
        "knn_mean_cm": float(dists_arr[:, 1:].mean()) * 100.0,
        "bbox_m3": float(extent[0] * extent[1] * extent[2]),
    }


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


class PGOMapTransformer(Transformer[PointCloud2, PointCloud2]):
    """Run PGO over a lidar stream and emit its accumulated global map.

    Reads per-frame body pose from ``obs.pose`` (7-tuple
    ``(x, y, z, qx, qy, qz, qw)``). Frames without a pose are skipped.

    Args:
        emit_every: Yield current global map every *n* frames. ``1``
            (default) = yield after every frame. ``0`` = yield only on
            upstream exhaustion.
        loop_score: Optional float stream — appends each detected loop
            closure's ICP fitness (lower = better) at the keyframe's ts.
        pose_jump: Optional float stream — for each loop closure event,
            appends ``max ||Δt_global||`` over all keyframes (m): the
            largest spatial correction PGO applied to any past pose.
        **pgo_cfg: Forwarded to :class:`PGOConfig`.
    """

    def __init__(
        self,
        *,
        emit_every: int = 1,
        loop_score: Stream[float] | None = None,
        pose_jump: Stream[float] | None = None,
        **pgo_cfg: Any,
    ) -> None:
        self.emit_every = emit_every
        self._cfg = PGOConfig(**pgo_cfg)
        self._loop_score = loop_score
        self._pose_jump = pose_jump

    def _make_obs(
        self, pgo: _SimplePGO, last_obs: Observation[PointCloud2], count: int
    ) -> Observation[PointCloud2]:
        cloud_np = pgo.build_global_map(self._cfg.global_map_voxel_size)
        pc = PointCloud2.from_numpy(cloud_np, frame_id=self._cfg.world_frame, timestamp=last_obs.ts)
        return last_obs.derive(
            data=pc,
            pose=None,
            tags={**last_obs.tags, "frame_count": count, "key_poses": pgo.num_key_poses},
        )

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[PointCloud2]]:
        pgo = _SimplePGO(self._cfg)
        last_obs: Observation[PointCloud2] | None = None
        count = 0

        for obs in upstream:
            if obs.pose is None:
                continue

            x, y, z, qx, qy, qz, qw = obs.pose
            r = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            t = np.array([x, y, z])

            points, _ = obs.data.as_numpy()
            if len(points) == 0:
                continue

            if self._cfg.unregister_input:
                body_pts = (r.T @ (points[:, :3].T - t[:, None])).T
            else:
                body_pts = points[:, :3]

            if pgo.add_key_pose(r, t, obs.ts, body_pts):
                pgo.search_for_loops()

                pre_t: np.ndarray | None = None
                if pgo._cache_pairs:
                    if self._loop_score is not None:
                        for pair in pgo._cache_pairs:
                            self._loop_score.append(float(pair["score"]), ts=obs.ts)
                    if self._pose_jump is not None:
                        pre_t = np.stack([kp.t_global.copy() for kp in pgo._key_poses])

                pgo.smooth_and_update()

                if pre_t is not None and self._pose_jump is not None:
                    post_t = np.stack([kp.t_global for kp in pgo._key_poses])
                    max_shift = float(np.linalg.norm(post_t - pre_t, axis=1).max())
                    self._pose_jump.append(max_shift, ts=obs.ts)

            last_obs = obs
            count += 1

            if self.emit_every > 0 and count % self.emit_every == 0:
                yield self._make_obs(pgo, last_obs, count)

        if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
            yield self._make_obs(pgo, last_obs, count)


def pgo_trajectories(
    stream: Any,
    *,
    loop_score: Stream[float] | None = None,
    pose_jump: Stream[float] | None = None,
    global_map_voxel_size: float | None = None,
    **pgo_cfg: Any,
) -> tuple[Any, ...]:
    """Run PGO over a lidar stream, return (drifted_path, corrected_path[, map]).

    ``drifted`` is the raw odometry pose at each keyframe (pre-PGO).
    ``corrected`` is iSAM2's optimized pose after all loop closures
    have settled. Useful for plotting both as overlays on the
    voxels-only global map to see exactly where PGO compensated.

    ``loop_score`` and ``pose_jump`` are the same optional side-streams
    that :class:`PGOMapTransformer` exposes (per-loop-closure ICP
    fitness and max keyframe shift, respectively).

    If ``global_map_voxel_size`` is given, also returns the PGO global
    map as a 3rd element — keyframe body clouds re-projected through
    their final corrected poses and voxel-downsampled at the given
    resolution. This is essentially free (body clouds are cached) but
    has the duplicate-wall artifact described in
    :func:`pgo_then_voxels`'s docstring.

    Each path is a :class:`dimos.msgs.nav_msgs.Path.Path`; pass to
    :class:`dimos.memory2.vis.space.elements.Polyline` to render in
    a ``Space``.
    """
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Path import Path

    cfg = PGOConfig(**pgo_cfg)
    pgo = _SimplePGO(cfg)
    for obs in stream:
        if obs.pose is None:
            continue
        x, y, z, qx, qy, qz, qw = obs.pose
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

            pre_t: np.ndarray | None = None
            if pgo._cache_pairs:
                if loop_score is not None:
                    for pair in pgo._cache_pairs:
                        loop_score.append(float(pair["score"]), ts=obs.ts)
                if pose_jump is not None:
                    pre_t = np.stack([kp.t_global.copy() for kp in pgo._key_poses])

            pgo.smooth_and_update()

            if pre_t is not None and pose_jump is not None:
                post_t = np.stack([kp.t_global for kp in pgo._key_poses])
                pose_jump.append(float(np.linalg.norm(post_t - pre_t, axis=1).max()), ts=obs.ts)

    def _make_path(rs: list[np.ndarray], ts: list[np.ndarray], stamps: list[float]) -> Path:
        poses = []
        for r_kp, t_kp, ts_kp in zip(rs, ts, stamps, strict=True):
            q = Rotation.from_matrix(r_kp).as_quat()
            poses.append(
                PoseStamped(
                    ts=ts_kp,
                    position=[float(t_kp[0]), float(t_kp[1]), float(t_kp[2])],
                    orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
                )
            )
        return Path(poses=poses, frame_id=cfg.world_frame)

    timestamps = [kp.timestamp for kp in pgo._key_poses]
    drifted = _make_path(
        [kp.r_local for kp in pgo._key_poses],
        [kp.t_local for kp in pgo._key_poses],
        timestamps,
    )
    corrected = _make_path(
        [kp.r_global for kp in pgo._key_poses],
        [kp.t_global for kp in pgo._key_poses],
        timestamps,
    )

    if global_map_voxel_size is not None and pgo.num_key_poses > 0:
        cloud_np = pgo.build_global_map(global_map_voxel_size)
        global_map = PointCloud2.from_numpy(cloud_np, frame_id=cfg.world_frame)
        return drifted, corrected, global_map
    return drifted, corrected


def apply_pgo_corrections(
    stream: Any,
    drifted_path: Any,
    corrected_path: Any,
    *,
    voxel_size: float = 0.05,
    block_count: int = 2_000_000,
    device: str = "CUDA:0",
    frame_id: str = FRAME_MAP,
) -> PointCloud2:
    """Pass 2 of two-pass PGO mapping, given the keyframe paths from a
    prior :func:`pgo_trajectories` run.

    Re-streams every lidar frame through :class:`VoxelGrid`, transforming
    each frame's world cloud by the rigid drift correction interpolated
    (SLERP for rotation, linear for translation) between the surrounding
    keyframes' corrections (``drifted -> corrected``).

    Use this when you've already called :func:`pgo_trajectories` and want
    to avoid re-running PGO. The result is identical to what
    :func:`pgo_then_voxels` would produce.
    """
    from scipy.spatial.transform import Slerp

    from dimos.mapping.voxels import VoxelGrid

    drifted_poses = drifted_path.poses
    corrected_poses = corrected_path.poses
    if len(drifted_poses) != len(corrected_poses):
        raise ValueError("drifted_path and corrected_path must have matching pose counts")

    if len(drifted_poses) < 2:
        # No correction possible — fall back to plain voxels insertion
        grid = VoxelGrid(voxel_size=voxel_size, block_count=block_count, device=device)
        try:
            for obs in stream:
                grid.add_frame(obs.data)
            return grid.get_global_pointcloud2()
        finally:
            grid.dispose()

    def _quat_to_R(q: Any) -> np.ndarray:
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    kf_ts = np.array([ps.ts for ps in corrected_poses])
    # Per-keyframe rigid drift correction: T_corr = T_global @ T_local.inv()
    R_corr_list: list[np.ndarray] = []
    t_corr_list: list[np.ndarray] = []
    for d, c in zip(drifted_poses, corrected_poses, strict=True):
        r_local = _quat_to_R(d.orientation)
        t_local = np.array([d.position.x, d.position.y, d.position.z])
        r_global = _quat_to_R(c.orientation)
        t_global = np.array([c.position.x, c.position.y, c.position.z])
        R_corr = r_global @ r_local.T
        R_corr_list.append(R_corr)
        t_corr_list.append(t_global - R_corr @ t_local)
    t_corrs = np.stack(t_corr_list)
    rot_slerp = Slerp(kf_ts, Rotation.from_matrix(np.stack(R_corr_list)))

    grid = VoxelGrid(
        voxel_size=voxel_size, block_count=block_count, device=device, frame_id=frame_id
    )
    try:
        n_inserted = 0
        for obs in stream:
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
        return grid.get_global_pointcloud2()
    finally:
        grid.dispose()


def pgo_then_voxels(
    stream: Any,
    *,
    voxel_size: float = 0.05,
    block_count: int = 2_000_000,
    device: str = "CUDA:0",
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
    "smear of slightly-offset re-projections" that
    ``PGOMapTransformer.build_global_map`` produces.
    """
    from scipy.spatial.transform import Slerp

    from dimos.mapping.voxels import VoxelGrid

    cfg = PGOConfig(**pgo_cfg)
    pgo = _SimplePGO(cfg)

    n_frames = 0
    for obs in stream:
        if obs.pose is None:
            continue
        x, y, z, qx, qy, qz, qw = obs.pose
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

    grid = VoxelGrid(voxel_size=voxel_size, block_count=block_count, device=device)
    try:
        if n_kf < 2:
            for obs in stream:
                grid.add_frame(obs.data)
            return grid.get_global_pointcloud2()

        kf_ts = np.array([kp.timestamp for kp in pgo._key_poses])
        # Per-keyframe rigid drift correction: T_corr = T_global @ T_local.inv()
        R_corr_list = [kp.r_global @ kp.r_local.T for kp in pgo._key_poses]
        t_corr_list = [
            kp.t_global - (kp.r_global @ kp.r_local.T) @ kp.t_local for kp in pgo._key_poses
        ]
        t_corrs = np.stack(t_corr_list)
        rot_slerp = Slerp(kf_ts, Rotation.from_matrix(np.stack(R_corr_list)))

        n_inserted = 0
        for obs in stream:
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


def process_scan(
    pgo: _SimplePGO,
    cloud: PointCloud2,
    r_local: np.ndarray,
    t_local: np.ndarray,
    ts: float,
    unregister_input: bool,
) -> tuple[Odometry, Transform] | None:
    """Add a keyframe, run loop closure, return messages to publish (None on empty cloud).

    Caller must hold ``pgo``'s lock during this call.
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
        if self.config.publish_global_map:
            self._thread = threading.Thread(target=self._publish_loop, daemon=True)
            self._thread.start()
        logger.info(
            "PGO module started (gtsam iSAM2)",
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
        corrected_odom, tf_msg = result
        self.corrected_odometry.publish(corrected_odom)
        self.tf.publish(tf_msg)

    def _publish_corrected_odom(self, r: np.ndarray, t: np.ndarray, ts: float) -> None:
        self.corrected_odometry.publish(build_corrected_odometry(r, t, ts))

    def _publish_map_odom_tf(self, r_offset: np.ndarray, t_offset: np.ndarray, ts: float) -> None:
        """Publish the map → odom correction transform."""
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
                        PointCloud2.from_numpy(
                            cloud_np, frame_id=self.config.world_frame, timestamp=now
                        )
                    )
                self._last_global_map_time = t0

            elapsed = time.monotonic() - t0
            sleep_time = max(DEFAULT_THREAD_JOIN_TIMEOUT, interval - elapsed)
            time.sleep(sleep_time)
