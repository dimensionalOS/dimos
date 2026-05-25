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

"""PGO drift corrections as composable Stream stages.

A lidar/odom stream comes in with poses that drift over time — the robot's
estimate of where it is in the world slowly diverges from ground truth as
small per-frame errors accumulate. PGO ("pose graph optimization") detects
revisited places via loop closure and pulls the trajectory back into
self-consistency. The result is a stream of *corrections*: a Transform per
keyframe that maps every drifted ("raw") pose to its corrected one.

Pipeline:

    lidar: Stream[PointCloud2]                         # pose-stamped scans
        -> pgo_keyframes(...)             -> Stream[Keyframe]    # one per kf
        -> keyframes_to_corrections(...)  -> Stream[Transform]   # world_corrected <- world_raw
        -> apply_corrections(any_stream, corrections) -> Stream[T]  # obs.pose shuffled

`pgo_keyframes` runs ISAM2 + ICP loop closure across the input, emitting
each kept keyframe with both its raw odom pose (`local`) and its optimized
pose (`optimized`) as Transforms. Pass `loop_closures_out=[]` to also
collect each accepted ICP loop edge as a `LoopClosure` (source+target
optimized poses + ICP fitness) — useful for visualizing the pose graph.

`keyframes_to_corrections` is pure arithmetic: per keyframe, the drift
correction is `optimized @ local^-1` (Transform composition). The resulting
stream has one entry per keyframe — sparse in time.

`make_interpolator` materializes the sparse correction stream into a
ts -> Transform function: SLERP for rotation, linear lerp for translation,
clipping out-of-range queries to the endpoints.

`apply_corrections` shuffles `obs.pose` on any stream using the interpolated
correction at each obs.ts. It's read-only on `obs.data`, so the same lidar
stream can be re-applied — or a different recording from the same physical
space, if the corrections generalize.

`gtsam` is imported lazily inside hot helpers so importing this module
stays cheap and gtsam-free for consumers that only need `Keyframe` /
`apply_corrections`.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, TypedDict, TypeVar, Unpack

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import gtsam  # type: ignore[import-not-found,import-untyped]

T = TypeVar("T")

FRAME_WORLD_CORRECTED = "world_corrected"
FRAME_WORLD_RAW = "world_raw"
FRAME_BODY = "body"

logger = setup_logger()


class PGOConfig(BaseConfig):
    # Keyframe detection
    key_pose_delta_trans: float = 0.5
    key_pose_delta_deg: float = 10.0

    # Loop closure
    loop_search_radius: float = 2.0
    loop_search_radius_local: float = 5.0
    loop_time_thresh: float = 20.0
    loop_score_thresh: float = 0.3
    loop_score_thresh_tight: float = 0.05  # early-exit threshold: ICP rmse ~22cm
    loop_submap_half_range: int = 20
    loop_candidates_per_iter: int = 5
    min_icp_inliers: int = 10
    min_keyframes_for_loop_search: int = 10
    loop_closure_extra_iterations: int = 4
    submap_resolution: float = 0.2
    min_loop_detect_duration: float = 5.0

    # ICP
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 1.0


class PGOKwargs(TypedDict, total=False):
    """Typed kwargs for `pgo_keyframes`; mirrors `PGOConfig` fields."""

    key_pose_delta_trans: float
    key_pose_delta_deg: float
    loop_search_radius: float
    loop_search_radius_local: float
    loop_time_thresh: float
    loop_score_thresh: float
    loop_score_thresh_tight: float
    loop_submap_half_range: int
    loop_candidates_per_iter: int
    min_icp_inliers: int
    min_keyframes_for_loop_search: int
    loop_closure_extra_iterations: int
    submap_resolution: float
    min_loop_detect_duration: float
    max_icp_iterations: int
    max_icp_correspondence_dist: float


@dataclass(frozen=True)
class Keyframe:
    """Keyframe emitted by `pgo_keyframes`.

    `local` is the body pose in the odom (`world_raw`) frame, `optimized` is
    the body pose in the drift-corrected (`world_corrected`) frame.
    """

    ts: float
    local: Transform
    optimized: Transform


@dataclass(frozen=True)
class LoopClosure:
    """An accepted ICP loop-closure edge in the pose graph.

    `source` and `target` are the final optimized body poses (in
    `world_corrected`) of the two keyframes the loop connected. `score` is
    the ICP fitness used as the loop's translation variance — lower is a
    tighter match.
    """

    source: Transform
    target: Transform
    score: float


def pgo_keyframes(
    stream: Stream[PointCloud2],
    *,
    on_frame: Callable[[Any], None] | None = None,
    loop_closures_out: list[LoopClosure] | None = None,
    **pgo_cfg: Unpack[PGOKwargs],
) -> Stream[Keyframe]:
    """Run PGO across a pose-stamped point-cloud stream; emit one obs per keyframe.

    If `loop_closures_out` is provided, accepted loop edges are appended to
    it (in detection order) using each keyframe's final optimized pose.
    """
    cfg = PGOConfig(**pgo_cfg)
    pgo = _PGO(cfg)

    for obs in stream:
        if on_frame is not None:
            on_frame(obs)
        if obs.pose is None:
            continue
        # Skip placeholder poses (origin position OR zero quaternion).
        if obs.pose[0] == 0 and obs.pose[1] == 0 and obs.pose[2] == 0:
            continue
        if (
            obs.pose[3] == 0
            and obs.pose[4] == 0
            and obs.pose[5] == 0
            and (obs.pose[6] == 0 or obs.pose[6] == 1)
        ):
            continue
        local_pose = _obs_to_pose3(obs)
        pgo.process(local_pose, obs.ts, obs.data)

    mem = MemoryStore()
    out: Stream[Keyframe] = mem.stream("keyframes", Keyframe)
    for kf in pgo.finalize():
        out.append(kf, ts=kf.ts)
    if loop_closures_out is not None:
        loop_closures_out.extend(pgo.loop_closures())
    return out


def keyframes_to_corrections(keyframes: Stream[Keyframe]) -> Stream[Transform]:
    """Per-keyframe drift correction as Transform(world_corrected <- world_raw)."""
    mem = MemoryStore()
    out: Stream[Transform] = mem.stream("corrections", Transform)
    for obs in keyframes:
        kf = obs.data
        # world_corrected <- body <- world_raw composes to world_corrected <- world_raw.
        drift = kf.optimized + kf.local.inverse()
        out.append(drift, ts=kf.ts)
    return out


def make_interpolator(corrections: Stream[Transform]) -> Callable[[float], Transform]:
    """Materialize corrections once; return a fast ts -> Transform lookup."""
    ts_list: list[float] = []
    R_list: list[np.ndarray] = []
    t_list: list[np.ndarray] = []
    for obs in corrections:
        R, t = _r_t_from_transform(obs.data)
        # obs.ts is authoritative; obs.data.ts can be mutated by Transform's
        # ts=0.0 -> time.time() fallback in its constructor.
        ts_list.append(obs.ts)
        R_list.append(R)
        t_list.append(t)

    if not ts_list:
        raise ValueError("empty corrections stream")

    # Slerp needs ≥2 keyframes. Pad len==1 with a duplicate so the
    # general path handles it; clip-to-endpoints behavior is unchanged.
    if len(ts_list) == 1:
        ts_list.append(ts_list[0] + 1e-6)
        R_list.append(R_list[0])
        t_list.append(t_list[0])

    ts_arr = np.array(ts_list)
    R_stack = np.stack(R_list)
    t_stack = np.stack(t_list)
    slerp = Slerp(ts_arr, Rotation.from_matrix(R_stack))

    def interp(ts: float) -> Transform:
        ts_clip = float(np.clip(ts, ts_arr[0], ts_arr[-1]))
        R = slerp([ts_clip])[0].as_matrix()
        idx = int(np.searchsorted(ts_arr, ts_clip))
        if idx == 0:
            t = t_stack[0]
        elif idx >= len(ts_arr):
            t = t_stack[-1]
        else:
            t_lo, t_hi = ts_arr[idx - 1], ts_arr[idx]
            alpha = (ts_clip - t_lo) / (t_hi - t_lo) if t_hi > t_lo else 0.0
            t = (1 - alpha) * t_stack[idx - 1] + alpha * t_stack[idx]
        return _transform_from_r_t(R, t, ts=float(ts))

    return interp


def apply_corrections(
    stream: Stream[T],
    corrections: Stream[Transform],
) -> Stream[T]:
    """Shuffle obs.pose on `stream` by the interpolated correction at each obs.ts.

    `obs.data` is untouched. Frames with `obs.pose is None` pass through
    unchanged. Out-of-range `obs.ts` get the endpoint correction (clipped).
    """
    interp = make_interpolator(corrections)

    def xf(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        for obs in upstream:
            if obs.pose is None:
                yield obs
                continue
            raw_tf = Transform.from_pose(FRAME_BODY, obs.pose_stamped)
            # Transform.__add__ composes: (T_corr + T_raw) applies T_corr after T_raw.
            # Observation normalizes Transform back to 7-tuple via __post_init__.
            corrected = interp(obs.ts) + raw_tf
            yield obs.derive(data=obs.data, pose=corrected)

    return stream.transform(xf)


def _r_t_from_transform(tf: Transform) -> tuple[np.ndarray, np.ndarray]:
    q = tf.rotation
    R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
    return R, t


def _transform_from_r_t(R: np.ndarray, t: np.ndarray, *, ts: float) -> Transform:
    return Transform(
        translation=Vector3(float(t[0]), float(t[1]), float(t[2])),
        rotation=Quaternion.from_rotation_matrix(R),
        frame_id=FRAME_WORLD_CORRECTED,
        child_frame_id=FRAME_WORLD_RAW,
        ts=ts,
    )


def _obs_to_pose3(obs: Observation[Any]) -> gtsam.Pose3:
    """Convert an observation's stored pose tuple directly to a `gtsam.Pose3`."""
    import gtsam  # type: ignore[import-not-found,import-untyped]

    if obs.pose is None:
        raise LookupError("No pose set on this observation")
    x, y, z, qx, qy, qz, qw = obs.pose
    return gtsam.Pose3(
        gtsam.Rot3.Quaternion(float(qw), float(qx), float(qy), float(qz)),
        gtsam.Point3(float(x), float(y), float(z)),
    )


@dataclass
class _KeyPose:
    local: gtsam.Pose3  # odom-frame pose at capture
    optimized: gtsam.Pose3  # drift-corrected pose
    timestamp: float
    body_cloud: PointCloud2  # voxel-downsampled, body frame


@dataclass
class _LoopPair:
    source: int
    target: int
    offset: gtsam.Pose3  # source pose in target's frame
    score: float


class _PGO:
    """Incremental PGO: gtsam ISAM2 over keyframes with ICP loop closures.

    Call `process` per frame (odom pose + body-frame points). Call
    `finalize` once at the end for the sorted, deduped keyframe list.
    """

    def __init__(self, config: PGOConfig) -> None:
        import gtsam  # type: ignore[import-not-found,import-untyped]

        self._gtsam = gtsam
        self._cfg = config
        self._key_poses: list[_KeyPose] = []
        self._pending_loops: list[_LoopPair] = []
        self._accepted_loops: list[_LoopPair] = []
        self._last_loop_ts: float | None = None
        self._world_correction: gtsam.Pose3 = gtsam.Pose3()  # identity

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.01)
        params.relinearizeSkip = 1
        self._isam2 = gtsam.ISAM2(params)
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

    def process(
        self,
        local_pose: gtsam.Pose3,
        ts: float,
        world_cloud: PointCloud2,
    ) -> None:
        if len(world_cloud) == 0:
            return
        if not self._is_keyframe(local_pose):
            return
        # Unregister: lift world-frame scan back into body frame using its
        # odom pose, so PGO can re-project it via the optimized pose later.
        body_cloud = world_cloud.transform(
            _pose3_to_transform(local_pose.inverse(), ts=ts)
        ).voxel_downsample(self._cfg.submap_resolution)
        self._add_keyframe(local_pose, ts, body_cloud)
        self._search_for_loops()
        self._smooth_and_update()

    def finalize(self) -> list[Keyframe]:
        """Return keyframes sorted by ts, with duplicate-ts entries dropped."""
        kps = sorted(self._key_poses, key=lambda kp: kp.timestamp)
        out: list[Keyframe] = []
        for i, kp in enumerate(kps):
            if i > 0 and kp.timestamp <= kps[i - 1].timestamp:
                continue
            out.append(
                Keyframe(
                    ts=kp.timestamp,
                    local=_pose3_to_transform(
                        kp.local,
                        ts=kp.timestamp,
                        frame_id=FRAME_WORLD_RAW,
                        child_frame_id=FRAME_BODY,
                    ),
                    optimized=_pose3_to_transform(
                        kp.optimized,
                        ts=kp.timestamp,
                        frame_id=FRAME_WORLD_CORRECTED,
                        child_frame_id=FRAME_BODY,
                    ),
                )
            )
        return out

    def loop_closures(self) -> list[LoopClosure]:
        """Accepted loop edges, resolved to final optimized poses."""
        out: list[LoopClosure] = []
        for pair in self._accepted_loops:
            src = self._key_poses[pair.source]
            tgt = self._key_poses[pair.target]
            out.append(
                LoopClosure(
                    source=_pose3_to_transform(
                        src.optimized,
                        ts=src.timestamp,
                        frame_id=FRAME_WORLD_CORRECTED,
                        child_frame_id=FRAME_BODY,
                    ),
                    target=_pose3_to_transform(
                        tgt.optimized,
                        ts=tgt.timestamp,
                        frame_id=FRAME_WORLD_CORRECTED,
                        child_frame_id=FRAME_BODY,
                    ),
                    score=pair.score,
                )
            )
        return out

    def _is_keyframe(self, local_pose: gtsam.Pose3) -> bool:
        if not self._key_poses:
            return True
        delta = self._key_poses[-1].local.inverse().compose(local_pose)
        delta_trans = float(np.linalg.norm(np.asarray(delta.translation())))
        # Rotation magnitude from trace: cos(theta) = (tr(R) - 1) / 2.
        R = delta.rotation().matrix()
        cos_theta = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
        delta_deg = float(np.degrees(np.arccos(cos_theta)))
        return (
            delta_trans > self._cfg.key_pose_delta_trans or delta_deg > self._cfg.key_pose_delta_deg
        )

    def _add_keyframe(
        self,
        local_pose: gtsam.Pose3,
        ts: float,
        body_cloud: PointCloud2,
    ) -> None:
        gtsam = self._gtsam
        idx = len(self._key_poses)
        optimized = self._world_correction.compose(local_pose)

        self._values.insert(idx, optimized)

        if idx == 0:
            noise = gtsam.noiseModel.Diagonal.Variances(np.full(6, 1e-12))
            self._graph.add(gtsam.PriorFactorPose3(idx, optimized, noise))
        else:
            last_local = self._key_poses[-1].local
            between = last_local.inverse().compose(local_pose)
            noise = gtsam.noiseModel.Diagonal.Variances(
                np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6])
            )
            self._graph.add(gtsam.BetweenFactorPose3(idx - 1, idx, between, noise))

        self._key_poses.append(
            _KeyPose(
                local=local_pose,
                optimized=optimized,
                timestamp=ts,
                body_cloud=body_cloud,
            )
        )

    def _get_submap(self, idx: int, half_range: int) -> PointCloud2:
        lo = max(0, idx - half_range)
        hi = min(len(self._key_poses) - 1, idx + half_range)
        if lo > hi:
            return PointCloud2()
        cloud = self._key_poses[lo].body_cloud.transform(
            _pose3_to_transform(self._key_poses[lo].optimized, ts=self._key_poses[lo].timestamp)
        )
        for i in range(lo + 1, hi + 1):
            kp = self._key_poses[i]
            cloud = cloud + kp.body_cloud.transform(
                _pose3_to_transform(kp.optimized, ts=kp.timestamp)
            )
        return cloud.voxel_downsample(self._cfg.submap_resolution)

    def _search_for_loops(self) -> None:
        if len(self._key_poses) < self._cfg.min_keyframes_for_loop_search:
            return

        cur_ts = self._key_poses[-1].timestamp
        if (
            self._last_loop_ts is not None
            and cur_ts - self._last_loop_ts < self._cfg.min_loop_detect_duration
        ):
            return

        cur_idx = len(self._key_poses) - 1
        cur_kp = self._key_poses[-1]
        cur_opt_t = np.asarray(cur_kp.optimized.translation())
        cur_loc_t = np.asarray(cur_kp.local.translation())

        from scipy.spatial import KDTree

        # Search both optimized and local (odom) frames and union. After
        # the robot is disturbed, optimized drift can exceed the search
        # radius; the local frame is drift-free over short horizons so
        # revisits still appear in range there.
        opt_positions = np.array(
            [np.asarray(kp.optimized.translation()) for kp in self._key_poses[:-1]]
        )
        loc_positions = np.array(
            [np.asarray(kp.local.translation()) for kp in self._key_poses[:-1]]
        )
        # Tight radius in optimized frame (precise revisits). Only fall
        # back to wider local-frame search when the optimized-frame search
        # returns nothing — drift has pushed neighbors out of range and
        # we need relocalization. For easy datasets, this keeps the
        # candidate set tight (no extra noise from far neighbors).
        opt_idxs = KDTree(opt_positions).query_ball_point(cur_opt_t, self._cfg.loop_search_radius)
        if opt_idxs:
            idxs = set(opt_idxs)
        else:
            idxs = set(KDTree(loc_positions).query_ball_point(cur_loc_t, self._cfg.loop_search_radius_local))
        if not idxs:
            return

        # Sort by min(opt_dist, loc_dist) and filter out recent keyframes.
        candidates = []
        for i in idxs:
            if abs(cur_ts - self._key_poses[i].timestamp) <= self._cfg.loop_time_thresh:
                continue
            d_opt = float(np.linalg.norm(np.asarray(self._key_poses[i].optimized.translation()) - cur_opt_t))
            d_loc = float(np.linalg.norm(np.asarray(self._key_poses[i].local.translation()) - cur_loc_t))
            candidates.append((min(d_opt, d_loc), i))
        if not candidates:
            return
        candidates.sort()

        # Try top-K candidates; keep the best (lowest-score) one that
        # passes the threshold. Early-exit when ICP is very tight (the
        # common case for easy datasets — saves K-1 redundant ICP runs).
        # The multi-candidate fallback only fires when the closest match
        # is weak, i.e. exactly the relocalization-after-disturbance case.
        source = self._get_submap(cur_idx, 0)
        best: tuple[int, Transform, float] | None = None
        for _, loop_idx in candidates[: self._cfg.loop_candidates_per_iter]:
            target = self._get_submap(loop_idx, self._cfg.loop_submap_half_range)
            icp_tf, fitness = _icp(
                source,
                target,
                max_iter=self._cfg.max_icp_iterations,
                max_dist=self._cfg.max_icp_correspondence_dist,
                min_inliers=self._cfg.min_icp_inliers,
            )
            if fitness <= self._cfg.loop_score_thresh and (best is None or fitness < best[2]):
                best = (loop_idx, icp_tf, fitness)
                if fitness <= self._cfg.loop_score_thresh_tight:
                    break
        if best is None:
            return
        loop_idx, icp_tf, fitness = best

        # icp_tf takes cur_kp.optimized -> refined pose (correcting the drift).
        # offset = loop_kp.optimized^-1 * refined = relative pose from loop to cur.
        icp_pose = _transform_to_pose3(icp_tf)
        refined = icp_pose.compose(cur_kp.optimized)
        offset = self._key_poses[loop_idx].optimized.between(refined)

        self._pending_loops.append(
            _LoopPair(
                source=cur_idx,
                target=loop_idx,
                offset=offset,
                score=fitness,
            )
        )
        self._last_loop_ts = cur_ts
        logger.info(
            "Loop closure detected",
            source=cur_idx,
            target=loop_idx,
            score=round(fitness, 4),
        )

    def _smooth_and_update(self) -> None:
        gtsam = self._gtsam
        has_loop = bool(self._pending_loops)

        for pair in self._pending_loops:
            # Pose3 noise is [rx, ry, rz, x, y, z]. The two halves have
            # different units (rad² vs m²), so a uniform variance silently
            # makes one half pathological. Use ICP fitness as the
            # *translation* variance and a generous fixed rotation variance
            # — loops shouldn't be trusted to fix rotation tightly without
            # normals + p2plane.
            trans_var = max(0.01, float(pair.score))  # >= sigma_trans = 10 cm
            rot_var = 0.01  # sigma_rot ~ 5.7 deg (p2plane ICP gives reliable rotation)
            noise = gtsam.noiseModel.Diagonal.Variances(
                np.array([rot_var, rot_var, rot_var, trans_var, trans_var, trans_var])
            )
            self._graph.add(gtsam.BetweenFactorPose3(pair.target, pair.source, pair.offset, noise))
        self._accepted_loops.extend(self._pending_loops)
        self._pending_loops.clear()

        self._isam2.update(self._graph, self._values)
        self._isam2.update()
        if has_loop:
            for _ in range(self._cfg.loop_closure_extra_iterations):
                self._isam2.update()
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

        estimates = self._isam2.calculateBestEstimate()
        for i in range(len(self._key_poses)):
            self._key_poses[i].optimized = estimates.atPose3(i)

        last = self._key_poses[-1]
        self._world_correction = last.optimized.compose(last.local.inverse())


def _pose3_to_transform(
    pose: gtsam.Pose3,
    *,
    ts: float,
    frame_id: str = "",
    child_frame_id: str = "",
) -> Transform:
    """PGO-internal: build a Transform from a Pose3."""
    t = np.asarray(pose.translation())
    return Transform(
        translation=Vector3(float(t[0]), float(t[1]), float(t[2])),
        rotation=Quaternion.from_rotation_matrix(pose.rotation().matrix()),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        ts=ts,
    )


def _transform_to_pose3(tf: Transform) -> gtsam.Pose3:
    """PGO-internal: build a Pose3 from a Transform."""
    import gtsam  # type: ignore[import-not-found,import-untyped]

    return gtsam.Pose3(tf.to_matrix())


def _icp(
    source: PointCloud2,
    target: PointCloud2,
    max_iter: int = 50,
    max_dist: float = 1.0,
    tol: float = 1e-6,
    min_inliers: int = 10,
    init: Transform | None = None,
) -> tuple[Transform, float]:
    """Point-to-plane ICP using Open3D's tensor pipeline.

    Returns ``(tf, fitness)`` where ``fitness`` is mean squared inlier
    distance (m^2) — the GTSAM noise model in `_smooth_and_update` uses
    this directly as sigma_trans squared. On rejection (too few points or
    zero correspondences) returns the identity transform and inf fitness.
    """

    if len(source) < min_inliers or len(target) < min_inliers:
        return Transform.identity(), float("inf")

    src_pcd = source.pointcloud_tensor
    tgt_pcd = target.pointcloud_tensor

    # Normals on the target enable point-to-plane ICP — converges tighter
    # than point-to-point on indoor scenes (walls give unambiguous normals
    # that resolve the slide-along-wall ambiguity).
    tgt_pcd.estimate_normals(max_nn=30, radius=0.3)

    device = src_pcd.device
    init_T = o3c.Tensor(
        init.to_matrix() if init is not None else np.eye(4),
        dtype=o3c.float64,
        device=device,
    )

    # Silence Open3D's "0 correspondence" warning — we deliberately use a
    # tight max_correspondence_distance and reject loops with poor fitness.
    # Treat singular-system failures as rejection rather than aborting.
    try:
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
    except RuntimeError:
        return Transform.identity(), float("inf")

    if float(result.fitness) == 0.0:
        return Transform.identity(), float("inf")

    T_mat = result.transformation.numpy()
    rmse = float(result.inlier_rmse)
    return _transform_from_r_t(T_mat[:3, :3], T_mat[:3, 3], ts=source.ts), rmse * rmse
