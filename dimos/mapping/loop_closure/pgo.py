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

"""PGO drift corrections as a memory2 Transformer.

A lidar/odom stream comes in with poses that drift over time — the robot's
estimate of where it is in the world slowly diverges from ground truth as
small per-frame errors accumulate. PGO ("pose graph optimization") detects
revisited places via loop closure and pulls the trajectory back into
self-consistency.

Two public types:

- ``PGO`` is ``Transformer[PointCloud2, PoseGraph]``. Apply it to a lidar
  stream and it emits one cumulative ``PoseGraph`` snapshot per loop-closure
  event (the only state changes that meaningfully restructure the optimized
  poses), plus a final emit at end-of-stream so ``.last()`` always returns
  something even on loop-less recordings.
- ``PoseGraph`` is a frozen dataclass carrying ``keyframes`` (nodes) and
  ``loops`` (edges), *and* is itself ``Transformer[Any, Any]``. Apply via
  ``stream.transform(graph)`` to rewrite ``obs.pose`` from the raw to the
  drift-corrected frame; or call ``graph.correct(pose)`` for a one-off.

Typical usage::

    graph = lidar.transform(PGO()).last().data        # batch
    corrected = some_stream.transform(graph)          # apply
    for snapshot in lidar.transform(PGO()): ...       # live viz

``gtsam`` is imported lazily inside hot helpers so importing this module
stays cheap and gtsam-free for consumers that only need ``PoseGraph``.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, TypedDict, TypeVar, Unpack, cast

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory2.transform import Transformer
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
    loop_time_thresh: float = 20.0
    loop_score_thresh: float = 0.3
    loop_submap_half_range: int = 10
    min_icp_inliers: int = 10
    min_keyframes_for_loop_search: int = 10
    loop_closure_extra_iterations: int = 4
    submap_resolution: float = 0.2
    min_loop_detect_duration: float = 5.0

    # ICP
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 1.0

    # Noise variances on the GTSAM between-factor (Pose3 is [rx, ry, rz, x, y, z]).
    # Defaults are tuned for a Go2-class ground robot: tight z because the
    # platform is planar; isotropic rotation. Loosen on aerial / 6-DoF arms.
    odom_rot_var: float = 1e-6
    odom_trans_var_xy: float = 1e-4
    odom_trans_var_z: float = 1e-6

    # Loop-closure rotation variance. Translation variance is supplied
    # per-edge from ICP fitness, floored at 1e-4 (sigma_trans ~ 1 cm).
    loop_rot_var: float = 0.05


class PGOKwargs(TypedDict, total=False):
    """Typed kwargs for `pgo_keyframes`; mirrors `PGOConfig` fields."""

    key_pose_delta_trans: float
    key_pose_delta_deg: float
    loop_search_radius: float
    loop_time_thresh: float
    loop_score_thresh: float
    loop_submap_half_range: int
    min_icp_inliers: int
    min_keyframes_for_loop_search: int
    loop_closure_extra_iterations: int
    submap_resolution: float
    min_loop_detect_duration: float
    max_icp_iterations: int
    max_icp_correspondence_dist: float
    odom_rot_var: float
    odom_trans_var_xy: float
    odom_trans_var_z: float
    loop_rot_var: float


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


@dataclass(frozen=True)
class PoseGraph(Transformer[Any, Any]):
    """Output of PGO: the keyframe nodes and loop edges of the optimized graph.

    Apply via ``stream.transform(graph)`` to rewrite each obs's pose into
    the drift-corrected frame; the data payload passes through untouched.
    For one-off pose corrections call :meth:`correct` directly.
    """

    keyframes: tuple[Keyframe, ...] = ()
    loops: tuple[LoopClosure, ...] = ()

    def correct(self, pose: Transform) -> Transform:
        """Map a raw-world pose into the drift-corrected frame at ``pose.ts``."""
        # Transform.__add__ composes: (T_corr + T_raw) applies T_corr after T_raw.
        return self.correction_at(pose.ts) + pose

    def correction_at(self, ts: float) -> Transform:
        """The raw ``world_corrected <- world_raw`` transform at ``ts``.

        Useful when applying the correction to non-pose data (e.g. point
        clouds): ``pointcloud.transform(graph.correction_at(obs.ts))``.
        """
        return self._interp()(ts)

    def __call__(self, upstream: Iterator[Observation[Any]]) -> Iterator[Observation[Any]]:
        """Rewrite obs.pose via :meth:`correct`; pass through pose-less obs unchanged."""
        for obs in upstream:
            ps = obs.pose_stamped
            if ps is None:
                yield obs
                continue
            raw_tf = Transform.from_pose(FRAME_BODY, ps)
            yield obs.derive(data=obs.data, pose=self.correct(raw_tf))

    def _interp(self) -> Callable[[float], Transform]:
        """Lazy slerp/lerp drift-correction lookup keyed by ts."""
        cached = self.__dict__.get("_interp_cache")
        if cached is not None:
            return cast("Callable[[float], Transform]", cached)

        if not self.keyframes:
            raise ValueError("PoseGraph has no keyframes")

        # Per-keyframe drift: world_corrected <- body <- world_raw.
        ts_list = [kf.ts for kf in self.keyframes]
        drifts = [(kf.optimized + kf.local.inverse()) for kf in self.keyframes]
        quat_list = [tf.rotation.to_numpy() for tf in drifts]
        t_list = [tf.translation.to_numpy() for tf in drifts]

        # Slerp needs ≥2 keyframes; pad a len==1 list with a near-duplicate.
        if len(ts_list) == 1:
            ts_list.append(ts_list[0] + 1e-6)
            quat_list.append(quat_list[0])
            t_list.append(t_list[0])

        ts_arr = np.array(ts_list)
        t_stack = np.stack(t_list)
        slerp = Slerp(ts_arr, Rotation.from_quat(np.stack(quat_list)))

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
            return Transform(
                translation=Vector3(t),
                rotation=Quaternion.from_rotation_matrix(R),
                frame_id=FRAME_WORLD_CORRECTED,
                child_frame_id=FRAME_WORLD_RAW,
                ts=float(ts),
            )

        # frozen=True blocks plain attribute writes; use object.__setattr__.
        object.__setattr__(self, "_interp_cache", interp)
        return interp


class PGO(Transformer[PointCloud2, "PoseGraph"]):
    """Pose-graph optimization as a memory2 Transformer.

    Emits one cumulative :class:`PoseGraph` snapshot per state change. By
    default (``emit_on="loop"``) that's one emit per accepted loop closure
    plus a final emit at end-of-stream so ``.last()`` always returns
    something even on loop-less recordings. ``emit_on="keyframe"`` emits on
    every new keyframe — noisier, useful for live progress viz.
    """

    def __init__(
        self,
        *,
        emit_on: Literal["loop", "keyframe"] = "loop",
        **cfg: Unpack[PGOKwargs],
    ) -> None:
        self._cfg = PGOConfig(**cfg)
        self._emit_on = emit_on

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[PoseGraph]]:
        pgo = _PGOState(self._cfg)
        last_kf_count = 0
        last_loop_count = 0
        last_kf_ts: float | None = None

        for obs in upstream:
            pose = obs.pose
            if pose is None:
                continue
            # Placeholder filter: zero translation OR uninitialized (all-zero)
            # quaternion. Identity rotation (qw=1) is valid and stays.
            if pose.position.is_zero() or pose.orientation.is_zero():
                continue
            pgo.process(_obs_to_pose3(obs), obs.ts, obs.data)

            n_kf = len(pgo._key_poses)
            n_loops = len(pgo._accepted_loops)
            if n_kf == last_kf_count and n_loops == last_loop_count:
                continue
            last_kf_ts = pgo._key_poses[-1].timestamp if pgo._key_poses else last_kf_ts

            should_emit = (self._emit_on == "loop" and n_loops > last_loop_count) or (
                self._emit_on == "keyframe"
            )
            last_kf_count = n_kf
            last_loop_count = n_loops
            if should_emit and last_kf_ts is not None:
                yield Observation(ts=last_kf_ts, data_type=PoseGraph, _data=pgo.snapshot())

        # Final emit: guarantees `.last()` returns something even on
        # loop-less recordings (and catches any tail state since the last emit).
        if last_kf_ts is not None:
            yield Observation(ts=last_kf_ts, data_type=PoseGraph, _data=pgo.snapshot())


def _obs_to_pose3(obs: Observation[Any]) -> gtsam.Pose3:
    """Convert an observation's pose to a `gtsam.Pose3`."""
    import gtsam  # type: ignore[import-not-found,import-untyped]

    pose = obs.pose
    if pose is None:
        raise LookupError("No pose set on this observation")
    t, r = pose.position, pose.orientation
    return gtsam.Pose3(
        gtsam.Rot3.Quaternion(r.w, r.x, r.y, r.z),
        gtsam.Point3(t.x, t.y, t.z),
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


class _PGOState:
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
            _pose3_to_transform(
                local_pose.inverse(),
                ts=ts,
                frame_id=FRAME_BODY,
                child_frame_id=FRAME_WORLD_RAW,
            )
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

    def snapshot(self) -> PoseGraph:
        """Immutable view of the current pose graph."""
        return PoseGraph(
            keyframes=tuple(self.finalize()),
            loops=tuple(self.loop_closures()),
        )

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
            cfg = self._cfg
            noise = gtsam.noiseModel.Diagonal.Variances(
                np.array(
                    [
                        cfg.odom_rot_var,
                        cfg.odom_rot_var,
                        cfg.odom_rot_var,
                        cfg.odom_trans_var_xy,
                        cfg.odom_trans_var_xy,
                        cfg.odom_trans_var_z,
                    ]
                )
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
            _pose3_to_transform(
                self._key_poses[lo].optimized,
                ts=self._key_poses[lo].timestamp,
                frame_id=FRAME_WORLD_CORRECTED,
                child_frame_id=FRAME_BODY,
            )
        )
        for i in range(lo + 1, hi + 1):
            kp = self._key_poses[i]
            cloud = cloud + kp.body_cloud.transform(
                _pose3_to_transform(
                    kp.optimized,
                    ts=kp.timestamp,
                    frame_id=FRAME_WORLD_CORRECTED,
                    child_frame_id=FRAME_BODY,
                )
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
        cur_t = np.asarray(cur_kp.optimized.translation())

        from scipy.spatial import KDTree

        positions = np.array(
            [np.asarray(kp.optimized.translation()) for kp in self._key_poses[:-1]]
        )
        tree = KDTree(positions)
        idxs = tree.query_ball_point(cur_t, self._cfg.loop_search_radius)
        if not idxs:
            return

        # query_ball_point doesn't sort by distance — do it ourselves and
        # filter by min time gap to avoid closing loops to recent keyframes.
        candidates = [
            (
                float(
                    np.linalg.norm(np.asarray(self._key_poses[i].optimized.translation()) - cur_t)
                ),
                i,
            )
            for i in idxs
            if abs(cur_ts - self._key_poses[i].timestamp) > self._cfg.loop_time_thresh
        ]
        if not candidates:
            return
        candidates.sort()
        loop_idx = candidates[0][1]

        target = self._get_submap(loop_idx, self._cfg.loop_submap_half_range)
        source = self._get_submap(cur_idx, 0)

        icp_tf, fitness = _icp(
            source,
            target,
            max_iter=self._cfg.max_icp_iterations,
            max_dist=self._cfg.max_icp_correspondence_dist,
            min_inliers=self._cfg.min_icp_inliers,
        )
        if fitness > self._cfg.loop_score_thresh:
            return

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

        # 1. Inject the loop closure factors into the active optimization graph
        for pair in self._pending_loops:
            trans_var = max(1e-4, float(pair.score))
            rot_var = self._cfg.loop_rot_var
            base_noise = gtsam.noiseModel.Diagonal.Variances(
                np.array([rot_var, rot_var, rot_var, trans_var, trans_var, trans_var])
            )
            robust_noise = gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Huber.Create(1.345), base_noise
            )
            self._graph.add(
                gtsam.BetweenFactorPose3(pair.target, pair.source, pair.offset, robust_noise)
            )
        self._accepted_loops.extend(self._pending_loops)
        self._pending_loops.clear()

        # Cache keyframe positions right before we run the optimization
        poses_before_update = [np.asarray(kp.optimized.translation()) for kp in self._key_poses]

        # 2. Run the first ISAM2 update with our raw graph and initial values
        # This safely registers new keys into the engine without crashing
        self._isam2.update(self._graph, self._values)

        # Capture the initial Chi2 baseline error right after variables are registered
        chi2_before = self._isam2.getFactorsUnsafe().error(self._isam2.getLinearizationPoint())

        # 3. If a loop is active, perform the extra relinearization passes
        if has_loop:
            for _ in range(self._cfg.loop_closure_extra_iterations):
                self._isam2.update()

        # Capture the optimized Chi2 error value after updates run
        chi2_after = self._isam2.getFactorsUnsafe().error(self._isam2.getLinearizationPoint())

        # Clear active factor graph buffers
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

        # 4. Pull the calculated best estimates out to optimize our local trajectory array
        estimates = self._isam2.calculateBestEstimate()
        for i in range(len(self._key_poses)):
            self._key_poses[i].optimized = estimates.atPose3(i)

        # 5. Extract our displacement tracking measurements (Metric B)
        max_jump = 0.0
        mean_jump = 0.0
        if poses_before_update and len(poses_before_update) == len(self._key_poses):
            displacements = [
                float(
                    np.linalg.norm(
                        np.asarray(self._key_poses[i].optimized.translation())
                        - poses_before_update[i]
                    )
                )
                for i in range(len(self._key_poses))
            ]
            if displacements:
                max_jump = max(displacements)
                mean_jump = np.mean(displacements)

        # 6. Stream status updates out to stdout console
        if has_loop:
            logger.info(
                "PGO Optimization Stats",
                loop_closed=has_loop,
                chi2_initial=round(float(chi2_before), 2),
                chi2_optimized=round(float(chi2_after), 2),
                max_graph_jump_meters=round(max_jump, 4),
                mean_graph_jump_meters=round(mean_jump, 4),
            )
        else:
            if len(self._key_poses) % 50 == 0:
                logger.info(
                    "PGO Odom Status Check",
                    total_keyframes=len(self._key_poses),
                    current_chi2=round(float(chi2_after), 2),
                )

        last = self._key_poses[-1]
        self._world_correction = last.optimized.compose(last.local.inverse())


def _pose3_to_transform(
    pose: gtsam.Pose3,
    *,
    ts: float,
    frame_id: str,
    child_frame_id: str,
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
    """Pure Python-native global registration using the core mechanics of TEASER++.
    Decouples translation and rotation using TIMs, prunes feature mismatches via
    distance checks, solves rotation via GNC-TLS, and resolves translation via 1D Voting.
    """
    if len(source) < min_inliers or len(target) < min_inliers:
        return Transform.identity(), float("inf")

    src_pcd = source.pointcloud_tensor
    tgt_pcd = target.pointcloud_tensor

    # 1. Generate FPFH geometric features to find candidate point pairs
    src_pcd.estimate_normals(max_nn=30, radius=0.3)
    tgt_pcd.estimate_normals(max_nn=30, radius=0.3)
    src_fpfh = o3d.t.pipelines.registration.compute_fpfh_feature(
        src_pcd, max_nn=100, radius=0.5
    ).numpy()
    tgt_fpfh = o3d.t.pipelines.registration.compute_fpfh_feature(
        tgt_pcd, max_nn=100, radius=0.5
    ).numpy()

    src_pts = src_pcd.point.positions.numpy()
    tgt_pts = tgt_pcd.point.positions.numpy()

    # Nearest neighbor matching in feature space
    from scipy.spatial import KDTree

    tree = KDTree(tgt_fpfh)
    _, matches = tree.query(src_fpfh, k=1)

    A = src_pts.T  # Shape: (3, N)
    B = tgt_pts[matches].T  # Shape: (3, N)
    N = A.shape[1]

    # 2. TEASER++ First Step: Construct Translation Invariant Measurements (TIMs)
    # Pairwise subtraction completely cancels out the global translation vector t
    A_tims = []
    B_tims = []
    # Pair points sequentially to keep calculation complexity at O(N) rather than O(N^2)
    for i in range(N - 1):
        A_tims.append(A[:, i] - A[:, i + 1])
        B_tims.append(B[:, i] - B[:, i + 1])

    A_tims = np.stack(A_tims, axis=1)  # Shape: (3, N-1)
    B_tims = np.stack(B_tims, axis=1)  # Shape: (3, N-1)

    # Coarse Pruning Check: If absolute distances change across frames, it's an outlier match
    dist_A = np.linalg.norm(A_tims, axis=0)
    dist_B = np.linalg.norm(B_tims, axis=0)
    inline_mask = np.abs(dist_A - dist_B) < max_dist

    if np.sum(inline_mask) < min_inliers - 1:
        return Transform.identity(), float("inf")

    A_tims_f = A_tims[:, inline_mask]
    B_tims_f = B_tims[:, inline_mask]

    # 3. TEASER++ Second Step: Solve Rotation via Graduated Non-Convexity (GNC-TLS)
    # Set the target boundary threshold matching your noise limits
    cbar2 = max_dist**2
    mu = 1.0  # Initial convexity control parameter
    R_est = np.eye(3)
    weights = np.ones(A_tims_f.shape[1])

    for _gnc_iter in range(20):
        # Weighted Kabsch/SVD alignment algorithm to get current rotation update
        total_weight = weights.sum()
        if total_weight == 0:
            return Transform.identity(), float("inf")
        centroid_A = np.sum(A_tims_f * weights, axis=1, keepdims=True) / total_weight
        centroid_B = np.sum(B_tims_f * weights, axis=1, keepdims=True) / total_weight
        
        A_centered = A_tims_f - centroid_A
        B_centered = B_tims_f - centroid_B

        H = (A_centered * weights) @ B_centered.T
        U, _, Vt = np.linalg.svd(H)

        # Check for reflection condition to keep matrix valid inside SO(3)
        d = np.linalg.det(U @ Vt)
        S = np.eye(3)
        if d < 0:
            S[2, 2] = -1
        R_est = Vt.T @ S @ U.T

        # Update tracking residuals under new rotation
        residuals2 = np.sum((B_tims_f - R_est @ A_tims_f) ** 2, axis=0)

        # Truncated Least Squares weight assignment rules
        th_low = (mu / (mu + 1)) * cbar2
        th_high = ((mu + 1) / mu) * cbar2

        new_weights = np.zeros_like(weights)
        new_weights[residuals2 <= th_low] = 1.0
        new_weights[residuals2 >= th_high] = 0.0

        middle_mask = (residuals2 > th_low) & (residuals2 < th_high)
        if np.any(middle_mask):
            new_weights[middle_mask] = (
                np.sqrt((cbar2 * mu * (mu + 1)) / residuals2[middle_mask]) - mu
            )

        weights = new_weights
        mu *= 1.4  # Advance the non-convex sharpening schedule
        if mu > 1e4:
            break

    # 4. TEASER++ Third Step: Extract Translation via Adaptive 1D Consensus Voting
    # Rotate source cloud toward target frame to find remaining translation offset entries
    A_rotated = R_est @ A
    t_candidates = B - A_rotated  # Shape: (3, N)

    # Resolve translation independently across each spatial coordinate axis [X, Y, Z]
    t_final = np.zeros(3)
    for axis in range(3):
        candidates = t_candidates[axis, :]
        # Fast 1D histogram bin checking to extract global consensus peak
        counts, bins = np.histogram(candidates, bins=50)
        best_bin = np.argmax(counts)
        # Final axis translation value is the average of items sitting inside the winning bin
        in_bin_mask = (candidates >= bins[best_bin]) & (candidates <= bins[best_bin + 1])
        t_final[axis] = np.mean(candidates[in_bin_mask]) if np.any(in_bin_mask) else 0.0

    # 5. Local Optimization Refinement: Run a quick point-to-plane ICP
    # Pass the robust global TEASER++ answer as an accurate initialization guess
    teaser_T = np.eye(4)
    teaser_T[0:3, 0:3] = R_est
    teaser_T[0:3, 3] = t_final

    device = src_pcd.device
    init_T = o3c.Tensor(teaser_T, dtype=o3c.float64, device=device)

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

    if float(result.fitness) == 0.0:
        return Transform.identity(), float("inf")

    tf = Transform.from_matrix(result.transformation.numpy(), ts=source.ts)
    rmse = float(result.inlier_rmse)

    return tf, rmse * rmse
