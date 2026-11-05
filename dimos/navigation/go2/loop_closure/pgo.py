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

"""Pose graph optimization: lidar keyframes, ICP loop closures, gtsam ISAM2.

- ``PGO``: ``Transformer[PointCloud2, PoseGraph]``, a cumulative snapshot per
  accepted loop plus one at end of stream.
- ``PoseGraph``: frozen keyframes and loops; as a ``Transformer`` it rewrites
  ``obs.pose`` from ``world_raw`` into ``world_corrected``.
- ``PGOState``: the incremental solver ``PGOMap`` drives live, premap included.

Typical usage::

    graph = lidar.transform(PGO()).last().data        # batch
    corrected = some_stream.transform(graph)          # apply
    for snapshot in lidar.transform(PGO()): ...       # live viz

``gtsam`` is imported lazily, so ``PoseGraph`` alone doesn't need it.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, TypedDict, TypeVar, cast

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.transform import Transformer
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.premap import ROS_GTSAM_SWAP
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import gtsam  # type: ignore[import-not-found,import-untyped]
    from typing_extensions import Unpack  # TODO(PY311): import from typing

    from dimos.navigation.go2.loop_closure.premap import Premap

T = TypeVar("T")

FRAME_WORLD_CORRECTED = "world_corrected"
FRAME_WORLD_RAW = "world_raw"
FRAME_BODY = "body"
FRAME_WORLD = "world"
FRAME_MAP = "map"

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

    # Odometry between-factor variances, gtsam order [rx, ry, rz, x, y, z]; tight z for a ground robot.
    odom_rot_var: float = 1e-6
    odom_trans_var_xy: float = 1e-4
    odom_trans_var_z: float = 1e-6

    # Loop rotation variance; translation variance is the ICP fitness, floored at 1e-4.
    loop_rot_var: float = 0.05

    # Premap attachment: this many agreeing fixes in a row settle it.
    reloc_settle_after: int = 3
    # A fix this close to the attachment agrees with it.
    reloc_agree_m: float = 0.25
    reloc_agree_deg: float = 2.0
    reloc_trans_sigma: float = 0.2
    reloc_rot_sigma_deg: float = 2.0
    # A disagreeing fix must outscore the attachment on the same window by this much to move it.
    reloc_move_margin: float = 0.05
    # Premap loop closures ICP a keyframe against the premap cropped to this radius.
    premap_crop_m: float = 15.0


class PGOKwargs(TypedDict, total=False):
    """Typed kwargs for `PGO`; mirrors `PGOConfig` fields."""

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
    reloc_settle_after: int
    reloc_agree_m: float
    reloc_agree_deg: float
    reloc_trans_sigma: float
    reloc_rot_sigma_deg: float
    reloc_move_margin: float
    premap_crop_m: float


@dataclass(frozen=True)
class Keyframe:
    """A pose graph node.

    `local` is the body pose in the odom (`world_raw`) frame, `optimized` is
    the body pose in `world_corrected`.
    """

    ts: float
    local: Transform
    optimized: Transform


@dataclass(frozen=True)
class LoopClosure:
    """An accepted ICP loop-closure edge in the pose graph.

    `source` and `target` are the two keyframes' optimized body poses in
    `world_corrected`. `score` is the edge's translation variance (floored ICP
    fitness, m^2); lower is tighter.
    """

    source: Transform
    target: Transform
    score: float


@dataclass(frozen=True)
class PoseGraph(Transformer[Any, Any]):
    """Output of PGO: the keyframe nodes and loop edges of the optimized graph.

    As a ``Transformer`` it rewrites each obs's pose into ``world_corrected``,
    passing data through.
    """

    keyframes: tuple[Keyframe, ...] = ()
    loops: tuple[LoopClosure, ...] = ()

    def correct(self, pose: Transform) -> Transform:
        """Map a raw-world pose into the drift-corrected frame at ``pose.ts``."""
        # Transform.__add__ composes: (T_corr + T_raw) applies T_corr after T_raw.
        return self.correction_at(pose.ts) + pose

    def correction_at(self, ts: float) -> Transform:
        """The ``world_corrected <- world_raw`` transform at ``ts``."""
        R, t = self._corrections(np.array([ts]))
        return Transform(
            translation=Vector3(t[0]),
            rotation=Quaternion.from_rotation_matrix(R[0].as_matrix()),
            frame_id=FRAME_WORLD_CORRECTED,
            child_frame_id=FRAME_WORLD_RAW,
            ts=float(ts),
        )

    def place(self, cloud: PointCloud2) -> PointCloud2:
        """Move each point by the correction for its capture time, anchored at the latest keyframe.

        Capture time is the cloud's per-point ``stamps`` when present, else
        ``cloud.ts``. The latest keyframe's correction maps to identity, so
        current data stays where raw odometry puts it and older data moves.
        """
        pts = cloud.points_f32()
        if not len(pts):
            return cloud
        stamps = cloud.stamps_f64()
        if stamps is None:
            stamps = np.full(len(pts), cloud.ts)
        unique, inverse = np.unique(stamps, return_inverse=True)
        R, t = self._corrections(unique)
        now_R, now_t = self._corrections(np.array([np.inf]))
        anchor = now_R[0].inv()
        R, t = anchor * R, anchor.apply(t - now_t[0])
        return PointCloud2.from_numpy(
            R[inverse].apply(pts.astype(np.float64)) + t[inverse],
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
        )

    def __call__(self, upstream: Iterator[Observation[Any]]) -> Iterator[Observation[Any]]:
        """Rewrite obs.pose via :meth:`correct`; pass through pose-less obs unchanged."""
        for obs in upstream:
            ps = obs.pose_stamped
            if ps is None:
                yield obs
                continue
            raw_tf = Transform.from_pose(FRAME_BODY, ps)
            yield obs.derive(data=obs.data, pose=self.correct(raw_tf))

    def _corrections(self, ts: np.ndarray) -> tuple[Rotation, np.ndarray]:
        """Vectorized drift correction: slerp rotation, lerp translation, clipped to the keyframes."""
        ts_arr, slerp, t_stack = self._drifts()
        clipped = np.clip(ts, ts_arr[0], ts_arr[-1])
        t = np.stack([np.interp(clipped, ts_arr, t_stack[:, i]) for i in range(3)], axis=1)
        return slerp(clipped), t

    def _drifts(self) -> tuple[np.ndarray, Slerp, np.ndarray]:
        """Lazy per-keyframe drift table (ts, rotation slerp, translations)."""
        cached = self.__dict__.get("_drifts_cache")
        if cached is not None:
            return cast("tuple[np.ndarray, Slerp, np.ndarray]", cached)

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
        table = (ts_arr, Slerp(ts_arr, Rotation.from_quat(np.stack(quat_list))), np.stack(t_list))
        # frozen dataclass
        object.__setattr__(self, "_drifts_cache", table)
        return table


class PGO(Transformer[PointCloud2, "PoseGraph"]):
    """Pose-graph optimization as a memory Transformer.

    Emits a cumulative :class:`PoseGraph` per accepted loop (``emit_on="loop"``)
    or per keyframe (``"keyframe"``), and once more at end of stream.
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
        pgo = PGOState(self._cfg)
        last_kf_count = 0
        last_loop_count = 0
        last_kf_ts: float | None = None

        for obs in upstream:
            pose = obs.pose
            if pose is None:
                continue
            # placeholder poses: zero translation or all-zero quaternion
            if pose.position.is_zero() or pose.orientation.is_zero():
                continue
            pgo.process(_obs_to_pose3(obs), obs.ts, obs.data)

            n_kf = len(pgo._key_poses)
            n_loops = pgo.n_loops
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

        # so `.last()` has something on loop-less recordings
        if last_kf_ts is not None:
            yield Observation(ts=last_kf_ts, data_type=PoseGraph, _data=pgo.snapshot())


def _obs_to_pose3(obs: Observation[Any]) -> gtsam.Pose3:
    """Convert an observation's pose to a `gtsam.Pose3`."""
    pose = obs.pose
    if pose is None:
        raise LookupError("No pose set on this observation")
    return _pose_to_pose3(pose)


def _pose_to_pose3(pose: Pose) -> gtsam.Pose3:
    import gtsam  # type: ignore[import-not-found,import-untyped]

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


@dataclass
class _Edge:
    """One between edge: keyframe ``b`` measured in keyframe ``a``, in gtsam's covariance order."""

    a: int
    b: int
    measured: np.ndarray  # 4x4
    covariance: np.ndarray  # 6x6, [rx, ry, rz, x, y, z]
    kind: Literal["premap", "live", "join"]
    robust: bool = False


class PGOState:
    """Incremental PGO: gtsam ISAM2 over keyframes with ICP loop closures.

    Call `process` per frame (odom pose + body-frame points). Call
    `finalize` once at the end for the sorted, deduped keyframe list.

    With a premap, its keyframes and edges join the solver pinned at its first
    keyframe; once attached, the solver's frame is ``map``. ``join`` edges
    (`relocalize` fixes and live-to-premap loops) belong to the attachment:
    moving it drops them and rebuilds the solver from each session's own edges.

    The graph is `_edges`; gtsam factors are only ever made from it. Premap
    keyframe ``j`` is key ``j`` and live keyframe ``i`` is key ``n_premap + i``.
    """

    def __init__(self, config: PGOConfig, premap: Premap | None = None) -> None:
        import gtsam  # type: ignore[import-not-found,import-untyped]

        self._gtsam = gtsam
        self._cfg = config
        self._key_poses: list[_KeyPose] = []
        self._last_loop_ts: float | None = None
        self._world_correction: gtsam.Pose3 = gtsam.Pose3()  # identity

        self._isam2 = self._new_isam2()
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()
        self._edges: list[_Edge] = []
        self._n_loops = 0  # live loop edges, counted as they land

        self._premap = premap
        self._p = len(premap) if premap is not None else 0
        self._attached = False
        self._agreeing = 0
        self._last_premap_loop_ts: float | None = None
        self.n_attaches = 0
        self.n_premap_loops = 0
        # each keyframe's raw odom pose and estimate, refreshed in _flush
        self._stamps = np.empty(0)
        self._local = np.empty((0, 4, 4))
        self._optimized = np.empty((0, 4, 4))
        self._premap_poses = np.empty((0, 4, 4))
        self._premap_coarse: tuple[np.ndarray, ...] = ()
        self._premap_reach = np.empty(0)
        if premap is not None:
            self._premap_poses = premap.poses.copy()
            self._premap_coarse = tuple(
                PointCloud2.from_numpy(scan).voxel_downsample(config.submap_resolution).points_f32()
                if len(scan)
                else scan
                for scan in premap.scans
            )
            # how far each premap keyframe's points reach from it
            self._premap_reach = np.array(
                [
                    np.linalg.norm(c[:, :2], axis=1).max() if len(c) else 0.0
                    for c in self._premap_coarse
                ]
            )
            for (a, b), m, cov in zip(
                premap.edges, premap.edge_poses, premap.edge_covariances, strict=True
            ):
                gtsam_cov = cov[ROS_GTSAM_SWAP][:, ROS_GTSAM_SWAP]
                self._add_edge(_Edge(int(a), int(b), m, gtsam_cov, "premap"))
            self._graph.add(self._premap_pin())
            for j, m in enumerate(premap.poses):
                self._values.insert(j, gtsam.Pose3(m))

    def _new_isam2(self) -> Any:
        params = self._gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.01)
        params.relinearizeSkip = 1
        return self._gtsam.ISAM2(params)

    def _premap_pin(self) -> Any:
        assert self._premap is not None
        pinned = self._gtsam.noiseModel.Diagonal.Variances(np.full(6, 1e-12))
        return self._gtsam.PriorFactorPose3(0, self._gtsam.Pose3(self._premap.poses[0]), pinned)

    def _factor(self, edge: _Edge) -> Any:
        gtsam = self._gtsam
        noise = gtsam.noiseModel.Gaussian.Covariance(edge.covariance)
        if edge.robust:
            noise = gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Cauchy.Create(1.0), noise
            )
        return gtsam.BetweenFactorPose3(edge.a, edge.b, gtsam.Pose3(edge.measured), noise)

    def _add_edge(self, edge: _Edge) -> None:
        self._edges.append(edge)
        self._graph.add(self._factor(edge))
        if edge.kind == "live" and edge.b != edge.a + 1:
            self._n_loops += 1

    def _reset(self, premap_poses: np.ndarray, live_poses: np.ndarray) -> None:
        """A fresh solver over `_edges` from these poses."""
        gtsam = self._gtsam
        self._isam2 = self._new_isam2()
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()
        if self._premap is not None:
            self._graph.add(self._premap_pin())
        for edge in self._edges:
            self._graph.add(self._factor(edge))
        for j, m in enumerate(premap_poses):
            self._values.insert(j, gtsam.Pose3(m))
        for i, m in enumerate(live_poses):
            self._values.insert(self._p + i, gtsam.Pose3(m))
        self._flush(self._cfg.loop_closure_extra_iterations)

    def _live_gauge(self, at: gtsam.Pose3) -> Any:
        noise = self._gtsam.noiseModel.Diagonal.Variances(np.full(6, 1e-12))
        return self._gtsam.PriorFactorPose3(self._p, at, noise)

    @property
    def attached(self) -> bool:
        return self._attached

    @property
    def settled(self) -> bool:
        return self._attached and self._agreeing >= self._cfg.reloc_settle_after

    def keyframe_stamps(self) -> np.ndarray:
        return self._stamps

    def keyframe_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """Each keyframe's raw odom pose and its pose in the solver's frame: (K, 4, 4) each."""
        return self._local, self._optimized

    def graph_from_world(self) -> np.ndarray:
        """The robot-anchored world frame into the solver's frame (``map`` once attached): 4x4."""
        return np.asarray(self._world_correction.matrix())

    def joint_graph(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Every keyframe and every edge, premap keyframes first, indexed like the keys.

        Stamps (N,), poses in the solver's frame (N, 4, 4), edges (E, 2), each edge's
        measurement (E, 4, 4) and covariance (E, 6, 6) in gtsam's [rx, ry, rz, x, y, z]
        order: the factors themselves, so a saved map is the same graph, not its shape.
        """
        premap_stamps = self._premap.stamps if self._premap is not None else np.empty(0)
        stamps = np.concatenate([premap_stamps, self.keyframe_stamps()])
        poses = np.concatenate([self._premap_poses, self._optimized]).reshape(-1, 4, 4)
        edges = np.array([(e.a, e.b) for e in self._edges], dtype=np.int64).reshape(-1, 2)
        measured = np.array([e.measured for e in self._edges])
        covariances = np.array([e.covariance for e in self._edges]).reshape(-1, 6, 6)
        return stamps, poses, edges, measured.reshape(-1, 4, 4), covariances

    def premap_poses(self) -> np.ndarray:
        """The premap keyframes' current estimates in ``map``: (K, 4, 4)."""
        return self._premap_poses

    def world_to_map(self) -> Transform | None:
        """Where ``map`` sits in the robot-anchored world frame, once attached."""
        if not self._attached:
            return None
        # attached, the solver works in map, so the latest correction is map <- raw odom
        return _pose3_to_transform(
            self._world_correction.inverse(),
            ts=self._key_poses[-1].timestamp,
            frame_id=FRAME_WORLD,
            child_frame_id=FRAME_MAP,
        )

    def relocalize(
        self,
        fix: Transform,
        fitness: float = 1.0,
        current_fitness: float | None = None,
    ) -> None:
        """A relocalizer's ``world -> map`` fix, measured in the raw odom frame at ``fix.ts``.

        ``current_fitness`` scores the current attachment on the same window; a
        disagreeing fix moves the attachment only when it scores higher.
        """
        if self._premap is None or not self._key_poses:
            return
        k = max(0, int(np.searchsorted(self._stamps, fix.ts, side="right")) - 1)
        kp = self._key_poses[k]
        # map as the solver's current frame sees it, if this fix is right
        implied = kp.optimized.compose(kp.local.inverse()).compose(_transform_to_pose3(fix))
        join = self._reloc_edge(k, implied)
        # judged at keyframe k, from the same premap keyframe as the graph has it now
        current = np.linalg.inv(self._premap_poses[join.a]) @ self._optimized[k]
        if self._attached and self._agree(
            self._gtsam.Pose3(join.measured), self._gtsam.Pose3(current)
        ):
            self._add_edge(join)
            self._flush(self._cfg.loop_closure_extra_iterations)
            self._agreeing += 1
            if self._agreeing == self._cfg.reloc_settle_after:
                logger.info("PGO premap: settled", agreeing_fixes=self._agreeing)
            return
        margin = self._cfg.reloc_move_margin
        if self._attached and current_fitness is not None and fitness <= current_fitness + margin:
            logger.info(
                "PGO premap: kept the attachment over a disagreeing fix",
                fitness=round(fitness, 3),
                current_fitness=round(current_fitness, 3),
            )
            # settling takes agreeing fixes in a row; once settled, fixes from unmapped places can't undo it
            if not self.settled:
                self._agreeing = 0
            return
        if self._attached:
            logger.info("PGO premap: relocalization changed its mind, moving the attachment")
        self._edges = [e for e in self._edges if e.kind != "join"] + [join]
        to_map = np.asarray(implied.inverse().matrix())
        self._last_premap_loop_ts = None
        self._attached = True
        self._reset(self._premap.poses, to_map @ self._optimized)
        self._agreeing = 1
        self.n_attaches += 1
        logger.info("PGO premap: attached", attaches=self.n_attaches)

    def _agree(self, a: gtsam.Pose3, b: gtsam.Pose3) -> bool:
        d = a.between(b)
        dist_m = float(np.linalg.norm(np.asarray(d.translation())))
        angle_deg = float(np.degrees(np.linalg.norm(self._gtsam.Rot3.Logmap(d.rotation()))))
        return dist_m <= self._cfg.reloc_agree_m and angle_deg <= self._cfg.reloc_agree_deg

    def _reloc_edge(self, k: int, implied: gtsam.Pose3) -> _Edge:
        """Live keyframe ``k`` where the fix puts it, from the nearest premap keyframe as stored."""
        assert self._premap is not None
        stored = self._premap.poses
        in_map = np.asarray(implied.inverse().matrix()) @ self._optimized[k]
        j = int(np.argmin(np.linalg.norm(stored[:, :3, 3] - in_map[:3, 3], axis=1)))
        rot = np.radians(self._cfg.reloc_rot_sigma_deg)
        trans = self._cfg.reloc_trans_sigma
        cov = np.diag(np.array([rot, rot, rot, trans, trans, trans]) ** 2)
        measured = np.linalg.inv(stored[j]) @ in_map  # k in j, as the matcher saw it
        return _Edge(j, self._p + k, measured, cov, "join", robust=True)

    @property
    def n_keyframes(self) -> int:
        return len(self._key_poses)

    @property
    def n_loops(self) -> int:
        return self._n_loops

    def _live_loops(self) -> list[_Edge]:
        return [e for e in self._edges if e.kind == "live" and e.b != e.a + 1]

    def keyframe_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """Every keyframe in the graph, anchored at the latest: positions (K, 3), xyzw quats (K, 4).

        Once attached that is the premap's keyframes too, first.
        """
        if not self._key_poses:
            return np.empty((0, 3)), np.empty((0, 4))
        poses = self._anchored_poses()
        return poses[:, :3, 3], Rotation.from_matrix(poses[:, :3, :3]).as_quat()

    def loop_segments(self) -> tuple[np.ndarray, np.ndarray]:
        """Every edge but odometry, as endpoints (L, 2, 3), with its translation variance (L,)."""
        if not self._key_poses:
            return np.empty((0, 2, 3)), np.empty(0)
        positions = self._anchored_poses()[:, :3, 3]
        # the premap is in a frame of its own until attached, so only the live edges then
        offset = 0 if self._attached else self._p
        drawn = [e for e in self._edges if e.b != e.a + 1 and (self._attached or e.kind == "live")]
        if not drawn:
            return np.empty((0, 2, 3)), np.empty(0)
        ends = np.array([(e.a - offset, e.b - offset) for e in drawn])
        variance = np.array([np.diag(e.covariance)[3:].mean() for e in drawn])
        return positions[ends], variance

    def _anchored_poses(self) -> np.ndarray:
        """The keyframes ``keyframe_poses`` draws, in the robot-anchored world frame: (K, 4, 4)."""
        _, live = self.keyframe_matrices()
        poses = np.concatenate([self._premap_poses, live]) if self._attached else live
        return np.asarray(np.linalg.inv(self.graph_from_world()) @ poses)

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
        # kept in body frame, placed by the optimized pose
        body_cloud = world_cloud.transform(
            _pose3_to_transform(
                local_pose.inverse(),
                ts=ts,
                frame_id=FRAME_BODY,
                child_frame_id=FRAME_WORLD_RAW,
            )
        ).voxel_downsample(self._cfg.submap_resolution)
        self._add_keyframe(local_pose, ts, body_cloud)
        self._smooth_and_update(self._search_for_loops())

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
        for edge in self._live_loops():
            src = self._key_poses[edge.b - self._p]
            tgt = self._key_poses[edge.a - self._p]
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
                    score=float(np.diag(edge.covariance)[3:].mean()),
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
        idx = len(self._key_poses)
        key = self._p + idx
        optimized = self._world_correction.compose(local_pose)

        self._values.insert(key, optimized)

        if idx == 0:
            self._graph.add(self._live_gauge(optimized))
        else:
            between = self._key_poses[-1].local.inverse().compose(local_pose)
            cfg = self._cfg
            cov = np.diag(
                [
                    cfg.odom_rot_var,
                    cfg.odom_rot_var,
                    cfg.odom_rot_var,
                    cfg.odom_trans_var_xy,
                    cfg.odom_trans_var_xy,
                    cfg.odom_trans_var_z,
                ]
            )
            self._add_edge(_Edge(key - 1, key, np.asarray(between.matrix()), cov, "live"))
        self._stamps = np.append(self._stamps, ts)
        self._local = np.concatenate([self._local, [np.asarray(local_pose.matrix())]])
        self._optimized = np.concatenate([self._optimized, [np.asarray(optimized.matrix())]])

        self._key_poses.append(
            _KeyPose(
                local=local_pose,
                optimized=optimized,
                timestamp=ts,
                body_cloud=body_cloud,
            )
        )

    def _get_submap(
        self, idx: int, half_range: int, *, exclude_idx: int | None = None
    ) -> PointCloud2:
        lo = max(0, idx - half_range)
        hi = min(len(self._key_poses) - 1, idx + half_range)
        indices = [i for i in range(lo, hi + 1) if i != exclude_idx]
        if not indices:
            return PointCloud2()
        first = self._key_poses[indices[0]]
        cloud = first.body_cloud.transform(
            _pose3_to_transform(
                first.optimized,
                ts=first.timestamp,
                frame_id=FRAME_WORLD_CORRECTED,
                child_frame_id=FRAME_BODY,
            )
        )
        for i in indices[1:]:
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

    def _search_for_loops(self) -> _LoopPair | None:
        if len(self._key_poses) < self._cfg.min_keyframes_for_loop_search:
            return None

        cur_ts = self._key_poses[-1].timestamp
        if (
            self._last_loop_ts is not None
            and cur_ts - self._last_loop_ts < self._cfg.min_loop_detect_duration
        ):
            return None

        cur_idx = len(self._key_poses) - 1
        cur_kp = self._key_poses[-1]
        # the nearest keyframe in reach, old enough not to be the recent trail
        dist = np.linalg.norm(self._optimized[:-1, :3, 3] - self._optimized[-1, :3, 3], axis=1)
        old = np.abs(cur_ts - self._stamps[:-1]) > self._cfg.loop_time_thresh
        dist[~old | (dist > self._cfg.loop_search_radius)] = np.inf
        if not np.isfinite(dist).any():
            return None
        loop_idx = int(np.argmin(dist))

        # The target window must not contain the source scan itself.
        target = self._get_submap(loop_idx, self._cfg.loop_submap_half_range, exclude_idx=cur_idx)
        source = self._get_submap(cur_idx, 0)

        icp_tf, fitness = _icp(
            source,
            target,
            max_iter=self._cfg.max_icp_iterations,
            max_dist=self._cfg.max_icp_correspondence_dist,
            min_inliers=self._cfg.min_icp_inliers,
        )
        if fitness > self._cfg.loop_score_thresh:
            return None

        # icp_tf takes cur_kp.optimized -> refined pose (correcting the drift).
        # offset = loop_kp.optimized^-1 * refined = relative pose from loop to cur.
        icp_pose = _transform_to_pose3(icp_tf)
        refined = icp_pose.compose(cur_kp.optimized)
        offset = self._key_poses[loop_idx].optimized.between(refined)

        self._last_loop_ts = cur_ts
        logger.info(
            "Loop closure detected",
            source=cur_idx,
            target=loop_idx,
            score=round(fitness, 4),
        )
        return _LoopPair(source=cur_idx, target=loop_idx, offset=offset, score=fitness)

    def _smooth_and_update(self, loop: _LoopPair | None) -> None:
        has_loop = loop is not None
        if loop is not None:
            # ICP fitness (m^2) as translation variance, a loose fixed rotation variance
            trans_var = max(1e-4, float(loop.score))
            rot_var = self._cfg.loop_rot_var
            cov = np.diag([rot_var, rot_var, rot_var, trans_var, trans_var, trans_var])
            a, b = self._p + loop.target, self._p + loop.source
            self._add_edge(_Edge(a, b, np.asarray(loop.offset.matrix()), cov, "live"))
        has_loop = self._search_premap() or has_loop

        # extra relinearization only when a loop lands
        self._flush(self._cfg.loop_closure_extra_iterations if has_loop else 0)

    def _flush(self, extra_iterations: int) -> None:
        """Hand the pending factors and values to ISAM2 and read the estimates back."""
        gtsam = self._gtsam
        self._isam2.update(self._graph, self._values)
        for _ in range(extra_iterations):
            self._isam2.update()
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

        estimates = self._isam2.calculateBestEstimate()
        for i in range(len(self._key_poses)):
            self._key_poses[i].optimized = estimates.atPose3(self._p + i)
        self._optimized = np.array(
            [np.asarray(kp.optimized.matrix()) for kp in self._key_poses]
        ).reshape(-1, 4, 4)
        if self._premap is not None:
            self._premap_poses = np.array([estimates.atPose3(j).matrix() for j in range(self._p)])

        last = self._key_poses[-1]
        self._world_correction = last.optimized.compose(last.local.inverse())

    def _search_premap(self) -> bool:
        """Close a loop from the newest keyframe into the nearest premap keyframe. True if one landed."""
        if not self._attached or self._premap is None:
            return False
        cur_ts = self._key_poses[-1].timestamp
        if (
            self._last_premap_loop_ts is not None
            and cur_ts - self._last_premap_loop_ts < self._cfg.min_loop_detect_duration
        ):
            return False
        cur_idx = len(self._key_poses) - 1
        cur = self._key_poses[cur_idx].optimized
        poses = self._premap_poses
        at = np.asarray(cur.translation())
        # the premap's points around here, from every keyframe whose points can reach the crop
        reach = self._cfg.premap_crop_m + self._premap_reach
        near = np.linalg.norm(poses[:, :2, 3] - at[:2], axis=1) <= reach
        placed = [
            scan @ poses[i, :3, :3].T + poses[i, :3, 3]
            for i, scan in enumerate(self._premap_coarse)
            if near[i] and len(scan)
        ]
        if not placed:
            return False
        pts = np.concatenate(placed)
        pts = pts[np.linalg.norm(pts[:, :2] - at[:2], axis=1) <= self._cfg.premap_crop_m]
        if len(pts) < self._cfg.min_icp_inliers:
            return False
        j = int(np.argmin(np.linalg.norm(poses[:, :3, 3] - at, axis=1)))
        target = PointCloud2.from_numpy(pts.astype(np.float32)).voxel_downsample(
            self._cfg.submap_resolution
        )
        source = self._get_submap(cur_idx, 0)
        icp_tf, fitness = _icp(
            source,
            target,
            max_iter=self._cfg.max_icp_iterations,
            max_dist=self._cfg.max_icp_correspondence_dist,
            min_inliers=self._cfg.min_icp_inliers,
        )
        if fitness > self._cfg.loop_score_thresh:
            logger.info("PGO premap loop refused", keyframe=cur_idx, score=round(fitness, 4))
            return False
        refined = np.asarray(_transform_to_pose3(icp_tf).compose(cur).matrix())
        rot_var = self._cfg.loop_rot_var
        trans_var = max(1e-4, fitness)
        # fitness-gated like a live loop, so no robust kernel to mute it
        cov = np.diag([rot_var] * 3 + [trans_var] * 3)
        offset = np.linalg.inv(poses[j]) @ refined
        self._add_edge(_Edge(j, self._p + cur_idx, offset, cov, "join"))
        self._last_premap_loop_ts = cur_ts
        self.n_premap_loops += 1
        logger.info(
            "PGO premap loop closed",
            keyframe=cur_idx,
            premap_keyframe=j,
            score=round(fitness, 4),
            premap_loops=self.n_premap_loops,
        )
        return True


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
    """Point-to-plane ICP using Open3D's tensor pipeline.

    Returns ``(tf, fitness)``, fitness being the mean squared inlier distance
    (m^2); identity and inf when rejected.
    """
    import open3d as o3d  # type: ignore[import-untyped]
    import open3d.core as o3c  # type: ignore[import-untyped]

    if len(source) < min_inliers or len(target) < min_inliers:
        return Transform.identity(), float("inf")

    src_pcd = source.pointcloud_tensor
    tgt_pcd = target.pointcloud_tensor

    tgt_pcd.estimate_normals(max_nn=30, radius=0.3)

    device = src_pcd.device
    init_T = o3c.Tensor(
        init.to_matrix() if init is not None else np.eye(4),
        dtype=o3c.float64,
        device=device,
    )

    # 0-correspondence warnings are expected with a tight max_dist
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

    # unlabeled: callers only read .to_matrix()
    tf = Transform.from_matrix(result.transformation.numpy(), ts=source.ts)
    rmse = float(result.inlier_rmse)
    return tf, rmse * rmse
