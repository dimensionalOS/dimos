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

"""Robust multi-sighting fiducial-marker pose aggregation.

Time-cluster same-marker sightings, gate each glimpse independently, and reduce
each cluster to ONE robust pose: the medoid refined by Huber-weighted IRLS
(weighted-mean translation + Markley quaternion eigen-mean), re-weighting each
iteration so a lingering bad glimpse keeps losing influence.

The fused pose is a frame-agnostic 7-vector ``[x, y, z, qx, qy, qz, qw]``; the
reloc ``FiducialPrior`` fuses ``world_T_marker`` (frame-invariant, so cluster
drift does not enter). Only :func:`view_quality` reads a camera-relative pose --
the caller passes ``distance``/``view_angle`` scalars in, keeping the fuse-pose
and the geometry-gate pose distinct.

Two entry points:
 - :func:`aggregate_visits` -- batch: gate + cluster a whole recording into one
   estimate per (marker, visit). Offline benefit harness.
 - :class:`TagAggregator` -- streaming: sliding window per marker, fused on
   demand once ``min_observations`` are in-window. Live prior.

No up-front sharpness gate: the detector's ``QualityWindow`` already drops
blurry frames before these post-detection observations arrive.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

# Per-glimpse gate thresholds; a glimpse must clear every one whose input is
# available (see gate_reason).
DEFAULT_MAX_DISTANCE_M = 1.0  # camera->tag range past which perspective is too weak
DEFAULT_MAX_VIEW_ANGLE_DEG = 45.0  # line-of-sight vs tag normal; grazing views mis-solve
DEFAULT_MAX_REPROJ_PX = 2.0  # RMS solvePnP corner reprojection error
DEFAULT_MIN_TAG_PX = 24.0  # tag side length in pixels (sqrt of quad area)
DEFAULT_MAX_LINEAR_SPEED_MPS = 0.5  # camera motion; faster == motion blur risk
DEFAULT_MAX_ANGULAR_SPEED_DPS = 50.0
DEFAULT_CLUSTER_GAP_S = 5.0  # sightings farther apart in time are separate visits
DEFAULT_MIN_OBSERVATIONS = 3  # clusters thinner than this are unreliable
DEFAULT_ROTATION_WEIGHT_M_PER_RAD = 0.5  # 1 rad of rot ~ 0.5 m of trans in pose distance
DEFAULT_HUBER_DELTA_M = 0.05  # residual (m) past which a sample is down-weighted
_HUBER_ITERATIONS = 5  # IRLS weights settle within ~5 iters at huber_delta_m scale


@dataclass(frozen=True)
class AggregationConfig:
    """Per-glimpse gate thresholds + clustering/fusion knobs.

    Frozen value type (dimos convention for pure-number tunables). Default
    tuning is the autoresearch harness's job (#2137).
    """

    max_distance_m: float = DEFAULT_MAX_DISTANCE_M
    max_view_angle_deg: float = DEFAULT_MAX_VIEW_ANGLE_DEG
    max_reproj_px: float = DEFAULT_MAX_REPROJ_PX
    min_tag_px: float = DEFAULT_MIN_TAG_PX
    max_linear_speed_mps: float = DEFAULT_MAX_LINEAR_SPEED_MPS
    max_angular_speed_dps: float = DEFAULT_MAX_ANGULAR_SPEED_DPS
    cluster_gap_s: float = DEFAULT_CLUSTER_GAP_S
    min_observations: int = DEFAULT_MIN_OBSERVATIONS
    rotation_weight_m_per_rad: float = DEFAULT_ROTATION_WEIGHT_M_PER_RAD
    huber_delta_m: float = DEFAULT_HUBER_DELTA_M
    time_window_s: float = DEFAULT_CLUSTER_GAP_S  # streaming sliding-window span


@dataclass(frozen=True)
class TagObservation:
    """One gated tag glimpse feeding the aggregator.

    ``pose`` is the frame-invariant fuse pose ``(x, y, z, qx, qy, qz, qw)``
    (``world_T_marker`` in the reloc prior). Quality fields drive the gates; a
    ``None`` field disables ITS gate only, so a pixel-less wire-delivered
    observation degrades gracefully.
    """

    ts: float
    marker_id: int
    pose: tuple[float, float, float, float, float, float, float]
    distance_m: float | None = None
    view_angle_deg: float | None = None
    reproj_px: float | None = None
    tag_px: float | None = None
    speed_mps_dps: tuple[float, float] | None = None


@dataclass(frozen=True)
class TagEstimate:
    """One robust fused pose for a marker over a cluster/window of glimpses."""

    marker_id: int
    pose: tuple[float, float, float, float, float, float, float]  # fused 7-vec, obs' frame
    n_observations: int
    ts: float  # latest contributing glimpse's timestamp


def pose7_from_matrix(matrix: np.ndarray) -> tuple[float, float, float, float, float, float, float]:
    """4x4 homogeneous transform -> ``(x, y, z, qx, qy, qz, qw)``."""
    translation = matrix[:3, 3]
    quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()  # x, y, z, w
    return (
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    )


def matrix_from_pose7(pose: tuple[float, ...] | list[float]) -> np.ndarray:
    """``(x, y, z, qx, qy, qz, qw)`` -> 4x4 homogeneous transform."""
    matrix = np.eye(4)
    matrix[:3, 3] = pose[:3]
    matrix[:3, :3] = Rotation.from_quat(pose[3:7]).as_matrix()
    return matrix


def view_quality(pose_cam: tuple[float, ...] | list[float]) -> tuple[float, float]:
    """``(distance_m, view_angle_deg)`` for a tag pose in the CAMERA optical frame.

    view_angle is line-of-sight vs the tag normal (0 == head-on). Camera-relative
    only: on the fuse-pose (world_T_marker) distance would be from world origin,
    meaningless as a gate.
    """
    translation = np.array(pose_cam[:3], dtype=np.float64)
    distance = float(np.linalg.norm(translation))
    normal = Rotation.from_quat(pose_cam[3:7]).as_matrix()[:, 2]
    line_of_sight = translation / (distance + 1e-9)
    cos_angle = abs(float(np.dot(line_of_sight, normal)))
    view_angle = math.degrees(math.acos(min(1.0, cos_angle)))
    return distance, view_angle


def tag_pixel_size(corners_px: np.ndarray) -> float:
    """Tag side length in pixels: sqrt of the corner quad's image area (shoelace).

    Small == few tag pixels == unreliable pose. Shoelace matches
    ``cv2.contourArea`` for a simple quad and keeps this core OpenCV-free.
    """
    quad = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    x, y = quad[:, 0], quad[:, 1]
    area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    return math.sqrt(area)


def gate_reason(obs: TagObservation, config: AggregationConfig) -> str | None:
    """Rejection reason, or ``None`` if the glimpse clears every gate whose input
    is present. A ``None`` field skips ITS gate so a pixel-less or odometry-less
    observation degrades to what it CAN evaluate rather than failing shut."""
    if obs.reproj_px is not None and obs.reproj_px > config.max_reproj_px:
        return "reproj"
    if obs.tag_px is not None and obs.tag_px < config.min_tag_px:
        return "small"
    if obs.distance_m is not None and obs.distance_m > config.max_distance_m:
        return "far"
    if obs.view_angle_deg is not None and obs.view_angle_deg > config.max_view_angle_deg:
        return "oblique"
    if obs.speed_mps_dps is not None and (
        obs.speed_mps_dps[0] > config.max_linear_speed_mps
        or obs.speed_mps_dps[1] > config.max_angular_speed_dps
    ):
        return "motion"
    return None


def cluster_by_time(
    observations: list[TagObservation], gap_s: float
) -> list[list[TagObservation]]:
    """Group same-marker observations into visits. A new cluster begins whenever
    the time gap to the previous same-marker observation exceeds ``gap_s``."""
    by_marker: dict[int, list[TagObservation]] = defaultdict(list)
    for obs in observations:
        by_marker[obs.marker_id].append(obs)
    clusters: list[list[TagObservation]] = []
    for marker_obs in by_marker.values():
        marker_obs.sort(key=lambda o: o.ts)
        current = [marker_obs[0]]
        for obs in marker_obs[1:]:
            if obs.ts - current[-1].ts > gap_s:
                clusters.append(current)
                current = [obs]
            else:
                current.append(obs)
        clusters.append(current)
    return clusters


def _pose_distance(
    a: tuple[float, ...], b: tuple[float, ...], rotation_weight_m_per_rad: float
) -> float:
    """Combined translation + weighted-rotation distance between two 7-vec poses."""
    translation = float(np.linalg.norm(np.array(a[:3]) - np.array(b[:3])))
    rotation = 2.0 * math.acos(min(1.0, abs(float(np.dot(a[3:7], b[3:7])))))
    return translation + rotation_weight_m_per_rad * rotation


def cluster_medoid(
    cluster: list[TagObservation], rotation_weight_m_per_rad: float
) -> TagObservation:
    """The observation whose pose is most central (min total pose-distance to the
    rest) -- a robust, outlier-resistant seed for the Huber refinement."""
    poses = [obs.pose for obs in cluster]
    best_index, best_cost = 0, float("inf")
    for i in range(len(poses)):
        cost = sum(
            _pose_distance(poses[i], poses[j], rotation_weight_m_per_rad)
            for j in range(len(poses))
            if j != i
        )
        if cost < best_cost:
            best_cost, best_index = cost, i
    return cluster[best_index]


def _huber_weights(residuals: np.ndarray, delta: float) -> np.ndarray:
    """IRLS Huber weights: 1 inside ``delta``, decaying as ``delta / r`` past it.
    https://en.wikipedia.org/wiki/Huber_loss
    """
    weights = np.ones_like(residuals)
    outside = residuals > delta
    weights[outside] = delta / residuals[outside]  # Huber IRLS https://en.wikipedia.org/wiki/Huber_loss
    return weights


def robust_cluster_pose(
    cluster: list[TagObservation],
    rotation_weight_m_per_rad: float,
    huber_delta_m: float,
) -> tuple[float, float, float, float, float, float, float]:
    """Cluster representative: the medoid, refined by Huber-weighted IRLS --
    weighted-mean translation + Markley quaternion eigen-mean (largest
    eigenvector of the sign-aligned quaternions' weighted scatter). Re-weighting
    each iteration keeps down-weighting a lingering bad glimpse (e.g. a mirror-
    flip PnP solution the ambiguity gate missed).
    Markley quaternion averaging: https://ntrs.nasa.gov/citations/20070017872
    """
    medoid = cluster_medoid(cluster, rotation_weight_m_per_rad)
    if len(cluster) < 2:
        return medoid.pose
    poses = np.array([obs.pose for obs in cluster], dtype=np.float64)
    translations, quaternions = poses[:, :3], poses[:, 3:7]
    # Sign-align quaternions to the medoid hemisphere: q and -q are the same
    # rotation, so averaging without this can cancel to garbage.
    reference = np.array(medoid.pose[3:7], dtype=np.float64)
    signs = np.sign(quaternions @ reference)
    signs[signs == 0] = 1.0
    quaternions = quaternions * signs[:, None]
    estimate_translation = np.array(medoid.pose[:3], dtype=np.float64)
    estimate_quaternion = reference.copy()
    # delta in radians so the rotation residual shares the translation delta's
    # robustness scale (pose_distance weights 1 rad ~ rotation_weight m).
    delta_rad = huber_delta_m / max(rotation_weight_m_per_rad, 1e-9)
    for _ in range(_HUBER_ITERATIONS):
        weights_t = _huber_weights(
            np.linalg.norm(translations - estimate_translation, axis=1), huber_delta_m
        )
        estimate_translation = (weights_t[:, None] * translations).sum(0) / weights_t.sum()
        angular_residual = 2.0 * np.arccos(
            np.clip(np.abs(quaternions @ estimate_quaternion), 0.0, 1.0)
        )
        weights_r = _huber_weights(angular_residual, delta_rad)
        scatter = (
            weights_r[:, None, None] * np.einsum("ni,nj->nij", quaternions, quaternions)
        ).sum(0)
        estimate_quaternion = np.linalg.eigh(scatter)[1][:, -1]  # Markley quaternion eigen-mean (2007) https://ntrs.nasa.gov/citations/20070017872
        if estimate_quaternion @ reference < 0:
            estimate_quaternion = -estimate_quaternion
    fused = (*estimate_translation.tolist(), *estimate_quaternion.tolist())
    return fused  # type: ignore[return-value]


def aggregate_visits(
    observations: list[TagObservation], config: AggregationConfig
) -> tuple[list[TagEstimate], dict[str, int]]:
    """Batch: gate every glimpse, time-cluster into visits, drop thin clusters,
    and Huber-refine each surviving cluster to one estimate.

    Returns the per-visit estimates (ts-sorted) and a per-reason rejection
    tally (blur/reproj/... counts) so a caller can log WHY glimpses were cut.
    """
    rejected: dict[str, int] = defaultdict(int)
    kept: list[TagObservation] = []
    for obs in observations:
        reason = gate_reason(obs, config)
        if reason is not None:
            rejected[reason] += 1
        else:
            kept.append(obs)

    estimates: list[TagEstimate] = []
    for cluster in cluster_by_time(kept, config.cluster_gap_s):
        if len(cluster) < config.min_observations:
            rejected["thin"] += len(cluster)
            continue
        pose = robust_cluster_pose(
            cluster, config.rotation_weight_m_per_rad, config.huber_delta_m
        )
        estimates.append(
            TagEstimate(
                marker_id=cluster[0].marker_id,
                pose=pose,
                n_observations=len(cluster),
                ts=max(o.ts for o in cluster),
            )
        )
    estimates.sort(key=lambda e: e.ts)
    return estimates, dict(rejected)


class TagAggregator:
    """Streaming robust pose per marker over a sliding time window.

    ``observe()`` gates a glimpse and appends it to its marker's window, purging
    glimpses older than ``time_window_s`` from the newest. ``robust_estimate()``
    Huber-fuses once a marker has ``min_observations`` in-window.
    """

    def __init__(self, config: AggregationConfig) -> None:
        self._config = config
        self._by_marker: dict[int, list[TagObservation]] = defaultdict(list)

    def observe(self, obs: TagObservation) -> str | None:
        """Feed one glimpse; return its rejection reason, or ``None`` if kept."""
        reason = gate_reason(obs, self._config)
        if reason is not None:
            return reason
        buf = self._by_marker[obs.marker_id]
        buf.append(obs)
        # Purge relative to this marker's newest glimpse: a marker that left
        # view keeps its window until it returns (then stale glimpses drop). The
        # prior's own age gate, not this buffer, decides carry-forward staleness.
        newest = obs.ts
        self._by_marker[obs.marker_id] = [
            o for o in buf if newest - o.ts <= self._config.time_window_s
        ]
        return None

    def robust_estimate(self, marker_id: int) -> TagEstimate | None:
        """Huber-fused pose for a marker's in-window glimpses, or ``None`` when
        fewer than ``min_observations`` are available."""
        buf = self._by_marker.get(marker_id, [])
        if len(buf) < self._config.min_observations:
            return None
        pose = robust_cluster_pose(
            buf, self._config.rotation_weight_m_per_rad, self._config.huber_delta_m
        )
        return TagEstimate(
            marker_id=marker_id,
            pose=pose,
            n_observations=len(buf),
            ts=max(o.ts for o in buf),
        )
