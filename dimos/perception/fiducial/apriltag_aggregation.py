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

"""Robust multi-sighting fiducial-marker pose aggregation: time-cluster same-marker sightings, gate each, reduce each cluster to one pose via Huber-weighted IRLS (weighted-mean translation + Markley quaternion eigen-mean)."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

# Frame-invariant marker pose 7-vector: (x, y, z, qx, qy, qz, qw), meters + xyzw.
Pose7 = tuple[float, float, float, float, float, float, float]

# Per-glimpse gate thresholds (see gate_reason).
DEFAULT_MAX_DISTANCE_M = 1.0  # camera->tag range past which perspective is too weak
DEFAULT_MAX_VIEW_ANGLE_DEG = 45.0  # line-of-sight vs tag normal; grazing views mis-solve
DEFAULT_MAX_REPROJ_PX = 2.0  # RMS solvePnP corner reprojection error
DEFAULT_MIN_TAG_PX = 24.0  # tag side length in pixels (sqrt of quad area)
# streaming: span back from the newest glimpse, not a between-sightings gap
DEFAULT_TIME_WINDOW_S = 5.0
DEFAULT_MIN_OBSERVATIONS = 3  # clusters thinner than this are unreliable
DEFAULT_ROTATION_WEIGHT_M_PER_RAD = 0.5  # 1 rad of rot ~ 0.5 m of trans in pose distance
DEFAULT_HUBER_DELTA_M = 0.05  # residual (m) past which a sample is down-weighted
_HUBER_ITERATIONS = 5  # IRLS weights settle within ~5 iters at huber_delta_m scale

# Reference geometry where tag_noise_scale == 1.0 (pose carries unscaled covariance).
REF_DISTANCE_M = 0.4  # camera->tag range a close, in-gate glimpse sits at
REF_REPROJ_PX = 1.0  # RMS corner misfit of a sharp glimpse
MIN_SCALE_DISTANCE_M = 0.2  # m; nearer than this the range term stops shrinking variance
MIN_SCALE_REPROJ_PX = 0.5  # px; sharper than this the reproj term stops shrinking variance
MIN_TAG_NOISE_SCALE = 0.25  # dimensionless floor, so no read claims near-zero variance


@dataclass(frozen=True)
class AggregationConfig:
    """Per-glimpse gate thresholds + clustering/aggregation knobs."""

    max_distance_m: float = DEFAULT_MAX_DISTANCE_M
    max_view_angle_deg: float = DEFAULT_MAX_VIEW_ANGLE_DEG
    max_reproj_px: float = DEFAULT_MAX_REPROJ_PX
    min_tag_px: float = DEFAULT_MIN_TAG_PX
    min_observations: int = DEFAULT_MIN_OBSERVATIONS
    rotation_weight_m_per_rad: float = DEFAULT_ROTATION_WEIGHT_M_PER_RAD
    huber_delta_m: float = DEFAULT_HUBER_DELTA_M
    time_window_s: float = DEFAULT_TIME_WINDOW_S  # streaming sliding-window span


@dataclass(frozen=True)
class TagObservation:
    """One gated tag glimpse; a ``None`` quality field disables ITS gate only."""

    ts: float
    marker_id: int
    pose: Pose7
    distance_m: float | None = None
    view_angle_deg: float | None = None
    reproj_px: float | None = None
    tag_px: float | None = None


@dataclass(frozen=True)
class TagEstimate:
    """One robust aggregated pose for a marker over a cluster/window of glimpses."""

    pose: Pose7  # aggregated 7-vec, obs' frame
    n_observations: int
    # Cluster MEDIANS, not the last glimpse's -- health of the whole cluster's pose.
    distance_m: float | None = None
    reproj_px: float | None = None


def pose7_from_matrix(matrix: np.ndarray) -> Pose7:
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


def view_quality(optical_T_marker: tuple[float, ...] | list[float]) -> tuple[float, float]:
    """``(distance_m, view_angle_deg)`` for a tag pose in the CAMERA optical frame (view_angle is line-of-sight vs the tag normal, 0 == head-on; camera-relative for the gate)."""
    translation = np.array(optical_T_marker[:3], dtype=np.float64)
    distance_m = float(np.linalg.norm(translation))
    normal = Rotation.from_quat(optical_T_marker[3:7]).as_matrix()[:, 2]
    line_of_sight = translation / (distance_m + 1e-9)  # m; guards zero range
    cos_angle = abs(float(np.dot(line_of_sight, normal)))
    view_angle_deg = math.degrees(math.acos(min(1.0, cos_angle)))
    return distance_m, view_angle_deg


def tag_side_px(corners_px: np.ndarray) -> float:
    """Tag side length in pixels: sqrt of the corner quad's image area (shoelace)."""
    quad = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    x, y = quad[:, 0], quad[:, 1]
    # shoelace: matches cv2.contourArea for a simple quad, keeps this core OpenCV-free
    area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    return math.sqrt(area)


def tag_noise_scale(distance_m: float | None, reproj_px: float | None) -> float:
    """Dimensionless variance inflation for one aggregated tag pose (jnav tag_noise), quadratic in range x reproj_px; a ``None`` input reads as the reference."""
    distance_m = REF_DISTANCE_M if distance_m is None else distance_m
    reproj_px = REF_REPROJ_PX if reproj_px is None else reproj_px
    range_term = (max(distance_m, MIN_SCALE_DISTANCE_M) / REF_DISTANCE_M) ** 2
    reproj_term = (max(reproj_px, MIN_SCALE_REPROJ_PX) / REF_REPROJ_PX) ** 2
    return max(range_term * reproj_term, MIN_TAG_NOISE_SCALE)


def tag_covariance(pose: tuple[float, ...] | list[float], scale: float) -> np.ndarray:
    """6x6 covariance for an aggregated tag pose, built in the tag frame and rotated into the pose's frame."""
    R = Rotation.from_quat(pose[3:7]).as_matrix()
    covariance = np.zeros((6, 6))
    # ROS is TRANSLATION-first, so these blocks sit swapped vs jnav's rotation-first GTSAM Pose3.
    # m^2: 5 cm in-plane, 50 cm along the tag normal (PnP's weak axis)
    covariance[:3, :3] = R @ np.diag([0.0025, 0.0025, 0.25]) @ R.T
    # rad^2: 11.5 deg about the mirror axes, 2.9 deg spin
    covariance[3:, 3:] = R @ np.diag([0.04, 0.04, 0.0025]) @ R.T
    return covariance * scale


def gate_reason(obs: TagObservation, config: AggregationConfig) -> str | None:
    """Rejection reason, or ``None`` if the glimpse clears every gate whose input is present (a ``None`` field skips ITS gate rather than failing shut)."""
    if obs.reproj_px is not None and obs.reproj_px > config.max_reproj_px:
        return "reproj"
    if obs.tag_px is not None and obs.tag_px < config.min_tag_px:
        return "small"
    if obs.distance_m is not None and obs.distance_m > config.max_distance_m:
        return "far"
    if obs.view_angle_deg is not None and obs.view_angle_deg > config.max_view_angle_deg:
        return "oblique"
    return None


def _pose_distance(
    a: tuple[float, ...], b: tuple[float, ...], rotation_weight_m_per_rad: float
) -> float:
    """Combined translation + weighted-rotation distance between two 7-vec poses."""
    translation_m = float(np.linalg.norm(np.array(a[:3]) - np.array(b[:3])))
    rotation_rad = 2.0 * math.acos(min(1.0, abs(float(np.dot(a[3:7], b[3:7])))))
    return translation_m + rotation_weight_m_per_rad * rotation_rad


def cluster_medoid(
    cluster: list[TagObservation], rotation_weight_m_per_rad: float
) -> TagObservation:
    """The observation whose pose is most central (min total pose-distance) -- a robust seed for the Huber refinement."""
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
    """IRLS Huber weights: 1 inside ``delta``, decaying as ``delta / r`` past it. https://en.wikipedia.org/wiki/Huber_loss"""
    weights = np.ones_like(residuals)
    outside = residuals > delta
    weights[outside] = (
        delta / residuals[outside]
    )  # Huber IRLS https://en.wikipedia.org/wiki/Huber_loss
    return weights


def robust_cluster_pose(
    cluster: list[TagObservation],
    rotation_weight_m_per_rad: float,
    huber_delta_m: float,
) -> Pose7:
    """Cluster representative: the medoid refined by Huber-weighted IRLS -- weighted-mean translation + Markley quaternion eigen-mean https://ntrs.nasa.gov/citations/20070017872."""
    medoid = cluster_medoid(cluster, rotation_weight_m_per_rad)
    if len(cluster) < 2:
        return medoid.pose
    poses = np.array([obs.pose for obs in cluster], dtype=np.float64)
    translations, quaternions = poses[:, :3], poses[:, 3:7]
    # Sign-align to the medoid hemisphere: q and -q are one rotation, else the mean cancels.
    reference = np.array(medoid.pose[3:7], dtype=np.float64)
    signs = np.sign(quaternions @ reference)
    signs[signs == 0] = 1.0
    quaternions = quaternions * signs[:, None]
    estimate_translation = np.array(medoid.pose[:3], dtype=np.float64)
    estimate_quaternion = reference.copy()
    # delta in radians so the rotation residual shares the translation delta's robustness scale.
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
        estimate_quaternion = np.linalg.eigh(scatter)[1][
            :, -1
        ]  # Markley quaternion eigen-mean (2007) https://ntrs.nasa.gov/citations/20070017872
        if estimate_quaternion @ reference < 0:
            estimate_quaternion = -estimate_quaternion
    aggregated = (*estimate_translation.tolist(), *estimate_quaternion.tolist())
    return aggregated  # type: ignore[return-value]


def _median_present(values: list[float | None]) -> float | None:
    """Median over the members that carry the field, ``None`` when none do -- median not mean, so one in-gate outlier cannot inflate the health signal."""
    present = [v for v in values if v is not None]
    return float(np.median(present)) if present else None


def aggregate_by_marker_id(
    observations: list[TagObservation],
    rotation_weight_m_per_rad: float = DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
    huber_delta_m: float = DEFAULT_HUBER_DELTA_M,
) -> dict[int, tuple[Pose7, int]]:
    """Batch: group observations by marker_id and fuse each group to one robust pose; id -> (7-vec, n)."""
    by_id: dict[int, list[TagObservation]] = defaultdict(list)
    for obs in observations:
        by_id[obs.marker_id].append(obs)
    return {
        marker_id: (
            robust_cluster_pose(group, rotation_weight_m_per_rad, huber_delta_m),
            len(group),
        )
        for marker_id, group in by_id.items()
    }


class TagAggregator:
    """Streaming robust pose per marker over a sliding time window."""

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
        # Purge relative to this marker's newest glimpse, not wall time, so a paused marker keeps its window.
        newest = obs.ts
        self._by_marker[obs.marker_id] = [
            o for o in buf if newest - o.ts <= self._config.time_window_s
        ]
        return None

    def robust_estimate(self, marker_id: int) -> TagEstimate | None:
        """Huber-aggregated pose for a marker's in-window glimpses, or ``None`` when fewer than ``min_observations`` are available."""
        buf = self._by_marker.get(marker_id, [])
        if len(buf) < self._config.min_observations:
            return None
        pose = robust_cluster_pose(
            buf, self._config.rotation_weight_m_per_rad, self._config.huber_delta_m
        )
        return TagEstimate(
            pose=pose,
            n_observations=len(buf),
            distance_m=_median_present([o.distance_m for o in buf]),
            reproj_px=_median_present([o.reproj_px for o in buf]),
        )
