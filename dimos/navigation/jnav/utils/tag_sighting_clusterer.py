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

"""Clustering AprilTag detections into landmark sightings, shared by the live TagSightings publisher and the
offline sightings CSV: per-glimpse quality gates, then a tag's detections within a cooldown become one sighting,
published at its medoid once enough agree with it (Huber outlier test) and republished under its id when it moves.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

DEFAULT_MAX_DISTANCE_M = 1.0  # camera-to-tag range; PnP depth error grows with range
DEFAULT_MAX_VIEW_ANGLE_DEG = 45.0  # off-axis view; PnP pose flips grow toward grazing views
DEFAULT_CLUSTER_GAP_SEC = (
    5.0  # a tag unseen this long has been left; the next glimpse is a new visit
)
DEFAULT_ROTATION_WEIGHT_M_PER_RAD = (
    0.5  # metres a radian of tag-pose rotation counts as when comparing detections
)
# A blurry tag still solves a pose; these reject the bad glimpses up front.
DEFAULT_MIN_SHARPNESS = 60.0  # Laplacian variance over the tag ROI
DEFAULT_MAX_REPROJ_PX = 2.0  # RMS solvePnP corner reprojection error
DEFAULT_MIN_TAG_PX = 24.0  # tag side length in pixels (sqrt of quad area)
DEFAULT_MAX_LINEAR_SPEED_MPS = (
    0.5  # robot speed past which a glimpse is motion-blurred / time-skewed
)
DEFAULT_MAX_ANGULAR_SPEED_DPS = 50.0  # robot turn rate, same reason
DEFAULT_MIN_OBSERVATIONS = 3  # clusters thinner than this are unreliable
DEFAULT_HUBER_DELTA_M = 0.05  # residual past which a sample is down-weighted

# One tag observation: ts, marker_id, t_cam_marker (xyz + xyzw quat), and the
# per-glimpse quality fields (sharpness, reproj_px, tag_px, speed).
Detection = dict[str, Any]


def view_quality(t_cam_marker: list[float]) -> tuple[float, float]:
    """(distance_m, view_angle_deg) for a tag pose in the camera optical frame.

    distance is the camera->tag range; view_angle is the angle between the line
    of sight and the tag's surface normal (0 = perfectly head-on)."""
    translation = np.array(t_cam_marker[:3], dtype=np.float64)
    distance = float(np.linalg.norm(translation))
    normal = Rotation.from_quat(t_cam_marker[3:7]).as_matrix()[:, 2]
    line_of_sight = translation / (distance + 1e-9)
    cos_angle = abs(float(np.dot(line_of_sight, normal)))
    view_angle = math.degrees(math.acos(min(1.0, cos_angle)))
    return distance, view_angle


def _pose_distance(a: list[float], b: list[float], rotation_weight_m_per_rad: float) -> float:
    return float(_pose_distances(np.array([a]), np.array([b]), rotation_weight_m_per_rad)[0, 0])


def _pose_distances(
    poses: np.ndarray, others: np.ndarray, rotation_weight_m_per_rad: float
) -> np.ndarray:
    """Translation + weighted rotation angle between every row of `poses` and of `others` (xyz + xyzw quat)."""
    translation = np.linalg.norm(poses[:, None, :3] - others[None, :, :3], axis=2)
    cos_half = np.minimum(1.0, np.abs(poses[:, 3:7] @ others[:, 3:7].T))
    distances: np.ndarray = translation + rotation_weight_m_per_rad * 2.0 * np.arccos(cos_half)
    return distances


def _huber_weights(residuals: np.ndarray, delta: float) -> np.ndarray:
    """IRLS Huber weights: 1 inside `delta`, decaying as delta/r past it."""
    weights = np.ones_like(residuals)
    outside = residuals > delta
    weights[outside] = delta / residuals[outside]
    return weights


def glimpse_passes(
    detection: Detection,
    *,
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
    max_view_angle_deg: float = DEFAULT_MAX_VIEW_ANGLE_DEG,
    min_sharpness: float = DEFAULT_MIN_SHARPNESS,
    max_reproj_px: float = DEFAULT_MAX_REPROJ_PX,
    min_tag_px: float = DEFAULT_MIN_TAG_PX,
    max_linear_speed_mps: float = DEFAULT_MAX_LINEAR_SPEED_MPS,
    max_angular_speed_dps: float = DEFAULT_MAX_ANGULAR_SPEED_DPS,
) -> str | None:
    """None when a glimpse passes every per-glimpse quality gate, else the rejection reason
    (a gate whose field the detection lacks passes)."""
    if detection.get("sharpness") is not None and detection["sharpness"] < min_sharpness:
        return "blur"
    if detection.get("reproj_px") is not None and detection["reproj_px"] > max_reproj_px:
        return "reproj"
    if detection.get("tag_px") is not None and detection["tag_px"] < min_tag_px:
        return "small"
    if detection.get("t_cam_marker") is not None:
        distance, view_angle = view_quality(detection["t_cam_marker"])
        if distance > max_distance_m:
            return "far"
        if view_angle > max_view_angle_deg:
            return "oblique"
    speed = detection.get("speed")
    if speed is not None and (speed[0] > max_linear_speed_mps or speed[1] > max_angular_speed_dps):
        return "motion"
    return None


# A detection's tag pose in a frame that only drifts slowly (odometry), where same-tag detections are compared.
STABLE_POSE_KEY = "t_odom_marker"


@dataclass(frozen=True)
class SightingClustering:
    """How `SightingClusterer` turns a tag's detections into sightings. The glimpse gates are loose on purpose: the
    medoid and its Huber outlier test reject the noisy glimpses, and a glance never reaches `min_inliers`."""

    # per-glimpse gates (`glimpse_passes`), each looser than its DEFAULT_* single-glimpse value
    max_distance_m: float = 2.0  # camera-to-tag range; PnP depth error grows with range
    max_view_angle_deg: float = 60.0  # off-axis view; PnP pose flips grow toward grazing views
    min_sharpness: float = 30.0  # Laplacian variance over the tag ROI (blur)
    max_reproj_px: float = 3.0  # RMS solvePnP corner reprojection error
    min_tag_px: float = 16.0  # tag side in pixels; corners get unreliable below this
    max_linear_speed_mps: float = (
        1.0  # robot speed; motion blur and camera/odom time skew grow with it
    )
    max_angular_speed_dps: float = 90.0  # robot turn rate; same reason as the linear gate
    # the same tag seen again within this long joins the open sighting, whose medoid is recomputed
    cooldown_s: float = DEFAULT_CLUSTER_GAP_SEC
    # a sighting is first published once this many detections agree with its medoid
    min_inliers: int = DEFAULT_MIN_OBSERVATIONS
    # residual from the medoid past which a detection is down-weighted, and the Huber weight below which it is an outlier
    huber_delta_m: float = DEFAULT_HUBER_DELTA_M
    min_huber_weight: float = 0.25
    rotation_weight_m_per_rad: float = DEFAULT_ROTATION_WEIGHT_M_PER_RAD
    # a published sighting is republished under its id (superseding it) once its medoid moves further than this
    republish_shift_m: float = 0.02

    def gate_thresholds(self) -> dict[str, float]:
        """The `glimpse_passes` keyword thresholds."""
        return {
            "max_distance_m": self.max_distance_m,
            "max_view_angle_deg": self.max_view_angle_deg,
            "min_sharpness": self.min_sharpness,
            "max_reproj_px": self.max_reproj_px,
            "min_tag_px": self.min_tag_px,
            "max_linear_speed_mps": self.max_linear_speed_mps,
            "max_angular_speed_dps": self.max_angular_speed_dps,
        }


@dataclass(frozen=True)
class SightingUpdate:
    """A sighting to (re)publish at `publish_ts`: its medoid detection; a later update with the same id supersedes it."""

    sighting_id: int
    publish_ts: float
    medoid: Detection
    inliers: int
    detections: int
    supersedes: bool


def cluster_inliers(
    cluster: list[Detection],
    rotation_weight_m_per_rad: float,
    huber_delta_m: float,
    min_huber_weight: float,
    pose_key: str = STABLE_POSE_KEY,
) -> tuple[Detection, list[Detection]]:
    """The medoid and the detections whose Huber weight about it is at least `min_huber_weight` (the rest are
    outliers); the medoid is re-picked among the inliers until they stop changing."""
    poses = np.array([detection[pose_key] for detection in cluster], dtype=np.float64)
    members = np.arange(len(cluster))
    for _ in range(len(cluster)):
        medoid = members[
            int(
                np.argmin(
                    _pose_distances(poses[members], poses[members], rotation_weight_m_per_rad).sum(
                        1
                    )
                )
            )
        ]
        residuals = _pose_distances(poses, poses[[medoid]], rotation_weight_m_per_rad)[:, 0]
        inliers = np.flatnonzero(_huber_weights(residuals, huber_delta_m) >= min_huber_weight)
        if np.array_equal(inliers, members):
            break
        members = inliers
    return cluster[medoid], [cluster[i] for i in members]


@dataclass
class _OpenSighting:
    detections: list[Detection] = field(default_factory=list)
    sighting_id: int | None = None
    published_pose: list[float] | None = None


class SightingClusterer:
    """Streams tag detections (time order, each with `STABLE_POSE_KEY`) into sightings: a tag's detections within
    `cooldown_s` of each other are one sighting, published once `min_inliers` agree with their medoid and republished
    under the same id whenever that medoid moves. The live publisher and the offline sightings CSV both run this."""

    def __init__(self, clustering: SightingClustering | None = None) -> None:
        self.clustering = clustering or SightingClustering()
        self.rejected: dict[str, int] = defaultdict(int)
        self._open: dict[int, _OpenSighting] = {}
        self._next_sighting_id = 0

    def add(self, detection: Detection) -> SightingUpdate | None:
        """Take one detection; the sighting update it causes, if any."""
        clustering = self.clustering
        reason = glimpse_passes(detection, **clustering.gate_thresholds())
        if reason is not None:
            self.rejected[reason] += 1
            return None
        marker = detection["marker_id"]
        sighting = self._open.get(marker)
        if (
            sighting is None
            or detection["ts"] - sighting.detections[-1]["ts"] > clustering.cooldown_s
        ):
            sighting = self._open[marker] = _OpenSighting()
        sighting.detections.append(detection)
        medoid, inliers = cluster_inliers(
            sighting.detections,
            clustering.rotation_weight_m_per_rad,
            clustering.huber_delta_m,
            clustering.min_huber_weight,
        )
        if len(inliers) < clustering.min_inliers:
            return None
        pose = medoid[STABLE_POSE_KEY]
        if sighting.published_pose is not None and (
            _pose_distance(pose, sighting.published_pose, clustering.rotation_weight_m_per_rad)
            <= clustering.republish_shift_m
        ):
            return None
        supersedes = sighting.sighting_id is not None
        if sighting.sighting_id is None:
            sighting.sighting_id = self._next_sighting_id
            self._next_sighting_id += 1
        sighting.published_pose = list(pose)
        return SightingUpdate(
            sighting_id=sighting.sighting_id,
            publish_ts=float(detection["ts"]),
            medoid=medoid,
            inliers=len(inliers),
            detections=len(sighting.detections),
            supersedes=supersedes,
        )
