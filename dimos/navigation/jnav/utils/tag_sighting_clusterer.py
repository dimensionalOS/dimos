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

"""TagSightings' clustering of AprilTag detections into landmark sightings: per-glimpse gates, then a cooldown window per tag, published at its medoid."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

# One tag observation: ts, marker_id, the stable-frame tag pose and optionally t_cam_marker (xyz + xyzw quat).
Detection = dict[str, Any]


def view_quality(t_cam_marker: list[float]) -> tuple[float, float]:
    """(distance_m, view_angle_deg) of a camera-frame tag pose: range, and line of sight to tag normal angle (0 = head-on)."""
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
    detection: Detection, *, max_distance_m: float, max_view_angle_deg: float
) -> str | None:
    """None when a glimpse passes the range and view-angle gates (which need `t_cam_marker`), else the rejection reason."""
    if detection.get("t_cam_marker") is not None:
        distance, view_angle = view_quality(detection["t_cam_marker"])
        if distance > max_distance_m:
            return "far"
        if view_angle > max_view_angle_deg:
            return "oblique"
    return None


# A detection's tag pose in a frame that only drifts slowly (odometry), where same-tag detections are compared.
STABLE_POSE_KEY = "t_odom_marker"


@dataclass(frozen=True)
class SightingClustering:
    """How `SightingClusterer` turns detections into sightings; loose glimpse gates, as the medoid's Huber test drops noisy ones."""

    # per-glimpse gates (`glimpse_passes`)
    max_distance_m: float = 2.0  # camera-to-tag range; PnP depth error grows with range
    max_view_angle_deg: float = 60.0  # off-axis view; PnP pose flips grow toward grazing views
    # the same tag seen again within this long joins the open sighting, whose medoid is over its last this-long of detections
    cooldown_s: float = 5.0
    # a sighting is first published once this many detections agree with its medoid
    min_inliers: int = 3
    # residual from the medoid past which a detection is down-weighted, and the Huber weight below which it is an outlier
    huber_delta_m: float = 0.05
    min_huber_weight: float = 0.25
    # metres a radian of tag-pose rotation counts as when comparing detections
    rotation_weight_m_per_rad: float = 0.5
    # a published sighting is republished under its id (superseding it) once its medoid moves further than this
    republish_shift_m: float = 0.02


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
    """The medoid and its inliers (Huber weight >= `min_huber_weight`), the medoid re-picked among them until they settle."""
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
    """Streams tag detections (time order, with `STABLE_POSE_KEY`) into sightings: published once `min_inliers` of a tag's last `cooldown_s` of detections agree with their medoid, republished under the same id when it moves."""

    def __init__(self, clustering: SightingClustering | None = None) -> None:
        self.clustering = clustering or SightingClustering()
        self.rejected: dict[str, int] = defaultdict(int)
        self._open: dict[int, _OpenSighting] = {}
        self._next_sighting_id = 0

    def add(self, detection: Detection) -> SightingUpdate | None:
        """Take one detection; the sighting update it causes, if any."""
        clustering = self.clustering
        reason = glimpse_passes(
            detection,
            max_distance_m=clustering.max_distance_m,
            max_view_angle_deg=clustering.max_view_angle_deg,
        )
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
        while detection["ts"] - sighting.detections[0]["ts"] > clustering.cooldown_s:
            sighting.detections.pop(0)
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
