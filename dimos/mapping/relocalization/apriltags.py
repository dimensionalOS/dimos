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

"""AprilTag landmarks: per-glimpse gates, then one robust pose per tag.

Same-id detections are clustered in time (a new cluster after ``gap_sec`` without
the tag), each cluster is reduced to its medoid refined by Huber-weighted IRLS, thin
clusters are dropped, and the clusters of one tag (separate visits) are reduced the
same way into the tag's pose. Ported from jnav's ``utils/apriltags.py``, with poses
already in a fixed frame instead of the camera's.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math
from typing import Any

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_HUBER_ITERATIONS = 5


@dataclass(frozen=True)
class TagGates:
    """Per-glimpse gates (jnav's defaults); a field a detection lacks is not gated on."""

    max_distance_m: float = 1.0
    max_view_angle_deg: float = 45.0
    min_sharpness: float = 60.0  # Laplacian variance over the tag ROI
    max_reproj_px: float = 2.0  # RMS solvePnP corner reprojection error
    min_tag_px: float = 24.0  # tag side length in pixels
    max_linear_speed_mps: float = 0.5
    max_angular_speed_dps: float = 50.0


@dataclass(frozen=True)
class TagClustering:
    gap_sec: float = 5.0
    min_observations: int = 3  # clusters thinner than this are unreliable
    huber_delta_m: float = 0.05  # residual past which a sample is down-weighted
    rotation_weight_m_per_rad: float = 0.5


@dataclass
class TagDetection:
    """One glimpse of a tag: its pose (xyz + xyzw quaternion) in a fixed frame."""

    ts: float
    marker_id: int
    pose: list[float]


def view_quality(tag_in_camera: list[float]) -> tuple[float, float]:
    """(distance_m, view_angle_deg) of a tag pose in the camera optical frame."""
    translation = np.array(tag_in_camera[:3], dtype=np.float64)
    distance = float(np.linalg.norm(translation))
    normal = Rotation.from_quat(tag_in_camera[3:7]).as_matrix()[:, 2]
    cos_angle = abs(float(np.dot(translation / (distance + 1e-9), normal)))
    return distance, math.degrees(math.acos(min(1.0, cos_angle)))


def rejection(tags: dict[str, Any], tag_in_camera: list[float], gates: TagGates) -> str | None:
    """Why a glimpse is rejected (None: kept), from the detector's recorded quality fields."""
    if tags.get("sharpness", math.inf) < gates.min_sharpness:
        return "blur"
    if tags.get("reproj_px", 0.0) > gates.max_reproj_px:
        return "reproj"
    if tags.get("tag_px", math.inf) < gates.min_tag_px:
        return "small"
    distance, view_angle = view_quality(tag_in_camera)
    if distance > gates.max_distance_m:
        return "far"
    if view_angle > gates.max_view_angle_deg:
        return "oblique"
    if tags.get("lin_speed", 0.0) > gates.max_linear_speed_mps:
        return "fast"
    if tags.get("ang_speed", 0.0) > gates.max_angular_speed_dps:
        return "spinning"
    return None


def cluster_by_time(detections: list[TagDetection], gap_sec: float) -> list[list[TagDetection]]:
    """Same-marker detections grouped; a new cluster starts after a gap over ``gap_sec``."""
    clusters: list[list[TagDetection]] = []
    by_marker: dict[int, list[TagDetection]] = defaultdict(list)
    for detection in detections:
        by_marker[detection.marker_id].append(detection)
    for marker_detections in by_marker.values():
        marker_detections.sort(key=lambda detection: detection.ts)
        current = [marker_detections[0]]
        for detection in marker_detections[1:]:
            if detection.ts - current[-1].ts > gap_sec:
                clusters.append(current)
                current = [detection]
            else:
                current.append(detection)
        clusters.append(current)
    return clusters


def _pose_distance(a: list[float], b: list[float], rotation_weight_m_per_rad: float) -> float:
    translation = float(np.linalg.norm(np.array(a[:3]) - np.array(b[:3])))
    rotation = 2.0 * math.acos(min(1.0, abs(float(np.dot(a[3:7], b[3:7])))))
    return translation + rotation_weight_m_per_rad * rotation


def cluster_medoid(cluster: list[TagDetection], rotation_weight_m_per_rad: float) -> TagDetection:
    """The detection with the least total pose distance to the rest."""
    costs = [
        sum(_pose_distance(a.pose, b.pose, rotation_weight_m_per_rad) for b in cluster)
        for a in cluster
    ]
    return cluster[int(np.argmin(costs))]


def _huber_weights(residuals: np.ndarray, delta: float) -> np.ndarray:
    """IRLS Huber weights: 1 inside ``delta``, decaying as delta/r past it."""
    weights = np.ones_like(residuals)
    outside = residuals > delta
    weights[outside] = delta / residuals[outside]
    return weights


def robust_cluster_pose(cluster: list[TagDetection], clustering: TagClustering) -> TagDetection:
    """The medoid refined by Huber-weighted IRLS: weighted-mean translation and weighted
    quaternion mean (Markley eigen method), re-weighted each iteration so an outlier keeps
    losing influence."""
    medoid = cluster_medoid(cluster, clustering.rotation_weight_m_per_rad)
    if len(cluster) < 2:
        return medoid
    poses = np.array([detection.pose for detection in cluster], dtype=np.float64)
    translations, quaternions = poses[:, :3], poses[:, 3:7]
    reference = np.array(medoid.pose[3:7])
    signs = np.sign(quaternions @ reference)
    signs[signs == 0] = 1.0
    quaternions = quaternions * signs[:, None]
    estimate_translation = np.array(medoid.pose[:3])
    estimate_quaternion = reference.copy()
    delta_rad = clustering.huber_delta_m / max(clustering.rotation_weight_m_per_rad, 1e-9)
    for _ in range(_HUBER_ITERATIONS):
        weights_t = _huber_weights(
            np.linalg.norm(translations - estimate_translation, axis=1), clustering.huber_delta_m
        )
        estimate_translation = (weights_t[:, None] * translations).sum(0) / weights_t.sum()
        angular_residual = 2.0 * np.arccos(
            np.clip(np.abs(quaternions @ estimate_quaternion), 0.0, 1.0)
        )
        weights_r = _huber_weights(angular_residual, delta_rad)
        scatter = (
            weights_r[:, None, None] * np.einsum("ni,nj->nij", quaternions, quaternions)
        ).sum(0)
        estimate_quaternion = np.linalg.eigh(scatter)[1][:, -1]
        if estimate_quaternion @ reference < 0:
            estimate_quaternion = -estimate_quaternion
    return TagDetection(
        ts=medoid.ts,
        marker_id=medoid.marker_id,
        pose=[*estimate_translation.tolist(), *estimate_quaternion.tolist()],
    )


def cluster_tags(
    detections: list[TagDetection], clustering: TagClustering
) -> dict[int, TagDetection]:
    """One robust pose per tag: per-visit clusters first, then across its visits."""
    by_marker: dict[int, list[TagDetection]] = defaultdict(list)
    for cluster in cluster_by_time(detections, clustering.gap_sec):
        if len(cluster) >= clustering.min_observations:
            by_marker[cluster[0].marker_id].append(robust_cluster_pose(cluster, clustering))
    return {
        marker_id: robust_cluster_pose(visits, clustering)
        for marker_id, visits in sorted(by_marker.items())
    }


def as_pose(detection: TagDetection, frame_id: str) -> PoseStamped:
    return PoseStamped(
        ts=detection.ts,
        frame_id=frame_id,
        position=detection.pose[:3],
        orientation=detection.pose[3:7],
    )


def pose_list(pose: Any) -> list[float]:
    """xyz + xyzw of a Pose/PoseStamped/Transform."""
    position = pose.position if hasattr(pose, "position") else pose.translation
    orientation = pose.orientation if hasattr(pose, "orientation") else pose.rotation
    return [
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]


def tag_frame(marker_id: int, prefix: str) -> str:
    return f"{prefix}{marker_id}"


def recording_tags(
    path: str,
    streams: tuple[str, ...],
    map_frame: str,
    gates: TagGates,
    clustering: TagClustering,
    tf_tolerance: float = 0.2,
) -> dict[int, TagDetection]:
    """The clustered tags of a ``.db``/``.mcap``, in its map cloud's frame (*map_frame*).

    Reads the first of *streams* the recording has: glimpses as ``PoseStamped`` with
    ``marker_id`` (and the detector's quality fields, gated on) in the observation tags,
    or a marker detector's ``Detection3DArray`` (id = marker id). Each is placed in
    *map_frame* through the recording's own tf at its time.
    """
    from dimos.mapping.relocalization.module import find_stream
    from dimos.memory.cli.dataset import open_store
    from dimos.protocol.tf.tf import MultiTBuffer

    glimpses: list[tuple[float, int, dict[str, Any], PoseStamped]] = []
    with open_store(path) as store:
        names = store.list_streams()
        name = next((found for s in streams if (found := find_stream(names, s))), None)
        if name is None:
            logger.info(f"{path} has none of {streams}, no prior AprilTags")
            return {}
        # payloads load lazily, so read them while the store is open
        for obs in store.stream(name).order_by("ts"):
            if isinstance(obs.data, Detection3DArray):
                glimpses += [
                    (float(obs.ts), marker_id, {}, pose)
                    for marker_id, pose in marker_poses(obs.data)
                ]
            else:
                tags = dict(obs.tags or {})
                glimpses.append((float(obs.ts), int(tags["marker_id"]), tags, obs.data))
        tf = MultiTBuffer(buffer_size=math.inf)
        tf_name = find_stream(names, "tf")
        if tf_name is not None:
            for message in store.stream(tf_name):
                tf.receive_transform(*message.data.transforms)

    rejected: dict[str, int] = defaultdict(int)
    detections: list[TagDetection] = []
    for ts, marker_id, tags, pose in glimpses:
        tag_in_camera = pose_list(pose)
        why = rejection(tags, tag_in_camera, gates) if tags else None
        if why is not None:
            rejected[why] += 1
            continue
        if pose.frame_id == map_frame:
            placed = tag_in_camera
        else:
            map_to_camera = tf.get(map_frame, pose.frame_id, ts, tf_tolerance)
            if map_to_camera is None:
                rejected["no tf"] += 1
                continue
            placed = pose_list(map_to_camera + Transform.from_pose("tag", pose))
        detections.append(TagDetection(ts, marker_id, placed))
    clustered = cluster_tags(detections, clustering)
    logger.info(
        f"AprilTags from {path}:{name}: {len(glimpses)} glimpses, kept {len(detections)}, "
        f"rejected {dict(rejected)}, {len(clustered)} tags {sorted(clustered)}"
    )
    return clustered


def tags_message(tags: dict[int, TagDetection], frame_id: str, ts: float) -> Detection3DArray:
    """Clustered tags as one ``Detection3DArray``: id = marker id, bbox.center = its pose."""
    detections = []
    for marker_id, tag in tags.items():
        detection = Detection3D()
        detection.header = Header(tag.ts, frame_id)
        detection.id = str(marker_id)
        detection.results = [
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(class_id=str(marker_id), score=1.0)
            )
        ]
        detection.results_length = 1
        pose = as_pose(tag, frame_id)
        detection.bbox = BoundingBox3D(
            center=Pose(position=pose.position, orientation=pose.orientation)
        )
        detections.append(detection)
    return Detection3DArray(
        detections_length=len(detections), header=Header(ts, frame_id), detections=detections
    )


def marker_poses(message: Detection3DArray) -> list[tuple[int, PoseStamped]]:
    """(marker id, pose) of each detection, as a marker detector publishes them."""
    out = []
    for detection in message.detections[: message.detections_length]:
        center = detection.bbox.center
        frame_id = str(detection.header.frame_id) or message.frame_id
        out.append(
            (
                int(detection.id),
                PoseStamped(
                    ts=message.ts,
                    frame_id=frame_id,
                    position=[center.position.x, center.position.y, center.position.z],
                    orientation=[
                        center.orientation.x,
                        center.orientation.y,
                        center.orientation.z,
                        center.orientation.w,
                    ],
                ),
            )
        )
    return out
