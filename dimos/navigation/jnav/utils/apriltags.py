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

"""AprilTag detection over a mem2.db color stream.

Detects tags in the `color_image` stream and rejects bad glimpses through several
independent gates — motion blur (per-tag sharpness), PnP misfit (reprojection
error), too-small / too-far / too-oblique views, and fast camera motion — then
clusters same-id detections in time and reduces each cluster to one robust pose
via a Huber-weighted refinement seeded at the cluster medoid. (Re)writes the
`april_tags` PoseStamped stream of tag-in-camera relative poses (solvePnP,
marker_id in each observation's tags) and returns those representatives for the
GTSAM solve.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass
from itertools import pairwise
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.fiducial.marker_pose import (
    create_aruco_detector,
    estimate_marker_pose,
    marker_reprojection_error,
)

DEFAULT_MAX_DISTANCE_M = 1.0
DEFAULT_MAX_VIEW_ANGLE_DEG = 45.0
DEFAULT_CLUSTER_GAP_SEC = 5.0
DEFAULT_ROTATION_WEIGHT_M_PER_RAD = 0.5
# A blurry tag still solves a pose; these reject the bad glimpses up front.
DEFAULT_MIN_SHARPNESS = 60.0  # Laplacian variance over the tag ROI
DEFAULT_MAX_REPROJ_PX = 2.0  # RMS solvePnP corner reprojection error
DEFAULT_MIN_TAG_PX = 24.0  # tag side length in pixels (sqrt of quad area)
DEFAULT_MAX_LINEAR_SPEED_MPS = 0.5
DEFAULT_MAX_ANGULAR_SPEED_DPS = 50.0
DEFAULT_MIN_OBSERVATIONS = 3  # clusters thinner than this are unreliable
DEFAULT_HUBER_DELTA_M = 0.05  # residual past which a sample is down-weighted
_HUBER_ITERATIONS = 5

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


def cluster_by_time(detections: list[Detection], gap_sec: float) -> list[list[Detection]]:
    """Group same-marker detections into clusters. A new cluster begins whenever
    the time gap to the previous same-marker detection exceeds gap_sec."""
    clusters: list[list[Detection]] = []
    by_marker: dict[int, list[Detection]] = defaultdict(list)
    for detection in detections:
        by_marker[detection["marker_id"]].append(detection)
    for marker_detections in by_marker.values():
        marker_detections.sort(key=lambda detection: detection["ts"])
        current = [marker_detections[0]]
        for detection in marker_detections[1:]:
            if detection["ts"] - current[-1]["ts"] > gap_sec:
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


def cluster_medoid(cluster: list[Detection], rotation_weight_m_per_rad: float) -> Detection:
    """The detection whose pose is most central (min total spatial+rotational
    distance to the rest) — a robust representative of the cluster."""
    poses = [detection["t_cam_marker"] for detection in cluster]
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


def tag_pixel_size(corners_pixels: np.ndarray) -> float:
    """Tag side length in pixels (sqrt of the quad's image area); small = unreliable."""
    import cv2

    quad = corners_pixels.reshape(4, 2).astype(np.float32)
    return float(math.sqrt(abs(cv2.contourArea(quad))))


def tag_sharpness(gray: np.ndarray, corners_pixels: np.ndarray) -> float:
    """Laplacian variance over the tag's bounding box — low under motion blur."""
    import cv2

    quad = corners_pixels.reshape(4, 2)
    x_min, y_min = np.floor(quad.min(0)).astype(int)
    x_max, y_max = np.ceil(quad.max(0)).astype(int)
    height, width = gray.shape[:2]
    x_min, y_min = max(int(x_min), 0), max(int(y_min), 0)
    x_max, y_max = min(int(x_max), width), min(int(y_max), height)
    if x_max - x_min < 2 or y_max - y_min < 2:
        return 0.0
    return float(cv2.Laplacian(gray[y_min:y_max, x_min:x_max], cv2.CV_64F).var())


def _pose_xyz_quat(pose: Any) -> np.ndarray | None:
    """Best-effort (x,y,z,qx,qy,qz,qw) from an observation pose (tuple/list or a msg
    with .position/.orientation); None if it isn't a usable pose."""
    if pose is None:
        return None
    if hasattr(pose, "position") and hasattr(pose, "orientation"):
        position, orientation = pose.position, pose.orientation
        return np.array(
            [
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )
    try:
        values = [float(component) for component in pose]
    except TypeError:
        return None
    return np.array(values[:7]) if len(values) >= 7 else None


def _camera_speeds(images: list[Any]) -> tuple[dict[float, tuple[float, float]], bool]:
    """Per-image (linear m/s, angular deg/s) from consecutive posed frames. The second
    return is False when too few frames carry poses to estimate motion (gate disabled)."""
    posed = [
        (float(obs.ts), pose)
        for obs in images
        if (pose := _pose_xyz_quat(getattr(obs, "pose", None))) is not None
    ]
    posed.sort(key=lambda item: item[0])
    speeds: dict[float, tuple[float, float]] = {}
    for (timestamp_a, pose_a), (timestamp_b, pose_b) in pairwise(posed):
        dt = timestamp_b - timestamp_a
        if dt <= 0:
            continue
        linear = float(np.linalg.norm(pose_b[:3] - pose_a[:3])) / dt
        cos_half = min(1.0, abs(float(np.dot(pose_a[3:7], pose_b[3:7]))))
        angular = math.degrees(2.0 * math.acos(cos_half)) / dt
        speeds[timestamp_b] = (linear, angular)
    return speeds, len(posed) >= 2


def _huber_weights(residuals: np.ndarray, delta: float) -> np.ndarray:
    """IRLS Huber weights: 1 inside `delta`, decaying as delta/r past it."""
    weights = np.ones_like(residuals)
    outside = residuals > delta
    weights[outside] = delta / residuals[outside]
    return weights


def robust_cluster_pose(
    cluster: list[Detection], rotation_weight_m_per_rad: float, huber_delta_m: float
) -> Detection:
    """Cluster representative: the medoid, then refined by Huber-weighted IRLS — a
    weighted-mean translation and weighted quaternion mean (Markley eigen method),
    re-weighting each iteration so a lingering bad glimpse keeps losing influence."""
    medoid = cluster_medoid(cluster, rotation_weight_m_per_rad)
    if len(cluster) < 2:
        return medoid
    poses = np.array([detection["t_cam_marker"] for detection in cluster], dtype=np.float64)
    translations, quaternions = poses[:, :3], poses[:, 3:7]
    reference = np.array(medoid["t_cam_marker"][3:7])
    signs = np.sign(quaternions @ reference)
    signs[signs == 0] = 1.0
    quaternions = quaternions * signs[:, None]
    estimate_translation = np.array(medoid["t_cam_marker"][:3])
    estimate_quaternion = reference.copy()
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
        estimate_quaternion = np.linalg.eigh(scatter)[1][:, -1]
        if estimate_quaternion @ reference < 0:
            estimate_quaternion = -estimate_quaternion
    return {
        **medoid,
        "t_cam_marker": [*estimate_translation.tolist(), *estimate_quaternion.tolist()],
    }


# Streaming / incremental API

_DEFAULT_MARKER_LENGTH = 0.10
_DEFAULT_DICTIONARY = "DICT_APRILTAG_36h11"


def detect_raw_detections(
    store: Any,
    intrinsics: np.ndarray,
    distortion: np.ndarray,
    *,
    image_stream: str = "color_image",
    marker_length: float = 0.10,
    dictionary: str = "DICT_APRILTAG_36h11",
) -> tuple[list[Detection], bool, int]:
    """Every valid-PnP tag detection over `image_stream`, unfiltered, with gate diagnostics."""
    import cv2

    detector = create_aruco_detector(dictionary)
    raw_detections: list[Detection] = []
    images = store.stream(image_stream, Image).to_list()
    speed_by_ts, speed_available = _camera_speeds(images)
    for image_obs in images:
        image = image_obs.data
        bgr = image.numpy() if hasattr(image, "numpy") else np.asarray(image.data)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        all_corners, marker_ids, _ = detector.detectMarkers(bgr)
        if marker_ids is None:
            continue
        for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=False):
            pose = estimate_marker_pose(corners, marker_length, intrinsics, distortion)
            if pose is None:
                continue
            rotation_vector, translation_vector = pose
            quaternion = Rotation.from_rotvec(rotation_vector.reshape(3)).as_quat()  # x,y,z,w
            translation = translation_vector.reshape(3)
            tag_in_camera = [
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ]
            raw_detections.append(
                {
                    "ts": float(image_obs.ts),
                    "marker_id": int(marker_id),
                    "t_cam_marker": tag_in_camera,
                    "sharpness": tag_sharpness(gray, corners),
                    "reproj_px": marker_reprojection_error(
                        corners,
                        marker_length,
                        intrinsics,
                        distortion,
                        rotation_vector,
                        translation_vector,
                    ),
                    "tag_px": tag_pixel_size(corners),
                    "speed": speed_by_ts.get(float(image_obs.ts)),
                }
            )
    return raw_detections, speed_available, len(images)


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
    """None when a glimpse passes every per-glimpse quality gate, else the rejection reason."""
    if detection["sharpness"] < min_sharpness:
        return "blur"
    if detection["reproj_px"] > max_reproj_px:
        return "reproj"
    if detection["tag_px"] < min_tag_px:
        return "small"
    distance, view_angle = view_quality(detection["t_cam_marker"])
    if distance > max_distance_m:
        return "far"
    if view_angle > max_view_angle_deg:
        return "oblique"
    speed = detection["speed"]
    if speed is not None and (speed[0] > max_linear_speed_mps or speed[1] > max_angular_speed_dps):
        return "motion"
    return None


def filter_glimpses(
    raw_detections: list[Detection],
    *,
    exclude_tags: Iterable[int] = (),
    **thresholds: float,
) -> list[Detection]:
    """Per-glimpse gating with NO clustering: every clean sighting survives, annotated
    with distance_m / view_angle_deg for downstream quality weighting."""
    excluded = set(exclude_tags)
    kept: list[Detection] = []
    for detection in raw_detections:
        if detection["marker_id"] in excluded:
            continue
        if glimpse_passes(detection, **thresholds) is not None:
            continue
        distance_m, view_angle_deg = view_quality(detection["t_cam_marker"])
        kept.append({**detection, "distance_m": distance_m, "view_angle_deg": view_angle_deg})
    return kept


def gate_detections(
    raw_detections: list[Detection],
    *,
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
    max_view_angle_deg: float = DEFAULT_MAX_VIEW_ANGLE_DEG,
    cluster_gap_sec: float = DEFAULT_CLUSTER_GAP_SEC,
    rotation_weight_m_per_rad: float = DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
    min_sharpness: float = DEFAULT_MIN_SHARPNESS,
    max_reproj_px: float = DEFAULT_MAX_REPROJ_PX,
    min_tag_px: float = DEFAULT_MIN_TAG_PX,
    max_linear_speed_mps: float = DEFAULT_MAX_LINEAR_SPEED_MPS,
    max_angular_speed_dps: float = DEFAULT_MAX_ANGULAR_SPEED_DPS,
    min_observations: int = DEFAULT_MIN_OBSERVATIONS,
    huber_delta_m: float = DEFAULT_HUBER_DELTA_M,
) -> tuple[list[Detection], dict[str, int], int]:
    """Gate + time-cluster raw detections into one Huber-refined medoid per cluster."""
    rejected: dict[str, int] = defaultdict(int)
    kept: list[Detection] = []
    for detection in raw_detections:
        reason = glimpse_passes(
            detection,
            max_distance_m=max_distance_m,
            max_view_angle_deg=max_view_angle_deg,
            min_sharpness=min_sharpness,
            max_reproj_px=max_reproj_px,
            min_tag_px=min_tag_px,
            max_linear_speed_mps=max_linear_speed_mps,
            max_angular_speed_dps=max_angular_speed_dps,
        )
        if reason is not None:
            rejected[reason] += 1
            continue
        kept.append(detection)

    detections: list[Detection] = []
    thin_clusters = 0
    for cluster in cluster_by_time(kept, cluster_gap_sec):
        if len(cluster) < min_observations:
            thin_clusters += 1
            continue
        detections.append(
            {
                **robust_cluster_pose(cluster, rotation_weight_m_per_rad, huber_delta_m),
                "n_observations": len(cluster),
            }
        )
    detections.sort(key=lambda detection: detection["ts"])
    return detections, dict(rejected), thin_clusters


def _write_tag_stream(
    store: Any, stream_name: str, detections: list[Detection], *, diagnostics: bool
) -> None:
    """(Re)write a PoseStamped tag stream, with gate diagnostics when `diagnostics`."""
    if stream_name in store.list_streams():
        store.delete_stream(stream_name)
    stream = store.stream(stream_name, PoseStamped)
    for detection in detections:
        pose = detection["t_cam_marker"]
        tags: dict[str, Any] = {"marker_id": detection["marker_id"]}
        if diagnostics:
            speed = detection.get("speed")
            distance, view_angle = view_quality(pose)
            tags.update(
                {
                    "sharpness": float(detection["sharpness"]),
                    "reproj_px": float(detection["reproj_px"]),
                    "tag_px": float(detection["tag_px"]),
                    "distance_m": float(distance),
                    "view_angle_deg": float(view_angle),
                    "lin_speed": float(speed[0]) if speed else -1.0,
                    "ang_speed": float(speed[1]) if speed else -1.0,
                }
            )
        stream.append(
            PoseStamped(ts=detection["ts"], position=pose[:3], orientation=pose[3:]),
            ts=detection["ts"],
            pose=tuple(pose),
            tags=tags,
        )


def read_raw_tag_stream(store: Any, stream_name: str) -> list[Detection]:
    """Read a persisted raw tag stream back into ``detect_raw_detections``-shaped dicts."""
    detections: list[Detection] = []
    for observation in store.stream(stream_name):
        pose = observation.data
        tags = observation.tags
        lin_speed = float(tags.get("lin_speed", -1.0))
        ang_speed = float(tags.get("ang_speed", -1.0))
        detections.append(
            {
                "ts": float(observation.ts),
                "marker_id": int(tags["marker_id"]),
                "t_cam_marker": [
                    pose.x,
                    pose.y,
                    pose.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                "sharpness": float(tags["sharpness"]),
                "reproj_px": float(tags["reproj_px"]),
                "tag_px": float(tags["tag_px"]),
                "speed": None if lin_speed < 0 else (lin_speed, ang_speed),
            }
        )
    return detections


def ensure_raw_tag_stream(
    store: Any,
    camera_matrix: np.ndarray | None,
    distortion: np.ndarray | None,
    *,
    raw_stream: str = "raw_april_tags",
    image_stream: str = "color_image",
    marker_length: float = 0.10,
    dictionary: str = "DICT_APRILTAG_36h11",
) -> bool:
    """Detect + persist the unfiltered tag stream when absent; returns whether it exists.

    ``camera_matrix``/``distortion`` come from the recording's CameraInfo stream (or None
    when it has no camera, in which case nothing can be detected)."""
    if raw_stream in store.list_streams():
        return True
    if camera_matrix is None or distortion is None or image_stream not in store.list_streams():
        return False
    print(
        f"{raw_stream} missing -- detecting AprilTags over {image_stream} "
        f"(tag_size={marker_length} m, dict={dictionary})...",
        flush=True,
    )
    raw_detections, _, n_images = detect_raw_detections(
        store,
        camera_matrix,
        distortion,
        image_stream=image_stream,
        marker_length=marker_length,
        dictionary=dictionary,
    )
    _write_tag_stream(store, raw_stream, raw_detections, diagnostics=True)
    print(
        f"wrote {raw_stream}: {len(raw_detections)} detections over {n_images} images", flush=True
    )
    return True


def write_april_streams(
    store: Any,
    raw_detections: list[Detection],
    filtered_detections: list[Detection],
    *,
    raw_stream: str = "raw_april_tags",
    filtered_stream: str = "april_tags",
) -> None:
    """Persist the unfiltered `raw_april_tags` and the gated `april_tags` streams."""
    _write_tag_stream(store, raw_stream, raw_detections, diagnostics=True)
    _write_tag_stream(store, filtered_stream, filtered_detections, diagnostics=False)


def _print_tag_summary(
    stream_name: str,
    n_raw: int,
    detections: list[Detection],
    rejected: dict[str, int],
    thin_clusters: int,
    speed_available: bool,
    n_images: int,
) -> None:
    in_spec = n_raw - sum(rejected.values())
    found_ids = sorted({detection["marker_id"] for detection in detections})
    gate_summary = ", ".join(f"{reason}={count}" for reason, count in sorted(rejected.items()))
    if not speed_available:
        gate_summary += (", " if gate_summary else "") + "motion-gate-off(no poses)"
    print(
        f"   {stream_name}: {n_raw} raw -> {in_spec} in-spec "
        f"-> {len(detections)} clusters (dropped {thin_clusters} thin), "
        f"markers {found_ids} (over {n_images} images)"
    )
    if gate_summary:
        print(f"   {stream_name} rejected: {gate_summary}")


def gate_params() -> dict[str, Any]:
    """The current AprilTag gate thresholds, for recording in a sidecar."""
    return {
        "max_distance_m": DEFAULT_MAX_DISTANCE_M,
        "max_view_angle_deg": DEFAULT_MAX_VIEW_ANGLE_DEG,
        "min_sharpness": DEFAULT_MIN_SHARPNESS,
        "max_reproj_px": DEFAULT_MAX_REPROJ_PX,
        "min_tag_px": DEFAULT_MIN_TAG_PX,
        "max_linear_speed_mps": DEFAULT_MAX_LINEAR_SPEED_MPS,
        "max_angular_speed_dps": DEFAULT_MAX_ANGULAR_SPEED_DPS,
        "min_observations": DEFAULT_MIN_OBSERVATIONS,
        "cluster_gap_sec": DEFAULT_CLUSTER_GAP_SEC,
        "huber_delta_m": DEFAULT_HUBER_DELTA_M,
        "rotation_weight_m_per_rad": DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
    }


def ensure_april_streams(
    store: Any,
    intrinsics: np.ndarray,
    distortion: np.ndarray,
    *,
    image_stream: str = "color_image",
    marker_length: float = 0.10,
    dictionary: str = "DICT_APRILTAG_36h11",
    raw_stream: str = "raw_april_tags",
    filtered_stream: str = "april_tags",
    exclude_tags: Iterable[int] = (),
    force: bool = False,
) -> list[Detection]:
    """Ensure both tag streams exist (`raw_april_tags`, `april_tags`); force=True rewrites.

    `exclude_tags` are kept in raw but dropped from filtered. Returns the filtered reps."""
    existing = set(store.list_streams())
    if not force and raw_stream in existing and filtered_stream in existing:
        return []
    raw_detections, speed_available, n_images = detect_raw_detections(
        store,
        intrinsics,
        distortion,
        image_stream=image_stream,
        marker_length=marker_length,
        dictionary=dictionary,
    )
    detections, rejected, thin_clusters = gate_detections(raw_detections)
    excluded = set(exclude_tags)
    if excluded:
        detections = [d for d in detections if d["marker_id"] not in excluded]
    write_april_streams(
        store, raw_detections, detections, raw_stream=raw_stream, filtered_stream=filtered_stream
    )
    print(f"   {raw_stream}: {len(raw_detections)} detections (unfiltered)")
    if excluded:
        print(f"   excluded dynamic tags from {filtered_stream}: {sorted(excluded)}")
    _print_tag_summary(
        filtered_stream,
        len(raw_detections),
        detections,
        rejected,
        thin_clusters,
        speed_available,
        n_images,
    )
    return detections


# April-tag agreement: a drift-quality metric for a corrected trajectory.
#
# A fixed April tag observed many times along a trajectory should map to ONE
# world position. Odometry drift scatters those estimates (the same tag lands in
# a different spot each lap); a good loop-closure / PGO correction pulls them
# back together. So the spread of a tag's repeated world-position estimates is a
# ground-truth-free measure of trajectory consistency — and the drop in spread
# from raw odometry to PGO-corrected odometry is how much PGO helped.

# sightings further apart than this are treated as separate visits
VISIT_GAP_S = 20.0


@dataclass(frozen=True)
class TagAgreement:
    """Per-tag spread of repeated world-position estimates (metres)."""

    tag_id: int
    observations: int
    spread: float  # RMS distance of estimates to their centroid


@dataclass(frozen=True)
class AgreementReport:
    """Agreement across all multiply-observed tags."""

    per_tag: tuple[TagAgreement, ...]
    mean_spread: float  # mean per-tag spread (metres); lower = better agreement
    total_observations: int

    @property
    def tag_count(self) -> int:
        return len(self.per_tag)


def tag_spread(positions: np.ndarray) -> float:
    """RMS distance of a tag's world-position estimates to their centroid."""
    if len(positions) < 2:
        return 0.0
    centroid = positions.mean(axis=0)
    return float(np.sqrt(np.mean(np.sum((positions - centroid) ** 2, axis=1))))


def agreement_report(
    tag_positions: dict[int, np.ndarray], *, min_observations: int = 2
) -> AgreementReport:
    """Score per-tag agreement from ``tag_id -> (N, 3) world positions``.

    Tags seen fewer than ``min_observations`` times carry no agreement signal and
    are excluded from ``mean_spread``.
    """
    per_tag: list[TagAgreement] = []
    total = 0
    for tag_id in sorted(tag_positions):
        positions = np.asarray(tag_positions[tag_id], dtype=np.float64).reshape(-1, 3)
        total += len(positions)
        if len(positions) < min_observations:
            continue
        per_tag.append(TagAgreement(tag_id, len(positions), tag_spread(positions)))
    mean_spread = float(np.mean([t.spread for t in per_tag])) if per_tag else 0.0
    return AgreementReport(tuple(per_tag), mean_spread, total)


def agreement_improvement(raw: AgreementReport, corrected: AgreementReport) -> float:
    """Fractional drop in mean spread from ``raw`` to ``corrected`` (1.0 = perfect).

    Positive means the correction tightened tag agreement; negative means it made
    it worse. Returns 0.0 if there's no raw spread to improve on.
    """
    if raw.mean_spread <= 0.0:
        return 0.0
    return (raw.mean_spread - corrected.mean_spread) / raw.mean_spread


def split_visits(times: list[float], *, gap_s: float) -> list[list[float]]:
    """Cluster sighting timestamps into visits separated by gaps > ``gap_s``."""
    visits: list[list[float]] = []
    for timestamp in sorted(times):
        if visits and timestamp - visits[-1][-1] <= gap_s:
            visits[-1].append(timestamp)
        else:
            visits.append([timestamp])
    return visits
