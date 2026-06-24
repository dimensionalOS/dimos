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
from collections.abc import Callable, Iterable
from copy import copy
from dataclasses import dataclass, field
from itertools import pairwise
import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image

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


def make_detector(dictionary_name: str) -> cv2.aruco.ArucoDetector:
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def _object_points(marker_length_m: float) -> np.ndarray:
    h = marker_length_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)


def estimate_marker_pose(
    corners_pixels: np.ndarray,
    marker_length_m: float,
    intrinsics: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    """solvePnP a single tag -> (rotation_vector, translation_vector) in the
    camera_optical frame, or None if it failed."""
    image_corners = corners_pixels.reshape(4, 1, 2).astype(np.float32)
    found, rotation_vector, translation_vector = cv2.solvePnP(
        _object_points(marker_length_m),
        image_corners,
        intrinsics,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return (rotation_vector, translation_vector) if found else None


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


def reprojection_error_px(
    corners_pixels: np.ndarray,
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
    marker_length_m: float,
    intrinsics: np.ndarray,
    distortion: np.ndarray,
) -> float:
    """RMS pixel distance between detected corners and the solvePnP pose reprojected
    back onto the image — a direct measure of how well the pose explains the tag."""
    projected, _ = cv2.projectPoints(
        _object_points(marker_length_m), rotation_vector, translation_vector, intrinsics, distortion
    )
    measured = corners_pixels.reshape(4, 2).astype(np.float64)
    diff = projected.reshape(4, 2) - measured
    return float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))


def tag_pixel_size(corners_pixels: np.ndarray) -> float:
    """Tag side length in pixels (sqrt of the quad's image area); small = unreliable."""
    quad = corners_pixels.reshape(4, 2).astype(np.float32)
    return float(math.sqrt(abs(cv2.contourArea(quad))))


def tag_sharpness(gray: np.ndarray, corners_pixels: np.ndarray) -> float:
    """Laplacian variance over the tag's bounding box — low under motion blur."""
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


def detect_apriltags(
    store: Any,
    intrinsics: np.ndarray,
    distortion: np.ndarray,
    image_stream: str = "color_image",
    stream_name: str = "april_tags",
    marker_length: float = 0.10,
    dictionary: str = "DICT_APRILTAG_36h11",
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
) -> list[Detection]:
    """Detect tags in `image_stream`, reject bad glimpses (blur, PnP misfit, small/
    far/oblique views, fast motion), cluster same-id detections by time, drop thin
    clusters, and (re)write the `april_tags` stream from one Huber-refined medoid
    representative per cluster. Returns that list of representatives."""
    detector = make_detector(dictionary)
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
                    "reproj_px": reprojection_error_px(
                        corners,
                        rotation_vector,
                        translation_vector,
                        marker_length,
                        intrinsics,
                        distortion,
                    ),
                    "tag_px": tag_pixel_size(corners),
                    "speed": speed_by_ts.get(float(image_obs.ts)),
                }
            )

    # Per-glimpse gates; count rejections per reason so thresholds are tunable.
    rejected: dict[str, int] = defaultdict(int)
    kept: list[Detection] = []
    for detection in raw_detections:
        if detection["sharpness"] < min_sharpness:
            rejected["blur"] += 1
            continue
        if detection["reproj_px"] > max_reproj_px:
            rejected["reproj"] += 1
            continue
        if detection["tag_px"] < min_tag_px:
            rejected["small"] += 1
            continue
        distance, view_angle = view_quality(detection["t_cam_marker"])
        if distance > max_distance_m:
            rejected["far"] += 1
            continue
        if view_angle > max_view_angle_deg:
            rejected["oblique"] += 1
            continue
        speed = detection["speed"]
        if speed is not None and (
            speed[0] > max_linear_speed_mps or speed[1] > max_angular_speed_dps
        ):
            rejected["motion"] += 1
            continue
        kept.append(detection)

    # One Huber-refined representative per time-clustered group; drop thin clusters.
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

    if stream_name in store.list_streams():
        store.delete_stream(stream_name)
    april_tag_stream = store.stream(stream_name, PoseStamped)
    for detection in detections:
        tag_in_camera = detection["t_cam_marker"]
        pose_stamped = PoseStamped(
            ts=detection["ts"], position=tag_in_camera[:3], orientation=tag_in_camera[3:]
        )
        april_tag_stream.append(
            pose_stamped,
            ts=detection["ts"],
            pose=tuple(tag_in_camera),
            tags={"marker_id": detection["marker_id"]},
        )

    found_ids = sorted({detection["marker_id"] for detection in detections})
    gate_summary = ", ".join(f"{reason}={count}" for reason, count in sorted(rejected.items()))
    if not speed_available:
        gate_summary += (", " if gate_summary else "") + "motion-gate-off(no poses)"
    print(
        f"   april_tags: {len(raw_detections)} raw -> {len(kept)} in-spec "
        f"-> {len(detections)} clusters (dropped {thin_clusters} thin), "
        f"markers {found_ids} (over {len(images)} images)"
    )
    if gate_summary:
        print(f"   april_tags rejected: {gate_summary}")
    return detections


# Streaming / incremental API

_DEFAULT_MARKER_LENGTH = 0.10
_DEFAULT_DICTIONARY = "DICT_APRILTAG_36h11"


@dataclass
class TagInfo:
    tag_number: float
    confidence: float
    pose: list[float]  # [x, y, z, qx, qy, qz, qw] relative to camera
    ts: float
    camera_frame_id: str = ""


@dataclass
class GroupingOptions:
    tag_ids_to_ignore: list[float] = field(default_factory=list)
    time_window: float = DEFAULT_CLUSTER_GAP_SEC
    max_distance: float = DEFAULT_MAX_DISTANCE_M
    min_confidence: float = 0.0


class AprilTagger:
    def __init__(
        self,
        camera_intrinsics: Any,
        grouping_options: Any,
        tags_ids_to_ignore: list[float],
    ) -> None:
        self.camera_intrinsics = camera_intrinsics
        self.grouping_options = grouping_options
        self.tags_ids_to_ignore = tags_ids_to_ignore

    def get_tag(self, img: Any) -> list[TagInfo]:
        return pure_get_tags(img, self.camera_intrinsics)

    def iter_april_tag_groups(
        self,
        next_observation: Any,
        options: Any = None,
        state: dict[str, Any] | None = None,
    ) -> tuple[list[TagInfo], dict[float, TagInfo], dict[float, TagInfo]]:
        if state is None:
            state = {}
        if options is None:
            options = self.grouping_options
        recent_tags, tag_estimates, finished_tags = pure_iter_april_tag_groups(
            next_observation,
            self.camera_intrinsics,
            options,
            state.get("recent_tags", []),
            state.get("tag_estimates", {}),
        )
        state["recent_tags"] = recent_tags
        state["tag_estimates"] = tag_estimates
        return recent_tags, tag_estimates, finished_tags


def pure_get_tags(img: Any, camera_intrinsics: Any) -> list[TagInfo]:
    """Detect AprilTags in an image and return relative poses via solvePnP.

    img: ndarray or an Image msg (frame_id/ts are carried onto each TagInfo).
    camera_intrinsics: dict with "intrinsics" (3x3 ndarray), and optionally
    "distortion", "marker_length", "dictionary".
    """
    intrinsics = camera_intrinsics["intrinsics"]
    distortion = camera_intrinsics.get("distortion", np.zeros(5))
    marker_length = camera_intrinsics.get("marker_length", _DEFAULT_MARKER_LENGTH)
    dictionary = camera_intrinsics.get("dictionary", _DEFAULT_DICTIONARY)

    camera_frame_id = getattr(img, "frame_id", "")
    ts = getattr(img, "ts", None)
    if ts is None:
        ts = time.time()

    if isinstance(img, np.ndarray):
        bgr = img
    elif hasattr(img, "numpy"):
        bgr = img.numpy()
    elif hasattr(img, "data"):
        bgr = np.asarray(img.data)
    else:
        bgr = np.asarray(img)
    if bgr.ndim == 2:
        bgr = cv2.cvtColor(bgr, cv2.COLOR_GRAY2BGR)

    detector = make_detector(dictionary)
    all_corners, marker_ids, _ = detector.detectMarkers(bgr)
    if marker_ids is None:
        return []

    tags: list[TagInfo] = []
    for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=False):
        pose_result = estimate_marker_pose(corners, marker_length, intrinsics, distortion)
        if pose_result is None:
            continue
        rotation_vector, translation_vector = pose_result
        quaternion = Rotation.from_rotvec(rotation_vector.reshape(3)).as_quat()
        translation = translation_vector.reshape(3)
        pose = [
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        ]
        reproj = reprojection_error_px(
            corners,
            rotation_vector,
            translation_vector,
            marker_length,
            intrinsics,
            distortion,
        )
        confidence = max(0.0, 1.0 - reproj / DEFAULT_MAX_REPROJ_PX)
        tags.append(
            TagInfo(
                tag_number=float(marker_id),
                confidence=confidence,
                pose=pose,
                ts=float(ts),
                camera_frame_id=camera_frame_id,
            )
        )
    return tags


def pure_iter_april_tag_groups(
    next_observation: Any,
    camera_intrinsics: Any,
    options: Any,
    recent_tags: list[TagInfo],
    tag_estimates: dict[float, TagInfo],
) -> tuple[list[TagInfo], dict[float, TagInfo], dict[float, TagInfo]]:
    """Returns (recent_tags, tag_estimates, finished_tags).

    finished_tags holds estimates whose tag id aged out of the time window this
    iteration — no fresher observation can revise them, so they are final."""
    tag_estimates = copy(tag_estimates)
    recent_tags = list(recent_tags)

    tags = pure_get_tags(next_observation.color_image, camera_intrinsics)
    recent_tags.extend(tags)

    tag_ids_to_ignore = set(getattr(options, "tag_ids_to_ignore", []))
    time_window = getattr(options, "time_window", DEFAULT_CLUSTER_GAP_SEC)
    max_distance = getattr(options, "max_distance", DEFAULT_MAX_DISTANCE_M)
    min_confidence = getattr(options, "min_confidence", 0.0)

    # Purge tags outside the time window
    if recent_tags:
        latest_ts = max(tag.ts for tag in recent_tags)
        recent_tags = [tag for tag in recent_tags if latest_ts - tag.ts <= time_window]

    all_recent_tag_ids = set(
        tag.tag_number for tag in recent_tags if tag.tag_number not in tag_ids_to_ignore
    )

    # Tag ids with an estimate but no surviving observations are finalized
    finished_tags = {
        tag_id: estimate
        for tag_id, estimate in tag_estimates.items()
        if tag_id not in all_recent_tag_ids
    }
    for tag_id in finished_tags:
        del tag_estimates[tag_id]

    for each_tag_id in all_recent_tag_ids:
        tags_for_id = [tag for tag in recent_tags if tag.tag_number == each_tag_id]

        # Filter by distance and confidence
        filtered = [
            tag
            for tag in tags_for_id
            if tag.confidence >= min_confidence
            and float(np.linalg.norm(tag.pose[:3])) <= max_distance
        ]
        if not filtered:
            continue

        # Pick the medoid (most central observation by pose distance)
        if len(filtered) == 1:
            tag_estimates[each_tag_id] = filtered[0]
        else:
            best_index = 0
            best_cost = float("inf")
            for i, tag_i in enumerate(filtered):
                cost = sum(
                    _pose_distance(tag_i.pose, tag_j.pose, DEFAULT_ROTATION_WEIGHT_M_PER_RAD)
                    for j, tag_j in enumerate(filtered)
                    if j != i
                )
                if cost < best_cost:
                    best_cost = cost
                    best_index = i
            tag_estimates[each_tag_id] = filtered[best_index]

    return recent_tags, tag_estimates, finished_tags


def load_intrinsics_json(path: Path) -> dict[str, Any]:
    raw = json.loads(path.read_text())
    config: dict[str, Any] = {
        "intrinsics": np.array(raw["intrinsics"], dtype=np.float64).reshape(3, 3),
        "distortion": np.array(raw.get("distortion", []), dtype=np.float64),
    }
    for key in ("marker_length", "dictionary"):
        if key in raw:
            config[key] = raw[key]
    return config


def sightings_from_observations(
    observations: Iterable[tuple[int, float]],
) -> dict[int, list[float]]:
    """Group ``(marker_id, timestamp)`` tag observations into ``tag_id -> [timestamps]``."""
    sightings: dict[int, list[float]] = {}
    for marker_id, timestamp in observations:
        sightings.setdefault(int(marker_id), []).append(float(timestamp))
    return sightings


def load_or_detect_sightings(
    stored: Iterable[tuple[int, float]],
    detect: Callable[[], Iterable[tuple[int, float]]],
) -> tuple[dict[int, list[float]], str]:
    """``tag_id -> [sighting timestamps]`` from already-stored tag observations,
    falling back to ``detect()`` when none are stored.

    Only marker_id + ts are used — tag positions are taken relative to the odom
    stream, so stored poses are never read. The caller supplies the stored
    iterable and the detect thunk, so this stays db-free. Returns the sightings
    plus a source label (``"stream"`` or ``"detected"``)."""
    sightings = sightings_from_observations(stored)
    if sightings:
        return sightings, "stream"
    return sightings_from_observations(detect()), "detected"


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
    detector = make_detector(dictionary)
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
                    "reproj_px": reprojection_error_px(
                        corners,
                        rotation_vector,
                        translation_vector,
                        marker_length,
                        intrinsics,
                        distortion,
                    ),
                    "tag_px": tag_pixel_size(corners),
                    "speed": speed_by_ts.get(float(image_obs.ts)),
                }
            )
    return raw_detections, speed_available, len(images)


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
        if detection["sharpness"] < min_sharpness:
            rejected["blur"] += 1
            continue
        if detection["reproj_px"] > max_reproj_px:
            rejected["reproj"] += 1
            continue
        if detection["tag_px"] < min_tag_px:
            rejected["small"] += 1
            continue
        distance, view_angle = view_quality(detection["t_cam_marker"])
        if distance > max_distance_m:
            rejected["far"] += 1
            continue
        if view_angle > max_view_angle_deg:
            rejected["oblique"] += 1
            continue
        speed = detection["speed"]
        if speed is not None and (
            speed[0] > max_linear_speed_mps or speed[1] > max_angular_speed_dps
        ):
            rejected["motion"] += 1
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
