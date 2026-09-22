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

"""Query-time object localization: text prompt to latest 3D pose and cloud.

Search memory with embeddings (SigLIP,
frame-level), open-vocabulary detection (OWLv2, calibrated per-box scores),
segmentation (EdgeTAM), projection to 3D through aligned depth. Two
algorithm rules distinguish it from a best-crop search:

* **Latest-pose semantics.** Among verified observations of the chosen
  support, the greatest timestamp wins. The answer is "where is it now",
  never "where did it match best".
* **Calibrated refusal.** Every stage carries a score and the answer can be
  ``None``: no accept-level detection, no multi-view confirmation, or an
  ambiguity between coexisting candidates below the refusal margin.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.embed import EmbedImages
from dimos.memory.tf import StreamTF
from dimos.memory.transform import throttle
from dimos.perception.detection.project import sees
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.memory import gates
from dimos.perception.memory.gates import OPTICAL_FRAME, TF_TOLERANCE, WORLD_FRAME
from dimos.perception.memory.types import Localization, LocalizePolicy, Support
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.memory.stream import Stream
    from dimos.models.embedding.siglip import SigLIPModel
    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
    from dimos.protocol.tf.tf import TFLookup

logger = setup_logger()

EMBED_HZ = 1.0
TOP_FRAMES = 12
TIME_BANDS = 6  # stratify retrieval across the window so late scans always compete
BOXES_PER_FRAME = 4
CONFIRM_FLOOR = 0.22  # geometric confirmation accept for re-detections
VERIFY_FRAMES = 20


@dataclass
class _ClusterObservation:
    ts: float
    score: float
    centroid: np.ndarray
    cloud: Any
    camera_position: np.ndarray
    detection: Detection3DPC


@dataclass
class _Cluster:
    center: np.ndarray
    observations: list[_ClusterObservation] = field(default_factory=list)

    def add(self, obs: _ClusterObservation) -> None:
        self.observations.append(obs)
        self.center = np.mean(np.stack([o.centroid for o in self.observations]), axis=0)

    @property
    def max_score(self) -> float:
        return max(o.score for o in self.observations)

    @property
    def latest(self) -> _ClusterObservation:
        return max(self.observations, key=lambda o: o.ts)

    @property
    def interval(self) -> tuple[float, float]:
        times = [o.ts for o in self.observations]
        return min(times), max(times)

    @property
    def n_views(self) -> int:
        return len({tuple(np.round(o.camera_position, 2)) for o in self.observations})

    @property
    def extent(self) -> np.ndarray:
        points = np.concatenate([np.asarray(o.cloud.pointcloud.points) for o in self.observations])
        extent: np.ndarray = points.max(axis=0) - points.min(axis=0)
        return extent


@dataclass
class LocalizeTrace:
    """Intermediate artifacts collected for rendering; filled when passed in."""

    detection_frames: list[Any] = field(default_factory=list)  # Observation[ImageDetections2D]
    matched: list[tuple[float, Detection3DPC]] = field(default_factory=list)
    verified: list[tuple[float, Detection3DPC]] = field(default_factory=list)
    answer: Detection3DPC | None = None
    backdrop_ts: float | None = None


def _quaternion_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(rotation).as_quat()
    return (float(x), float(y), float(z), float(w))


def _azimuth_coverage(observations: list[_ClusterObservation], center: np.ndarray) -> float:
    directions = []
    for obs in observations:
        v = obs.camera_position - center
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            directions.append(v / norm)
    if not directions:
        return 0.0
    dirs = np.stack(directions)
    azimuth = np.arctan2(dirs[:, 1], dirs[:, 0])
    bins = set(((azimuth + math.pi) / (2 * math.pi) * 8).astype(int) % 8)
    return len(bins) / 8.0


def _axes_observed(
    observations: list[_ClusterObservation], center: np.ndarray
) -> tuple[bool, bool, bool]:
    directions = []
    for obs in observations:
        v = obs.camera_position - center
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            directions.append(v / norm)
    if not directions:
        return (False, False, False)
    dirs = np.stack(directions)
    return tuple(bool((np.abs(dirs[:, i]) > 0.3).any()) for i in range(3))  # type: ignore[return-value]


class _DetectionCache:
    """One OWLv2 + EdgeTAM pass per unique frame, shared across queries and clusters."""

    def __init__(self, detector: Any, segmenter: Any, queries: list[str], floor: float) -> None:
        self.detector = detector
        self.segmenter = segmenter
        self.queries = queries
        self.floor = floor
        self._cache: dict[float, ImageDetections2D] = {}

    def detect(self, image: Any, query: str) -> ImageDetections2D:
        key = image.ts
        cached = self._cache.get(key)
        if cached is None:
            detections: ImageDetections2D = self.detector.query_detections(
                image, self.queries, threshold=self.floor
            )
            ranked = sorted(detections.detections, key=lambda d: -d.confidence)
            cached = ImageDetections2D(
                image,
                [
                    det
                    for label in self.queries
                    for det in [d for d in ranked if d.name == label][:BOXES_PER_FRAME]
                ],
            )
            if len(cached):
                cached = self.segmenter.segment(cached)
            self._cache[key] = cached
        return cached.filter(lambda d: d.name == query)


def _lift(
    detections: ImageDetections2D,
    store: Any,
    tf: TFLookup,
    camera_info: CameraInfo,
    optical_frame: str,
    world_frame: str,
    tf_tolerance: float,
    policy: LocalizePolicy,
    plane: Any | None = None,
) -> list[tuple[Detection3DPC, np.ndarray]]:
    """Depth-lift 2D detections; returns valid (detection3d, camera_position) pairs."""
    depth = gates.depth_at(store, detections.ts)
    transform = tf.get(optical_frame, world_frame, detections.ts, tf_tolerance)
    if depth is None or transform is None:
        return []
    pose = gates.camera_pose(tf, detections.ts, optical_frame, world_frame, tf_tolerance)
    if pose is None:
        return []
    camera = np.array([pose.position.x, pose.position.y, pose.position.z])

    lifted = ImageDetections3DPC.from_depth(detections, depth, camera_info, transform)
    valid: list[tuple[Detection3DPC, np.ndarray]] = []
    for det3d in lifted:
        points = np.asarray(det3d.pointcloud.pointcloud.points)
        if len(points) < policy.min_depth_points:
            continue
        extent = points.max(axis=0) - points.min(axis=0)
        if float(extent.max()) > policy.max_object_extent_m:
            continue
        ranges = np.linalg.norm(points - camera, axis=1)
        if float(np.median(ranges)) < policy.min_camera_range_m:
            continue
        if plane is not None:
            heights = plane.height_above(points)
            low = float(np.quantile(heights, 0.05))
            high = float(np.quantile(heights, 0.95))
            if low > policy.surface_patch_min_drop_m and high < policy.surface_patch_max_rise_m:
                continue
        valid.append((det3d, camera))
    return valid


def embed_index(
    store: Any,
    siglip: SigLIPModel,
    t0: float,
    t1: float,
    *,
    optical_frame: str = OPTICAL_FRAME,
    world_frame: str = WORLD_FRAME,
    tf_tolerance: float = TF_TOLERANCE,
) -> Stream[Any, Any]:
    """SigLIP-embedded, world-posed frame index at EMBED_HZ over the window.

    Built once per window and handed to every ``localize`` call on it: the
    embed forwards are what a second query would otherwise repeat.
    """
    tf = StreamTF.from_store(store)
    if tf is None:
        raise ValueError("recording has no tf stream")
    posed = (
        store.streams.color_image.after(t0)
        .before(t1)
        .transform(throttle(1.0 / EMBED_HZ))
        .map(
            lambda obs: obs.derive(
                data=obs.data,
                pose=gates.camera_pose(tf, obs.ts, optical_frame, world_frame, tf_tolerance),
            )
        )
        .filter(lambda obs: obs.pose is not None)
    )
    embedded: Stream[Any, Any] = posed.transform(EmbedImages(siglip)).materialize()
    logger.info(f"index: {embedded.count()} frames embedded over {t1 - t0:.1f}s")
    return embedded


def _retrieve(
    index: Stream[Any, Any],
    tf: TFLookup,
    query_embedding: Any,
    optical_frame: str,
    world_frame: str,
    tf_tolerance: float,
) -> list[Any]:
    """Top still frames by text similarity, stratified over time bands.

    Stratification is what keeps latest-pose semantics honest at the
    candidate stage: the top frames of the whole window may all be early,
    and a support that only exists late must still get a detection pass.
    """
    ranked = [
        obs
        for obs in index.search(query_embedding, k=max(index.count(), 1))
        if gates.camera_still(tf, obs.ts, optical_frame, world_frame, tf_tolerance)
    ]
    if not ranked:
        return []

    t0, t1 = index.get_time_range()
    bands = max(1, min(TIME_BANDS, int((t1 - t0) / 20)))
    per_band = max(1, TOP_FRAMES // bands)
    span = (t1 - t0) / bands
    selected: list[Any] = []
    chosen: set[float] = set()
    for band in range(bands):
        band_lo = t0 + band * span
        band_hi = band_lo + span
        in_band = [obs for obs in ranked if band_lo <= obs.ts < band_hi]
        for obs in in_band[:per_band]:
            if obs.ts not in chosen:
                chosen.add(obs.ts)
                selected.append(obs)
    for obs in ranked:  # fill remaining budget by global rank
        if len(selected) >= TOP_FRAMES:
            break
        if obs.ts not in chosen:
            chosen.add(obs.ts)
            selected.append(obs)
    return selected


def localize(
    store: Any,
    query: str | list[str],
    *,
    index: Stream[Any, Any],
    siglip: SigLIPModel,
    detector: Owlv2Detector,
    segmenter: EdgeTAMImageSegmenter,
    require_pose: bool = True,
    policy: LocalizePolicy | None = None,
    cloud_mode: str = "latest_visible",
    world_frame: str = WORLD_FRAME,
    optical_frame: str = OPTICAL_FRAME,
    tf_tolerance: float = TF_TOLERANCE,
    trace: LocalizeTrace | list[LocalizeTrace] | None = None,
) -> Localization | list[Localization | None] | None:
    """Latest unambiguous 3D localization of *query*, or ``None``.

    ``None`` is a first-class answer: nothing reached the accept score, no
    support was confirmed from a second viewpoint, or the best candidate had
    no valid depth and ``require_pose`` holds. An ambiguity between
    coexisting candidates is returned with ``ambiguity_margin`` below
    ``refusal_margin`` - a flagged hit, never a silent guess.

    A list *query* runs every label through one shared detection cache -
    OWLv2 takes the whole list per frame - and returns one result per label,
    in input order; ``trace`` then takes a list of the same length.

    The index and the three models belong to the caller: nothing here is
    loaded or stopped, so one process can call this repeatedly on warm
    weights, and every query on one window reuses the same embeddings. The
    window is the index's - build it with :func:`embed_index`.
    """
    policy = policy or LocalizePolicy()
    tf = StreamTF.from_store(store)
    if tf is None:
        raise ValueError("recording has no tf stream")
    camera_info = store.streams.camera_info.first().data

    queries = [query] if isinstance(query, str) else query
    traces: list[LocalizeTrace | None] = (
        list(trace) if isinstance(trace, list) else [trace] * len(queries)
    )
    cache = _DetectionCache(detector, segmenter, queries, policy.candidate_floor)
    results = [
        _localize_one(
            store,
            q,
            index=index,
            siglip=siglip,
            cache=cache,
            tf=tf,
            camera_info=camera_info,
            require_pose=require_pose,
            policy=policy,
            cloud_mode=cloud_mode,
            world_frame=world_frame,
            optical_frame=optical_frame,
            tf_tolerance=tf_tolerance,
            trace=t,
        )
        for q, t in zip(queries, traces, strict=True)
    ]
    return results[0] if isinstance(query, str) else results


def _localize_one(
    store: Any,
    query: str,
    *,
    index: Stream[Any, Any],
    siglip: SigLIPModel,
    cache: _DetectionCache,
    tf: TFLookup,
    camera_info: CameraInfo,
    require_pose: bool,
    policy: LocalizePolicy,
    cloud_mode: str,
    world_frame: str,
    optical_frame: str,
    tf_tolerance: float,
    trace: LocalizeTrace | None,
) -> Localization | None:
    # Pass 1 - SigLIP: rank the indexed frames by the query.
    query_embedding = siglip.embed_text(query)
    frames = _retrieve(index, tf, query_embedding, optical_frame, world_frame, tf_tolerance)
    logger.info(f"localize '{query}': {len(frames)} candidate frames of {index.count()} embedded")
    if not frames:
        return None

    from dimos.perception.memory.support_plane import fit_support_plane

    plane = fit_support_plane(
        store, tf, camera_info, frames, optical_frame, world_frame, tf_tolerance
    )

    # Pass 2 - OWLv2 + EdgeTAM: detect, segment, lift, verify.
    clusters: list[_Cluster] = []
    ungrounded_best: tuple[float, float] | None = None  # (score, ts)
    processed: set[float] = set()

    def _absorb(frame_obs: Any, is_verify: bool) -> None:
        nonlocal ungrounded_best
        if frame_obs.ts in processed:
            return
        processed.add(frame_obs.ts)
        detections = cache.detect(frame_obs.data, query)
        if not len(detections):
            return
        if trace is not None and not is_verify:
            trace.detection_frames.append(frame_obs.derive(data=detections))
        lifted = _lift(
            detections,
            store,
            tf,
            camera_info,
            optical_frame,
            world_frame,
            tf_tolerance,
            policy,
            plane,
        )
        for det2d in detections:
            if not any(d.track_id == det2d.track_id for d, _ in lifted):
                best = (det2d.confidence, det2d.ts)
                if ungrounded_best is None or best[0] > ungrounded_best[0]:
                    ungrounded_best = best
        for det3d, camera in lifted:
            observation = _ClusterObservation(
                ts=det3d.ts,
                score=det3d.confidence,
                centroid=np.asarray(det3d.pointcloud.pointcloud.points).mean(axis=0),
                cloud=det3d.pointcloud,
                camera_position=camera,
                detection=det3d,
            )
            if trace is not None:
                (trace.verified if is_verify else trace.matched).append((det3d.ts, det3d))
            for cluster in clusters:
                distance = float(np.linalg.norm(observation.centroid - cluster.center))
                if distance <= policy.cluster_radius_m:
                    cluster.add(observation)
                    break
            else:
                clusters.append(_Cluster(center=observation.centroid, observations=[observation]))

    for frame_obs in frames:
        _absorb(frame_obs, is_verify=False)
    logger.info(f"detection: {len(clusters)} support candidates")

    # Cross-view verification: a support seen from one pose only is
    # unconfirmed. Frames whose camera could observe the support are found
    # geometrically (near + sees with occlusion), then re-detected.
    clusters.sort(key=lambda c: -c.max_score)
    for cluster in list(clusters[:4]):
        predicate = sees(
            cluster.center,
            camera_info,
            tf=tf,
            world_frame=world_frame,
            optical_frame=optical_frame,
            time_tolerance=tf_tolerance,
            extent=np.minimum(cluster.extent, 0.4),
            # A large object overflows close-up frames; a third of its box in
            # view is still a usable re-detection pass, and those close-ups
            # are exactly the distinct viewpoints verification needs.
            min_fraction=0.35,
            depth=lambda obs: gates.depth_at(store, obs.ts),
            max_range=1.6,
        )
        observing = [
            obs
            for obs in index.near(cluster.center, radius=1.6)
            if obs.ts not in processed
            and gates.camera_still(tf, obs.ts, optical_frame, world_frame, tf_tolerance)
            and predicate(obs)
        ]
        if len(observing) > VERIFY_FRAMES:
            # Even spread that always includes the endpoints: dropping the
            # latest seeing frames would bias the latest-pose answer early.
            picks = np.unique(np.linspace(0, len(observing) - 1, VERIFY_FRAMES).astype(int))
            observing = [observing[i] for i in picks]
        for frame_obs in observing:
            _absorb(frame_obs, is_verify=True)

    verified = [
        c for c in clusters if c.max_score >= policy.accept_score and c.n_views >= policy.min_views
    ]
    logger.info(
        "verification: "
        + ", ".join(
            f"score={c.max_score:.2f} views={c.n_views} obs={len(c.observations)}" for c in clusters
        )
    )

    if not verified:
        if ungrounded_best is not None and ungrounded_best[0] >= policy.accept_score:
            if require_pose:
                logger.info("best candidate has no valid depth and require_pose is set")
                return None
            score, ts = ungrounded_best
            return Localization(
                instance_id="query-0",
                semantic_score=score,
                identity_score=0.0,
                ambiguity_margin=1.0,
                position_world_xyz=None,
                orientation_world_xyzw=None,
                frame_id="world",
                support=None,
                pose_timestamp=ts,
                geometry_timestamp=ts,
                last_seen_timestamp=ts,
                point_cloud=None,
                cloud_mode=cloud_mode,
                coverage=0.0,
                n_views=1,
                reason="no_valid_depth",
            )
        return None

    winner = max(verified, key=lambda c: c.latest.ts)
    w_lo, w_hi = winner.interval
    rival_scores = [
        c.max_score
        for c in verified
        if c is not winner
        and not (c.interval[1] < w_lo or c.interval[0] > w_hi)  # coexisting in time
    ]
    margin = winner.max_score - max(rival_scores) if rival_scores else 1.0
    reason = "ambiguous_between_coexisting_candidates" if margin < policy.refusal_margin else None

    latest = winner.latest
    points = np.asarray(latest.cloud.pointcloud.points)
    aabb_min, aabb_max = points.min(axis=0), points.max(axis=0)
    try:
        orientation = _quaternion_from_matrix(np.asarray(latest.cloud.oriented_bounding_box.R))
    except Exception:
        orientation = (0.0, 0.0, 0.0, 1.0)
    center = (aabb_min + aabb_max) / 2
    extent = np.maximum(aabb_max - aabb_min, 0.005)
    sigma = (
        np.stack([o.centroid for o in winner.observations]).std(axis=0)
        if len(winner.observations) > 1
        else np.full(3, 0.01)
    )
    support = Support(
        center_xyz=(float(center[0]), float(center[1]), float(center[2])),
        extent_xyz_m=(float(extent[0]), float(extent[1]), float(extent[2])),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        sigma_xyz_m=(float(sigma[0]), float(sigma[1]), float(sigma[2])),
        coverage=_azimuth_coverage(winner.observations, winner.center),
        axes_observed=_axes_observed(winner.observations, winner.center),
        frame_id="world",
    )

    if trace is not None:
        trace.answer = latest.detection
        trace.backdrop_ts = latest.ts

    return Localization(
        instance_id="query-0",
        semantic_score=winner.max_score,
        identity_score=min(1.0, winner.n_views / 4.0),
        ambiguity_margin=margin,
        position_world_xyz=(
            float(latest.centroid[0]),
            float(latest.centroid[1]),
            float(latest.centroid[2]),
        ),
        orientation_world_xyzw=orientation,
        frame_id="world",
        support=support,
        pose_timestamp=latest.ts,
        geometry_timestamp=latest.ts,
        last_seen_timestamp=latest.ts,
        point_cloud=latest.cloud,
        cloud_mode=cloud_mode,
        coverage=support.coverage,
        n_views=winner.n_views,
        reason=reason,
    )
