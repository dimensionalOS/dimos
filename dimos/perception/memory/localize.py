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
segmentation (EdgeTAM), projection to 3D through the rig's geometry - an
aligned depth stream or a registered pointcloud stream. Two
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
from dimos.memory.transform import throttle
from dimos.perception.detection.identity import Identity, spatial
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.memory.rig import Rig
from dimos.perception.memory.types import Localization, LocalizePolicy, Support
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.stream import Stream
    from dimos.models.embedding.siglip import SigLIPModel
    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC

logger = setup_logger()

TIME_BANDS = 6  # stratify retrieval across the window so late scans always compete
BOXES_PER_FRAME = 4
# Frames per OWLv2 forward. The forward's softmax transient is fp32 even
# under autocast (~580MB per image at 960px), so the profitable batch is
# whatever VRAM holds beyond the resident models - 1 on an 8GB card.
DETECT_BATCH = 1
CONFIRM_FLOOR = 0.22  # geometric confirmation accept for re-detections
VERIFY_FRAMES = 20


# A support candidate is an identity group: the member sightings of one
# object. Everything a group reports is a plain function over its members.


def _centroid(det: Detection3DPC) -> np.ndarray:
    centroid: np.ndarray = np.asarray(det.pointcloud.pointcloud.points).mean(axis=0)
    return centroid


def _camera_position(det: Detection3DPC) -> np.ndarray:
    position = (-det.transform).translation
    return np.array([position.x, position.y, position.z])


def _group_center(members: list[Detection3DPC]) -> np.ndarray:
    center: np.ndarray = np.mean(np.stack([_centroid(d) for d in members]), axis=0)
    return center


def _max_score(members: list[Detection3DPC]) -> float:
    return max(d.confidence for d in members)


def _latest(members: list[Detection3DPC]) -> Detection3DPC:
    return max(members, key=lambda d: d.ts)


def _interval(members: list[Detection3DPC]) -> tuple[float, float]:
    times = [d.ts for d in members]
    return min(times), max(times)


def _n_views(members: list[Detection3DPC]) -> int:
    return len({tuple(np.round(_camera_position(d), 2)) for d in members})


def _group_extent(members: list[Detection3DPC]) -> np.ndarray:
    points = np.concatenate([np.asarray(d.pointcloud.pointcloud.points) for d in members])
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


def _azimuth_coverage(members: list[Detection3DPC], center: np.ndarray) -> float:
    directions = []
    for det in members:
        v = _camera_position(det) - center
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            directions.append(v / norm)
    if not directions:
        return 0.0
    dirs = np.stack(directions)
    azimuth = np.arctan2(dirs[:, 1], dirs[:, 0])
    bins = set(((azimuth + math.pi) / (2 * math.pi) * 8).astype(int) % 8)
    return len(bins) / 8.0


def _axes_observed(members: list[Detection3DPC], center: np.ndarray) -> tuple[bool, bool, bool]:
    directions = []
    for det in members:
        v = _camera_position(det) - center
        norm = np.linalg.norm(v)
        if norm > 1e-6:
            directions.append(v / norm)
    if not directions:
        return (False, False, False)
    dirs = np.stack(directions)
    return tuple(bool((np.abs(dirs[:, i]) > 0.3).any()) for i in range(3))  # type: ignore[return-value]


class _DetectionCache:
    """One OWLv2 + EdgeTAM + lift pass per unique frame, shared across queries and clusters."""

    def __init__(
        self, detector: Any, segmenter: Any, rig: Rig, queries: list[str], floor: float
    ) -> None:
        self.detector = detector
        self.segmenter = segmenter
        self.rig = rig
        self.queries = queries
        self.floor = floor
        self._cache: dict[float, ImageDetections2D] = {}
        self._lifted: dict[float, list[Detection3DPC]] = {}

    def _ingest(self, image: Any, detections: ImageDetections2D) -> None:
        ranked = sorted(detections.detections, key=lambda d: -d.confidence)
        kept = [
            det
            for label in self.queries
            for det in [d for d in ranked if d.name == label][:BOXES_PER_FRAME]
        ]
        for i, det in enumerate(kept):
            det.track_id = i
        cached = ImageDetections2D(image, kept)
        if len(cached):
            cached = self.segmenter.segment(cached)
        self._cache[image.ts] = cached

    def prefetch(self, images: list[Any]) -> None:
        """Detect and segment every uncached frame, batching the OWLv2 forwards."""
        todo: dict[float, Any] = {}
        for image in images:
            if image.ts not in self._cache and image.ts not in todo:
                todo[image.ts] = image
        pending = list(todo.values())
        for start in range(0, len(pending), DETECT_BATCH):
            chunk = pending[start : start + DETECT_BATCH]
            for image, detections in zip(
                chunk,
                self.detector.query_detections_batch(chunk, self.queries, threshold=self.floor),
                strict=True,
            ):
                self._ingest(image, detections)

    def detect(self, image: Any, query: str) -> ImageDetections2D:
        if image.ts not in self._cache:
            self._ingest(
                image, self.detector.query_detections(image, self.queries, threshold=self.floor)
            )
        return self._cache[image.ts].filter(lambda d: d.name == query)

    def lift(self, ts: float) -> list[Detection3DPC]:
        """All of a frame's detections lifted once, whatever their label."""
        if ts not in self._lifted:
            lifted = self.rig.lift(self._cache[ts])
            self._lifted[ts] = list(lifted) if lifted is not None else []
        return self._lifted[ts]


def _lift(
    detections: ImageDetections2D,
    cache: _DetectionCache,
    rig: Rig,
    policy: LocalizePolicy,
    plane: Any | None = None,
) -> list[tuple[Detection3DPC, np.ndarray]]:
    """Gate a query's share of the frame's shared lift; (detection3d, camera_position) pairs."""
    pose = rig.camera_pose(detections.ts)
    if pose is None:
        return []
    camera = np.array([pose.position.x, pose.position.y, pose.position.z])

    track_ids = {det.track_id for det in detections}
    valid: list[tuple[Detection3DPC, np.ndarray]] = []
    for det3d in cache.lift(detections.ts):
        if det3d.track_id not in track_ids:
            continue
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
    rig: Rig | None = None,
) -> Stream[Any, Any]:
    """SigLIP-embedded, world-posed frame index at the rig's embed rate.

    Built once per window and handed to every ``localize`` call on it: the
    embed forwards are what a second query would otherwise repeat.
    """
    rig = rig or Rig.from_store(store)
    posed = (
        rig.color.after(t0)
        .before(t1)
        .transform(throttle(1.0 / rig.embed_hz))
        .map(lambda obs: obs.derive(data=obs.data, pose=rig.index_pose(obs)))
        .filter(lambda obs: obs.pose is not None)
    )
    embedded: Stream[Any, Any] = posed.transform(EmbedImages(siglip)).materialize()
    logger.info(f"index: {embedded.count()} frames embedded over {t1 - t0:.1f}s")
    return embedded


def _retrieve(index: Stream[Any, Any], rig: Rig, query_embedding: Any, budget: int) -> list[Any]:
    """Top still frames by text similarity, stratified over time bands.

    Stratification is what keeps latest-pose semantics honest at the
    candidate stage: the top frames of the whole window may all be early,
    and a support that only exists late must still get a detection pass.
    """
    ranked = [
        obs
        for obs in index.search(query_embedding, k=max(index.count(), 1))
        if rig.camera_still(obs.ts)
    ]
    if not ranked:
        return []

    t0, t1 = index.get_time_range()
    bands = max(1, min(TIME_BANDS, int((t1 - t0) / 20)))
    per_band = max(1, budget // bands)
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
        if len(selected) >= budget:
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
    rig: Rig | None = None,
    require_pose: bool = True,
    policy: LocalizePolicy | None = None,
    cloud_mode: str = "latest_visible",
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

    The index, the rig and the three models belong to the caller: nothing
    here is loaded or stopped, so one process can call this repeatedly on
    warm weights, and every query on one window reuses the same embeddings.
    The window is the index's - build it with :func:`embed_index`. Without a
    ``rig`` the store's shape decides one, and without a ``policy`` the rig
    supplies scale-appropriate defaults.
    """
    rig = rig or Rig.from_store(store)
    policy = policy or rig.default_localize_policy()

    queries = [query] if isinstance(query, str) else query
    traces: list[LocalizeTrace | None] = (
        list(trace) if isinstance(trace, list) else [trace] * len(queries)
    )
    cache = _DetectionCache(detector, segmenter, rig, queries, policy.candidate_floor)
    results = [
        _localize_one(
            q,
            index=index,
            siglip=siglip,
            cache=cache,
            rig=rig,
            require_pose=require_pose,
            policy=policy,
            cloud_mode=cloud_mode,
            trace=t,
        )
        for q, t in zip(queries, traces, strict=True)
    ]
    return results[0] if isinstance(query, str) else results


def _localize_one(
    query: str,
    *,
    index: Stream[Any, Any],
    siglip: SigLIPModel,
    cache: _DetectionCache,
    rig: Rig,
    require_pose: bool,
    policy: LocalizePolicy,
    cloud_mode: str,
    trace: LocalizeTrace | None,
) -> Localization | None:
    # Pass 1 - SigLIP: rank the indexed frames by the query.
    query_embedding = siglip.embed_text(query)
    frames = _retrieve(index, rig, query_embedding, policy.retrieval_frames)
    logger.info(f"localize '{query}': {len(frames)} candidate frames of {index.count()} embedded")
    if not frames:
        return None

    from dimos.perception.memory.support_plane import fit_support_plane

    plane = fit_support_plane(rig, frames)
    cache.prefetch([obs.data for obs in frames])

    # Pass 2 - OWLv2 + EdgeTAM: detect, segment, lift, verify. Support
    # candidates are identity groups: sightings joined by the spatial
    # is_same strategy at the policy's cluster radius.
    identity = Identity(is_same=spatial(policy.cluster_radius_m))
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
        lifted = _lift(detections, cache, rig, policy, plane)
        for det2d in detections:
            if not any(d.track_id == det2d.track_id for d, _ in lifted):
                best = (det2d.confidence, det2d.ts)
                if ungrounded_best is None or best[0] > ungrounded_best[0]:
                    ungrounded_best = best
        for det3d, _camera in lifted:
            if trace is not None:
                (trace.verified if is_verify else trace.matched).append((det3d.ts, det3d))
            identity.add(det3d)

    for frame_obs in frames:
        _absorb(frame_obs, is_verify=False)
    logger.info(f"detection: {len(identity.groups)} support candidates")

    # Cross-view verification: a support seen from one pose only is
    # unconfirmed. Frames whose camera could observe the support are found
    # geometrically (near + sees with occlusion), then re-detected.
    for members in sorted(identity.groups, key=_max_score, reverse=True)[:4]:
        center = _group_center(members)
        predicate = rig.sees(
            center,
            extent=np.minimum(_group_extent(members), 0.4),
            # A large object overflows close-up frames; a third of its box in
            # view is still a usable re-detection pass, and those close-ups
            # are exactly the distinct viewpoints verification needs.
            min_fraction=0.35,
            max_range=policy.verify_radius_m,
        )
        observing = [
            obs
            for obs in index.near(center, radius=policy.verify_radius_m)
            if obs.ts not in processed and rig.camera_still(obs.ts) and predicate(obs)
        ]
        if len(observing) > VERIFY_FRAMES:
            # Even spread that always includes the endpoints: dropping the
            # latest seeing frames would bias the latest-pose answer early.
            picks = np.unique(np.linspace(0, len(observing) - 1, VERIFY_FRAMES).astype(int))
            observing = [observing[i] for i in picks]
        cache.prefetch([obs.data for obs in observing])
        for frame_obs in observing:
            _absorb(frame_obs, is_verify=True)

    verified = [
        members
        for members in identity.groups
        if _max_score(members) >= policy.accept_score and _n_views(members) >= policy.min_views
    ]
    logger.info(
        "verification: "
        + ", ".join(
            f"score={_max_score(g):.2f} views={_n_views(g)} obs={len(g)}" for g in identity.groups
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
                frame_id=rig.world_frame,
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

    winner = max(verified, key=lambda g: _latest(g).ts)
    w_lo, w_hi = _interval(winner)
    rival_scores = [
        _max_score(g)
        for g in verified
        if g is not winner
        and not (_interval(g)[1] < w_lo or _interval(g)[0] > w_hi)  # coexisting in time
    ]
    margin = _max_score(winner) - max(rival_scores) if rival_scores else 1.0
    reason = "ambiguous_between_coexisting_candidates" if margin < policy.refusal_margin else None

    latest = _latest(winner)
    points = np.asarray(latest.pointcloud.pointcloud.points)
    aabb_min, aabb_max = points.min(axis=0), points.max(axis=0)
    try:
        orientation = _quaternion_from_matrix(np.asarray(latest.pointcloud.oriented_bounding_box.R))
    except Exception:
        orientation = (0.0, 0.0, 0.0, 1.0)
    center = (aabb_min + aabb_max) / 2
    extent = np.maximum(aabb_max - aabb_min, 0.005)
    sigma = (
        np.stack([_centroid(d) for d in winner]).std(axis=0)
        if len(winner) > 1
        else np.full(3, 0.01)
    )
    winner_center = _group_center(winner)
    support = Support(
        center_xyz=(float(center[0]), float(center[1]), float(center[2])),
        extent_xyz_m=(float(extent[0]), float(extent[1]), float(extent[2])),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        sigma_xyz_m=(float(sigma[0]), float(sigma[1]), float(sigma[2])),
        coverage=_azimuth_coverage(winner, winner_center),
        axes_observed=_axes_observed(winner, winner_center),
        frame_id=rig.world_frame,
    )

    if trace is not None:
        trace.answer = latest
        trace.backdrop_ts = latest.ts

    latest_centroid = _centroid(latest)
    return Localization(
        instance_id="query-0",
        semantic_score=_max_score(winner),
        identity_score=min(1.0, _n_views(winner) / 4.0),
        ambiguity_margin=margin,
        position_world_xyz=(
            float(latest_centroid[0]),
            float(latest_centroid[1]),
            float(latest_centroid[2]),
        ),
        orientation_world_xyzw=orientation,
        frame_id=rig.world_frame,
        support=support,
        pose_timestamp=latest.ts,
        geometry_timestamp=latest.ts,
        last_seen_timestamp=latest.ts,
        point_cloud=latest.pointcloud,
        cloud_mode=cloud_mode,
        coverage=support.coverage,
        n_views=_n_views(winner),
        reason=reason,
    )
