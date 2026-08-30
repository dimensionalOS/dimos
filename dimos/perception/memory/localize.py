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

* **Latest-pose semantics.** Every verified instance is returned,
  latest-seen first, and each instance's position follows its latest
  sighting: "where is it now", never "where did it match best". The
  instance's cloud is the union of every viewpoint that saw it.
* **Calibrated refusal.** Every stage carries a score and the answer can be
  empty: no accept-level detection, no multi-view confirmation. Coexisting
  same-label instances below the refusal margin are flagged, never merged
  or silently dropped.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory.embed import EmbedImages
from dimos.memory.transform import QualityWindow, peaks
from dimos.perception.detection.identity import Identity, fused, spatial
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.memory.rig import CLOUD_MIN_POINTS, Rig
from dimos.perception.memory.types import Localization, LocalizePolicy, Support
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import PoseTuple
    from dimos.models.embedding.siglip import SigLIPModel
    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
    from dimos.perception.memory.identity_store import IdentityStore

logger = setup_logger()

_SCORE_CACHE_MAX = 8192


# A support candidate is an identity group: the member sightings of one
# object. Everything a group reports is a plain function over its members.


def _similarity(obs: Any) -> float:
    return float(obs.similarity)


def _settled(index: Stream[Any, Any], spacing: float) -> set[int]:
    """Ids of the index frames left once sub-spacing duplicates are dropped.

    The index emits the sharpest frame per window from a fixed phase, so a
    camera that moves between windows leaves both the settled frame and the
    blurred one taken while it was still moving. Their poses differ, so the
    blurred one passes as a second viewpoint and lifts to a displaced cloud.
    A pair closer than half the index window is one window's content split
    by that phase; only the sharper of the two survives here.
    """
    ids: set[int] = set()
    last_id = -1
    last_ts = -math.inf
    last_sharpness = -1.0
    for obs in index.order_by("ts"):
        sharpness = float(obs.data.sharpness)
        if obs.ts - last_ts < spacing:
            if sharpness <= last_sharpness:
                continue
            ids.discard(last_id)
        ids.add(obs.id)
        last_id, last_ts, last_sharpness = obs.id, obs.ts, sharpness
    return ids


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


@dataclass
class LocalizeTrace:
    """Intermediate artifacts collected for rendering; filled when passed in."""

    detection_frames: list[Any] = field(default_factory=list)  # Observation[ImageDetections2D]
    matched: list[tuple[float, Detection3DPC]] = field(default_factory=list)
    verified: list[tuple[float, Detection3DPC]] = field(default_factory=list)
    answers: list[list[Detection3DPC]] = field(default_factory=list)  # sightings per instance
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


def _lift(
    detections: ImageDetections2D[Any],
    rig: Rig,
    policy: LocalizePolicy,
    plane: Any | None = None,
) -> list[Detection3DPC]:
    """Gate a frame's lifted detections."""
    pose = rig.camera_pose(detections.ts)
    if pose is None:
        return []
    camera = np.array([pose.position.x, pose.position.y, pose.position.z])
    lifted = rig.lift(detections, plane)
    if lifted is None:
        return []

    floor = policy.min_depth_points if rig.cloud is None else CLOUD_MIN_POINTS
    valid: list[Detection3DPC] = []
    for det3d in lifted:
        points = np.asarray(det3d.pointcloud.pointcloud.points)
        if len(points) < floor:
            continue
        extent = points.max(axis=0) - points.min(axis=0)
        if float(extent.max()) > policy.max_object_extent_m:
            continue
        ranges = np.linalg.norm(points - camera, axis=1)
        if float(np.median(ranges)) < policy.min_camera_range_m:
            continue
        if plane is not None:
            heights = rig.support_heights(detections.ts, plane, points)
            low = float(np.quantile(heights, 0.05))
            high = float(np.quantile(heights, 0.95))
            if low > policy.surface_patch_min_drop_m and high < policy.surface_patch_max_rise_m:
                continue
        valid.append(det3d)
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
        .filter(lambda obs: obs.data.brightness > 0.1)
        .transform(QualityWindow(lambda img: img.sharpness, window=1.0 / rig.embed_hz))
        .map(lambda obs: obs.derive(data=obs.data, pose=rig.index_pose(obs)))
        .filter(lambda obs: obs.pose is not None)
    )
    embedded: Stream[Any, Any] = posed.transform(EmbedImages(siglip)).materialize()
    logger.info(f"index: {embedded.count()} frames embedded over {t1 - t0:.1f}s")
    return embedded


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
    trace: LocalizeTrace | list[LocalizeTrace] | None = None,
    identity_store: IdentityStore | None = None,
) -> list[Localization] | list[list[Localization]]:
    """Every verified 3D instance of *query*, latest-seen first.

    Each instance's ``point_cloud`` is the union of every viewpoint that saw
    it, and its position follows the latest sighting. An empty list is a
    first-class answer: nothing reached the accept score, no support was
    confirmed from a second viewpoint, or the best candidate had no valid
    depth and ``require_pose`` holds. Coexisting instances of one label are
    all returned; each carries ``ambiguity_margin`` against its rivals and
    is flagged below ``refusal_margin`` - never a silent guess.

    A list *query* shares one detection pass: every label's semantic peaks
    mark its sightings, each peak takes the frames adjacent to it in time
    until the verification policy's viewpoints are covered, and each unique
    selected frame is scored against every label, segmented and lifted once.
    One instance list per label, in input order. ``trace`` then takes a list
    of the same length.

    The index, the rig and the three models belong to the caller: nothing
    here is loaded or stopped, so one process can call this repeatedly on
    warm weights, and every query on one window reuses the same embeddings.
    The window is the index's - build it with :func:`embed_index`. Without a
    ``rig`` the store's shape decides one, and without a ``policy`` the rig
    supplies scale-appropriate defaults.

    An ``identity_store`` makes evidence cumulative: each label's groups
    persist across calls, frames the store already ingested for a label are
    skipped entirely, and an object stays answerable after it leaves the
    window, its position still following the latest sighting. Without one,
    every call verifies from scratch inside its window.
    """
    rig = rig or Rig.from_store(store)
    policy = policy or rig.default_localize_policy()

    queries = [query] if isinstance(query, str) else query
    traces: list[LocalizeTrace | None] = (
        list(trace) if isinstance(trace, list) else [trace] * len(queries)
    )

    index_count = index.count()
    settled = _settled(index, 0.5 / rig.embed_hz)
    source = index.filter(lambda obs: obs.id in settled)
    candidate_ids: set[int] = set()
    expanded: set[float] = set()
    anchor_x = 0.0
    anchor_y = 0.0
    anchor_count = 0

    for q in queries:
        query_embedding = siglip.embed_text(q)
        sightings = list(
            source.search(query_embedding)
            .order_by("ts")
            .transform(peaks(key=_similarity, distance=1.0))
        )
        # A peak is never the window's last sample, and an instance's position
        # follows its latest sighting: the tail's best frame is one too.
        sightings.extend(
            source.after(sightings[-1].ts if sightings else 0.0).search(query_embedding, k=1)
        )
        peak_count = 0
        for peak in sightings:
            peak_pose = cast("PoseTuple", peak.pose_tuple)
            peak_count += 1
            anchor_count += 1
            anchor_x += peak_pose[0]
            anchor_y += peak_pose[1]
            candidate_ids.add(peak.id)
            if peak.ts in expanded:
                continue
            expanded.add(peak.ts)
            gathered: Stream[Any, Any] = source.near(
                peak.pose_stamped, radius=policy.verify_radius_m
            ).transform(QualityWindow(lambda img: img.sharpness, window=0.5))
            for obs in gathered:
                candidate_ids.add(obs.id)
        logger.info(f"localize {q!r}: {peak_count} semantic peaks of {index_count} embedded")

    candidates = index.filter(lambda obs: obs.id in candidate_ids).order_by("ts")
    candidate_count = len(candidate_ids)
    logger.info(f"detection: {candidate_count} candidate frames for {len(queries)} labels")

    plane = None
    if candidate_count:
        from dimos.perception.memory.support_plane import fit_support_plane

        mx = anchor_x / anchor_count
        my = anchor_y / anchor_count
        cell = (round(mx / 2.0), round(my / 2.0))
        plane = rig._plane_cache.get(cell)
        if plane is None:
            stride = max(1, candidate_count // 5)
            keyframes = []
            for i, obs in enumerate(candidates):
                if i % stride == 0:
                    keyframes.append(obs)
                    if len(keyframes) == 5:
                        break
            plane = fit_support_plane(rig, keyframes)
            if plane is not None:
                rig._plane_cache[cell] = plane

    if identity_store is None:
        entries = None
        identities = [
            Identity(is_same=spatial(policy.cluster_radius_m), merge=fused(policy.fuse_voxel_m))
            for _ in queries
        ]
        ingested: list[set[float]] = [set() for _ in queries]
        ungrounded: list[tuple[float, float] | None] = [None] * len(queries)  # (score, ts)
    else:
        entries = [
            identity_store.get_or_create(
                q, spatial(policy.cluster_radius_m), fused(policy.fuse_voxel_m)
            )
            for q in queries
        ]
        identities = [entry.identity for entry in entries]
        ingested = [entry.ingested for entry in entries]
        ungrounded = [entry.ungrounded for entry in entries]

    floor = policy.candidate_floor
    cache = detector.score_cache

    def _detect(upstream: Iterator[Any]) -> Iterator[Any]:
        for obs in upstream:
            active = [j for j in range(len(queries)) if obs.ts not in ingested[j]]
            if not active:
                continue
            if any((obs.ts, queries[j], floor) not in cache for j in active):
                boxes, rows = detector.query_score_rows_batch([obs.data], queries, threshold=floor)[
                    0
                ]
                for j, q in enumerate(queries):
                    keep = rows[:, j] >= floor
                    cache[(obs.ts, q, floor)] = (boxes[keep], rows[keep, j])
                    if len(cache) > _SCORE_CACHE_MAX:
                        cache.popitem(last=False)

            rows_per_label: list[tuple[int, tuple[Any, Any]]] = []
            for j in active:
                key = (obs.ts, queries[j], floor)
                cache.move_to_end(key)
                rows_per_label.append((j, cache[key]))
                ingested[j].add(obs.ts)
            if not any(len(scores) for _j, (_boxes, scores) in rows_per_label):
                continue
            img = obs.data
            detections: list[Detection2DBBox] = []
            for j, (boxes, scores) in rows_per_label:
                for box, score in zip(boxes, scores, strict=True):
                    det = Detection2DBBox(
                        bbox=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                        track_id=len(detections),
                        class_id=j,
                        confidence=float(score),
                        name=queries[j],
                        ts=img.ts,
                        image=img,
                    )
                    if det.is_valid():
                        detections.append(det)
            if detections:
                yield obs.derive(data=ImageDetections2D(image=img, detections=detections))

    def _ingest(upstream: Iterator[Any]) -> Iterator[Any]:
        for obs in upstream:
            frame = obs.data
            lifted = _lift(frame, rig, policy, plane)
            grounded = {det3d.track_id for det3d in lifted}
            for det2d in frame:
                j = det2d.class_id
                best = ungrounded[j]
                if det2d.track_id not in grounded and (best is None or det2d.confidence > best[0]):
                    ungrounded[j] = (det2d.confidence, det2d.ts)
            for det3d in lifted:
                label_trace = traces[det3d.class_id]
                if label_trace is not None:
                    label_trace.matched.append((det3d.ts, det3d))
                identities[det3d.class_id].add(det3d)
            for j in range(len(queries)):
                label_trace = traces[j]
                if label_trace is None:
                    continue
                label_dets = [det for det in frame if det.class_id == j]
                if label_dets:
                    label_trace.detection_frames.append(
                        obs.derive(data=ImageDetections2D(image=frame.image, detections=label_dets))
                    )
            yield obs

    candidates.transform(_detect).map(lambda obs: obs.derive(data=segmenter.segment(obs.data))).transform(_ingest).drain()

    if entries is not None:
        for entry, best in zip(entries, ungrounded, strict=True):
            entry.ungrounded = best

    results = [
        _finalize(
            q,
            identity=identities[j],
            ungrounded_best=ungrounded[j],
            rig=rig,
            require_pose=require_pose,
            policy=policy,
            trace=traces[j],
        )
        for j, q in enumerate(queries)
    ]
    return results[0] if isinstance(query, str) else results


def _finalize(
    query: str,
    *,
    identity: Identity,
    ungrounded_best: tuple[float, float] | None,
    rig: Rig,
    require_pose: bool,
    policy: LocalizePolicy,
    trace: LocalizeTrace | None,
) -> list[Localization]:
    verified = [
        (merged, members)
        for merged, members in zip(identity.merged, identity.groups, strict=True)
        if _max_score(members) >= policy.accept_score and _n_views(members) >= policy.min_views
    ]
    logger.info(
        f"verification {query!r}: "
        + ", ".join(
            f"score={_max_score(g):.2f} views={_n_views(g)} obs={len(g)}" for g in identity.groups
        )
    )
    if trace is not None:
        for _merged, members in verified:
            for det3d in members:
                trace.verified.append((det3d.ts, det3d))

    if not verified:
        if ungrounded_best is not None and ungrounded_best[0] >= policy.accept_score:
            if require_pose:
                logger.info(f"{query!r}: best candidate has no valid depth and require_pose is set")
                return []
            score, ts = ungrounded_best
            return [
                Localization(
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
                    coverage=0.0,
                    n_views=1,
                    reason="no_valid_depth",
                )
            ]
        return []

    verified.sort(key=lambda pair: _latest(pair[1]).ts, reverse=True)
    instances: list[Localization] = []
    for k, (merged, members) in enumerate(verified):
        m_lo, m_hi = _interval(members)
        rival_scores = [
            _max_score(others)
            for _m, others in verified
            if others is not members
            and not (_interval(others)[1] < m_lo or _interval(others)[0] > m_hi)  # coexisting
        ]
        margin = _max_score(members) - max(rival_scores) if rival_scores else 1.0
        reason = (
            "ambiguous_between_coexisting_candidates" if margin < policy.refusal_margin else None
        )

        latest = _latest(members)
        union = merged.pointcloud
        points = np.asarray(union.pointcloud.points)
        aabb_min, aabb_max = points.min(axis=0), points.max(axis=0)
        try:
            orientation = _quaternion_from_matrix(
                np.asarray(latest.pointcloud.oriented_bounding_box.R)
            )
        except Exception:
            orientation = (0.0, 0.0, 0.0, 1.0)
        center = (aabb_min + aabb_max) / 2
        extent = np.maximum(aabb_max - aabb_min, 0.005)
        sigma = (
            np.stack([_centroid(d) for d in members]).std(axis=0)
            if len(members) > 1
            else np.full(3, 0.01)
        )
        group_center = _group_center(members)
        support = Support(
            center_xyz=(float(center[0]), float(center[1]), float(center[2])),
            extent_xyz_m=(float(extent[0]), float(extent[1]), float(extent[2])),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            sigma_xyz_m=(float(sigma[0]), float(sigma[1]), float(sigma[2])),
            coverage=_azimuth_coverage(members, group_center),
            axes_observed=_axes_observed(members, group_center),
            frame_id=rig.world_frame,
        )

        if trace is not None:
            trace.answers.append(members)
            if k == 0:
                trace.backdrop_ts = latest.ts

        latest_centroid = _centroid(latest)
        instances.append(
            Localization(
                instance_id=f"query-{k}",
                semantic_score=_max_score(members),
                identity_score=min(1.0, _n_views(members) / 4.0),
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
                point_cloud=union,
                coverage=support.coverage,
                n_views=_n_views(members),
                reason=reason,
            )
        )
    return instances
