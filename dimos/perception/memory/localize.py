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
from dimos.memory.transform import QualityWindow, peaks
from dimos.perception.detection.identity import Identity, spatial
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
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


# A support candidate is an identity group: the member sightings of one
# object. Everything a group reports is a plain function over its members.


def _similarity(obs: Any) -> float:
    return float(obs.similarity)


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


def _lift(
    detections: ImageDetections2D,
    rig: Rig,
    policy: LocalizePolicy,
    plane: Any | None = None,
) -> list[Detection3DPC]:
    """Gate a frame's lifted detections."""
    pose = rig.camera_pose(detections.ts)
    if pose is None:
        return []
    camera = np.array([pose.position.x, pose.position.y, pose.position.z])
    lifted = rig.lift(detections)
    if lifted is None:
        return []

    valid: list[Detection3DPC] = []
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
    cloud_mode: str = "latest_visible",
    trace: LocalizeTrace | list[LocalizeTrace] | None = None,
) -> Localization | list[Localization | None] | None:
    """Latest unambiguous 3D localization of *query*, or ``None``.

    ``None`` is a first-class answer: nothing reached the accept score, no
    support was confirmed from a second viewpoint, or the best candidate had
    no valid depth and ``require_pose`` holds. An ambiguity between
    coexisting candidates is returned with ``ambiguity_margin`` below
    ``refusal_margin`` - a flagged hit, never a silent guess.

    A list *query* shares one detection pass: every label's semantic peaks
    select frames, and each unique frame is scored against every label in a
    single OWLv2 forward, segmented and lifted once. One result per label,
    in input order. ``trace`` then takes a list of the same length.

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

    peaks_per_label: list[Stream[Any, Any]] = []
    for q in queries:
        query_embedding = siglip.embed_text(q)
        label_peaks: Stream[Any, Any] = (
            index.search(query_embedding)
            .order_by("ts")
            .transform(peaks(key=_similarity, distance=1.0))
            .materialize()
        )
        logger.info(
            f"localize {q!r}: {label_peaks.count()} semantic peaks of {index.count()} embedded"
        )
        peaks_per_label.append(label_peaks)

    frames: dict[float, Any] = {}
    expanded: set[float] = set()
    for label_peaks in peaks_per_label:
        for peak in label_peaks:
            frames.setdefault(peak.ts, peak)
            if peak.ts in expanded:
                continue
            expanded.add(peak.ts)
            nearby: Stream[Any, Any] = index.near(
                peak.pose_stamped, radius=policy.verify_radius_m
            ).transform(QualityWindow(lambda img: img.sharpness, window=0.5))
            for obs in nearby:
                frames.setdefault(obs.ts, obs)
    ordered = sorted(frames.values(), key=lambda obs: obs.ts)
    logger.info(f"detection: {len(ordered)} candidate frames for {len(queries)} labels")

    if not ordered:
        results: list[Localization | None] = [None] * len(queries)
        return None if isinstance(query, str) else results

    from dimos.perception.memory.support_plane import fit_support_plane

    plane = fit_support_plane(rig, ordered)
    identities = [Identity(is_same=spatial(policy.cluster_radius_m)) for _ in queries]
    ungrounded: list[tuple[float, float] | None] = [None] * len(queries)  # (score, ts)

    for obs in ordered:
        boxes, rows = detector.query_score_rows(obs.data, queries, threshold=policy.candidate_floor)
        candidates: list[Detection2DBBox] = []
        for box, row in zip(boxes, rows, strict=True):
            bbox = (float(box[0]), float(box[1]), float(box[2]), float(box[3]))
            for j, score in enumerate(row):
                if score < policy.candidate_floor:
                    continue
                det = Detection2DBBox(
                    bbox=bbox,
                    track_id=len(candidates),
                    class_id=j,
                    confidence=float(score),
                    name=queries[j],
                    ts=obs.data.ts,
                    image=obs.data,
                )
                if det.is_valid() and det.bbox_2d_volume() > 3000:
                    candidates.append(det)
        if not candidates:
            continue

        frame = segmenter.segment(ImageDetections2D(image=obs.data, detections=candidates))
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
        for j, label_trace in enumerate(traces):
            if label_trace is None:
                continue
            label_dets = [det for det in frame if det.class_id == j]
            if label_dets:
                label_trace.detection_frames.append(
                    obs.derive(data=ImageDetections2D(image=obs.data, detections=label_dets))
                )

    results = [
        _finalize(
            q,
            identity=identities[j],
            ungrounded_best=ungrounded[j],
            rig=rig,
            require_pose=require_pose,
            policy=policy,
            cloud_mode=cloud_mode,
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
    cloud_mode: str,
    trace: LocalizeTrace | None,
) -> Localization | None:
    verified = [
        members
        for members in identity.groups
        if _max_score(members) >= policy.accept_score and _n_views(members) >= policy.min_views
    ]
    logger.info(
        f"verification {query!r}: "
        + ", ".join(
            f"score={_max_score(g):.2f} views={_n_views(g)} obs={len(g)}" for g in identity.groups
        )
    )
    if trace is not None:
        for members in verified:
            for det3d in members:
                trace.verified.append((det3d.ts, det3d))

    if not verified:
        if ungrounded_best is not None and ungrounded_best[0] >= policy.accept_score:
            if require_pose:
                logger.info(f"{query!r}: best candidate has no valid depth and require_pose is set")
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
