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

SigLIP frame retrieval, OWLv2 detection, EdgeTAM segmentation, then a lift to
3D through the rig's geometry. Two rules distinguish it from a best-crop
search: every verified instance is returned latest-seen first and positioned
by its latest sighting ("where is it now"), and every stage carries a score so
the answer can be empty rather than a guess.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.memory.embed import EmbedImages
from dimos.memory.transform import QualityWindow, peaks
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.memory.rig import Rig
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

logger = setup_logger()

_SCORE_CACHE_MAX = 8192

# One row per accepted sighting. Everything verification reports is a reduction
# over these columns, so no detection - and no image - is retained.
M_TS, M_SCORE, M_CX, M_CY, M_CZ, M_KX, M_KY, M_KZ = range(8)
M_WIDTH = 8


@dataclass
class IndexFrame:
    """What frame selection needs. The pixels stay in the color stream."""

    ts: float
    frame_id: str
    sharpness: float


@dataclass
class LocalizeTrace:
    """Artifacts a renderer needs; collected only when one is passed in."""

    detection_frames: list[Any] = field(default_factory=list)
    answers: list[list[Detection3DPC]] = field(default_factory=list)
    backdrop_ts: float | None = None
    first_match_ts: float | None = None


@dataclass
class Groups:
    """Sightings of one label grouped into objects, cumulative across calls.

    Parallel per-group arrays. The only geometry held is each group's fused
    union and its latest member's own cloud, which the answer's orientation
    comes from; per-sighting evidence is eight floats in a flat row.
    """

    centers: list[np.ndarray] = field(default_factory=list)  # running weighted center
    clouds: list[PointCloud2] = field(default_factory=list)  # fused union
    weights: list[np.ndarray | None] = field(default_factory=list)  # per fused point
    raw: list[float] = field(default_factory=list)  # raw points folded in so far
    latest: list[PointCloud2] = field(default_factory=list)
    latest_ts: list[float] = field(default_factory=list)
    members: list[list[float]] = field(default_factory=list)  # flat, M_WIDTH per row
    trace: list[list[Detection3DPC]] = field(default_factory=list)
    ingested: set[float] = field(default_factory=set)  # frame ts already lifted
    ungrounded: tuple[float, float] | None = None  # best (score, ts) with no depth

    def add(self, det: Detection3DPC, radius: float, voxel: float) -> int:
        """Assign one sighting to its object, folding its cloud in."""
        points = np.asarray(det.pointcloud.pointcloud.points)
        centroid = points.mean(axis=0)
        camera = (-det.transform).translation

        hit = next(
            (i for i, c in enumerate(self.centers) if np.linalg.norm(c - centroid) <= radius), -1
        )
        if hit < 0:
            hit = len(self.centers)
            self.centers.append(centroid)
            self.clouds.append(det.pointcloud)
            self.weights.append(None)
            self.raw.append(float(len(points)))
            self.latest.append(det.pointcloud)
            self.latest_ts.append(det.ts)
            self.members.append([])
            self.trace.append([])
        else:
            self._fuse(hit, det.pointcloud.ts, points, centroid, voxel)
            if det.ts > self.latest_ts[hit]:
                self.latest[hit] = det.pointcloud
                self.latest_ts[hit] = det.ts

        self.members[hit].extend(
            (det.ts, float(det.confidence), *centroid, camera.x, camera.y, camera.z)
        )
        return hit

    def _fuse(
        self, i: int, ts: float, points: np.ndarray, centroid: np.ndarray, voxel: float
    ) -> None:
        """Voxel-average a sighting into the group's union.

        Each fused point carries the raw points behind it, so the union is a
        weighted mean over every viewpoint rather than a mean of means, and the
        running center follows the same counts.
        """
        held = np.asarray(self.clouds[i].pointcloud.points)
        na, nb = self.raw[i], float(len(points))
        self.centers[i] = (na * self.centers[i] + nb * centroid) / (na + nb)
        self.raw[i] = na + nb
        merged = np.vstack([held, points])
        ts = max(self.clouds[i].ts, ts)
        frame = self.clouds[i].frame_id

        if voxel > 0:
            w = np.ones(len(merged))
            weights = self.weights[i]
            if weights is not None:
                w[: len(weights)] = weights
            _, inverse = np.unique(
                np.floor(merged / voxel).astype(np.int64), axis=0, return_inverse=True
            )
            wsum = np.zeros(inverse.max() + 1)
            np.add.at(wsum, inverse, w)
            psum = np.zeros((len(wsum), 3))
            np.add.at(psum, inverse, merged * w[:, None])
            merged = psum / wsum[:, None]
            self.weights[i] = wsum
        self.clouds[i] = PointCloud2.from_numpy(merged, frame_id=frame, timestamp=ts)

    def rows(self, i: int) -> np.ndarray:
        return np.asarray(self.members[i]).reshape(-1, M_WIDTH)


def _similarity(obs: Any) -> float:
    return float(obs.similarity)


def _settled(index: Stream[Any, Any], spacing: float) -> set[int]:
    """Ids left once sub-spacing duplicates are dropped.

    The index emits the sharpest frame per window from a fixed phase, so a
    camera that moves between windows leaves both the settled frame and the
    blurred one taken while it was still moving. Their poses differ, so the
    blurred one would pass as a second viewpoint and lift to a displaced cloud.
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


def _lift(
    detections: ImageDetections2D[Any], rig: Rig, policy: LocalizePolicy, plane: Any | None
) -> list[Detection3DPC]:
    """Gate a frame's lifted detections."""
    pose = rig.camera_pose(detections.ts, detections.image.frame_id)
    if pose is None:
        return []
    lifted = rig.lift(detections, plane)
    if lifted is None:
        return []
    camera = np.array([pose.position.x, pose.position.y, pose.position.z])

    valid: list[Detection3DPC] = []
    for det3d in lifted:
        points = np.asarray(det3d.pointcloud.pointcloud.points)
        if len(points) < policy.min_points:
            continue
        if float((points.max(axis=0) - points.min(axis=0)).max()) > policy.max_object_extent_m:
            continue
        if float(np.median(np.linalg.norm(points - camera, axis=1))) < policy.min_camera_range_m:
            continue
        if plane is not None:
            heights = rig.support_heights(detections.ts, plane, points)
            # A cloud hugging the support is a patch of the surface, not an object.
            if (
                float(np.quantile(heights, 0.05)) > policy.surface_patch_min_drop_m
                and float(np.quantile(heights, 0.95)) < policy.surface_patch_max_rise_m
            ):
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

    Built once per window and handed to every ``localize`` call on it. Only the
    pose, the embedding and the sharpness are kept; the pixels are re-read from
    the color stream for the handful of frames that reach the detector, so the
    index costs bytes per frame instead of a decoded image.
    """
    rig = rig or Rig.from_store(store)
    embedded: Stream[Any, Any] = (
        rig.color.after(t0)
        .before(t1)
        .filter(lambda obs: obs.data.brightness > 0.1)
        .transform(QualityWindow(lambda img: img.sharpness, window=1.0 / rig.embed_hz))
        .map(lambda obs: obs.derive(data=obs.data, pose=rig.index_pose(obs)))
        .filter(lambda obs: obs.pose is not None)
        .transform(EmbedImages(siglip))
        .map(
            lambda obs: obs.derive(
                data=IndexFrame(obs.ts, obs.data.frame_id, float(obs.data.sharpness))
            )
        )
        .materialize()
    )
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
    groups: dict[str, Groups] | None = None,
) -> list[Localization] | list[list[Localization]]:
    """Every verified 3D instance of *query*, latest-seen first.

    An empty list is a first-class answer: nothing reached the accept score, no
    support was confirmed from a second viewpoint, or the best candidate had no
    valid depth and ``require_pose`` holds. Coexisting instances of one label
    are all returned, each carrying ``ambiguity_margin`` against its rivals.

    A list *query* shares one detection pass and returns one list per label, in
    input order; ``trace`` then takes a list of the same length. The index, the
    rig and the models belong to the caller. A ``groups`` dict makes evidence
    cumulative across calls: frames already ingested for a label are skipped,
    and an object stays answerable after it leaves the window.
    """
    rig = rig or Rig.from_store(store)
    policy = policy or rig.default_localize_policy()

    queries = [query] if isinstance(query, str) else query
    traces: list[LocalizeTrace | None] = (
        list(trace) if isinstance(trace, list) else [trace] * len(queries)
    )
    state = [(groups if groups is not None else {}).setdefault(q, Groups()) for q in queries]

    index_count = index.count()
    settled = _settled(index, policy.settled_window_fraction / rig.embed_hz)
    source = index.filter(lambda obs: obs.id in settled)
    candidate_ids: set[int] = set()
    expanded: set[float] = set()
    anchor = np.zeros(2)
    anchor_count = 0

    for q in queries:
        query_embedding = siglip.embed_text(q)
        sightings: list[Any] = list(
            source.search(query_embedding)
            .order_by("ts")
            .transform(
                peaks(
                    key=_similarity,
                    prominence=policy.peak_prominence,
                    distance=policy.peak_distance_s,
                    width=policy.peak_width_s,
                )
            )
        )
        # A peak is never the window's last sample, and an instance's position
        # follows its latest sighting: the tail's best frame is one too.
        sightings.extend(
            source.after(sightings[-1].ts if sightings else 0.0).search(
                query_embedding, k=policy.tail_k
            )
        )
        for peak in sightings:
            anchor += cast("PoseTuple", peak.pose_tuple)[:2]
            anchor_count += 1
            candidate_ids.add(peak.id)
            if peak.ts in expanded:
                continue
            expanded.add(peak.ts)
            gathered: Stream[Any, Any] = source.near(
                peak.pose_stamped, radius=policy.verify_radius_m
            ).transform(QualityWindow(lambda f: f.sharpness, window=policy.verify_window_s))
            candidate_ids.update(obs.id for obs in gathered)
        logger.info(f"localize {q!r}: {len(sightings)} semantic peaks of {index_count} embedded")

    candidates = index.filter(lambda obs: obs.id in candidate_ids).order_by("ts")
    candidate_count = len(candidate_ids)
    logger.info(f"detection: {candidate_count} candidate frames for {len(queries)} labels")

    plane = None
    if candidate_count:
        from dimos.perception.memory.support_plane import fit_support_plane

        mean = anchor / anchor_count
        cell = (round(mean[0] / policy.plane_cell_m), round(mean[1] / policy.plane_cell_m))
        plane = rig.plane_cache.get(cell)
        if plane is None:
            stride = max(1, candidate_count // policy.plane_keyframes)
            plane = fit_support_plane(
                rig,
                [obs for i, obs in enumerate(candidates) if i % stride == 0][
                    : policy.plane_keyframes
                ],
            )
            if plane is not None:
                rig.plane_cache[cell] = plane

    floor = policy.candidate_floor
    cache = detector.score_cache

    def _detect(upstream: Iterator[Any]) -> Iterator[Any]:
        for obs in upstream:
            active = [j for j in range(len(queries)) if obs.ts not in state[j].ingested]
            if not active:
                continue
            keys = [(obs.ts, queries[j], floor) for j in active]
            img = rig.frame_at(obs) if any(k not in cache for k in keys) else None
            if img is not None:
                boxes, rows = detector.query_score_rows_batch([img], queries, threshold=floor)[0]
                for j, q in enumerate(queries):
                    keep = rows[:, j] >= floor
                    cache[(obs.ts, q, floor)] = (boxes[keep], rows[keep, j])
                    if len(cache) > _SCORE_CACHE_MAX:
                        cache.popitem(last=False)
            for key in keys:
                cache.move_to_end(key)
            for j in active:
                state[j].ingested.add(obs.ts)
            if not any(len(cache[key][1]) for key in keys):
                continue
            img = img if img is not None else rig.frame_at(obs)

            detections: list[Detection2DBBox] = []
            for j, key in zip(active, keys, strict=True):
                for box, score in zip(*cache[key], strict=True):
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
                group = state[det2d.class_id]
                best = group.ungrounded
                if det2d.track_id not in grounded and (best is None or det2d.confidence > best[0]):
                    group.ungrounded = (det2d.confidence, det2d.ts)
            for det3d in lifted:
                j = det3d.class_id
                slot = state[j].add(det3d, policy.cluster_radius_m, policy.fuse_voxel_m)
                if traces[j] is not None:
                    state[j].trace[slot].append(det3d)
                    if traces[j].first_match_ts is None:  # type: ignore[union-attr]
                        traces[j].first_match_ts = det3d.ts  # type: ignore[union-attr]
            for j, label_trace in enumerate(traces):
                label_dets = [det for det in frame if det.class_id == j]
                if label_trace is not None and label_dets:
                    label_trace.detection_frames.append(
                        obs.derive(data=ImageDetections2D(image=frame.image, detections=label_dets))
                    )
            yield obs

    candidates.transform(_detect).map(
        lambda obs: obs.derive(data=segmenter.segment(obs.data))
    ).transform(_ingest).drain()

    results = [
        _finalize(q, state[j], rig, require_pose, policy, traces[j]) for j, q in enumerate(queries)
    ]
    return results[0] if isinstance(query, str) else results


def _orientation(cloud: PointCloud2) -> tuple[float, float, float, float]:
    from scipy.spatial.transform import Rotation

    try:
        x, y, z, w = Rotation.from_matrix(np.asarray(cloud.oriented_bounding_box.R)).as_quat()
    except Exception:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(x), float(y), float(z), float(w))


def _finalize(
    query: str,
    group: Groups,
    rig: Rig,
    require_pose: bool,
    policy: LocalizePolicy,
    trace: LocalizeTrace | None,
) -> list[Localization]:
    rows = [group.rows(i) for i in range(len(group.centers))]
    views = [len(np.unique(np.round(r[:, M_KX : M_KZ + 1], 2), axis=0)) for r in rows]
    verified = [
        i
        for i, r in enumerate(rows)
        if r[:, M_SCORE].max() >= policy.accept_score and views[i] >= policy.min_views
    ]
    logger.info(
        f"verification {query!r}: "
        + ", ".join(
            f"score={r[:, M_SCORE].max():.2f} views={views[i]} obs={len(r)}"
            for i, r in enumerate(rows)
        )
    )

    if not verified:
        best = group.ungrounded
        if best is None or best[0] < policy.accept_score:
            return []
        if require_pose:
            logger.info(f"{query!r}: best candidate has no valid depth and require_pose is set")
            return []
        score, ts = best
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

    verified.sort(key=lambda i: rows[i][:, M_TS].max(), reverse=True)
    instances: list[Localization] = []
    for k, i in enumerate(verified):
        mine = rows[i]
        score = float(mine[:, M_SCORE].max())
        lo, hi = float(mine[:, M_TS].min()), float(mine[:, M_TS].max())
        rivals = [
            float(rows[j][:, M_SCORE].max())
            for j in verified
            if j != i
            and not (rows[j][:, M_TS].max() < lo or rows[j][:, M_TS].min() > hi)  # coexisting
        ]
        margin = score - max(rivals) if rivals else 1.0

        union = group.clouds[i]
        points = np.asarray(union.pointcloud.points)
        aabb_min, aabb_max = points.min(axis=0), points.max(axis=0)
        centroids = mine[:, M_CX : M_CZ + 1]
        offsets = mine[:, M_KX : M_KZ + 1] - centroids.mean(axis=0)  # center to camera
        norms = np.linalg.norm(offsets, axis=1)
        dirs = offsets[norms > 1e-6] / norms[norms > 1e-6, None]
        if len(dirs):
            octant = ((np.arctan2(dirs[:, 1], dirs[:, 0]) + math.pi) / (2 * math.pi) * 8).astype(
                int
            )
            coverage = len(set(octant % 8)) / 8.0
            axes = tuple(bool((np.abs(dirs[:, a]) > 0.3).any()) for a in range(3))
        else:
            coverage, axes = 0.0, (False, False, False)

        center = (aabb_min + aabb_max) / 2
        extent = np.maximum(aabb_max - aabb_min, 0.005)
        sigma = centroids.std(axis=0) if len(mine) > 1 else np.full(3, 0.01)
        if trace is not None:
            trace.answers.append(group.trace[i])
            if k == 0:
                trace.backdrop_ts = hi

        newest = mine[mine[:, M_TS].argmax()]
        instances.append(
            Localization(
                instance_id=f"query-{k}",
                semantic_score=score,
                identity_score=min(1.0, views[i] / 4.0),
                ambiguity_margin=margin,
                position_world_xyz=(
                    float(newest[M_CX]),
                    float(newest[M_CY]),
                    float(newest[M_CZ]),
                ),
                orientation_world_xyzw=_orientation(group.latest[i]),
                frame_id=rig.world_frame,
                support=Support(
                    center_xyz=tuple(float(v) for v in center),  # type: ignore[arg-type]
                    extent_xyz_m=tuple(float(v) for v in extent),  # type: ignore[arg-type]
                    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                    sigma_xyz_m=tuple(float(v) for v in sigma),  # type: ignore[arg-type]
                    coverage=coverage,
                    axes_observed=cast("tuple[bool, bool, bool]", axes),
                    frame_id=rig.world_frame,
                ),
                pose_timestamp=hi,
                geometry_timestamp=hi,
                last_seen_timestamp=hi,
                point_cloud=union,
                coverage=coverage,
                n_views=views[i],
                reason=(
                    "ambiguous_between_coexisting_candidates"
                    if margin < policy.refusal_margin
                    else None
                ),
            )
        )
    return instances
