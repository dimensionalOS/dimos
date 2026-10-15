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

"""Query-time object localization: text prompts to 3D instances.

Embedder frame retrieval, detection, segmentation, then a lift to 3D through
the camera's aligned depth or the rig's lidar, and geometric association. The
models are the caller's, behind the interfaces in ``dandetect``. Every verified
instance is returned latest-seen first and positioned by its latest sighting,
and every stage carries a score so the answer can be empty rather than a guess.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.embed import EmbedImages
from dimos.memory.transform import QualityWindow, peaks
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.localize.association import (
    M_CX,
    M_CY,
    M_CZ,
    M_KX,
    M_KZ,
    M_SCORE,
    M_TS,
    FrameKey,
    Groups,
)
from dimos.perception.localize.rig import Rig
from dimos.perception.localize.support_plane import (
    SupportPlane,
    fit_support_plane,
)
from dimos.perception.localize.types import Localization, LocalizeParams
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory.stream import Stream
    from dimos.models.embedding.base import EmbeddingModel
    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
    from dimos.perception.localize.dandetect import Detector, Segmenter

logger = setup_logger()


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


def _similarity(obs: Any) -> float:
    return float(obs.similarity)


def _per_camera(counts: dict[str, int]) -> str:
    return ", ".join(f"{frame} {n}" for frame, n in counts.items())


def _box_key(box: Any) -> tuple[int, ...]:
    return tuple(round(float(v)) for v in box)


def _settled(index: Stream[Any, Any], spacing: float) -> set[int]:
    """Ids left once one camera's sub-spacing duplicates are dropped.

    The index emits the sharpest frame per window from a fixed phase, so a
    camera that moves between windows leaves both the settled frame and the
    blurred one taken while it was still moving. Their poses differ, so the
    blurred one would pass as a second viewpoint and lift to a displaced cloud.
    Frames of different cameras are never duplicates.
    """
    ids: set[int] = set()
    last: dict[str | None, tuple[int, float, float]] = {}
    for obs in index.order_by("ts"):
        camera = obs.tags.get("camera")
        last_id, last_ts, last_sharpness = last.get(camera, (-1, -math.inf, -1.0))
        sharpness = float(obs.data.sharpness)
        if obs.ts - last_ts < spacing:
            if sharpness <= last_sharpness:
                continue
            ids.discard(last_id)
        ids.add(obs.id)
        last[camera] = (obs.id, obs.ts, sharpness)
    return ids


def _lift(
    detections: ImageDetections2D[Any], rig: Rig, params: LocalizeParams, plane: SupportPlane | None
) -> list[Detection3DPC]:
    """Lift a frame's detections through its camera's geometry and gate them."""
    transform = rig.world_to_optical(detections.ts, detections.image.frame_id)
    lifted = rig.lift(detections, transform) if transform is not None else None
    if transform is None or lifted is None:
        return []
    camera = (-transform).translation.to_numpy()

    valid: list[Detection3DPC] = []
    for det3d in lifted:
        points = np.asarray(det3d.pointcloud.pointcloud.points)
        if len(points) < params.min_points:
            continue
        if float((points.max(axis=0) - points.min(axis=0)).max()) > params.max_object_extent_m:
            continue
        if float(np.median(np.linalg.norm(points - camera, axis=1))) < params.min_camera_range_m:
            continue
        if plane is not None:
            heights = plane.height_above(points)
            # A cloud hugging the support is a patch of the surface, not an object.
            if (
                float(np.quantile(heights, 0.05)) > params.surface_patch_min_drop_m
                and float(np.quantile(heights, 0.95)) < params.surface_patch_max_rise_m
            ):
                continue
        valid.append(det3d)
    return valid


def index_frames(rig: Rig, feed: Stream[Any, Any]) -> Stream[Any, Any]:
    """One camera's frames worth embedding: lit, sharpest per slot, world-posed, camera-tagged."""
    return (
        feed.filter(lambda obs: obs.data.brightness > 0.1)
        .transform(QualityWindow(lambda img: img.sharpness, window=1.0 / rig.embed_hz))
        .map(
            lambda obs: obs.derive(
                data=obs.data,
                pose=rig.camera_pose(obs.ts, obs.data.frame_id),
                tags={**obs.tags, "camera": obs.data.frame_id},
            )
        )
        .filter(lambda obs: obs.pose is not None)
    )


def embed_index(rig: Rig, embedder: EmbeddingModel, t0: float, t1: float) -> Stream[Any, Any]:
    """Embedded, world-posed frame index over ``[t0, t1]``, every camera merged in time.

    Built once per window and handed to every ``localize`` call on it. Only the
    pose, the embedding and the sharpness are kept; the pixels are re-read from
    the color stream for the frames that reach the detector.
    """
    first, *rest = [
        index_frames(rig, cam.color.after(t0).before(t1)) for cam in rig.cameras.values()
    ]
    feed = (
        first.transform(lambda upstream: heapq.merge(upstream, *rest, key=lambda obs: obs.ts))
        if rest
        else first
    )
    embedded: Stream[Any, Any] = (
        feed.transform(EmbedImages(embedder))
        .map(
            lambda obs: obs.derive(
                data=IndexFrame(obs.ts, obs.data.frame_id, float(obs.data.sharpness))
            )
        )
        .materialize()
    )
    logger.info(f"index: {embedded.count()} frames embedded over {t1 - t0:.1f}s")
    return embedded


def retrieve(
    queries: list[str],
    *,
    index: Stream[Any, Any],
    rig: Rig,
    embedder: EmbeddingModel,
    params: LocalizeParams,
) -> Stream[Any, Any]:
    """The index frames worth a detector pass for ``queries``, in time order.

    Per query and camera: the embedder's similarity peaks and the tail's best
    frame, each joined by the sharpest frames posed within ``verify_radius_m``.
    """
    index_count = index.count()
    settled = _settled(index, params.settled_window_fraction / rig.embed_hz)
    source = index.filter(lambda obs: obs.id in settled)
    feeds = {frame: source.tags(camera=frame) for frame in rig.cameras}

    candidate_ids: set[int] = set()
    expanded: set[FrameKey] = set()
    for q in queries:
        query_embedding = embedder.embed_text(q)
        sightings: list[Any] = []
        found_by: dict[str, int] = {}
        for frame, feed in feeds.items():
            found: list[Any] = list(
                feed.search(query_embedding)
                .order_by("ts")
                .transform(
                    peaks(
                        key=_similarity,
                        prominence=params.peak_prominence,
                        distance=params.peak_distance_s,
                        width=params.peak_width_s,
                    )
                )
            )
            # A peak is never the window's last sample, and an instance's position
            # follows its latest sighting: the tail's best frame is one too.
            found.extend(
                feed.after(found[-1].ts if found else 0.0).search(query_embedding, k=params.tail_k)
            )
            sightings.extend(found)
            found_by[frame] = len(found)
        for peak in sightings:
            candidate_ids.add(peak.id)
            peak_key = (peak.tags["camera"], peak.ts)
            if peak_key in expanded:
                continue
            expanded.add(peak_key)
            for feed in feeds.values():
                gathered: Stream[Any, Any] = feed.near(
                    peak.pose_stamped, radius=params.verify_radius_m
                ).transform(QualityWindow(lambda f: f.sharpness, window=params.verify_window_s))
                candidate_ids.update(obs.id for obs in gathered)
        logger.info(
            f"localize {q!r}: {len(sightings)} semantic peaks of {index_count} embedded "
            f"({_per_camera(found_by)})"
        )

    candidates = index.filter(lambda obs: obs.id in candidate_ids).order_by("ts")
    per_camera = dict.fromkeys(feeds, 0)
    for obs in candidates:
        per_camera[obs.tags["camera"]] += 1
    logger.info(
        f"detection: {len(candidate_ids)} candidate frames for {len(queries)} labels "
        f"({_per_camera(per_camera)})"
    )
    return candidates


def support_planes(
    rig: Rig, candidates: Stream[Any, Any], params: LocalizeParams
) -> dict[str, SupportPlane | None]:
    """Each camera's support plane, from the call's candidates; it must exist before any lift."""
    by_camera: dict[str, list[Any]] = {frame: [] for frame in rig.cameras}
    for obs in candidates:
        by_camera[obs.tags["camera"]].append(obs)

    planes: dict[str, SupportPlane | None] = {}
    for frame, keyframes in by_camera.items():
        if not keyframes:
            continue
        stride = max(1, len(keyframes) // params.plane_keyframes)
        fit_from = keyframes[::stride][: params.plane_keyframes]
        # one plane per camera and area: a moving camera meets other surfaces
        x, y = np.mean([obs.pose_tuple[:2] for obs in fit_from], axis=0)
        key = (frame, round(float(x) / params.plane_cell_m), round(float(y) / params.plane_cell_m))
        if key not in rig.plane_cache:
            plane = fit_support_plane(rig, fit_from, frame)
            if plane is not None:
                rig.plane_cache[key] = plane
        planes[frame] = rig.plane_cache.get(key)
    return planes


def detect(
    candidates: Stream[Any, Any],
    queries: list[str],
    *,
    rig: Rig,
    detector: Detector,
    state: list[Groups],
    params: LocalizeParams,
    pending: dict[FrameKey, list[int]],
) -> Stream[Any, Any]:
    """Each candidate frame's boxes per query, from one detector pass per frame.

    A frame every query has read under ``params`` is skipped, and so is a box
    already folded into a query's groups. A frame left without boxes is marked
    read here; any other goes into ``pending`` for :func:`associate` to mark.
    """
    floor = params.candidate_floor

    def _frames(upstream: Iterator[Any]) -> Iterator[Any]:
        for obs in upstream:
            frame_key = (obs.tags["camera"], obs.ts)
            active = [
                j for j, group in enumerate(state) if (frame_key, params) not in group.ingested
            ]
            if not active:
                continue
            img = rig.frame_at(obs)
            boxes, rows = detector.query_score_rows(img, queries, threshold=floor)

            detections: list[Detection2DBBox] = []
            for j in active:
                keep = rows[:, j] >= floor
                for box, score in zip(boxes[keep], rows[keep, j], strict=True):
                    if (frame_key, _box_key(box)) in state[j].folded:
                        continue
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
            if not detections:
                for j in active:
                    state[j].ingested.add((frame_key, params))
                continue
            pending[frame_key] = active
            yield obs.derive(data=ImageDetections2D(image=img, detections=detections))

    return candidates.transform(_frames)


def segment(detections: Stream[Any, Any], segmenter: Segmenter) -> Stream[Any, Any]:
    """Each frame's boxes as masks, carried with the detector's boxes they came from."""
    return detections.map(lambda obs: obs.derive(data=(obs.data, segmenter.segment(obs.data))))


def associate(
    frames: Stream[Any, Any],
    *,
    rig: Rig,
    params: LocalizeParams,
    planes: dict[str, SupportPlane | None],
    state: list[Groups],
    pending: dict[FrameKey, list[int]],
    traces: list[LocalizeTrace | None],
) -> Stream[Any, Any]:
    """Each segmented frame lifted to 3D, folded into its queries' groups, then marked read."""

    def _frames(upstream: Iterator[Any]) -> Iterator[Any]:
        for obs in upstream:
            boxes, frame = obs.data
            frame_key = (obs.tags["camera"], obs.ts)
            for det3d in _lift(frame, rig, params, planes.get(frame_key[0])):
                j = det3d.class_id
                slot = state[j].add(det3d, params.cluster_radius_m, params.fuse_voxel_m)
                state[j].folded.add((frame_key, _box_key(boxes.detections[det3d.track_id].bbox)))
                if traces[j] is not None:
                    state[j].trace[slot].append(det3d)
            for j, label_trace in enumerate(traces):
                label_dets = [det for det in frame if det.class_id == j]
                if label_trace is not None and label_dets:
                    label_trace.detection_frames.append(
                        obs.derive(data=ImageDetections2D(image=frame.image, detections=label_dets))
                    )
            # read only once its sightings are in, so a frame a call failed on is read again
            for j in pending.pop(frame_key):
                state[j].ingested.add((frame_key, params))
            yield obs

    return frames.transform(_frames)


def localize(
    queries: list[str],
    *,
    index: Stream[Any, Any],
    rig: Rig,
    embedder: EmbeddingModel,
    detector: Detector,
    segmenter: Segmenter,
    params: LocalizeParams,
    traces: list[LocalizeTrace] | None = None,
    groups: dict[str, Groups] | None = None,
) -> list[list[Localization]]:
    """Every verified 3D instance of each query, latest-seen first, one list per query.

    The queries share one detection pass. An empty list is a first-class
    answer: nothing reached the accept score or no support was confirmed from
    a second viewpoint. Coexisting instances of one label are all returned,
    each carrying ``ambiguity_margin`` against its rivals. ``traces``, one per
    query, collect what a renderer needs. A ``groups`` dict makes evidence
    cumulative across calls: frames already read for a label under the same
    params are skipped, a box is lifted in once, and an object stays answerable
    after it leaves the window. A frame counts as read once its sightings are
    in, so a frame a failed call left behind is read again.
    """
    label_traces: list[LocalizeTrace | None] = (
        list(traces) if traces is not None else [None] * len(queries)
    )
    state = [(groups if groups is not None else {}).setdefault(q, Groups()) for q in queries]

    candidates = retrieve(queries, index=index, rig=rig, embedder=embedder, params=params)
    planes = support_planes(rig, candidates, params)
    # detect hands each frame with boxes to associate, which marks it read
    pending: dict[FrameKey, list[int]] = {}
    detections = detect(
        candidates, queries, rig=rig, detector=detector, state=state, params=params, pending=pending
    )
    associate(
        segment(detections, segmenter),
        rig=rig,
        params=params,
        planes=planes,
        state=state,
        pending=pending,
        traces=label_traces,
    ).drain()

    return [_finalize(q, state[j], rig, params, label_traces[j]) for j, q in enumerate(queries)]


def _finalize(
    query: str,
    group: Groups,
    rig: Rig,
    params: LocalizeParams,
    trace: LocalizeTrace | None,
) -> list[Localization]:
    rows = [group.rows(i) for i in range(len(group.centers))]
    views = [len(np.unique(np.round(r[:, M_KX : M_KZ + 1], 2), axis=0)) for r in rows]
    verified = [
        i
        for i, r in enumerate(rows)
        if r[:, M_SCORE].max() >= params.accept_score and views[i] >= params.min_views
    ]
    logger.info(
        f"verification {query!r}: "
        + ", ".join(
            f"score={r[:, M_SCORE].max():.2f} views={views[i]} obs={len(r)}"
            for i, r in enumerate(rows)
        )
    )

    verified.sort(key=lambda i: rows[i][:, M_TS].max(), reverse=True)
    instances: list[Localization] = []
    for i in verified:
        mine = rows[i]
        score = float(mine[:, M_SCORE].max())
        lo, hi = float(mine[:, M_TS].min()), float(mine[:, M_TS].max())
        rivals = [
            float(rows[j][:, M_SCORE].max())
            for j in verified
            if j != i
            and not (rows[j][:, M_TS].max() < lo or rows[j][:, M_TS].min() > hi)  # coexisting
        ]
        if trace is not None:
            trace.answers.append(group.trace[i])
        newest = mine[mine[:, M_TS].argmax()]
        instances.append(
            Localization(
                instance_id=group.ids[i],
                semantic_score=score,
                ambiguity_margin=score - max(rivals) if rivals else 1.0,
                position_world_xyz=(float(newest[M_CX]), float(newest[M_CY]), float(newest[M_CZ])),
                frame_id=rig.world_frame,
                last_seen_timestamp=hi,
                point_cloud=group.clouds[i],
                n_views=views[i],
            )
        )
    return instances
