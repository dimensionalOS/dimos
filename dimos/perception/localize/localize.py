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
models are the caller's, behind the interfaces in ``dandetect``. A window is
answered only from sightings inside it; ``memento`` keeps every sighting, so a
span read before is answered from there and only the unread gap is detected.
Every verified instance is returned latest-seen first and positioned by its
latest sighting, and every stage carries a score so the answer can be empty
rather than a guess.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import heapq
import json
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.embed import EmbedImages
from dimos.memory.transform import QualityWindow, peaks
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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
    Groups,
)
from dimos.perception.localize.memento import Gap, Memento, Reader, Sighting, Span
from dimos.perception.localize.rig import Rig
from dimos.perception.localize.support_plane import (
    SupportPlane,
    fit_support_plane,
)
from dimos.perception.localize.types import Localization, LocalizeParams
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

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
    """Artifacts a renderer needs; collected only when one is passed in.

    ``answers`` holds each answer's lifted detections in this process, ``spans``
    its first and last sighting.
    """

    detection_frames: list[Any] = field(default_factory=list)
    answers: list[list[Detection3DPC]] = field(default_factory=list)
    spans: list[Span] = field(default_factory=list)


def _similarity(obs: Any) -> float:
    return float(obs.similarity)


def _per_camera(counts: dict[str, int]) -> str:
    return ", ".join(f"{frame} {n}" for frame, n in counts.items())


def _inside(gap: Gap) -> Callable[[Any], bool]:
    return lambda obs: obs.ts in gap


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
    gaps: dict[str, Gap],
    *,
    index: Stream[Any, Any],
    rig: Rig,
    embedder: EmbeddingModel,
    params: LocalizeParams,
) -> Stream[Any, Any]:
    """The index frames worth a detector pass for each query's unread gap, in time order.

    Per query and camera, inside the query's gap: the embedder's similarity
    peaks and the tail's best frame, each joined by the sharpest frames posed
    within ``verify_radius_m``.
    """
    index_count = index.count()
    settled = _settled(index, params.settled_window_fraction / rig.embed_hz)
    source = index.filter(lambda obs: obs.id in settled)

    candidate_ids: set[int] = set()
    expanded: set[tuple[str, float, Gap]] = set()
    for q, gap in gaps.items():
        feeds = {frame: source.tags(camera=frame).filter(_inside(gap)) for frame in rig.cameras}
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
            peak_key = (peak.tags["camera"], peak.ts, gap)
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
    per_camera = dict.fromkeys(rig.cameras, 0)
    for obs in candidates:
        per_camera[obs.tags["camera"]] += 1
    logger.info(
        f"detection: {len(candidate_ids)} candidate frames for {len(gaps)} labels "
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
    gaps: dict[str, Gap],
    rig: Rig,
    detector: Detector,
    params: LocalizeParams,
) -> Stream[Any, Any]:
    """Each candidate frame's boxes for the queries whose unread gap holds it.

    One detector pass per frame scores every query; a box is kept only for a
    query that has not read the frame.
    """
    floor = params.candidate_floor

    def _frames(upstream: Iterator[Any]) -> Iterator[Any]:
        for obs in upstream:
            active = [
                j
                for j, q in enumerate(queries)
                if q in gaps and obs.ts in gaps[q] and queries.index(q) == j
            ]
            if not active:
                continue
            img = rig.frame_at(obs)
            boxes, rows = detector.query_score_rows(img, queries, threshold=floor)

            detections: list[Detection2DBBox] = []
            for j in active:
                keep = rows[:, j] >= floor
                for box, score in zip(boxes[keep], rows[keep, j], strict=True):
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

    return candidates.transform(_frames)


def segment(detections: Stream[Any, Any], segmenter: Segmenter) -> Stream[Any, Any]:
    """Each frame's boxes as masks."""
    return detections.map(lambda obs: obs.derive(data=segmenter.segment(obs.data)))


def lift(
    frames: Stream[Any, Any],
    queries: list[str],
    *,
    rig: Rig,
    params: LocalizeParams,
    planes: dict[str, SupportPlane | None],
    traces: list[LocalizeTrace | None],
) -> Iterator[tuple[str, Sighting]]:
    """Each segmented frame lifted to 3D: one sighting per surviving mask, under its query.

    A sighting carries its lifted detection only when a trace will render it.
    """
    keep = any(trace is not None for trace in traces)
    for obs in frames:
        frame = obs.data
        camera = obs.tags["camera"]
        for det3d in _lift(frame, rig, params, planes.get(camera)):
            viewpoint = (-det3d.transform).translation
            yield (
                queries[det3d.class_id],
                Sighting(
                    ts=obs.ts,
                    camera=camera,
                    score=float(det3d.confidence),
                    points=np.asarray(det3d.pointcloud.pointcloud.points),
                    viewpoint=(viewpoint.x, viewpoint.y, viewpoint.z),
                    detection=det3d if keep else None,
                ),
            )
        for j, trace in enumerate(traces):
            label_dets = [det for det in frame if det.name == queries[j]]
            if trace is not None and label_dets:
                trace.detection_frames.append(
                    obs.derive(data=ImageDetections2D(image=frame.image, detections=label_dets))
                )


def localize(
    queries: list[str],
    *,
    index: Stream[Any, Any],
    window: Span,
    head: float,
    rig: Rig,
    embedder: EmbeddingModel,
    detector: Detector,
    segmenter: Segmenter,
    params: LocalizeParams,
    memento: Memento | None = None,
    traces: list[LocalizeTrace] | None = None,
) -> list[list[Localization]]:
    """Every verified 3D instance of each query seen inside ``window``, latest-seen first.

    One list per query; the queries share one detection pass. ``window`` is the
    closed span the answer covers, ``index`` its embedded frames and ``head``
    the newest frame there is. An empty list is a first-class answer: nothing
    in the window reached the accept score or was confirmed from a second
    viewpoint. Coexisting instances of one label are all returned, each
    carrying ``ambiguity_margin`` against its rivals, and an object that moved
    inside the window is an instance per place. ``memento`` keeps every
    sighting read: a span a label has read under the same params, the answer
    gates aside, is answered from it with no detector pass, so a window
    overlapping earlier ones detects only its unread gap. ``traces``, one per
    query, collect what a renderer needs.
    """
    label_traces: list[LocalizeTrace | None] = (
        list(traces) if traces is not None else [None] * len(queries)
    )

    def read(gaps: dict[str, Gap]) -> dict[str, list[Sighting]]:
        candidates = retrieve(gaps, index=index, rig=rig, embedder=embedder, params=params)
        planes = support_planes(rig, candidates, params)
        detections = detect(
            candidates, queries, gaps=gaps, rig=rig, detector=detector, params=params
        )
        found: dict[str, list[Sighting]] = {q: [] for q in gaps}
        for label, sighting in lift(
            segment(detections, segmenter),
            queries,
            rig=rig,
            params=params,
            planes=planes,
            traces=label_traces,
        ):
            found[label].append(sighting)
        return found

    return answer_window(
        queries,
        memento=memento if memento is not None else Memento(),
        key=memory_key(params),
        window=window,
        head=head,
        read=read,
        params=params,
        world_frame=rig.world_frame,
        traces=label_traces,
    )


def answer_window(
    queries: list[str],
    *,
    memento: Memento,
    key: str,
    window: Span,
    head: float,
    read: Reader,
    params: LocalizeParams,
    world_frame: str,
    traces: list[LocalizeTrace | None],
) -> list[list[Localization]]:
    """Each query's verified instances, built only from its sightings inside ``window``.

    ``read`` runs on the unread gaps alone; everything else comes from ``memento``.
    """
    found, gaps = memento.recall(queries, key, window, head, read)
    for q in queries:
        logger.info(
            f"memento {q!r}: read {gaps[q].seconds:.1f}s of the {window[1] - window[0]:.1f}s "
            f"window, {len(found[q])} sightings in it"
        )
    return [_finalize(q, found[q], params, world_frame, traces[j]) for j, q in enumerate(queries)]


def memory_key(params: LocalizeParams) -> str:
    """What a span counts as read under: the params without the answer gates, as a digest."""
    text = json.dumps(asdict(params.read_key()), sort_keys=True)
    return hashlib.blake2s(text.encode(), digest_size=8).hexdigest()


def _finalize(
    query: str,
    sightings: list[Sighting],
    params: LocalizeParams,
    world_frame: str,
    trace: LocalizeTrace | None,
) -> list[Localization]:
    group = Groups()
    for sighting in sightings:
        group.add(sighting, params.cluster_radius_m, params.fuse_voxel_m)
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
            trace.answers.append(
                [s.detection for s in group.sightings[i] if s.detection is not None]
            )
            trace.spans.append((lo, hi))
        newest = mine[mine[:, M_TS].argmax()]
        instances.append(
            Localization(
                instance_id=_instance_id(query, group.sightings[i][0]),
                semantic_score=score,
                ambiguity_margin=score - max(rivals) if rivals else 1.0,
                position_world_xyz=(float(newest[M_CX]), float(newest[M_CY]), float(newest[M_CZ])),
                frame_id=world_frame,
                first_seen_timestamp=lo,
                last_seen_timestamp=hi,
                point_cloud=PointCloud2.from_numpy(
                    group.clouds[i], frame_id=world_frame, timestamp=hi
                ),
                n_views=views[i],
            )
        )
    return instances


def _instance_id(query: str, first: Sighting) -> str:
    """An instance named by its label and earliest sighting, so a repeated window names it alike."""
    return hashlib.blake2s(f"{query}:{first.id}".encode(), digest_size=4).hexdigest()
