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

"""Lightweight persistable localize evidence (no RGB / point clouds).

Written to ``audit/localize_evidence.json`` after a Dan localize pass so a later
``generate --with-localize --localize-evidence PATH`` can skip Dan and rebuild
clips from recording frames at stored sighting timestamps.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from dimos.evals.vqa.primitives.cross_view_identity import SightingView
from dimos.memory.cli.dataset import open_dataset
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.rig import Rig, window_tf
from dimos.perception.localize.types import Localization
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.evals.vqa.localize_recording import LocalizeWindowResult

logger = setup_logger()

EVIDENCE_VERSION = 1
# Reject reuse when the requested window drifts more than this from the dump.
WINDOW_MATCH_TOLERANCE_S = 0.05
# Nearest color frame when hydrating a sighting image.
SIGHTING_IMAGE_TOLERANCE_S = 0.15


class SightingEvidence(BaseModel):
    """One 2D observation timestamp + bbox (no pixels)."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    timestamp: float
    bbox: tuple[float, float, float, float]


class InstanceEvidence(BaseModel):
    """One verified localize instance with category name and sighting times."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    object_name: str = Field(min_length=1)
    instance_id: str = Field(min_length=1)
    position_world_xyz: tuple[float, float, float]
    first_seen: float
    n_views: int = Field(ge=1)
    semantic_score: float = Field(ge=0)
    last_seen_timestamp: float | None = None
    frame_id: str = "world"
    ambiguity_margin: float = 0.0
    sightings: tuple[SightingEvidence, ...] = ()


class LocalizeEvidenceDocument(BaseModel):
    """Disk form of a localize window — small enough to commit / reuse."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    version: int = EVIDENCE_VERSION
    dataset: str = Field(min_length=1)
    after: float
    before: float
    queries: tuple[str, ...] = ()
    instances: tuple[InstanceEvidence, ...] = ()


def evidence_from_window_result(
    result: LocalizeWindowResult,
    *,
    dataset: str,
    after: float,
    before: float,
) -> LocalizeEvidenceDocument:
    """Serialize hits / first-seen / sighting timestamps (+ bbox) from a live pass."""
    views_by_query = getattr(result, "views_by_query", {}) or {}
    first_seen = getattr(result, "first_seen_by_instance", {}) or {}
    instances: list[InstanceEvidence] = []
    for object_name, hit in result.named_hits:
        by_id = views_by_query.get(object_name) or {}
        views = by_id.get(hit.instance_id) or ()
        sightings = tuple(
            SightingEvidence(timestamp=float(view.timestamp), bbox=tuple(view.bbox))  # type: ignore[arg-type]
            for view in views
        )
        earliest = first_seen.get(hit.instance_id)
        if earliest is None and sightings:
            earliest = min(s.timestamp for s in sightings)
        if earliest is None:
            earliest = float(hit.last_seen_timestamp)
        instances.append(
            InstanceEvidence(
                object_name=object_name,
                instance_id=str(hit.instance_id),
                position_world_xyz=tuple(hit.position_world_xyz),  # type: ignore[arg-type]
                first_seen=float(earliest),
                n_views=int(hit.n_views),
                semantic_score=float(hit.semantic_score),
                last_seen_timestamp=float(hit.last_seen_timestamp),
                frame_id=str(hit.frame_id),
                ambiguity_margin=float(hit.ambiguity_margin),
                sightings=sightings,
            )
        )
    return LocalizeEvidenceDocument(
        dataset=str(dataset),
        after=float(after),
        before=float(before),
        queries=tuple(result.queries),
        instances=tuple(instances),
    )


def write_localize_evidence(path: Path, document: LocalizeEvidenceDocument) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(document.model_dump_json(indent=2) + "\n", encoding="utf-8")


def load_localize_evidence(path: Path) -> LocalizeEvidenceDocument:
    return LocalizeEvidenceDocument.model_validate_json(path.read_text(encoding="utf-8"))


def assert_evidence_matches_window(
    document: LocalizeEvidenceDocument,
    *,
    dataset: str,
    after: float,
    before: float,
    localize_queries: Sequence[str] = (),
    tolerance_s: float = WINDOW_MATCH_TOLERANCE_S,
) -> None:
    """Raise ``ValueError`` when evidence cannot safely replace a Dan pass."""
    if document.version != EVIDENCE_VERSION:
        raise ValueError(
            f"localize evidence version {document.version} != supported {EVIDENCE_VERSION}"
        )
    if _dataset_key(document.dataset) != _dataset_key(dataset):
        raise ValueError(
            f"localize evidence dataset {document.dataset!r} does not match request {dataset!r}"
        )
    if abs(document.after - after) > tolerance_s or abs(document.before - before) > tolerance_s:
        raise ValueError(
            "localize evidence window "
            f"[{document.after}, {document.before}] does not match request "
            f"[{after}, {before}] (tolerance {tolerance_s}s)"
        )
    if localize_queries:
        available = set(document.queries) | {inst.object_name for inst in document.instances}
        missing = [q for q in localize_queries if q not in available]
        if missing:
            raise ValueError(
                f"localize evidence is missing requested queries: {missing}; "
                f"have {sorted(available)}"
            )


def _dataset_key(dataset: str) -> str:
    return str(Path(dataset).expanduser().resolve()) if Path(dataset).exists() else str(dataset)


def _stub_localization(instance: InstanceEvidence) -> Localization:
    cloud = PointCloud2.from_numpy(
        np.zeros((1, 3), dtype=np.float32),
        frame_id=instance.frame_id,
        timestamp=float(instance.last_seen_timestamp or instance.first_seen),
    )
    return Localization(
        instance_id=instance.instance_id,
        semantic_score=instance.semantic_score,
        ambiguity_margin=instance.ambiguity_margin,
        position_world_xyz=instance.position_world_xyz,
        frame_id=instance.frame_id,
        last_seen_timestamp=float(instance.last_seen_timestamp or instance.first_seen),
        point_cloud=cloud,
        n_views=instance.n_views,
    )


def _nearest_frame(
    color_frames: Sequence[tuple[float, Image]],
    timestamp: float,
    *,
    tolerance_s: float = SIGHTING_IMAGE_TOLERANCE_S,
) -> Image | None:
    if not color_frames:
        return None
    ts, image = min(color_frames, key=lambda row: abs(row[0] - timestamp))
    if abs(ts - timestamp) > tolerance_s:
        return None
    return image


def _nearest_frame_with_camera(
    color_frames: Sequence[tuple[float, Image]],
    color_frames_by_camera: Mapping[str, Sequence[tuple[float, Image]]] | None,
    timestamp: float,
    *,
    tolerance_s: float = SIGHTING_IMAGE_TOLERANCE_S,
) -> tuple[Image, str] | None:
    """Nearest RGB within tolerance; prefer an explicit per-camera stream when present."""
    best: tuple[float, Image, str] | None = None
    streams: list[tuple[str, Sequence[tuple[float, Image]]]] = []
    if color_frames_by_camera:
        streams.extend((frame_id, frames) for frame_id, frames in color_frames_by_camera.items())
    elif color_frames:
        streams.append(("", color_frames))
    for frame_id, frames in streams:
        if not frames:
            continue
        ts, image = min(frames, key=lambda row: abs(row[0] - timestamp))
        gap = abs(ts - timestamp)
        if gap > tolerance_s:
            continue
        if best is None or gap < best[0]:
            cam = frame_id or str(getattr(image, "frame_id", "") or "")
            best = (gap, image, cam)
    if best is None:
        return None
    return best[1], best[2]


def hydrate_views_and_hits(
    document: LocalizeEvidenceDocument,
    color_frames: Sequence[tuple[float, Image]],
    *,
    localize_queries: Sequence[str] = (),
    color_frames_by_camera: Mapping[str, Sequence[tuple[float, Image]]] | None = None,
) -> tuple[
    tuple[tuple[str, Localization], ...],
    dict[str, float],
    dict[str, dict[str, tuple[SightingView, ...]]],
]:
    """Rebuild named_hits / first_seen / views_by_query; pull RGB from ``color_frames``."""
    wanted = {q.strip() for q in localize_queries if q.strip()} or None
    named: list[tuple[str, Localization]] = []
    first_seen: dict[str, float] = {}
    views_by_query: dict[str, dict[str, tuple[SightingView, ...]]] = {}
    skipped_sightings = 0
    for instance in document.instances:
        if wanted is not None and instance.object_name not in wanted:
            continue
        hit = _stub_localization(instance)
        named.append((instance.object_name, hit))
        first_seen[instance.instance_id] = float(instance.first_seen)
        views: list[SightingView] = []
        for sighting in instance.sightings:
            matched = _nearest_frame_with_camera(
                color_frames, color_frames_by_camera, sighting.timestamp
            )
            if matched is None:
                skipped_sightings += 1
                continue
            image, camera_frame = matched
            views.append(
                SightingView(
                    query=instance.object_name,
                    instance_id=instance.instance_id,
                    image=image,
                    bbox=sighting.bbox,
                    confidence=instance.semantic_score,
                    timestamp=float(sighting.timestamp),
                    n_views=instance.n_views,
                    semantic_score=instance.semantic_score,
                    camera_frame=camera_frame,
                )
            )
        views_by_query.setdefault(instance.object_name, {})[instance.instance_id] = tuple(views)
    if skipped_sightings:
        logger.warning(
            f"localize evidence hydrate skipped {skipped_sightings} sighting(s) "
            f"(no color frame within {SIGHTING_IMAGE_TOLERANCE_S}s)"
        )
    return tuple(named), first_seen, views_by_query


@contextmanager
def localize_window_from_evidence(
    dataset: str | Path,
    document: LocalizeEvidenceDocument,
    after: float,
    before: float,
    *,
    localize_queries: Sequence[str] = (),
) -> Iterator[LocalizeWindowResult]:
    """Open the recording, load window color frames, hydrate lightweight evidence."""
    from dimos.evals.vqa.localize_recording import (
        LocalizeWindowResult,
        color_frames_by_camera_from_rig,
    )

    assert_evidence_matches_window(
        document,
        dataset=str(dataset),
        after=after,
        before=before,
        localize_queries=localize_queries,
    )
    store = open_dataset(dataset)
    store.start()
    try:
        rig = Rig.from_store(store)
        rig.tf = window_tf(store, after, before)
        if rig.lidar is not None:
            rig.lidar = rig.lidar.after(after).before(before)

        by_camera = color_frames_by_camera_from_rig(rig, after, before)
        color_frames = next(iter(by_camera.values())) if by_camera else ()

        named_hits, first_seen, views_by_query = hydrate_views_and_hits(
            document,
            color_frames,
            localize_queries=localize_queries,
            color_frames_by_camera=by_camera,
        )
        queries = (
            tuple(document.queries)
            if document.queries
            else tuple(sorted({name for name, _ in named_hits}))
        )
        logger.info(
            f"localize evidence reuse: {len(named_hits)} hit(s), "
            f"{len(color_frames)} color frame(s), queries={list(queries)}"
        )
        yield LocalizeWindowResult(
            named_hits=named_hits,
            rig=rig,
            queries=queries,
            first_seen_by_instance=first_seen,
            color_frames=tuple(color_frames),
            color_frames_by_camera=dict(by_camera),
            views_by_query=views_by_query,
        )
    finally:
        store.stop()
