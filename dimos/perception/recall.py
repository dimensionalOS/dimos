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

"""CLIP-based recall over recorded frames."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import hashlib
from itertools import groupby
from math import floor
import re
from typing import TYPE_CHECKING, Any, cast

from dimos.memory2.embed import EmbedImages
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.type.observation import EmbeddedObservation

logger = setup_logger()

DEFAULT_CLIP_MODEL = "openai/clip-vit-base-patch32"


def index_stream_name(model_id: str) -> str:
    slug = re.sub(r"[^0-9A-Za-z]+", "_", model_id).strip("_")
    suffix = (
        ""
        if model_id == DEFAULT_CLIP_MODEL
        else f"_{hashlib.sha256(model_id.encode()).hexdigest()[:16]}"
    )
    return f"color_image_clip_{slug}{suffix}"


def index_cursor_stream_name(model_id: str) -> str:
    return index_stream_name(model_id) + "_source_cursor"


def build_frame_clip_index(
    store: Store,
    *,
    model: Any,
    src: str = "color_image",
    model_id: str = DEFAULT_CLIP_MODEL,
    hz: float = 2.0,
    start: float | None = None,
    end: float | None = None,
    index_store: Store | None = None,
    source_tag: str | None = None,
    thumbnail_px: int = 192,
) -> int:
    """Backfill a CLIP index within ``start < ts <= end`` when bounds are supplied."""
    if hz <= 0:
        raise ValueError("hz must be positive")
    index = (index_store if index_store is not None else store).stream(
        index_stream_name(model_id), Image
    )
    src_stream = store.stream(src, Image).order_by("ts")
    if start is not None:
        src_stream = src_stream.after(start)
    if end is not None:
        src_stream = src_stream.time_range(float("-inf"), end)
    tags = {"rec": source_tag} if source_tag else {}
    indexed = index.tags(**tags) if tags else index
    if start is not None:
        indexed = indexed.time_range(floor(start * hz) / hz, float("inf"))
    indexed_buckets = {floor(obs.ts * hz) for obs in indexed}
    persisted_bucket_count = len(indexed_buckets)

    def best_in_new_buckets(upstream: Iterator[Any]) -> Iterator[Any]:
        for bucket, observations in groupby(upstream, key=lambda obs: floor(obs.ts * hz)):
            if bucket in indexed_buckets:
                continue
            indexed_buckets.add(bucket)
            yield max(observations, key=lambda obs: obs.data.sharpness)

    pipeline = (
        # Skip pose-less frames — they can't answer "where" (world-frame frames need no pose).
        src_stream.filter(
            lambda obs: obs.pose is not None or (obs.data.frame_id or "") in ("", "world")
        )
        .filter(lambda obs: obs.data.brightness > 0.1)
        .transform(best_in_new_buckets)
        .transform(EmbedImages(model))
    )
    n = 0
    for obs in pipeline:
        embedded = cast("EmbeddedObservation[Image]", obs)
        payload = obs.data
        if index_store is not None:
            payload, _scale = payload.resize_to_fit(thumbnail_px, thumbnail_px)
        index.append(
            payload,
            ts=obs.ts,
            pose=obs.pose,
            tags=tags,
            embedding=embedded.embedding,
        )
        n += 1
    claimed_bucket_count = len(indexed_buckets) - persisted_bucket_count
    if n < claimed_bucket_count:
        raise RuntimeError(
            f"embedding model returned {n} frame(s) for {claimed_bucket_count} selected bucket(s)"
        )
    logger.info(
        "build_frame_clip_index: indexed %d frame(s) into '%s'", n, index_stream_name(model_id)
    )
    return n


def confirm_object_position(
    store: Store,
    text: str,
    when_ts: float,
    *,
    detector: Any = None,
    visual_embedder: Any = None,
    window_s: float = 2.0,
) -> Any | None:
    """The object's position: two-frame DINO confirmation, or one-frame without DINO."""
    from dimos.perception.detection.world_belief import WorldBelief, WorldBeliefConfig
    from dimos.perception.scene_scan import ScanIncompleteError, SceneScanner

    scanner = SceneScanner(
        detector=detector,
        embed=visual_embedder is not None,
        visual_embedder=visual_embedder,
    )
    belief = WorldBelief(
        WorldBeliefConfig(min_frames=2 if visual_embedder is not None else 1, min_span_s=0.0)
    )
    try:
        scanner.scan(store, belief, prompt=[text], start=when_ts - window_s, end=when_ts + window_s)
    except ScanIncompleteError:
        return None
    matches = [o for o in belief.present() if o.name == text]
    if not matches:
        return None
    return max(matches, key=lambda o: float(o.confidence)).center


def recall(
    store: Store,
    text: str,
    *,
    model: Any,
    model_id: str = DEFAULT_CLIP_MODEL,
    k: int = 20,
    detector: Any = None,
    visual_embedder: Any = None,
    open_recording: Callable[[str], Any] | None = None,
    window_s: float = 2.0,
    max_moments: int = 5,
) -> tuple[EmbeddedObservation[Image] | None, Any | None]:
    """Return the best matching frame and detector-confirmed object center."""
    index = store.stream(index_stream_name(model_id), Image)
    hits = index.search(model.embed_text(text), k=k).to_list()
    hits.sort(key=lambda o: o.similarity or 0.0, reverse=True)
    if not hits:
        logger.info("recall('%s'): no hits (index built? run build_frame_clip_index)", text)
        return None, None
    if open_recording is not None:
        scanned: list[tuple[str, float]] = []
        for hit in hits:
            rec = hit.tags.get("rec") or ""
            if not rec or any(r == rec and abs(float(hit.ts) - t) <= window_s for r, t in scanned):
                continue
            if len(scanned) >= max_moments:
                break
            source = open_recording(rec)
            if source is None:
                continue
            scanned.append((rec, float(hit.ts)))
            with source as rec_store:
                center = confirm_object_position(
                    rec_store,
                    text,
                    float(hit.ts),
                    detector=detector,
                    visual_embedder=visual_embedder,
                    window_s=window_s,
                )
            if center is not None:
                if len(scanned) > 1:
                    logger.info(
                        "recall('%s'): argmax frame unconfirmed by detector; returning"
                        " verified moment %d (sim %.3f vs %.3f)",
                        text,
                        len(scanned),
                        hit.similarity or 0.0,
                        hits[0].similarity or 0.0,
                    )
                return hit, center
        logger.info(
            "recall('%s'): no detector-confirmed moment in top-%d hits (%d scanned)",
            text,
            len(hits),
            len(scanned),
        )
    return hits[0], None
