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

"""Frame-CLIP memory index + recall over recordings — "when/where did I last see X"."""

from __future__ import annotations

from collections.abc import Callable
import re
from typing import TYPE_CHECKING, Any

from dimos.memory2.embed import EmbedImages
from dimos.memory2.transform import QualityWindow
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.type.observation import EmbeddedObservation

logger = setup_logger()

DEFAULT_CLIP_MODEL = "openai/clip-vit-base-patch32"


def index_stream_name(model_id: str) -> str:
    return "color_image_clip_" + re.sub(r"[^0-9A-Za-z]+", "_", model_id).strip("_")


def _build_clip(model_id: str) -> Any:
    from dimos.models.embedding.clip import CLIPModel

    model = CLIPModel(model_name=model_id)
    model.start()
    return model


def build_frame_clip_index(
    store: Store,
    *,
    src: str = "color_image",
    model: Any = None,
    model_id: str = DEFAULT_CLIP_MODEL,
    hz: float = 2.0,
    start: float | None = None,
    index_store: Store | None = None,
    source_tag: str | None = None,
    thumbnail_px: int = 192,
) -> int:
    """Backfill a CLIP index over ``store``'s color frames. Returns the number indexed."""
    if model is None:
        model = _build_clip(model_id)
    index = (index_store if index_store is not None else store).stream(
        index_stream_name(model_id), Image
    )
    src_stream = store.stream(src, Image)
    if start is not None:
        src_stream = src_stream.time_range(start, float("inf"))
    pipeline = (
        # Skip pose-less frames — they can't answer "where" (world-frame frames need no pose).
        src_stream.filter(
            lambda obs: obs.pose is not None or (obs.data.frame_id or "") in ("", "world")
        )
        .filter(lambda obs: obs.data.brightness > 0.1)
        .transform(QualityWindow(lambda img: img.sharpness, window=1.0 / hz))
        .transform(EmbedImages(model))
    )
    n = 0
    for obs in pipeline:
        payload = obs.data
        if index_store is not None:
            payload, _scale = payload.resize_to_fit(thumbnail_px, thumbnail_px)
        index.append(
            payload,
            ts=obs.ts,
            pose=obs.pose,
            tags={"rec": source_tag} if source_tag else {},
            embedding=obs.embedding,
        )
        n += 1
    logger.info("build_frame_clip_index: indexed %d frame(s) into '%s'", n, index_stream_name(model_id))
    return n


def confirm_object_position(
    store: Store,
    text: str,
    when_ts: float,
    *,
    detector: Any = None,
    window_s: float = 2.0,
) -> Any | None:
    """The OBJECT's world position around a recalled moment, or None."""
    from dimos.perception.detection.world_belief import WorldBelief, WorldBeliefConfig
    from dimos.perception.scene_scan import SceneScanner

    scanner = SceneScanner(
        detector=detector,
        embed=False,  # position only — no identity galleries needed
    )
    belief = WorldBelief(WorldBeliefConfig(min_frames=2, min_span_s=0.0))
    present = scanner.scan(
        store, belief, prompt=[text], start=when_ts - window_s, end=when_ts + window_s
    )
    matches = [o for o in present if o.name == text]
    if not matches:
        return None
    return max(matches, key=lambda o: float(o.confidence)).center


def recall(
    store: Store,
    text: str,
    *,
    model: Any = None,
    model_id: str = DEFAULT_CLIP_MODEL,
    k: int = 20,
    detector: Any = None,
    open_recording: Callable[[str], Any] | None = None,
    window_s: float = 2.0,
    max_moments: int = 5,
) -> tuple[EmbeddedObservation[Image] | None, Any | None]:
    """Recall "when/where did I see ``text``" → ``(hit, object_center)``"""
    if model is None:
        model = _build_clip(model_id)
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
                    rec_store, text, float(hit.ts), detector=detector, window_s=window_s
                )
            if center is not None:
                if len(scanned) > 1:
                    logger.info(
                        "recall('%s'): argmax frame unconfirmed by detector; returning"
                        " verified moment %d (sim %.3f vs %.3f)",
                        text, len(scanned), hit.similarity or 0.0, hits[0].similarity or 0.0,
                    )
                return hit, center
        logger.info(
            "recall('%s'): no detector-confirmed moment in top-%d hits (%d scanned)",
            text, len(hits), len(scanned),
        )
    return hits[0], None
