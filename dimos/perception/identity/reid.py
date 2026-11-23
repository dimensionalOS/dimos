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

"""Crop-embedding ReID against recently lost tracks of the same class."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

from torch.nn import functional

from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.utils.sequential_ids import SequentialIds


def ema_embedding(old: Embedding, new: Embedding, alpha: float) -> Embedding:
    """Exponential moving average, then L2-normalize. *alpha* is the weight on *old*."""
    previous = old.to_torch()
    current = new.to_torch(previous.device)
    mixed = functional.normalize(alpha * previous + (1.0 - alpha) * current, dim=0)
    return Embedding(vector=mixed, timestamp=new.timestamp)


@dataclass
class _Active:
    identity_id: int
    name: str
    embedding: Embedding
    last_t: float
    frames: int


@dataclass
class _Lost:
    identity_id: int
    name: str
    embedding: Embedding
    last_t: float


class AppearanceReId:
    """Map BoT-SORT ids onto stable identities with crop embeddings.

    Motion-matched tracks keep their identity. A new BoT-SORT id searches the
    recently lost gallery of the same class name; a high cosine reuses that
    identity. Embeddings run on new/broken tracks, and every ``update_every``
    frames on a live track for the EMA slot — not a gallery search.
    """

    def __init__(
        self,
        embedder: EmbeddingModel,
        *,
        threshold: float = 0.75,
        ttl_s: float = 8.0,
        ema: float = 0.8,
        update_every: int = 5,
        crop_padding: int = 10,
    ) -> None:
        if not 0.0 <= ema <= 1.0:
            raise ValueError("ema must be in [0, 1]")
        if update_every < 1:
            raise ValueError("update_every must be >= 1")
        self._embedder = embedder
        self._threshold = threshold
        self._ttl_s = ttl_s
        self._ema = ema
        self._update_every = update_every
        self._crop_padding = crop_padding
        self._ids = SequentialIds()
        self._by_bot: dict[int, _Active] = {}
        self._lost: list[_Lost] = []

    def reset(self) -> None:
        self._ids = SequentialIds()
        self._by_bot = {}
        self._lost = []

    def assign(self, detections: Sequence[Detection2DBBox], image: Image) -> list[Detection2DBBox]:
        t = float(image.ts)
        live_bots = {det.track_id for det in detections if det.track_id >= 0}
        self._retire(live_bots, t)
        need_embed = [
            i
            for i, det in enumerate(detections)
            if det.track_id >= 0 and self._should_embed(det.track_id)
        ]
        embeddings = self._embed_crops([detections[i] for i in need_embed])
        by_index = dict(zip(need_embed, embeddings, strict=True))
        out: list[Detection2DBBox] = []
        for i, det in enumerate(detections):
            if det.track_id < 0:
                out.append(det)
                continue
            active = self._by_bot.get(det.track_id)
            emb = by_index.get(i)
            if active is None:
                if emb is None:
                    raise RuntimeError("new track missing crop embedding")
                ident = self._reuse_or_new(det.name, emb, t)
                self._by_bot[det.track_id] = _Active(ident, det.name, emb, t, 1)
                out.append(replace(det, track_id=ident))
                continue
            if emb is not None:
                active.embedding = ema_embedding(active.embedding, emb, self._ema)
            active.last_t = t
            active.frames += 1
            out.append(replace(det, track_id=active.identity_id))
        return out

    def _should_embed(self, bot_id: int) -> bool:
        active = self._by_bot.get(bot_id)
        if active is None:
            return True
        return active.frames % self._update_every == 0

    def _embed_crops(self, detections: Sequence[Detection2DBBox]) -> list[Embedding]:
        if not detections:
            return []
        crops = [det.cropped_image(padding=self._crop_padding) for det in detections]
        result = self._embedder.embed(*crops)
        if isinstance(result, Embedding):
            return [result]
        return list(result)

    def _retire(self, live_bots: set[int], t: float) -> None:
        for bot_id in [bot_id for bot_id in self._by_bot if bot_id not in live_bots]:
            active = self._by_bot.pop(bot_id)
            self._lost.append(
                _Lost(active.identity_id, active.name, active.embedding, active.last_t)
            )
        self._lost = [lost for lost in self._lost if (t - lost.last_t) <= self._ttl_s]

    def _reuse_or_new(self, name: str, embedding: Embedding, t: float) -> int:
        best_i = -1
        best_sim = self._threshold
        for i, lost in enumerate(self._lost):
            if lost.name != name:
                continue
            if (t - lost.last_t) > self._ttl_s:
                continue
            sim = embedding @ lost.embedding
            if sim >= best_sim:
                best_sim = sim
                best_i = i
        if best_i >= 0:
            return self._lost.pop(best_i).identity_id
        return self._ids.next() + 1
