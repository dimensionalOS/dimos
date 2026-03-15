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

from __future__ import annotations

from itertools import islice
from typing import TYPE_CHECKING, Any, TypeVar

from dimos.memory2.transform import Transformer

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation
    from dimos.models.embedding.base import EmbeddingModel

T = TypeVar("T")


def _batched(it: Iterator[T], n: int) -> Iterator[list[T]]:
    """Yield successive n-sized chunks from an iterator."""
    while True:
        batch = list(islice(it, n))
        if not batch:
            return
        yield batch


class EmbedImages(Transformer[Any, Any]):
    """Embed images using ``model.embed()``.

    Data type stays the same — observations are enriched with an
    ``.embedding`` field, yielding :class:`EmbeddedObservation` instances.
    """

    def __init__(self, model: EmbeddingModel, batch_size: int = 32) -> None:
        self.model = model
        self.batch_size = batch_size

    def __call__(self, upstream: Iterator[Observation[Any]]) -> Iterator[Observation[Any]]:
        for batch in _batched(upstream, self.batch_size):
            images = [obs.data for obs in batch]
            embeddings = self.model.embed(*images)
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            for obs, emb in zip(batch, embeddings, strict=False):
                yield obs.derive(data=obs.data, embedding=emb)


class EmbedText(Transformer[Any, Any]):
    """Embed text using ``model.embed_text()``.

    Data type stays the same — observations are enriched with an
    ``.embedding`` field, yielding :class:`EmbeddedObservation` instances.
    """

    def __init__(self, model: EmbeddingModel, batch_size: int = 32) -> None:
        self.model = model
        self.batch_size = batch_size

    def __call__(self, upstream: Iterator[Observation[Any]]) -> Iterator[Observation[Any]]:
        for batch in _batched(upstream, self.batch_size):
            texts = [str(obs.data) for obs in batch]
            embeddings = self.model.embed_text(*texts)
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            for obs, emb in zip(batch, embeddings, strict=False):
                yield obs.derive(data=obs.data, embedding=emb)
