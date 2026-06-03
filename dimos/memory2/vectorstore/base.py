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

from abc import abstractmethod
from typing import TYPE_CHECKING, Any

from dimos.core.resource import CompositeResource
from dimos.memory2.registry import qual
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from dimos.models.embedding.base import Embedding


class VectorStoreConfig(BaseConfig):
    pass


class VectorStore(Configurable, CompositeResource):
    """Pluggable storage and ANN index for embedding vectors.

    Separates vector indexing from metadata so backends can swap
    search strategies (brute-force, vec0, FAISS, Qdrant) independently.

    Same shape as BlobStore: ``put`` / ``search`` / ``delete``, keyed
    by ``(stream, observation_id)``.  Vector index creation is lazy — the
    first ``put`` for a stream determines dimensionality.
    """

    config: VectorStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)

    @abstractmethod
    def put(self, stream_name: str, key: int, embedding: Embedding) -> None:
        """Store an embedding vector for the given stream and observation id."""
        ...

    @abstractmethod
    def search(self, stream_name: str, query: Embedding, k: int | None) -> list[tuple[int, float]]:
        """Return top-k (observation_id, similarity) pairs, descending.

        ``k=None`` means "store default" — backends that require a bound
        (e.g. sqlite-vec) pick one internally; unbounded backends return all.
        """
        ...

    @abstractmethod
    def delete(self, stream_name: str, key: int) -> None:
        """Remove a vector. Silent if missing."""
        ...

    def serialize(self) -> dict[str, Any]:
        return {"class": qual(type(self)), "config": self.config.model_dump()}
