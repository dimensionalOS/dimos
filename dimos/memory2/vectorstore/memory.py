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

from typing import TYPE_CHECKING, Any

from dimos.memory2.vectorstore.base import VectorStore, VectorStoreConfig

if TYPE_CHECKING:
    from dimos.models.embedding.base import Embedding


class MemoryVectorStoreConfig(VectorStoreConfig):
    pass


class MemoryVectorStore(VectorStore):
    """In-memory brute-force vector store for testing.

    Stores embeddings in a dict keyed by ``(stream, observation_id)``.
    Search computes cosine similarity against all vectors in the stream.
    """

    config: MemoryVectorStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._vectors: dict[str, dict[int, Embedding]] = {}

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def put(self, stream_name: str, key: int, embedding: Embedding) -> None:
        self._vectors.setdefault(stream_name, {})[key] = embedding

    def search(self, stream_name: str, query: Embedding, k: int) -> list[tuple[int, float]]:
        vectors = self._vectors.get(stream_name, {})
        if not vectors:
            return []
        scored = [(key, float(emb @ query)) for key, emb in vectors.items()]
        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[:k]

    def delete(self, stream_name: str, key: int) -> None:
        vectors = self._vectors.get(stream_name, {})
        vectors.pop(key, None)
