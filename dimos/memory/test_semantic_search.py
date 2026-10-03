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

import numpy as np

from dimos.memory.module import SemanticSearch
from dimos.models.embedding.base import Embedding


class FakeEmbeddingModel:
    def embed_text(self, query: str) -> Embedding:
        return Embedding(np.array([1.0], dtype=np.float32))


def test_search_finds_strongest_match_when_vector_results_are_similarity_ordered(
    memory_store,
) -> None:
    searcher = object.__new__(SemanticSearch)
    searcher.model = FakeEmbeddingModel()  # type: ignore[assignment]
    searcher.embeddings = memory_store.stream("color_image_embedded", str)

    # Vector search returns these in similarity order, not timestamp order.
    similarities = [
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
        0.7,
        0.8,
        0.9,
        1.0,
        0.2,
        0.2,
        0.2,
        0.2,
        0.2,
        0.3,
        0.4,
        0.5,
        0.8,
        0.7,
        0.6,
        0.5,
        0.4,
        0.3,
        0.2,
        0.2,
        0.2,
        0.2,
        0.2,
    ]
    for ts, similarity in enumerate(similarities):
        searcher.embeddings.append(
            f"frame-{ts}",
            ts=float(ts),
            pose=(float(ts), 0.0, 0.0),
            embedding=Embedding(np.array([similarity])),
        )

    result = searcher.search("target")

    assert result.ts == 8.0
