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

"""Tests for memory transformers."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from dimos.memory.impl.sqlite import SqliteSession, SqliteStore
from dimos.memory.transformer import TextEmbeddingTransformer
from dimos.models.embedding.base import Embedding, EmbeddingModel

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.msgs.sensor_msgs.Image import Image


class FakeTextEmbedder(EmbeddingModel):
    device = "cpu"

    def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
        raise NotImplementedError

    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        results = []
        for text in texts:
            h = hash(text) % 1000 / 1000.0
            results.append(Embedding(np.array([h, 1.0 - h, 0.0, 0.0], dtype=np.float32)))
        return results if len(results) > 1 else results[0]


class SemanticFakeEmbedder(EmbeddingModel):
    """Embeds 'kitchen' texts to one region, everything else to another."""

    device = "cpu"

    def embed(self, *imgs: Image) -> Embedding | list[Embedding]:  # type: ignore[override]
        raise NotImplementedError

    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        results = []
        for text in texts:
            if "kitchen" in text.lower():
                results.append(Embedding(np.array([1.0, 0.0, 0.0], dtype=np.float32)))
            else:
                results.append(Embedding(np.array([0.0, 1.0, 0.0], dtype=np.float32)))
        return results if len(results) > 1 else results[0]


@pytest.fixture
def session(tmp_path: object) -> Iterator[SqliteSession]:
    from pathlib import Path

    assert isinstance(tmp_path, Path)
    store = SqliteStore(str(tmp_path / "test.db"))
    sess = store.session()
    yield sess
    sess.stop()
    store.stop()


class TestTextEmbeddingTransformer:
    """Test text -> embedding -> semantic search pipeline."""

    def test_text_to_embedding_backfill(self, session: SqliteSession) -> None:
        """Backfill: store text, transform to embeddings, search by text."""
        logs = session.stream("te_logs", str)
        logs.append("Robot navigated to kitchen", ts=1.0)
        logs.append("Battery low warning", ts=2.0)
        logs.append("Robot navigated to bedroom", ts=3.0)

        emb_stream = logs.transform(TextEmbeddingTransformer(FakeTextEmbedder())).store(
            "te_log_embeddings"
        )

        assert emb_stream.count() == 3

        results = emb_stream.search_embedding("Robot navigated to kitchen", k=1).fetch()
        assert len(results) == 1
        assert isinstance(results[0].data, Embedding)

        # project_to to get source text
        projected = (
            emb_stream.search_embedding("Robot navigated to kitchen", k=1).project_to(logs).fetch()
        )
        assert len(projected) == 1
        assert isinstance(projected[0].data, str)

    def test_text_embedding_live(self, session: SqliteSession) -> None:
        """Live mode: new text is embedded automatically."""
        logs = session.stream("te_live_logs", str)
        emb_stream = logs.transform(TextEmbeddingTransformer(FakeTextEmbedder()), live=True).store(
            "te_live_embs"
        )

        assert emb_stream.count() == 0  # no backfill

        logs.append("New log entry", ts=1.0)
        assert emb_stream.count() == 1

        logs.append("Another log entry", ts=2.0)
        assert emb_stream.count() == 2

    def test_text_embedding_search_and_project(self, session: SqliteSession) -> None:
        """search_embedding + project_to retrieves source text."""
        logs = session.stream("te_proj_logs", str)
        logs.append("Robot entered kitchen", ts=1.0)
        logs.append("Battery warning", ts=2.0)
        logs.append("Cleaning kitchen floor", ts=3.0)

        emb_stream = logs.transform(TextEmbeddingTransformer(SemanticFakeEmbedder())).store(
            "te_proj_embs"
        )

        results = emb_stream.search_embedding("kitchen", k=2).project_to(logs).fetch()
        assert len(results) == 2
        assert all("kitchen" in r.data.lower() for r in results)
