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

"""Tests for embedding layer: EmbeddedObservation, vector search, text search, transformers."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.models.embedding.base import Embedding


def _emb(vec: list[float]) -> Embedding:
    """Return a unit-normalized Embedding."""
    v = np.array(vec, dtype=np.float32)
    v /= np.linalg.norm(v) + 1e-10
    return Embedding(vector=v)


class TestEmbeddedObservation:
    def test_construction(self) -> None:
        emb = _emb([1, 0, 0])
        obs = EmbeddedObservation(id=0, ts=1.0, _data="hello", embedding=emb)
        assert obs.data == "hello"
        assert obs.embedding is emb
        assert obs.similarity is None

    def test_is_observation(self) -> None:
        obs = EmbeddedObservation(id=0, ts=1.0, _data="x", embedding=_emb([1, 0]))
        assert isinstance(obs, Observation)

    def test_derive_preserves_embedding(self) -> None:
        emb = _emb([1, 0, 0])
        obs = EmbeddedObservation(id=0, ts=1.0, _data="a", embedding=emb)
        derived = obs.derive(data="b")
        assert isinstance(derived, EmbeddedObservation)
        assert derived.embedding is emb
        assert derived.data == "b"

    def test_derive_replaces_embedding(self) -> None:
        old = _emb([1, 0, 0])
        new = _emb([0, 1, 0])
        obs = EmbeddedObservation(id=0, ts=1.0, _data="a", embedding=old)
        derived = obs.derive(data="a", embedding=new)
        assert derived.embedding is new

    def test_derive_preserves_similarity(self) -> None:
        obs = EmbeddedObservation(id=0, ts=1.0, _data="a", embedding=_emb([1, 0]), similarity=0.95)
        derived = obs.derive(data="b")
        assert derived.similarity == 0.95

    def test_observation_derive_promotes_to_embedded(self) -> None:
        obs = Observation(id=0, ts=1.0, _data="plain")
        emb = _emb([1, 0, 0])
        derived = obs.derive(data="plain", embedding=emb)
        assert isinstance(derived, EmbeddedObservation)
        assert derived.embedding is emb

    def test_observation_derive_without_embedding_stays_observation(self) -> None:
        obs = Observation(id=0, ts=1.0, _data="plain")
        derived = obs.derive(data="still plain")
        assert type(derived) is Observation


class TestListBackendEmbedding:
    def test_append_with_embedding(self, memory_store) -> None:
        s = memory_store.stream("vecs", str)
        emb = _emb([1, 0, 0])
        obs = s.append("hello", embedding=emb)
        assert isinstance(obs, EmbeddedObservation)
        assert obs.embedding is emb

    def test_append_without_embedding(self, memory_store) -> None:
        s = memory_store.stream("plain", str)
        obs = s.append("hello")
        assert type(obs) is Observation

    def test_search_returns_top_k(self, memory_store) -> None:
        s = memory_store.stream("vecs", str)
        s.append("north", embedding=_emb([0, 1, 0]))
        s.append("east", embedding=_emb([1, 0, 0]))
        s.append("south", embedding=_emb([0, -1, 0]))
        s.append("west", embedding=_emb([-1, 0, 0]))

        results = s.search(_emb([0, 1, 0]), k=2).to_list()
        assert len(results) == 2
        assert results[0].data == "north"
        assert results[0].similarity is not None
        assert results[0].similarity > 0.99

    def test_search_sorted_by_similarity(self, memory_store) -> None:
        s = memory_store.stream("vecs", str)
        s.append("far", embedding=_emb([0, -1, 0]))
        s.append("close", embedding=_emb([0.9, 0.1, 0]))
        s.append("exact", embedding=_emb([1, 0, 0]))

        results = s.search(_emb([1, 0, 0]), k=3).to_list()
        assert results[0].data == "exact"
        assert results[1].data == "close"
        assert results[2].data == "far"
        # Descending similarity
        assert results[0].similarity >= results[1].similarity >= results[2].similarity

    def test_search_skips_non_embedded(self, memory_store) -> None:
        s = memory_store.stream("mixed", str)
        s.append("plain")  # no embedding
        s.append("embedded", embedding=_emb([1, 0, 0]))

        results = s.search(_emb([1, 0, 0]), k=10).to_list()
        assert len(results) == 1
        assert results[0].data == "embedded"

    def test_search_with_filters(self, memory_store) -> None:
        s = memory_store.stream("vecs", str)
        s.append("early", ts=10.0, embedding=_emb([1, 0, 0]))
        s.append("late", ts=20.0, embedding=_emb([1, 0, 0]))

        # Only the late one should pass the after filter
        results = s.after(15.0).search(_emb([1, 0, 0]), k=10).to_list()
        assert len(results) == 1
        assert results[0].data == "late"

    def test_search_with_limit(self, memory_store) -> None:
        s = memory_store.stream("vecs", str)
        for i in range(10):
            s.append(f"item{i}", embedding=_emb([1, 0, 0]))

        # search k=5 then limit 2
        results = s.search(_emb([1, 0, 0]), k=5).limit(2).to_list()
        assert len(results) == 2

    def test_search_with_live_raises(self, memory_store) -> None:
        s = memory_store.stream("vecs", str)
        s.append("x", embedding=_emb([1, 0, 0]))
        with pytest.raises(TypeError, match="Cannot combine"):
            list(s.live().search(_emb([1, 0, 0]), k=5))


class TestTextSearch:
    def test_search_text_substring(self, memory_store) -> None:
        s = memory_store.stream("logs", str)
        s.append("motor fault detected")
        s.append("temperature normal")
        s.append("motor overheating")

        results = s.search_text("motor").to_list()
        assert len(results) == 2
        assert {r.data for r in results} == {"motor fault detected", "motor overheating"}

    def test_search_text_case_insensitive(self, memory_store) -> None:
        s = memory_store.stream("logs", str)
        s.append("Motor Fault")
        s.append("other event")

        results = s.search_text("motor fault").to_list()
        assert len(results) == 1

    def test_search_text_with_filters(self, memory_store) -> None:
        s = memory_store.stream("logs", str)
        s.append("motor fault", ts=10.0)
        s.append("motor warning", ts=20.0)
        s.append("motor fault", ts=30.0)

        results = s.after(15.0).search_text("fault").to_list()
        assert len(results) == 1
        assert results[0].ts == 30.0

    def test_search_text_no_match(self, memory_store) -> None:
        s = memory_store.stream("logs", str)
        s.append("all clear")

        results = s.search_text("motor").to_list()
        assert len(results) == 0


class TestSaveEmbeddings:
    def test_save_preserves_embeddings(self, memory_store) -> None:
        src = memory_store.stream("source", str)
        dst = memory_store.stream("dest", str)

        emb = _emb([1, 0, 0])
        src.append("item", embedding=emb)
        src.save(dst).drain()

        results = dst.to_list()
        assert len(results) == 1
        assert isinstance(results[0], EmbeddedObservation)
        # Same vector content (different Embedding instance after re-append)
        np.testing.assert_array_almost_equal(results[0].embedding.to_numpy(), emb.to_numpy())

    def test_save_mixed_embedded_and_plain(self, memory_store) -> None:
        src = memory_store.stream("source", str)
        dst = memory_store.stream("dest", str)

        src.append("plain")
        src.append("embedded", embedding=_emb([0, 1, 0]))
        src.save(dst).drain()

        results = dst.to_list()
        assert len(results) == 2
        assert type(results[0]) is Observation
        assert isinstance(results[1], EmbeddedObservation)


class _MockEmbeddingModel:
    """Fake EmbeddingModel that returns deterministic unit vectors."""

    device = "cpu"

    def embed(self, *images):
        vecs = []
        for img in images:
            rng = np.random.default_rng(hash(str(img)) % 2**32)
            v = rng.standard_normal(8).astype(np.float32)
            v /= np.linalg.norm(v)
            vecs.append(Embedding(vector=v))
        return vecs if len(vecs) > 1 else vecs[0]

    def embed_text(self, *texts):
        vecs = []
        for text in texts:
            rng = np.random.default_rng(hash(text) % 2**32)
            v = rng.standard_normal(8).astype(np.float32)
            v /= np.linalg.norm(v)
            vecs.append(Embedding(vector=v))
        return vecs if len(vecs) > 1 else vecs[0]


class TestEmbedTransformers:
    def test_embed_images_produces_embedded_observations(self, memory_store) -> None:
        from dimos.memory2.embed import EmbedImages

        model = _MockEmbeddingModel()
        s = memory_store.stream("imgs", str)
        s.append("img1", ts=1.0)
        s.append("img2", ts=2.0)

        results = s.transform(EmbedImages(model)).to_list()
        assert len(results) == 2
        for obs in results:
            assert isinstance(obs, EmbeddedObservation)
            assert isinstance(obs.embedding, Embedding)
            assert obs.embedding.to_numpy().shape == (8,)

    def test_embed_text_produces_embedded_observations(self, memory_store) -> None:
        from dimos.memory2.embed import EmbedText

        model = _MockEmbeddingModel()
        s = memory_store.stream("logs", str)
        s.append("motor fault", ts=1.0)
        s.append("all clear", ts=2.0)

        results = s.transform(EmbedText(model)).to_list()
        assert len(results) == 2
        for obs in results:
            assert isinstance(obs, EmbeddedObservation)
            assert isinstance(obs.embedding, Embedding)

    def test_embed_preserves_data(self, memory_store) -> None:
        from dimos.memory2.embed import EmbedText

        model = _MockEmbeddingModel()
        s = memory_store.stream("logs", str)
        s.append("hello", ts=1.0)

        result = s.transform(EmbedText(model)).first()
        assert result.data == "hello"

    def test_embed_then_search(self, memory_store) -> None:
        from dimos.memory2.embed import EmbedText

        model = _MockEmbeddingModel()
        s = memory_store.stream("logs", str)
        for i in range(10):
            s.append(f"log entry {i}", ts=float(i))

        embedded = s.transform(EmbedText(model))
        # Get the embedding for the first item, then search for similar
        first_emb = embedded.first().embedding
        results = embedded.search(first_emb, k=3).to_list()
        assert len(results) == 3
        # First result should be the exact match
        assert results[0].similarity is not None
        assert results[0].similarity > 0.99

    def test_embed_batching(self, memory_store) -> None:
        from dimos.memory2.embed import EmbedText

        call_sizes: list[int] = []

        class _TrackingModel(_MockEmbeddingModel):
            def embed_text(self, *texts):
                call_sizes.append(len(texts))
                return super().embed_text(*texts)

        model = _TrackingModel()
        s = memory_store.stream("logs", str)
        for i in range(5):
            s.append(f"entry {i}")

        list(s.transform(EmbedText(model, batch_size=2)))
        # 5 items with batch_size=2 → 3 calls (2, 2, 1)
        assert call_sizes == [2, 2, 1]


class TestPluggableVectorStore:
    """Verify that injecting a VectorStore via store config actually delegates search."""

    def test_append_stores_in_vector_store(self) -> None:
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.vectorstore.memory import MemoryVectorStore

        vs = MemoryVectorStore()
        with MemoryStore(vector_store=vs) as store:
            s = store.stream("vecs", str)
            s.append("hello", embedding=_emb([1, 0, 0]))
            s.append("world", embedding=_emb([0, 1, 0]))

        assert len(vs._vectors["vecs"]) == 2

    def test_append_without_embedding_skips_vector_store(self) -> None:
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.vectorstore.memory import MemoryVectorStore

        vs = MemoryVectorStore()
        with MemoryStore(vector_store=vs) as store:
            s = store.stream("plain", str)
            s.append("no embedding")

        assert "plain" not in vs._vectors

    def test_search_uses_vector_store(self) -> None:
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.vectorstore.memory import MemoryVectorStore

        vs = MemoryVectorStore()
        with MemoryStore(vector_store=vs) as store:
            s = store.stream("vecs", str)
            s.append("north", embedding=_emb([0, 1, 0]))
            s.append("east", embedding=_emb([1, 0, 0]))
            s.append("south", embedding=_emb([0, -1, 0]))
            s.append("west", embedding=_emb([-1, 0, 0]))

            results = s.search(_emb([0, 1, 0]), k=2).to_list()
            assert len(results) == 2
            assert results[0].data == "north"
            assert results[0].similarity is not None
            assert results[0].similarity > 0.99

    def test_search_with_filters_via_vector_store(self) -> None:
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.vectorstore.memory import MemoryVectorStore

        vs = MemoryVectorStore()
        with MemoryStore(vector_store=vs) as store:
            s = store.stream("vecs", str)
            s.append("early", ts=10.0, embedding=_emb([1, 0, 0]))
            s.append("late", ts=20.0, embedding=_emb([1, 0, 0]))

            # Filter + search: only "late" passes the after filter
            results = s.after(15.0).search(_emb([1, 0, 0]), k=10).to_list()
            assert len(results) == 1
            assert results[0].data == "late"

    def test_per_stream_vector_store_override(self) -> None:
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.vectorstore.memory import MemoryVectorStore

        vs_default = MemoryVectorStore()
        vs_override = MemoryVectorStore()
        with MemoryStore(vector_store=vs_default) as store:
            # Stream with default vector store
            s1 = store.stream("s1", str)
            s1.append("a", embedding=_emb([1, 0, 0]))

            # Stream with overridden vector store
            s2 = store.stream("s2", str, vector_store=vs_override)
            s2.append("b", embedding=_emb([0, 1, 0]))

        assert "s1" in vs_default._vectors
        assert "s1" not in vs_override._vectors
        assert "s2" in vs_override._vectors
        assert "s2" not in vs_default._vectors
