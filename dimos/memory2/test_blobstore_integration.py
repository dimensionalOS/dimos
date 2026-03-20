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

"""Tests for BlobStore integration with MemoryStore/Backend."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pytest

from dimos.memory2.blobstore.file import FileBlobStore
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.type.observation import _UNLOADED
from dimos.models.embedding.base import Embedding

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path


def _emb(vec: list[float]) -> Embedding:
    v = np.array(vec, dtype=np.float32)
    v /= np.linalg.norm(v) + 1e-10
    return Embedding(vector=v)


@pytest.fixture
def bs(tmp_path: Path) -> Iterator[FileBlobStore]:
    blob_store = FileBlobStore(root=str(tmp_path / "blobs"))
    blob_store.start()
    yield blob_store
    blob_store.stop()


@pytest.fixture
def store(bs: FileBlobStore) -> Iterator[MemoryStore]:
    with MemoryStore(blob_store=bs) as s:
        yield s


class TestBlobStoreIntegration:
    def test_append_stores_in_blobstore(self, bs: FileBlobStore, store: MemoryStore) -> None:
        s = store.stream("data", bytes)
        s.append(b"hello", ts=1.0)

        # Blob was written to the file store
        raw = bs.get("data", 0)
        assert len(raw) > 0

    def test_lazy_data_not_loaded_until_access(self, store: MemoryStore) -> None:
        s = store.stream("data", str)
        obs = s.append("payload", ts=1.0)

        # Data replaced with sentinel after append
        assert isinstance(obs._data, type(_UNLOADED))
        assert obs._loader is not None

    def test_lazy_data_loads_correctly(self, store: MemoryStore) -> None:
        s = store.stream("data", str)
        s.append("payload", ts=1.0)

        result = s.first()
        assert result.data == "payload"

    def test_eager_preloads_data(self, bs: FileBlobStore) -> None:
        with MemoryStore(blob_store=bs, eager_blobs=True) as store:
            s = store.stream("data", str)
            s.append("payload", ts=1.0)

            # Iterating with eager_blobs triggers load
            results = s.fetch()
            assert len(results) == 1
            # Data should be loaded (not _UNLOADED)
            assert not isinstance(results[0]._data, type(_UNLOADED))
            assert results[0].data == "payload"

    def test_per_stream_eager_override(self, store: MemoryStore) -> None:
        # Default: lazy
        lazy_stream = store.stream("lazy", str)
        lazy_stream.append("lazy-val", ts=1.0)

        # Override: eager
        eager_stream = store.stream("eager", str, eager_blobs=True)
        eager_stream.append("eager-val", ts=1.0)

        lazy_results = lazy_stream.fetch()
        eager_results = eager_stream.fetch()

        # Lazy: data stays unloaded until accessed
        assert lazy_results[0].data == "lazy-val"

        # Eager: data pre-loaded during iteration
        assert not isinstance(eager_results[0]._data, type(_UNLOADED))
        assert eager_results[0].data == "eager-val"

    def test_no_blobstore_unchanged(self) -> None:
        with MemoryStore() as store:
            s = store.stream("data", str)
            obs = s.append("inline", ts=1.0)

            # Without blob store, data stays inline
            assert obs._data == "inline"
            assert obs._loader is None
            assert obs.data == "inline"

    def test_blobstore_with_vector_search(self, bs: FileBlobStore) -> None:
        from dimos.memory2.vectorstore.memory import MemoryVectorStore

        vs = MemoryVectorStore()
        with MemoryStore(blob_store=bs, vector_store=vs) as store:
            s = store.stream("vecs", str)
            s.append("north", ts=1.0, embedding=_emb([0, 1, 0]))
            s.append("east", ts=2.0, embedding=_emb([1, 0, 0]))
            s.append("south", ts=3.0, embedding=_emb([0, -1, 0]))

            # Vector search triggers lazy load via obs.derive(data=obs.data, ...)
            results = s.search(_emb([0, 1, 0]), k=2).fetch()
            assert len(results) == 2
            assert results[0].data == "north"
            assert results[0].similarity > 0.99

    def test_blobstore_with_text_search(self, store: MemoryStore) -> None:
        s = store.stream("logs", str)
        s.append("motor fault", ts=1.0)
        s.append("temperature ok", ts=2.0)

        # Text search triggers lazy load via str(obs.data)
        results = s.search_text("motor").fetch()
        assert len(results) == 1
        assert results[0].data == "motor fault"

    def test_multiple_appends_get_unique_blobs(self, store: MemoryStore) -> None:
        s = store.stream("multi", str)
        s.append("first", ts=1.0)
        s.append("second", ts=2.0)
        s.append("third", ts=3.0)

        results = s.fetch()
        assert [r.data for r in results] == ["first", "second", "third"]

    def test_fetch_preserves_metadata(self, store: MemoryStore) -> None:
        s = store.stream("meta", str)
        s.append("val", ts=42.0, tags={"kind": "info"})

        result = s.first()
        assert result.ts == 42.0
        assert result.tags == {"kind": "info"}
        assert result.data == "val"
