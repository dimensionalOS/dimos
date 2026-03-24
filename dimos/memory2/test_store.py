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

"""Grid tests for Store implementations.

Runs the same test logic against every Store backend (MemoryStore, SqliteStore, ...).
The parametrized ``session`` fixture from conftest runs each test against both backends.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import pytest

from dimos.memory2.blobstore.base import BlobStore
from dimos.memory2.vectorstore.base import VectorStore

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store


class TestStoreBasic:
    """Core store operations that every backend must support."""

    def test_create_stream_and_append(self, session: Store) -> None:
        s = session.stream("images", bytes)
        obs = s.append(b"frame1", tags={"camera": "front"})

        assert obs.data == b"frame1"
        assert obs.tags["camera"] == "front"
        assert obs.ts > 0

    def test_append_multiple_and_fetch(self, session: Store) -> None:
        s = session.stream("sensor", float)
        s.append(1.0, ts=100.0)
        s.append(2.0, ts=200.0)
        s.append(3.0, ts=300.0)

        results = s.fetch()
        assert len(results) == 3
        assert [o.data for o in results] == [1.0, 2.0, 3.0]

    def test_iterate_stream(self, session: Store) -> None:
        s = session.stream("log", str)
        s.append("a", ts=1.0)
        s.append("b", ts=2.0)

        collected = [obs.data for obs in s]
        assert collected == ["a", "b"]

    def test_count(self, session: Store) -> None:
        s = session.stream("events", str)
        assert s.count() == 0
        s.append("x")
        s.append("y")
        assert s.count() == 2

    def test_first_and_last(self, session: Store) -> None:
        s = session.stream("data", int)
        s.append(10, ts=1.0)
        s.append(20, ts=2.0)
        s.append(30, ts=3.0)

        assert s.first().data == 10
        assert s.last().data == 30

    def test_first_empty_raises(self, session: Store) -> None:
        s = session.stream("empty", int)
        with pytest.raises(LookupError):
            s.first()

    def test_exists(self, session: Store) -> None:
        s = session.stream("check", str)
        assert not s.exists()
        s.append("hi")
        assert s.exists()

    def test_filter_after(self, session: Store) -> None:
        s = session.stream("ts_data", int)
        s.append(1, ts=10.0)
        s.append(2, ts=20.0)
        s.append(3, ts=30.0)

        results = s.after(15.0).fetch()
        assert [o.data for o in results] == [2, 3]

    def test_filter_before(self, session: Store) -> None:
        s = session.stream("ts_data", int)
        s.append(1, ts=10.0)
        s.append(2, ts=20.0)
        s.append(3, ts=30.0)

        results = s.before(25.0).fetch()
        assert [o.data for o in results] == [1, 2]

    def test_filter_time_range(self, session: Store) -> None:
        s = session.stream("ts_data", int)
        s.append(1, ts=10.0)
        s.append(2, ts=20.0)
        s.append(3, ts=30.0)

        results = s.time_range(15.0, 25.0).fetch()
        assert [o.data for o in results] == [2]

    def test_filter_tags(self, session: Store) -> None:
        s = session.stream("tagged", str)
        s.append("a", tags={"kind": "info"})
        s.append("b", tags={"kind": "error"})
        s.append("c", tags={"kind": "info"})

        results = s.tags(kind="info").fetch()
        assert [o.data for o in results] == ["a", "c"]

    def test_limit_and_offset(self, session: Store) -> None:
        s = session.stream("paged", int)
        for i in range(5):
            s.append(i, ts=float(i))

        page = s.offset(1).limit(2).fetch()
        assert [o.data for o in page] == [1, 2]

    def test_order_by_desc(self, session: Store) -> None:
        s = session.stream("ordered", int)
        s.append(1, ts=10.0)
        s.append(2, ts=20.0)
        s.append(3, ts=30.0)

        results = s.order_by("ts", desc=True).fetch()
        assert [o.data for o in results] == [3, 2, 1]

    def test_separate_streams_isolated(self, session: Store) -> None:
        a = session.stream("stream_a", str)
        b = session.stream("stream_b", str)

        a.append("in_a")
        b.append("in_b")

        assert [o.data for o in a] == ["in_a"]
        assert [o.data for o in b] == ["in_b"]

    def test_same_stream_on_repeated_calls(self, session: Store) -> None:
        s1 = session.stream("reuse", str)
        s2 = session.stream("reuse", str)
        assert s1 is s2

    def test_append_with_embedding(self, session: Store) -> None:
        import numpy as np

        from dimos.memory2.type.observation import EmbeddedObservation
        from dimos.models.embedding.base import Embedding

        s = session.stream("vectors", str)
        emb = Embedding(vector=np.array([1.0, 0.0, 0.0], dtype=np.float32))
        obs = s.append("hello", embedding=emb)
        assert isinstance(obs, EmbeddedObservation)
        assert obs.embedding is emb

    def test_search_top_k(self, session: Store) -> None:
        import numpy as np

        from dimos.models.embedding.base import Embedding

        def _emb(v: list[float]) -> Embedding:
            a = np.array(v, dtype=np.float32)
            return Embedding(vector=a / (np.linalg.norm(a) + 1e-10))

        s = session.stream("searchable", str)
        s.append("north", embedding=_emb([0, 1, 0]))
        s.append("east", embedding=_emb([1, 0, 0]))
        s.append("south", embedding=_emb([0, -1, 0]))

        results = s.search(_emb([0, 1, 0]), k=2).fetch()
        assert len(results) == 2
        assert results[0].data == "north"
        assert results[0].similarity > 0.99

    def test_search_text(self, session: Store) -> None:
        s = session.stream("logs", str)
        s.append("motor fault")
        s.append("temperature ok")

        # SqliteObservationStore blocks search_text to prevent full table scans
        try:
            results = s.search_text("motor").fetch()
        except NotImplementedError:
            pytest.skip("search_text not supported on this backend")
        assert len(results) == 1
        assert results[0].data == "motor fault"


class TestBlobLoading:
    """Verify lazy and eager blob loading paths."""

    def test_sqlite_lazy_by_default(self, sqlite_store: Store) -> None:
        """Default sqlite iteration uses lazy loaders — data is _UNLOADED until accessed."""
        from dimos.memory2.type.observation import _Unloaded

        s = sqlite_store.stream("lazy_test", str)
        s.append("hello", ts=1.0)
        s.append("world", ts=2.0)

        for obs in s:
            # Before accessing .data, _data should be the unloaded sentinel
            assert isinstance(obs._data, _Unloaded)
            assert obs._loader is not None
            # Accessing .data triggers the loader
            val = obs.data
            assert isinstance(val, str)
            # After loading, _loader is cleared
            assert obs._loader is None

    def test_sqlite_eager_loads_inline(self, sqlite_store: Store) -> None:
        """With eager_blobs=True, data is loaded via JOIN — no lazy loader."""
        from dimos.memory2.type.observation import _Unloaded

        s = sqlite_store.stream("eager_test", str, eager_blobs=True)
        s.append("hello", ts=1.0)
        s.append("world", ts=2.0)

        for obs in s:
            # Data should already be loaded — no lazy sentinel
            assert not isinstance(obs._data, _Unloaded)
            assert obs._loader is None
            assert isinstance(obs.data, str)

    def test_sqlite_lazy_and_eager_same_values(self, sqlite_store: Store) -> None:
        """Both paths must return identical data."""
        lazy_s = sqlite_store.stream("vals", str)
        lazy_s.append("alpha", ts=1.0, tags={"k": "v"})
        lazy_s.append("beta", ts=2.0, tags={"k": "w"})

        # Lazy read
        lazy_results = lazy_s.fetch()

        # Eager read — new stream handle with eager_blobs on same backend
        eager_s = sqlite_store.stream("vals", str, eager_blobs=True)
        eager_results = eager_s.fetch()

        assert [o.data for o in lazy_results] == [o.data for o in eager_results]
        assert [o.tags for o in lazy_results] == [o.tags for o in eager_results]
        assert [o.ts for o in lazy_results] == [o.ts for o in eager_results]

    def test_memory_lazy_with_blobstore(self, tmp_path) -> None:
        """MemoryStore with a BlobStore uses lazy loaders."""
        from dimos.memory2.blobstore.file import FileBlobStore
        from dimos.memory2.store.memory import MemoryStore
        from dimos.memory2.type.observation import _Unloaded

        bs = FileBlobStore(root=str(tmp_path / "blobs"))
        bs.start()
        with MemoryStore(blob_store=bs) as store:
            s = store.stream("mem_lazy", str)
            s.append("data1", ts=1.0)

            obs = s.first()
            # Backend replaces _data with _UNLOADED when blob_store is set
            assert isinstance(obs._data, _Unloaded)
            assert obs.data == "data1"
        bs.stop()


class SpyBlobStore(BlobStore):
    """BlobStore that records all calls for verification."""

    def __init__(self) -> None:
        super().__init__()
        self.puts: list[tuple[str, int, bytes]] = []
        self.gets: list[tuple[str, int]] = []
        self.store: dict[tuple[str, int], bytes] = {}

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def put(self, stream: str, key: int, data: bytes) -> None:
        self.puts.append((stream, key, data))
        self.store[(stream, key)] = data

    def get(self, stream: str, key: int) -> bytes:
        self.gets.append((stream, key))
        return self.store[(stream, key)]

    def delete(self, stream: str, key: int) -> None:
        self.store.pop((stream, key), None)


class SpyVectorStore(VectorStore):
    """VectorStore that records all calls for verification."""

    def __init__(self) -> None:
        super().__init__()
        self.puts: list[tuple[str, int]] = []
        self.searches: list[tuple[str, int]] = []
        self.vectors: dict[str, dict[int, Any]] = {}

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def put(self, stream: str, key: int, embedding: Any) -> None:
        self.puts.append((stream, key))
        self.vectors.setdefault(stream, {})[key] = embedding

    def search(self, stream: str, query: Any, k: int) -> list[tuple[int, float]]:
        self.searches.append((stream, k))
        vectors = self.vectors.get(stream, {})
        if not vectors:
            return []
        scored = [(key, float(emb @ query)) for key, emb in vectors.items()]
        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[:k]

    def delete(self, stream: str, key: int) -> None:
        self.vectors.get(stream, {}).pop(key, None)


@pytest.fixture
def memory_spy_session():
    from dimos.memory2.store.memory import MemoryStore

    blob_spy = SpyBlobStore()
    vec_spy = SpyVectorStore()
    with MemoryStore(blob_store=blob_spy, vector_store=vec_spy) as store:
        yield store, blob_spy, vec_spy


@pytest.fixture
def sqlite_spy_session(tmp_path):
    from dimos.memory2.store.sqlite import SqliteStore

    blob_spy = SpyBlobStore()
    vec_spy = SpyVectorStore()
    with SqliteStore(
        path=str(tmp_path / "spy.db"), blob_store=blob_spy, vector_store=vec_spy
    ) as store:
        yield store, blob_spy, vec_spy


@pytest.fixture(params=["memory_spy_session", "sqlite_spy_session"])
def spy_session(request: pytest.FixtureRequest):
    return request.getfixturevalue(request.param)


class TestStoreDelegation:
    """Verify all backends delegate to pluggable BlobStore and VectorStore."""

    def test_append_calls_blob_put(self, spy_session) -> None:
        store, blob_spy, _vec_spy = spy_session
        s = store.stream("blobs", str)
        s.append("first", ts=1.0)
        s.append("second", ts=2.0)

        assert len(blob_spy.puts) == 2
        assert all(stream == "blobs" for stream, _k, _d in blob_spy.puts)

    def test_iterate_calls_blob_get(self, spy_session) -> None:
        store, blob_spy, _vec_spy = spy_session
        s = store.stream("blobs", str)
        s.append("a", ts=1.0)
        s.append("b", ts=2.0)

        blob_spy.gets.clear()
        for obs in s:
            _ = obs.data
        assert len(blob_spy.gets) == 2

    def test_append_embedding_calls_vector_put(self, spy_session) -> None:
        import numpy as np

        from dimos.models.embedding.base import Embedding

        def _emb(v: list[float]) -> Embedding:
            a = np.array(v, dtype=np.float32)
            return Embedding(vector=a / (np.linalg.norm(a) + 1e-10))

        store, _blob_spy, vec_spy = spy_session
        s = store.stream("vecs", str)
        s.append("a", ts=1.0, embedding=_emb([1, 0, 0]))
        s.append("b", ts=2.0, embedding=_emb([0, 1, 0]))
        s.append("c", ts=3.0)  # no embedding

        assert len(vec_spy.puts) == 2

    def test_search_calls_vector_search(self, spy_session) -> None:
        import numpy as np

        from dimos.models.embedding.base import Embedding

        def _emb(v: list[float]) -> Embedding:
            a = np.array(v, dtype=np.float32)
            return Embedding(vector=a / (np.linalg.norm(a) + 1e-10))

        store, _blob_spy, vec_spy = spy_session
        s = store.stream("vecs", str)
        s.append("north", ts=1.0, embedding=_emb([0, 1, 0]))
        s.append("east", ts=2.0, embedding=_emb([1, 0, 0]))

        results = s.search(_emb([0, 1, 0]), k=2).fetch()
        assert len(vec_spy.searches) == 1
        assert results[0].data == "north"


class TestStandaloneComponents:
    """Verify each SQLite component works standalone with path= (no Store needed)."""

    def test_observation_store_standalone(self, tmp_path) -> None:
        from dimos.memory2.codecs.base import codec_for
        from dimos.memory2.observationstore.sqlite import SqliteObservationStore
        from dimos.memory2.type.filter import StreamQuery
        from dimos.memory2.type.observation import Observation

        db = str(tmp_path / "obs.db")
        codec = codec_for(str)
        with SqliteObservationStore(path=db, name="events", codec=codec) as store:
            obs = Observation(id=0, ts=1.0, _data="hello")
            row_id = store.insert(obs)
            store.commit()
            assert row_id == 1

            results = list(store.query(StreamQuery()))
            assert len(results) == 1
            assert results[0].ts == 1.0

    def test_blob_store_standalone(self, tmp_path) -> None:
        from dimos.memory2.blobstore.sqlite import SqliteBlobStore

        db = str(tmp_path / "blob.db")
        with SqliteBlobStore(path=db) as store:
            store.put("stream1", 1, b"data1")
            store.put("stream1", 2, b"data2")
            assert store.get("stream1", 1) == b"data1"
            assert store.get("stream1", 2) == b"data2"

    def test_vector_store_standalone(self, tmp_path) -> None:
        import numpy as np

        from dimos.memory2.vectorstore.sqlite import SqliteVectorStore
        from dimos.models.embedding.base import Embedding

        db = str(tmp_path / "vec.db")
        with SqliteVectorStore(path=db) as store:
            emb1 = Embedding(vector=np.array([1, 0, 0], dtype=np.float32))
            emb2 = Embedding(vector=np.array([0, 1, 0], dtype=np.float32))
            store.put("vecs", 1, emb1)
            store.put("vecs", 2, emb2)

            results = store.search("vecs", emb1, k=2)
            assert len(results) == 2
            assert results[0][0] == 1  # closest to emb1 is itself

    def test_conn_and_path_mutually_exclusive(self, tmp_path) -> None:
        import sqlite3

        from dimos.memory2.blobstore.sqlite import SqliteBlobStore
        from dimos.memory2.observationstore.sqlite import SqliteObservationStore
        from dimos.memory2.vectorstore.sqlite import SqliteVectorStore

        conn = sqlite3.connect(":memory:")
        db = str(tmp_path / "test.db")

        with pytest.raises(ValueError, match="either conn or path"):
            SqliteBlobStore(conn=conn, path=db)
        with pytest.raises(ValueError, match="either conn or path"):
            SqliteVectorStore(conn=conn, path=db)
        with pytest.raises(ValueError, match="either conn or path"):
            SqliteObservationStore(conn=conn, name="x", path=db)
        with pytest.raises(ValueError, match="either conn or path"):
            SqliteBlobStore()
        with pytest.raises(ValueError, match="either conn or path"):
            SqliteVectorStore()
        with pytest.raises(ValueError, match="either conn or path"):
            SqliteObservationStore(name="x")
        conn.close()


class TestStreamAccessor:
    """Test attribute-style stream access via store.streams."""

    def test_accessor_returns_same_stream(self, session: Store) -> None:
        s = session.stream("images", bytes)
        assert session.streams.images is s

    def test_accessor_dir_lists_streams(self, session: Store) -> None:
        session.stream("alpha", str)
        session.stream("beta", int)
        names = dir(session.streams)
        assert "alpha" in names
        assert "beta" in names

    def test_accessor_missing_raises(self, session: Store) -> None:
        with pytest.raises(AttributeError, match="nonexistent"):
            _ = session.streams.nonexistent

    def test_accessor_getitem(self, session: Store) -> None:
        s = session.stream("data", float)
        assert session.streams["data"] is s

    def test_accessor_getitem_missing_raises(self, session: Store) -> None:
        with pytest.raises(KeyError):
            session.streams["nope"]

    def test_accessor_repr(self, session: Store) -> None:
        session.stream("x", str)
        r = repr(session.streams)
        assert "x" in r
        assert "StreamAccessor" in r

    def test_accessor_dynamic(self, session: Store) -> None:
        assert "late" not in dir(session.streams)
        session.stream("late", str)
        assert "late" in dir(session.streams)
