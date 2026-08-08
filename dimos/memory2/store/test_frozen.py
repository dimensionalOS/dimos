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

from pathlib import Path
import sqlite3

import pytest

from dimos.memory2.blobstore.sqlite import SqliteBlobStore
from dimos.memory2.store.frozen import FrozenMemoryStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vectorstore.sqlite import SqliteVectorStore


@pytest.fixture
def recorded_stores(tmp_path: Path):
    source_path = tmp_path / "source.db"
    derived_path = tmp_path / "derived.db"
    with SqliteStore(path=str(source_path)) as source:
        source.stream("camera", str).append("past", ts=10.0)
        source.stream("camera", str).append("at-cutoff", ts=20.0)
        source.stream("camera", str).append("future", ts=30.0)
    with SqliteStore(path=str(derived_path)) as derived:
        derived.stream("global_map", str).append("map-10", ts=10.0)
        derived.stream("global_map", str).append("map-20", ts=20.0)
        derived.stream("global_map", str).append("map-30", ts=30.0)
    for path in (source_path, derived_path):
        with sqlite3.connect(path) as connection:
            connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            connection.execute("PRAGMA journal_mode=DELETE")
    yield source_path, derived_path


def test_frozen_memory_merges_stores_and_includes_cutoff(recorded_stores) -> None:
    source_path, derived_path = recorded_stores
    with FrozenMemoryStore(
        SqliteStore(path=str(source_path), must_exist=True, read_only=True),
        derived=SqliteStore(path=str(derived_path), must_exist=True, read_only=True),
        through_timestamp=20.0,
    ) as memory:
        assert memory.list_streams() == ["camera", "global_map"]
        assert [obs.data for obs in memory.streams.camera] == ["past", "at-cutoff"]
        assert memory.streams.global_map.last().data == "map-20"


def test_frozen_memory_rejects_mutation(recorded_stores) -> None:
    source_path, derived_path = recorded_stores
    with FrozenMemoryStore(
        SqliteStore(path=str(source_path), must_exist=True, read_only=True),
        derived=SqliteStore(path=str(derived_path), must_exist=True, read_only=True),
        through_timestamp=20.0,
    ) as memory:
        with pytest.raises(PermissionError, match="read-only SQLite store"):
            memory.streams.camera.append("nope", ts=15.0)
        with pytest.raises(PermissionError, match="frozen memory"):
            memory.delete_stream("camera")
        with pytest.raises(TypeError, match="cannot be created"):
            memory.stream("new", str)


def test_read_only_sqlite_store_does_not_create_wal(recorded_stores) -> None:
    source_path, _ = recorded_stores
    wal_path = Path(f"{source_path}-wal")
    shm_path = Path(f"{source_path}-shm")
    original_bytes = source_path.read_bytes()

    with SqliteStore(path=str(source_path), must_exist=True, read_only=True) as source:
        assert source.stream("camera").last().data == "future"
        with pytest.raises(PermissionError, match="read-only SQLite store"):
            source.stream("camera").append("nope")
        with pytest.raises(PermissionError, match="read-only store"):
            source.delete_stream("camera")
        with pytest.raises(KeyError, match="does not exist in read-only store"):
            source.stream("new", str)

    assert source_path.read_bytes() == original_bytes
    assert not wal_path.exists()
    assert not shm_path.exists()


def test_read_only_store_propagates_to_external_sqlite_components(tmp_path: Path) -> None:
    source_path = tmp_path / "source.db"
    blob_path = tmp_path / "blobs.db"
    vector_path = tmp_path / "vectors.db"
    blob_store = SqliteBlobStore(path=str(blob_path))
    vector_store = SqliteVectorStore(path=str(vector_path))
    with SqliteStore(path=str(source_path)) as source:
        stream = source.stream(
            "camera",
            str,
            blob_store=blob_store,
            vector_store=vector_store,
        )
        stream.append("frame", ts=1.0)
        blob_store._conn.commit()
    for path in (source_path, blob_path, vector_path):
        with sqlite3.connect(path) as connection:
            connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            connection.execute("PRAGMA journal_mode=DELETE")

    with SqliteStore(path=str(source_path), must_exist=True, read_only=True) as source:
        stream = source.stream("camera")
        assert stream.first().data == "frame"
        backend = stream._source
        assert backend is not None
        assert backend.blob_store is not None
        assert backend.vector_store is not None
        assert backend.blob_store._conn.execute("PRAGMA query_only").fetchone() == (1,)
        assert backend.vector_store._conn.execute("PRAGMA query_only").fetchone() == (1,)
        assert not Path(f"{blob_path}-wal").exists()
        assert not Path(f"{vector_path}-wal").exists()


def test_frozen_memory_rejects_stream_collisions(recorded_stores) -> None:
    source_path, _ = recorded_stores
    source = SqliteStore(path=str(source_path), must_exist=True, read_only=True)
    duplicate = SqliteStore(path=str(source_path), must_exist=True, read_only=True)
    try:
        with pytest.raises(ValueError, match="overlapping streams: camera"):
            FrozenMemoryStore(source, derived=duplicate, through_timestamp=20.0)
    finally:
        source.stop()
        duplicate.stop()
