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

"""Tests for RegistryStore and serialization round-trips."""

from __future__ import annotations

import pytest

from dimos.memory2.blobstore.file import FileBlobStore
from dimos.memory2.blobstore.sqlite import SqliteBlobStore, SqliteBlobStoreConfig
from dimos.memory2.notifier.subject import SubjectNotifier
from dimos.memory2.observationstore.sqlite import SqliteObservationStoreConfig
from dimos.memory2.registry import RegistryStore, deserialize_component, qual
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vectorstore.sqlite import SqliteVectorStore, SqliteVectorStoreConfig


class TestQual:
    def test_qual_blob_store(self) -> None:
        assert qual(SqliteBlobStore) == "dimos.memory2.blobstore.sqlite.SqliteBlobStore"

    def test_qual_file_blob_store(self) -> None:
        assert qual(FileBlobStore) == "dimos.memory2.blobstore.file.FileBlobStore"

    def test_qual_vector_store(self) -> None:
        assert qual(SqliteVectorStore) == "dimos.memory2.vectorstore.sqlite.SqliteVectorStore"

    def test_qual_notifier(self) -> None:
        assert qual(SubjectNotifier) == "dimos.memory2.notifier.subject.SubjectNotifier"


class TestRegistryStore:
    def test_put_get_round_trip(self, tmp_path) -> None:
        from dimos.memory2.utils.sqlite import open_sqlite_connection

        conn = open_sqlite_connection(str(tmp_path / "reg.db"))
        reg = RegistryStore(conn=conn)

        config = {"payload_module": "builtins.str", "codec_id": "pickle"}
        reg.put("my_stream", config)
        result = reg.get("my_stream")
        assert result == config
        conn.close()

    def test_get_missing(self, tmp_path) -> None:
        from dimos.memory2.utils.sqlite import open_sqlite_connection

        conn = open_sqlite_connection(str(tmp_path / "reg.db"))
        reg = RegistryStore(conn=conn)
        assert reg.get("nonexistent") is None
        conn.close()

    def test_list_streams(self, tmp_path) -> None:
        from dimos.memory2.utils.sqlite import open_sqlite_connection

        conn = open_sqlite_connection(str(tmp_path / "reg.db"))
        reg = RegistryStore(conn=conn)
        reg.put("a", {"x": 1})
        reg.put("b", {"x": 2})
        assert sorted(reg.list_streams()) == ["a", "b"]
        conn.close()

    def test_delete(self, tmp_path) -> None:
        from dimos.memory2.utils.sqlite import open_sqlite_connection

        conn = open_sqlite_connection(str(tmp_path / "reg.db"))
        reg = RegistryStore(conn=conn)
        reg.put("x", {"y": 1})
        reg.delete("x")
        assert reg.get("x") is None
        conn.close()

    def test_upsert(self, tmp_path) -> None:
        from dimos.memory2.utils.sqlite import open_sqlite_connection

        conn = open_sqlite_connection(str(tmp_path / "reg.db"))
        reg = RegistryStore(conn=conn)
        reg.put("x", {"v": 1})
        reg.put("x", {"v": 2})
        assert reg.get("x") == {"v": 2}
        conn.close()


class TestComponentSerialization:
    def test_sqlite_observation_store_config(self) -> None:
        cfg = SqliteObservationStoreConfig(page_size=512, path="test.db")
        dumped = cfg.model_dump()
        restored = SqliteObservationStoreConfig(**dumped)
        assert restored.page_size == 512

    def test_sqlite_blob_store_config(self) -> None:
        cfg = SqliteBlobStoreConfig(path="/tmp/test.db")
        dumped = cfg.model_dump()
        restored = SqliteBlobStoreConfig(**dumped)
        assert restored.path == "/tmp/test.db"

    def test_sqlite_blob_store_roundtrip(self, tmp_path) -> None:
        store = SqliteBlobStore(path=str(tmp_path / "blob.db"))
        data = store.serialize()
        assert data["class"] == qual(SqliteBlobStore)
        restored = deserialize_component(data)
        assert isinstance(restored, SqliteBlobStore)

    def test_file_blob_store_roundtrip(self, tmp_path) -> None:
        store = FileBlobStore(root=str(tmp_path / "blobs"))
        data = store.serialize()
        assert data["class"] == qual(FileBlobStore)
        restored = deserialize_component(data)
        assert isinstance(restored, FileBlobStore)
        assert str(restored._root) == str(tmp_path / "blobs")

    def test_sqlite_vector_store_config(self) -> None:
        cfg = SqliteVectorStoreConfig(path="/tmp/vec.db")
        dumped = cfg.model_dump()
        restored = SqliteVectorStoreConfig(**dumped)
        assert restored.path == "/tmp/vec.db"

    def test_sqlite_vector_store_roundtrip(self, tmp_path) -> None:
        store = SqliteVectorStore(path=str(tmp_path / "vec.db"))
        data = store.serialize()
        assert data["class"] == qual(SqliteVectorStore)
        restored = deserialize_component(data)
        assert isinstance(restored, SqliteVectorStore)

    def test_subject_notifier_roundtrip(self) -> None:
        notifier = SubjectNotifier()
        data = notifier.serialize()
        assert data["class"] == qual(SubjectNotifier)
        restored = deserialize_component(data)
        assert isinstance(restored, SubjectNotifier)

    def test_deserialize_component(self, tmp_path) -> None:
        store = FileBlobStore(root=str(tmp_path / "blobs"))
        data = store.serialize()
        restored = deserialize_component(data)
        assert isinstance(restored, FileBlobStore)


class TestBackendSerialization:
    def test_backend_serialize(self, tmp_path) -> None:
        from dimos.memory2.backend import Backend
        from dimos.memory2.codecs.pickle import PickleCodec
        from dimos.memory2.observationstore.memory import ListObservationStore

        backend = Backend(
            metadata_store=ListObservationStore(name="test"),
            codec=PickleCodec(),
            blob_store=FileBlobStore(root=str(tmp_path / "blobs")),
            notifier=SubjectNotifier(),
        )
        data = backend.serialize()
        assert data["codec_id"] == "pickle"
        assert data["blob_store"]["class"] == qual(FileBlobStore)
        assert data["notifier"]["class"] == qual(SubjectNotifier)


class TestStoreReopen:
    def test_reopen_preserves_data(self, tmp_path) -> None:
        """Create a store, write data, close, reopen, read back."""
        db = str(tmp_path / "test.db")
        with SqliteStore(path=db) as store:
            s = store.stream("nums", int)
            s.append(42, ts=1.0)
            s.append(99, ts=2.0)

        with SqliteStore(path=db) as store2:
            s2 = store2.stream("nums", int)
            assert s2.count() == 2
            obs = s2.fetch()
            assert [o.data for o in obs] == [42, 99]

    def test_reopen_preserves_codec(self, tmp_path) -> None:
        """Codec ID is stored and restored on reopen."""
        db = str(tmp_path / "codec.db")
        with SqliteStore(path=db) as store:
            s = store.stream("data", str, codec="pickle")
            s.append("hello", ts=1.0)

        with SqliteStore(path=db) as store2:
            s2 = store2.stream("data", str)
            assert s2.first().data == "hello"

    def test_reopen_preserves_eager_blobs(self, tmp_path) -> None:
        """eager_blobs override is stored in registry and restored on reopen."""
        db = str(tmp_path / "eager.db")
        with SqliteStore(path=db) as store:
            s = store.stream("data", str, eager_blobs=True)
            s.append("test", ts=1.0)

        with SqliteStore(path=db) as store2:
            stored = store2._registry.get("data")
            assert stored is not None
            assert stored["eager_blobs"] is True

    def test_reopen_preserves_file_blob_store(self, tmp_path) -> None:
        """FileBlobStore override is stored and restored on reopen."""
        db = str(tmp_path / "file_blob.db")
        blob_dir = str(tmp_path / "blobs")
        with SqliteStore(path=db) as store:
            fbs = FileBlobStore(root=blob_dir)
            fbs.start()
            s = store.stream("imgs", str, blob_store=fbs)
            s.append("image_data", ts=1.0)

        with SqliteStore(path=db) as store2:
            stored = store2._registry.get("imgs")
            assert stored is not None
            assert stored["blob_store"]["class"] == qual(FileBlobStore)
            assert stored["blob_store"]["config"]["root"] == blob_dir

    def test_reopen_type_mismatch_raises(self, tmp_path) -> None:
        """Opening a stream with a different payload type raises ValueError."""
        db = str(tmp_path / "mismatch.db")
        with SqliteStore(path=db) as store:
            store.stream("nums", int)

        with SqliteStore(path=db) as store2:
            with pytest.raises(ValueError, match="was created with type"):
                store2.stream("nums", str)

    def test_reopen_list_streams(self, tmp_path) -> None:
        """list_streams includes streams from registry on reopen."""
        db = str(tmp_path / "list.db")
        with SqliteStore(path=db) as store:
            store.stream("a", int)
            store.stream("b", str)

        with SqliteStore(path=db) as store2:
            assert sorted(store2.list_streams()) == ["a", "b"]

    def test_reopen_without_payload_type(self, tmp_path) -> None:
        """Reopening a known stream without payload_type works."""
        db = str(tmp_path / "no_type.db")
        with SqliteStore(path=db) as store:
            s = store.stream("data", str)
            s.append("hello", ts=1.0)

        with SqliteStore(path=db) as store2:
            s2 = store2.stream("data")
            assert s2.first().data == "hello"

    def test_reopen_preserves_page_size(self, tmp_path) -> None:
        """page_size is stored in registry and restored on reopen."""
        db = str(tmp_path / "page.db")
        with SqliteStore(path=db, page_size=512) as store:
            store.stream("data", str)

        with SqliteStore(path=db) as store2:
            stored = store2._registry.get("data")
            assert stored is not None
            assert stored["page_size"] == 512
