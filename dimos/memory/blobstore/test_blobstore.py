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

"""Grid tests for BlobStore implementations."""

from __future__ import annotations

from contextlib import ExitStack
from typing import TYPE_CHECKING

import pytest

from dimos.memory.blobstore.memory import MemoryBlobStore
from dimos.memory.blobstore.sqlite import SqliteBlobStore
from dimos.memory.utils.sqlite import open_disposable_sqlite_connection

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.memory.blobstore.base import BlobStore


class TestBlobStore:
    def test_put_get_roundtrip(self, blob_store: BlobStore) -> None:
        data = b"hello world"
        blob_store.put("stream_a", 1, data)
        assert blob_store.get("stream_a", 1) == data

    def test_get_missing_raises(self, blob_store: BlobStore) -> None:
        with pytest.raises(KeyError):
            blob_store.get("nonexistent", 999)

    def test_put_overwrite(self, blob_store: BlobStore) -> None:
        blob_store.put("s", 1, b"first")
        blob_store.put("s", 1, b"second")
        assert blob_store.get("s", 1) == b"second"

    def test_delete(self, blob_store: BlobStore) -> None:
        blob_store.put("s", 1, b"data")
        blob_store.delete("s", 1)
        with pytest.raises(KeyError):
            blob_store.get("s", 1)

    def test_delete_missing_raises(self, blob_store: BlobStore) -> None:
        with pytest.raises(KeyError):
            blob_store.delete("s", 999)

    def test_stream_isolation(self, blob_store: BlobStore) -> None:
        blob_store.put("a", 1, b"alpha")
        blob_store.put("b", 1, b"beta")
        assert blob_store.get("a", 1) == b"alpha"
        assert blob_store.get("b", 1) == b"beta"

    def test_large_blob(self, blob_store: BlobStore) -> None:
        data = bytes(range(256)) * 1000  # 256 KB
        blob_store.put("big", 0, data)
        assert blob_store.get("big", 0) == data
        assert blob_store.get("big", 0) == data

    def test_size_bytes(self, blob_store: BlobStore) -> None:
        assert blob_store.size_bytes("s") == 0
        blob_store.put("s", 1, b"12345")
        blob_store.put("s", 2, b"1234567890")
        assert blob_store.size_bytes("s") == 15
        blob_store.delete("s", 1)
        assert blob_store.size_bytes("s") == 10
        assert blob_store.size_bytes("other") == 0


class TestSqliteBlobStore:
    def test_owned_mutations_persist_across_reopen(self, tmp_path: Path) -> None:
        path = str(tmp_path / "blobs.db")
        with SqliteBlobStore(path=path) as store:
            store.put("s", 1, b"old")
            store.put("s", 2, b"remove")

        with SqliteBlobStore(path=path) as store:
            assert store.get("s", 1) == b"old"
            store.put("s", 1, b"new")
            store.delete("s", 2)

        with SqliteBlobStore(path=path) as store:
            assert store.get("s", 1) == b"new"
            with pytest.raises(KeyError):
                store.get("s", 2)

    def test_borrowed_transaction_remains_caller_owned(self, tmp_path: Path) -> None:
        with ExitStack() as stack:
            disposable, conn = open_disposable_sqlite_connection(tmp_path / "blobs.db")
            stack.callback(disposable.dispose)
            isolation_level = conn.isolation_level
            with SqliteBlobStore(conn=conn) as store:
                store.put("s", 1, b"committed")
                conn.commit()
                store.put("s", 2, b"pending")

            assert conn.isolation_level == isolation_level
            assert conn.in_transaction
            conn.rollback()
            assert conn.execute('SELECT data FROM "s_blob"').fetchall() == [(b"committed",)]


class TestMemoryBlobStore:
    def test_max_items(self) -> None:
        # rolling window of the newest blobs, per stream
        with MemoryBlobStore(max_items=2) as store:
            for key in (1, 2, 3):
                store.put("a", key, b"a%d" % key)
            store.put("b", 1, b"b1")

            with pytest.raises(KeyError):
                store.get("a", 1)
            assert [store.get("a", key) for key in (2, 3)] == [b"a2", b"a3"]
            assert store.get("b", 1) == b"b1"
