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

from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from dimos.memory2.blobstore.base import BlobStore


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
