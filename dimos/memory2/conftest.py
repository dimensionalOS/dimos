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

"""Shared fixtures for memory2 tests."""

from __future__ import annotations

import sqlite3
import tempfile
from typing import TYPE_CHECKING, cast

import pytest

from dimos.memory2.blobstore.file import FileBlobStore
from dimos.memory2.blobstore.sqlite import SqliteBlobStore
from dimos.memory2.store.memory import MemoryStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.models.embedding.clip import CLIPModel

if TYPE_CHECKING:
    from collections.abc import Iterator
    from pathlib import Path

    from dimos.memory2.blobstore.base import BlobStore
    from dimos.memory2.store.base import Store


@pytest.fixture(scope="module")
def clip() -> CLIPModel:
    return CLIPModel()


@pytest.fixture
def memory_store() -> Iterator[MemoryStore]:
    with MemoryStore() as store:
        yield store


@pytest.fixture
def memory_session(memory_store: MemoryStore) -> Iterator[MemoryStore]:
    """Alias: in the new architecture, the store IS the session."""
    yield memory_store


@pytest.fixture
def sqlite_store() -> Iterator[SqliteStore]:
    with tempfile.NamedTemporaryFile(suffix=".db") as f:
        store = SqliteStore(path=f.name)
        with store:
            yield store


@pytest.fixture
def sqlite_session(sqlite_store: SqliteStore) -> Iterator[SqliteStore]:
    """Alias: in the new architecture, the store IS the session."""
    yield sqlite_store


@pytest.fixture(params=["memory_store", "sqlite_store"])
def session(request: pytest.FixtureRequest) -> Store:
    """Parametrized fixture that runs tests against both backends.

    Named 'session' to minimize test changes — tests use session.stream() which
    now goes directly to Store.stream().
    """
    return cast("Store", request.getfixturevalue(request.param))


@pytest.fixture
def file_blob_store(tmp_path: Path) -> Iterator[FileBlobStore]:
    with FileBlobStore(root=str(tmp_path / "blobs")) as store:
        yield store


@pytest.fixture
def sqlite_blob_store() -> Iterator[SqliteBlobStore]:
    conn = sqlite3.connect(":memory:")
    with SqliteBlobStore(conn=conn) as store:
        yield store


@pytest.fixture(params=["file_blob_store", "sqlite_blob_store"])
def blob_store(request: pytest.FixtureRequest) -> BlobStore:
    return cast("BlobStore", request.getfixturevalue(request.param))
