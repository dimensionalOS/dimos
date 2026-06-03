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

import sqlite3
from typing import Any

from pydantic import Field, model_validator

from dimos.memory2.blobstore.base import BlobStore, BlobStoreConfig
from dimos.memory2.utils.sqlite import open_disposable_sqlite_connection
from dimos.memory2.utils.validation import validate_identifier


class SqliteBlobStoreConfig(BlobStoreConfig):
    conn: sqlite3.Connection | None = Field(default=None, exclude=True)
    path: str | None = None

    @model_validator(mode="after")
    def _conn_xor_path(self) -> SqliteBlobStoreConfig:
        if self.conn is not None and self.path is not None:
            raise ValueError("Specify either conn or path, not both")
        if self.conn is None and self.path is None:
            raise ValueError("Specify either conn or path")
        return self


class SqliteBlobStore(BlobStore):
    """Stores blobs in a separate SQLite table per stream.

    Table layout per stream::

        CREATE TABLE "{stream}_blob" (
            id   INTEGER PRIMARY KEY,
            data BLOB NOT NULL
        );

    Supports two construction modes:

    - ``SqliteBlobStore(conn=conn)`` — borrows an externally-managed connection.
    - ``SqliteBlobStore(path="file.db")`` — opens and owns its own connection.

    Does NOT commit; the caller (typically Backend) is responsible for commits.
    """

    config: SqliteBlobStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._conn: sqlite3.Connection = self.config.conn  # type: ignore[assignment]  # set in start() if None
        self._path = self.config.path
        self._tables: set[str] = set()

    def _ensure_table(self, stream_name: str) -> None:
        if stream_name in self._tables:
            return
        validate_identifier(stream_name)
        self._conn.execute(
            f'CREATE TABLE IF NOT EXISTS "{stream_name}_blob" '
            "(id INTEGER PRIMARY KEY, data BLOB NOT NULL)"
        )
        self._tables.add(stream_name)

    def start(self) -> None:
        if self._conn is None:
            assert self._path is not None
            disposable, self._conn = open_disposable_sqlite_connection(self._path)
            self.register_disposable(disposable)

    def put(self, stream_name: str, key: int, data: bytes) -> None:
        self._ensure_table(stream_name)
        self._conn.execute(
            f'INSERT OR REPLACE INTO "{stream_name}_blob" (id, data) VALUES (?, ?)',
            (key, data),
        )

    def get(self, stream_name: str, key: int) -> bytes:
        try:
            row = self._conn.execute(
                f'SELECT data FROM "{stream_name}_blob" WHERE id = ?', (key,)
            ).fetchone()
        except Exception:
            raise KeyError(f"No blob for stream={stream_name!r}, key={key}")
        if row is None:
            raise KeyError(f"No blob for stream={stream_name!r}, key={key}")
        result: bytes = row[0]
        return result

    def delete(self, stream_name: str, key: int) -> None:
        try:
            cur = self._conn.execute(f'DELETE FROM "{stream_name}_blob" WHERE id = ?', (key,))
        except Exception:
            raise KeyError(f"No blob for stream={stream_name!r}, key={key}") from None
        if cur.rowcount == 0:
            raise KeyError(f"No blob for stream={stream_name!r}, key={key}")
