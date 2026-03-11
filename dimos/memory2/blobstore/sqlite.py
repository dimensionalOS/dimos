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

from typing import TYPE_CHECKING

from dimos.memory2.backend import BlobStore

if TYPE_CHECKING:
    import sqlite3


class SqliteBlobStore(BlobStore):
    """Stores blobs in a separate SQLite table per stream.

    Table layout per stream::

        CREATE TABLE "{stream}_blob" (
            id   INTEGER PRIMARY KEY,
            data BLOB NOT NULL
        );

    Does NOT own the connection — lifecycle managed externally.
    """

    def __init__(self, conn: sqlite3.Connection) -> None:
        self._conn = conn
        self._tables: set[str] = set()

    def _ensure_table(self, stream: str) -> None:
        if stream in self._tables:
            return
        self._conn.execute(
            f'CREATE TABLE IF NOT EXISTS "{stream}_blob" '
            "(id INTEGER PRIMARY KEY, data BLOB NOT NULL)"
        )
        self._tables.add(stream)

    # ── Resource lifecycle ────────────────────────────────────────

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    # ── BlobStore interface ───────────────────────────────────────

    def put(self, stream: str, key: int, data: bytes) -> None:
        self._ensure_table(stream)
        self._conn.execute(
            f'INSERT OR REPLACE INTO "{stream}_blob" (id, data) VALUES (?, ?)',
            (key, data),
        )

    def get(self, stream: str, key: int) -> bytes:
        self._ensure_table(stream)
        row = self._conn.execute(
            f'SELECT data FROM "{stream}_blob" WHERE id = ?', (key,)
        ).fetchone()
        if row is None:
            raise KeyError(f"No blob for stream={stream!r}, key={key}")
        result: bytes = row[0]
        return result

    def delete(self, stream: str, key: int) -> None:
        self._ensure_table(stream)
        self._conn.execute(f'DELETE FROM "{stream}_blob" WHERE id = ?', (key,))
