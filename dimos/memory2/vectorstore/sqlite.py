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

import json
import sqlite3
from typing import TYPE_CHECKING, Any

from pydantic import Field, model_validator

from dimos.memory2.utils.sqlite import open_disposable_sqlite_connection
from dimos.memory2.utils.validation import validate_identifier
from dimos.memory2.vectorstore.base import VectorStore, VectorStoreConfig

if TYPE_CHECKING:
    from dimos.models.embedding.base import Embedding


class SqliteVectorStoreConfig(VectorStoreConfig):
    conn: sqlite3.Connection | None = Field(default=None, exclude=True)
    path: str | None = None

    @model_validator(mode="after")
    def _conn_xor_path(self) -> SqliteVectorStoreConfig:
        if self.conn is not None and self.path is not None:
            raise ValueError("Specify either conn or path, not both")
        if self.conn is None and self.path is None:
            raise ValueError("Specify either conn or path")
        return self


class SqliteVectorStore(VectorStore):
    """Vector store backed by sqlite-vec's vec0 virtual tables.

    Creates one virtual table per stream: ``"{stream}_vec"``.
    Dimensionality is determined lazily on the first ``put()``.

    Supports two construction modes:

    - ``SqliteVectorStore(conn=conn)`` — borrows an externally-managed connection.
    - ``SqliteVectorStore(path="file.db")`` — opens and owns its own connection.
    """

    config: SqliteVectorStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._conn: sqlite3.Connection = self.config.conn  # type: ignore[assignment]  # set in start() if None
        self._path = self.config.path
        self._tables: dict[str, int] = {}  # stream_name -> dimensionality

    def _ensure_table(self, stream_name: str, dim: int) -> None:
        if stream_name in self._tables:
            return
        validate_identifier(stream_name)
        self._conn.execute(
            f'CREATE VIRTUAL TABLE IF NOT EXISTS "{stream_name}_vec" '
            f"USING vec0(embedding float[{dim}] distance_metric=cosine)"
        )
        self._tables[stream_name] = dim

    def start(self) -> None:
        if self._conn is None:
            assert self._path is not None
            disposable, self._conn = open_disposable_sqlite_connection(self._path)
            self.register_disposable(disposable)

    def put(self, stream_name: str, key: int, embedding: Embedding) -> None:
        vec = embedding.to_numpy().tolist()
        self._ensure_table(stream_name, len(vec))
        self._conn.execute(
            f'INSERT OR REPLACE INTO "{stream_name}_vec" (rowid, embedding) VALUES (?, ?)',
            (key, json.dumps(vec)),
        )

    def search(self, stream_name: str, query: Embedding, k: int) -> list[tuple[int, float]]:
        validate_identifier(stream_name)
        vec = query.to_numpy().tolist()
        try:
            rows = self._conn.execute(
                f'SELECT rowid, distance FROM "{stream_name}_vec" WHERE embedding MATCH ? AND k = ?',
                (json.dumps(vec), k),
            ).fetchall()
        except sqlite3.OperationalError as e:
            if "no such table" in str(e):
                return []
            raise
        # vec0 cosine distance = 1 - cosine_similarity
        return [(int(row[0]), max(0.0, 1.0 - row[1])) for row in rows]

    def delete(self, stream_name: str, key: int) -> None:
        validate_identifier(stream_name)
        try:
            self._conn.execute(f'DELETE FROM "{stream_name}_vec" WHERE rowid = ?', (key,))
        except sqlite3.OperationalError as e:
            if "no such table" not in str(e):
                raise
