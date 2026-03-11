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
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.store import Session, Store

if TYPE_CHECKING:
    from collections.abc import Iterator
    import os

    from reactivex.abc import DisposableBase

    from dimos.memory2.backend import Backend
    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import StreamQuery
    from dimos.memory2.type import Observation

T = TypeVar("T")


class SqliteBackend(Generic[T]):
    """SQLite-backed observation storage for a single stream (table)."""

    def __init__(self, conn: sqlite3.Connection, name: str) -> None:
        self._conn = conn
        self._name = name

    @property
    def name(self) -> str:
        return self._name

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        raise NotImplementedError

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
    ) -> Observation[T]:
        raise NotImplementedError

    def count(self, query: StreamQuery) -> int:
        raise NotImplementedError

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        raise NotImplementedError


class SqliteSession(Session):
    """Session owning a single SQLite connection."""

    def __init__(self, conn: sqlite3.Connection) -> None:
        super().__init__()
        self._conn = conn

    def _create_backend(self, name: str, payload_type: type[Any] | None = None) -> Backend[Any]:
        return SqliteBackend(self._conn, name)

    def stop(self) -> None:
        super().stop()
        self._conn.close()


class SqliteStore(Store):
    """Store backed by a SQLite database file."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self._path = path

    def session(self) -> SqliteSession:
        conn = sqlite3.connect(self._path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        return SqliteSession(conn)
