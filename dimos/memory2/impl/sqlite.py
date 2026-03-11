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

from dataclasses import dataclass
import sqlite3
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.backend import BackendConfig
from dimos.memory2.livechannel.subject import SubjectChannel
from dimos.memory2.store import Session, Store, StoreConfig
from dimos.protocol.service.spec import Configurable

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.backend import Backend, LiveChannel
    from dimos.memory2.filter import StreamQuery
    from dimos.memory2.type import Observation

T = TypeVar("T")


class SqliteBackend(Configurable[BackendConfig], Generic[T]):
    """SQLite-backed observation storage for a single stream (table)."""

    default_config: type[BackendConfig] = BackendConfig

    def __init__(self, conn: sqlite3.Connection, name: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._conn = conn
        self._name = name
        self._channel: LiveChannel[T] = self.config.live_channel or SubjectChannel()

    @property
    def name(self) -> str:
        return self._name

    @property
    def live_channel(self) -> LiveChannel[T]:
        return self._channel

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        raise NotImplementedError

    def append(self, obs: Observation[T]) -> Observation[T]:
        raise NotImplementedError

    def count(self, query: StreamQuery) -> int:
        raise NotImplementedError


class SqliteSession(Session):
    """Session owning a single SQLite connection."""

    def __init__(self, conn: sqlite3.Connection, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._conn = conn

    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        return SqliteBackend(self._conn, name, **config)

    def list_streams(self) -> list[str]:
        # TODO: also query DB for persisted streams not yet opened
        return list(self._streams.keys())

    def delete_stream(self, name: str) -> None:
        self._streams.pop(name, None)
        # TODO: drop underlying table/rows from SQLite

    def stop(self) -> None:
        super().stop()
        self._conn.close()


@dataclass
class SqliteStoreConfig(StoreConfig):
    """Config for SQLite-backed store."""

    path: str = "memory.db"


class SqliteStore(Store):
    """Store backed by a SQLite database file."""

    default_config: type[SqliteStoreConfig] = SqliteStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    def session(self, **kwargs: Any) -> SqliteSession:
        conn = sqlite3.connect(self.config.path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        return SqliteSession(conn, **kwargs)
