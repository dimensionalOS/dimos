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

"""Stream registry: persists fully-resolved backend config per stream."""

from __future__ import annotations

import importlib
import json
import sqlite3
from typing import Any

from pydantic import Field

from dimos.protocol.service.spec import BaseConfig, Configurable


def qual(cls: type) -> str:
    """Fully qualified class name, e.g. 'dimos.memory2.blobstore.sqlite.SqliteBlobStore'."""
    return f"{cls.__module__}.{cls.__qualname__}"


def deserialize_component(data: dict[str, Any]) -> Any:
    """Instantiate a component from its ``{"class": ..., "config": ...}`` dict."""
    module_path, _, cls_name = data["class"].rpartition(".")
    mod = importlib.import_module(module_path)
    cls = getattr(mod, cls_name)
    return cls(**data["config"])


class RegistryStoreConfig(BaseConfig):
    conn: sqlite3.Connection = Field(exclude=True)


class RegistryStore(Configurable):
    """SQLite persistence for stream name -> config JSON."""

    config: RegistryStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._conn: sqlite3.Connection = self.config.conn
        self._conn.execute(
            "CREATE TABLE IF NOT EXISTS _streams ("
            "    name   TEXT PRIMARY KEY,"
            "    config TEXT NOT NULL"
            ")"
        )
        self._conn.commit()

    def get(self, name: str) -> dict[str, Any] | None:
        row = self._conn.execute("SELECT config FROM _streams WHERE name = ?", (name,)).fetchone()
        if row is None:
            return None
        return json.loads(row[0])  # type: ignore[no-any-return]

    def put(self, name: str, config: dict[str, Any]) -> None:
        self._conn.execute(
            "INSERT OR REPLACE INTO _streams (name, config) VALUES (?, ?)",
            (name, json.dumps(config)),
        )
        self._conn.commit()

    def delete(self, name: str) -> None:
        self._conn.execute("DELETE FROM _streams WHERE name = ?", (name,))
        self._conn.commit()

    def list_streams(self) -> list[str]:
        rows = self._conn.execute("SELECT name FROM _streams").fetchall()
        return [r[0] for r in rows]
