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

import os
import sqlite3
from typing import Annotated, Any

from pydantic import BeforeValidator

from dimos.memory2.backend import Backend
from dimos.memory2.blobstore.base import BlobStore
from dimos.memory2.blobstore.sqlite import SqliteBlobStore
from dimos.memory2.codecs.base import codec_id
from dimos.memory2.observationstore.sqlite import SqliteObservationStore
from dimos.memory2.registry import RegistryStore, deserialize_component, qual
from dimos.memory2.store.base import Store, StoreConfig
from dimos.memory2.utils.sqlite import open_disposable_sqlite_connection
from dimos.memory2.utils.validation import validate_identifier
from dimos.memory2.vectorstore.base import VectorStore
from dimos.memory2.vectorstore.sqlite import SqliteVectorStore


class SqliteStoreConfig(StoreConfig):
    """Config for SQLite-backed store."""

    path: Annotated[
        str, BeforeValidator(lambda v: os.fspath(v) if isinstance(v, os.PathLike) else v)
    ] = "memory.db"
    page_size: int = 256


class SqliteStore(Store):
    """Store backed by a SQLite database file."""

    config: SqliteStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._registry_conn = self._open_connection()
        self._registry = RegistryStore(conn=self._registry_conn)

    def _open_connection(self) -> sqlite3.Connection:
        """Open a new WAL-mode connection with sqlite-vec loaded."""
        disposable, connection = open_disposable_sqlite_connection(self.config.path)
        self.register_disposable(disposable)
        return connection

    def _assemble_backend(self, name: str, stored: dict[str, Any]) -> Backend[Any]:
        """Reconstruct a Backend from a stored config dict."""
        from dimos.memory2.codecs.base import _resolve_payload_type, codec_from_id

        payload_module = stored["payload_module"]
        codec = codec_from_id(stored["codec_id"], payload_module)
        data_type = _resolve_payload_type(payload_module)
        eager_blobs = stored.get("eager_blobs", False)
        page_size = stored.get("page_size", self.config.page_size)

        backend_conn = self._open_connection()

        # Reconstruct components from serialized config
        bs_data = stored.get("blob_store")
        if bs_data is not None:
            bs_cfg = bs_data.get("config", {})
            if bs_cfg.get("path") is None and bs_data["class"] == qual(SqliteBlobStore):
                bs: Any = SqliteBlobStore(conn=backend_conn)
            else:
                bs = deserialize_component(bs_data)
        else:
            bs = SqliteBlobStore(conn=backend_conn)

        vs_data = stored.get("vector_store")
        if vs_data is not None:
            vs_cfg = vs_data.get("config", {})
            if vs_cfg.get("path") is None and vs_data["class"] == qual(SqliteVectorStore):
                vs: Any = SqliteVectorStore(conn=backend_conn)
            else:
                vs = deserialize_component(vs_data)
        else:
            vs = SqliteVectorStore(conn=backend_conn)

        notifier_data = stored.get("notifier")
        if notifier_data is not None:
            notifier = deserialize_component(notifier_data)
        else:
            from dimos.memory2.notifier.subject import SubjectNotifier

            notifier = SubjectNotifier()

        blob_store_conn_match = isinstance(bs, SqliteBlobStore) and bs._conn is backend_conn

        metadata_store: SqliteObservationStore[Any] = SqliteObservationStore(
            conn=backend_conn,
            name=name,
            codec=codec,
            blob_store_conn_match=blob_store_conn_match and eager_blobs,
            page_size=page_size,
        )
        backend: Backend[Any] = Backend(
            metadata_store=metadata_store,
            codec=codec,
            data_type=data_type,
            blob_store=bs,
            vector_store=vs,
            notifier=notifier,
            eager_blobs=eager_blobs,
        )
        return backend

    @staticmethod
    def _serialize_backend(
        backend: Backend[Any], payload_module: str, page_size: int
    ) -> dict[str, Any]:
        """Serialize a backend's config for registry storage."""
        cfg: dict[str, Any] = {
            "payload_module": payload_module,
            "codec_id": codec_id(backend.codec),
            "eager_blobs": backend.eager_blobs,
            "page_size": page_size,
        }
        if backend.blob_store is not None:
            cfg["blob_store"] = backend.blob_store.serialize()
        if backend.vector_store is not None:
            cfg["vector_store"] = backend.vector_store.serialize()
        cfg["notifier"] = backend.notifier.serialize()
        return cfg

    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        validate_identifier(name)

        stored = self._registry.get(name)

        if stored is not None:
            # Load path: validate type, assemble from stored config
            if payload_type is not None:
                actual_module = f"{payload_type.__module__}.{payload_type.__qualname__}"
                if actual_module != stored["payload_module"]:
                    raise ValueError(
                        f"Stream {name!r} was created with type {stored['payload_module']}, "
                        f"but opened with {actual_module}"
                    )
            return self._assemble_backend(name, stored)

        # Create path: inject conn-shared defaults, then delegate to base
        if payload_type is None:
            raise TypeError(f"Stream {name!r} does not exist yet — payload_type is required")

        backend_conn = self._open_connection()

        # Inject conn-shared instances unless user provided overrides
        if not isinstance(config.get("blob_store"), BlobStore):
            config["blob_store"] = SqliteBlobStore(conn=backend_conn)
        if not isinstance(config.get("vector_store"), VectorStore):
            config["vector_store"] = SqliteVectorStore(conn=backend_conn)

        # Resolve codec early — needed for SqliteObservationStore
        codec = self._resolve_codec(payload_type, config.get("codec"))
        config["codec"] = codec

        # Create SqliteObservationStore with conn-sharing
        bs = config["blob_store"]
        blob_conn_match = isinstance(bs, SqliteBlobStore) and bs._conn is backend_conn
        eager_blobs = config.get("eager_blobs", False)
        obs_store: SqliteObservationStore[Any] = SqliteObservationStore(
            conn=backend_conn,
            name=name,
            codec=codec,
            blob_store_conn_match=blob_conn_match and eager_blobs,
            page_size=config.pop("page_size", self.config.page_size),
        )
        config["observation_store"] = obs_store

        backend = super()._create_backend(name, payload_type, **config)

        # Persist to registry
        payload_module = f"{payload_type.__module__}.{payload_type.__qualname__}"
        self._registry.put(
            name,
            self._serialize_backend(
                backend, payload_module, config["observation_store"].config.page_size
            ),
        )

        return backend

    def list_streams(self) -> list[str]:
        db_names = set(self._registry.list_streams())
        return sorted(db_names | set(self._streams.keys()))

    def delete_stream(self, name: str) -> None:
        super().delete_stream(name)
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}"')
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}_blob"')
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}_vec"')
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}_rtree"')
        self._registry.delete(name)

    def stop(self) -> None:
        super().stop()
        self._registry_conn.close()
