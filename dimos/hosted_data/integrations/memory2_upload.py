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

"""Create indexed memory2 snapshots and publish them to hosted storage."""

from __future__ import annotations

from contextlib import closing
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import tempfile
from typing import Any

from dimos.hosted_data.repository import (
    ReplayObject,
    sha256_file,
    upload_file,
)


@dataclass(frozen=True)
class Memory2StreamIndex:
    """Small, codec-independent summary for one memory2 stream."""

    name: str
    items: int
    start_time: float | None
    end_time: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class Memory2DatasetIndex:
    """Portable metadata used to discover and verify a remote memory2 dataset."""

    dataset: str
    filename: str
    size_bytes: int
    sha256: str
    sqlite_user_version: int
    created_at: str
    streams: tuple[Memory2StreamIndex, ...]
    object: ReplayObject | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "kind": "dimos-memory2-sqlite",
            "dataset": self.dataset,
            "filename": self.filename,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "sqlite_user_version": self.sqlite_user_version,
            "created_at": self.created_at,
            "streams": [stream.to_dict() for stream in self.streams],
            "object": self.object.to_dict() if self.object is not None else None,
        }


@dataclass(frozen=True)
class Memory2Upload:
    """Objects published for one consistent memory2 snapshot."""

    index: Memory2DatasetIndex
    dataset_object: ReplayObject
    index_object: ReplayObject

    def to_dict(self) -> dict[str, Any]:
        return {
            "dataset": self.index.to_dict(),
            "dataset_object": self.dataset_object.to_dict(),
            "index_object": self.index_object.to_dict(),
        }


def _validate_dataset_name(value: str) -> str:
    if not value or Path(value).name != value or value in {".", ".."}:
        raise ValueError("dataset must be a plain name without directories")
    return value


def snapshot_sqlite_database(source: str | Path, destination: str | Path) -> Path:
    """Create a standalone SQLite snapshot, including committed WAL data."""
    source_path = Path(source).resolve()
    destination_path = Path(destination).resolve()
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    if source_path.suffix != ".db":
        raise ValueError(f"memory2 upload currently requires a .db file, got {source_path.name!r}")
    if destination_path == source_path:
        raise ValueError("snapshot destination must differ from the source database")

    destination_path.parent.mkdir(parents=True, exist_ok=True)
    destination_path.unlink(missing_ok=True)
    source_uri = f"{source_path.as_uri()}?mode=ro"
    with (
        closing(sqlite3.connect(source_uri, uri=True, timeout=30.0)) as source_connection,
        closing(sqlite3.connect(destination_path, timeout=30.0)) as destination_connection,
    ):
        source_connection.backup(destination_connection)
        check = destination_connection.execute("PRAGMA integrity_check").fetchone()
        if check is None or check[0] != "ok":
            raise RuntimeError(f"memory2 snapshot failed integrity_check: {check!r}")
        destination_connection.execute("PRAGMA journal_mode=DELETE")
    return destination_path


def _quote_identifier(value: str) -> str:
    return '"' + value.replace('"', '""') + '"'


def index_memory2_snapshot(
    path: str | Path,
    *,
    dataset: str | None = None,
    remote_object: ReplayObject | None = None,
) -> Memory2DatasetIndex:
    """Inspect a standalone memory2 SQLite file without loading its codecs."""
    snapshot = Path(path)
    if not snapshot.is_file():
        raise FileNotFoundError(snapshot)

    with closing(sqlite3.connect(f"{snapshot.resolve().as_uri()}?mode=ro", uri=True)) as connection:
        user_version = int(connection.execute("PRAGMA user_version").fetchone()[0])
        registry_exists = connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = '_streams'"
        ).fetchone()
        if registry_exists is None:
            raise ValueError(f"{snapshot} is not a memory2 SQLite database")
        names = [
            str(row[0])
            for row in connection.execute("SELECT name FROM _streams ORDER BY name").fetchall()
        ]
        streams: list[Memory2StreamIndex] = []
        for name in names:
            table = _quote_identifier(name)
            row = connection.execute(f"SELECT COUNT(*), MIN(ts), MAX(ts) FROM {table}").fetchone()
            if row is None:
                raise RuntimeError(f"failed to index memory2 stream {name!r}")
            streams.append(
                Memory2StreamIndex(
                    name=name,
                    items=int(row[0]),
                    start_time=float(row[1]) if row[1] is not None else None,
                    end_time=float(row[2]) if row[2] is not None else None,
                )
            )

    return Memory2DatasetIndex(
        dataset=_validate_dataset_name(dataset or snapshot.stem),
        filename=snapshot.name,
        size_bytes=snapshot.stat().st_size,
        sha256=sha256_file(snapshot),
        sqlite_user_version=user_version,
        created_at=datetime.now(timezone.utc).isoformat(),
        streams=tuple(streams),
        object=remote_object,
    )


def write_memory2_index(path: str | Path, index: Memory2DatasetIndex) -> Path:
    """Write the completed dataset index."""
    destination = Path(path)
    destination.write_text(
        json.dumps(index.to_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return destination


def upload_memory2_dataset(
    *,
    path: str | Path,
    server_url: str,
    owner: str,
    repository: str,
    token: str | None = None,
    dataset: str | None = None,
    retries: int = 3,
    backoff_seconds: float = 0.5,
) -> Memory2Upload:
    """Snapshot and upload a memory2 database and its searchable JSON index."""
    source = Path(path)
    with tempfile.TemporaryDirectory(prefix="dimos-memory2-upload-") as temporary:
        temporary_dir = Path(temporary)
        snapshot = snapshot_sqlite_database(source, temporary_dir / source.name)
        local_index = index_memory2_snapshot(snapshot, dataset=dataset or source.stem)
        dataset_object = upload_file(
            server_url=server_url,
            owner=owner,
            repository=repository,
            path=snapshot,
            token=token,
            retries=retries,
            backoff_seconds=backoff_seconds,
        )
        completed_index = index_memory2_snapshot(
            snapshot,
            dataset=local_index.dataset,
            remote_object=dataset_object,
        )
        index_path = write_memory2_index(
            temporary_dir / f"{completed_index.dataset}.memory2.json",
            completed_index,
        )
        index_object = upload_file(
            server_url=server_url,
            owner=owner,
            repository=repository,
            path=index_path,
            token=token,
            retries=retries,
            backoff_seconds=backoff_seconds,
        )
        return Memory2Upload(
            index=completed_index,
            dataset_object=dataset_object,
            index_object=index_object,
        )
