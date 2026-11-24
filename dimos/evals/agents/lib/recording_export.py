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
# See the License for the applicable law for the specific language governing
# permissions and limitations under the License.

"""Copy selected Dataset streams into a case recording without decoding payloads."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import shutil
import sqlite3
from typing import Any

from dimos.constants import CACHE_DIR
from dimos.memory.stream import Stream
from dimos.memory.utils.sqlite import conn_lock
from dimos.memory.utils.validation import validate_identifier

_CACHE_DIR = CACHE_DIR / "evals" / "recording-exports"
_cache_files: dict[tuple[str, tuple[str, ...]], Path] = {}


@dataclass(frozen=True)
class _StreamCopy:
    name: str
    data_type: type
    ids: tuple[int, ...] | None


def recording_file(streams: Sequence[Stream[Any, Any]], path: Path) -> Path:
    """*streams* written to a memory store at *path*, under their own names,
    so a subprocess can open what the case selected and nothing more.

    Sqlite-backed selections copy metadata and blob rows in SQL so JPEG frames
    are not decoded and re-encoded. Identical unfiltered exports of the same
    source file are stored once and hardlinked into each case directory (copy
    only when the cache is on another filesystem). Eval workspaces still see
    ``recording.db``; WAL/SHM stay next to the opened path.
    """
    copies = _sqlite_copies(streams)
    if copies is None:
        return _recording_file_by_append(streams, path)
    src_path, selected = copies
    names = tuple(item.name for item in selected)
    cache_key = _cache_key(src_path, names) if all(item.ids is None for item in selected) else None
    if cache_key is None:
        _sql_copy(src_path, path, selected)
        return path
    cached = _cached_export(cache_key, path)
    if cached.is_file():
        _place(cached, path)
        return path
    cached.parent.mkdir(parents=True, exist_ok=True)
    tmp = cached.with_suffix(".tmp")
    _sql_copy(src_path, tmp, selected)
    tmp.replace(cached)
    _cache_files[cache_key] = cached
    _place(cached, path)
    return path


def _embedding_of(obs: Any, stream: Stream[Any, Any]) -> Any:
    """Vectors live in the vector table; iterating a sqlite stream does not attach them."""
    embedding = getattr(obs, "embedding", None)
    if embedding is not None:
        return embedding
    backend = getattr(stream, "_source", None)
    vectors = getattr(backend, "vector_store", None)
    name = stream.name
    if vectors is None or name is None:
        return None
    return vectors.get(name, obs.id)


def _recording_file_by_append(streams: Sequence[Stream[Any, Any]], path: Path) -> Path:
    from dimos.memory.store.sqlite import SqliteStore

    with SqliteStore(path=str(path)) as store:
        for stream in streams:
            if stream.name is None:
                raise ValueError("a stream must be bound to a store to be written out")
            target: Stream[Any, Any] = store.stream(stream.name, stream.data_type)
            for obs in stream:
                target.append(
                    obs.data,
                    ts=obs.ts,
                    pose=obs.pose_tuple,
                    tags=obs.tags,
                    embedding=_embedding_of(obs, stream),
                )
    return path


def _sqlite_copies(
    streams: Sequence[Stream[Any, Any]],
) -> tuple[str, tuple[_StreamCopy, ...]] | None:
    if not streams:
        return None
    src_path: str | None = None
    selected: list[_StreamCopy] = []
    for stream in streams:
        copy = _sqlite_copy(stream)
        if copy is None:
            return None
        path, item = copy
        if src_path is None:
            src_path = path
        elif path != src_path:
            return None
        selected.append(item)
    assert src_path is not None
    return src_path, tuple(selected)


def _sqlite_copy(stream: Stream[Any, Any]) -> tuple[str, _StreamCopy] | None:
    if stream._transform is not None or stream._query.live_buffer is not None:
        return None
    name = stream.name
    data_type = stream.data_type
    if name is None or data_type is None:
        return None
    source: Any = stream._source
    while isinstance(source, Stream):
        if source._transform is not None or source._query.live_buffer is not None:
            return None
        source = source._source
    store = getattr(source, "metadata_store", None)
    conn = getattr(store, "_conn", None)
    if conn is None:
        return None
    row = conn.execute("PRAGMA database_list").fetchone()
    if row is None or not row[2]:
        return None
    return str(Path(row[2]).resolve()), _StreamCopy(name, data_type, _selected_ids(stream))


def _selected_ids(stream: Stream[Any, Any]) -> tuple[int, ...] | None:
    query = stream._query
    if (
        not query.filters
        and query.limit_val is None
        and query.offset_val is None
        and query.search_vec is None
        and query.search_text is None
    ):
        return None
    return tuple(obs.id for obs in stream)


def _sql_copy(src_path: str, dest: Path, selected: Sequence[_StreamCopy]) -> None:
    from dimos.memory.store.sqlite import SqliteStore

    dest.parent.mkdir(parents=True, exist_ok=True)
    with SqliteStore(path=str(dest)) as store:
        for item in selected:
            store.stream(item.name, item.data_type)
        conn = store._registry_conn
        with conn_lock(conn):
            conn.execute("ATTACH DATABASE ? AS src", (src_path,))
            try:
                for item in selected:
                    _copy_stream_tables(conn, item)
                conn.commit()
                conn.execute("PRAGMA wal_checkpoint(TRUNCATE)")
            except BaseException:
                conn.rollback()
                raise
            finally:
                conn.execute("DETACH DATABASE src")


def _copy_stream_tables(conn: sqlite3.Connection, item: _StreamCopy) -> None:
    validate_identifier(item.name)
    id_list = _id_predicate(item.ids)
    _insert_rows(conn, item.name, id_list)
    if _src_table_exists(conn, f"{item.name}_blob"):
        conn.execute(
            f'CREATE TABLE IF NOT EXISTS main."{item.name}_blob" '
            "(id INTEGER PRIMARY KEY, data BLOB NOT NULL)"
        )
        _insert_rows(conn, f"{item.name}_blob", id_list)
    if _src_table_exists(conn, f"{item.name}_rtree"):
        _insert_rows(conn, f"{item.name}_rtree", id_list)
    if _src_table_exists(conn, f"{item.name}_vec"):
        _ensure_vec_table(conn, item.name)
        _insert_rows(
            conn,
            f"{item.name}_vec",
            None if item.ids is None else f"rowid IN ({','.join(str(i) for i in item.ids)})",
            columns="rowid, embedding",
        )


def _id_predicate(ids: tuple[int, ...] | None) -> str | None:
    if ids is None:
        return None
    if not ids:
        return "0"
    return f"id IN ({','.join(str(i) for i in ids)})"


def _insert_rows(
    conn: sqlite3.Connection, table: str, predicate: str | None, *, columns: str | None = None
) -> None:
    src = f'src."{table}"' if columns is None else f'src."{table}"'
    dest = f'main."{table}"'
    select = f"SELECT {columns} FROM {src}" if columns is not None else f"SELECT * FROM {src}"
    insert = (
        f"INSERT INTO {dest}({columns}) {select}"
        if columns is not None
        else f"INSERT INTO {dest} {select}"
    )
    if predicate is not None:
        insert = f"{insert} WHERE {predicate}"
    conn.execute(insert)


def _ensure_vec_table(conn: sqlite3.Connection, name: str) -> None:
    if _table_exists(conn, "main", f"{name}_vec"):
        return
    row = conn.execute(
        "SELECT sql FROM src.sqlite_master WHERE name = ?", (f"{name}_vec",)
    ).fetchone()
    if row is None or not row[0]:
        return
    conn.execute(row[0])


def _src_table_exists(conn: sqlite3.Connection, name: str) -> bool:
    return _table_exists(conn, "src", name)


def _table_exists(conn: sqlite3.Connection, schema: str, name: str) -> bool:
    row = conn.execute(
        f"SELECT 1 FROM {schema}.sqlite_master WHERE name = ? AND type IN ('table', 'view')",
        (name,),
    ).fetchone()
    return row is not None


def _cache_key(src_path: str, names: tuple[str, ...]) -> tuple[str, tuple[str, ...]]:
    return (src_path, names)


def _export_cache_dir(dest: Path) -> Path:
    """Keep unique exports on the same volume as the case dir so hardlinks work.

    Eval cases write ``<run>/<case>/recording.db``. Putting the cache under
    ``<run>/_recording_exports`` avoids copying 100× onto ``dimos-data`` while
    leaving XDG cache as the fallback for ad-hoc destinations.
    """
    dest = dest.resolve()
    if dest.name == "recording.db":
        return dest.parent.parent / "_recording_exports"
    return _CACHE_DIR


def _cached_export(key: tuple[str, tuple[str, ...]], dest: Path) -> Path:
    digest = hashlib.sha256(repr(key).encode()).hexdigest()[:24]
    return _export_cache_dir(dest) / f"{digest}.db"


def _place(src: Path, dest: Path) -> None:
    """Point *dest* at *src* without a second full copy when the volume allows it."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() or dest.is_symlink():
        dest.unlink()
    try:
        os.link(src, dest)
    except OSError:
        shutil.copy2(src, dest)
