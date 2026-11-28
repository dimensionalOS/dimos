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
from pathlib import Path
import sqlite3
import threading

from reactivex.disposable import Disposable

# ``sqlite3.Connection`` does not allow arbitrary attribute assignment nor
# weak references, so we keep the per-connection locks in a side-table keyed
# by the connection object's identity. Entries are removed on close via
# :func:`close_sqlite_connection` / the disposable from
# :func:`open_disposable_sqlite_connection`.
_locks: dict[int, threading.RLock] = {}
_locks_guard = threading.Lock()


def conn_lock(conn: sqlite3.Connection) -> threading.RLock:
    """Return the per-connection RLock for ``conn``, creating one on first call.

    Every sqlite-backed store that borrows a connection must serialize its
    conn/cursor accesses through this lock — sqlite3 connections opened with
    ``check_same_thread=False`` (as memory2 does, since backend components are
    constructed on one thread but read from many) are not safe for concurrent
    use otherwise. The same RLock is shared across all components that borrow
    the same connection, so cross-store conn access is serialized too.
    """
    key = id(conn)
    lock = _locks.get(key)
    if lock is not None:
        return lock
    with _locks_guard:
        lock = _locks.get(key)
        if lock is None:
            lock = threading.RLock()
            _locks[key] = lock
    return lock


def close_sqlite_connection(conn: sqlite3.Connection) -> None:
    """Close ``conn`` and drop its lock entry from the registry."""
    conn.close()
    with _locks_guard:
        _locks.pop(id(conn), None)


class RecordingRegistryError(ValueError):
    """The input database does not provide a readable dimOS stream registry."""


def connect_existing_sqlite(
    path: str | Path, *, read_only: bool, timeout: float = 5.0
) -> sqlite3.Connection:
    """Open an existing SQLite file without creating it or changing its journal mode."""
    path = Path(path).resolve()
    try:
        file_mode = path.stat().st_mode
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"SQLite database not found: {path}") from exc
    options = "mode=ro" if read_only else "mode=rw"
    if read_only:
        # A directory writable by another user can still acquire a live WAL.
        # Only bypass SQLite's locking on read-only media or explicitly sealed archives.
        sealed = (file_mode | path.parent.stat().st_mode) & 0o222 == 0
        if not sealed and hasattr(os, "statvfs") and not os.access(path.parent, os.W_OK):
            sealed = bool(os.statvfs(path.parent).f_flag & os.ST_RDONLY)
        if sealed and not any(Path(f"{path}{suffix}").exists() for suffix in ("-wal", "-journal")):
            options += "&immutable=1"
    return sqlite3.connect(
        f"{path.as_uri()}?{options}", uri=True, timeout=timeout, check_same_thread=False
    )


def connect_to_recording(path: str | Path, *, read_only: bool) -> sqlite3.Connection:
    """Open an existing recording without creating a file or initializing an unrelated DB."""
    try:
        conn = connect_existing_sqlite(path, read_only=read_only)
        try:
            # Validate on the same handle before WAL mode or registry setup can change the file.
            conn.execute("SELECT name, config FROM _streams LIMIT 0")
        except sqlite3.Error:
            conn.close()
            raise
    except sqlite3.Error as exc:
        raise RecordingRegistryError(
            f"Cannot read SQLite recording registry in {path}: {exc}"
        ) from exc
    return conn


def open_sqlite_connection(
    path: str | Path, *, existing_recording: bool = False, read_only: bool = False
) -> sqlite3.Connection:
    """Open a SQLite connection with sqlite-vec loaded, enabling WAL for writers."""
    import sqlite_vec

    if existing_recording:
        conn = connect_to_recording(path, read_only=read_only)
    elif read_only:
        conn = connect_existing_sqlite(path, read_only=True)
    else:
        conn = sqlite3.connect(path, check_same_thread=False)
    try:
        if not read_only:
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("PRAGMA synchronous=NORMAL")
        # WAL has one writer; concurrent writer threads outlast the 5s default
        conn.execute("PRAGMA busy_timeout=30000")
        conn.enable_load_extension(True)
        sqlite_vec.load(conn)
        conn.enable_load_extension(False)
    except BaseException:
        conn.close()
        raise
    return conn


def open_disposable_sqlite_connection(
    path: str | Path, *, existing_recording: bool = False, read_only: bool = False
) -> tuple[Disposable, sqlite3.Connection]:
    """Open a SQLite connection and return (disposable, connection).

    The disposable closes the connection and releases its lock when disposed.
    """
    conn = open_sqlite_connection(path, existing_recording=existing_recording, read_only=read_only)
    return Disposable(lambda: close_sqlite_connection(conn)), conn
