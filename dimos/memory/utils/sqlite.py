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

from pathlib import Path
import sqlite3
import threading

from reactivex.disposable import Disposable
import sqlite_vec

from dimos.utils.data import is_packaged_data

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


def read_only_sqlite_uri(path: str | Path) -> str:
    uri = Path(path).resolve().as_uri() + "?mode=ro"
    if is_packaged_data(path):
        # Even mode=ro can create -shm and -wal files. An immutable snapshot
        # needs no sidecars, but SQLite ignores outstanding WAL transactions.
        wal = Path(f"{Path(path).resolve()}-wal")
        if wal.exists() and wal.stat().st_size:
            raise sqlite3.OperationalError(
                f"Checkpoint {path} before packaging it in DIMOS_DATA_DIR"
            )
        uri += "&immutable=1"
    return uri


def open_sqlite_connection(path: str | Path, *, read_only: bool = False) -> sqlite3.Connection:
    """Open SQLite with sqlite-vec, using WAL only for writable connections."""
    if read_only or (str(path) != ":memory:" and is_packaged_data(path)):
        conn = sqlite3.connect(read_only_sqlite_uri(path), uri=True, check_same_thread=False)
    else:
        conn = sqlite3.connect(path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
    # WAL has one writer; concurrent writer threads outlast the 5s default
    conn.execute("PRAGMA busy_timeout=30000")
    conn.enable_load_extension(True)
    sqlite_vec.load(conn)
    conn.enable_load_extension(False)
    return conn


def open_disposable_sqlite_connection(
    path: str | Path, *, read_only: bool = False
) -> tuple[Disposable, sqlite3.Connection]:
    """Open SQLite and return (disposable, connection).

    The disposable closes the connection and releases its lock when disposed.
    """
    conn = open_sqlite_connection(path, read_only=read_only)
    return Disposable(lambda: close_sqlite_connection(conn)), conn
