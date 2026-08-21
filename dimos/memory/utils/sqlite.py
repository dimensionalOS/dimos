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

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
import sqlite3

from reactivex.disposable import Disposable


@contextmanager
def sqlite_transaction(connection: sqlite3.Connection) -> Iterator[None]:
    """Commit a SQLite write batch on success and roll it back on failure."""
    connection.execute("BEGIN")
    try:
        yield
    except BaseException:
        connection.rollback()
        raise
    else:
        connection.commit()


def open_sqlite_connection(path: str | Path) -> sqlite3.Connection:
    """Open a WAL-mode SQLite connection with sqlite-vec loaded."""
    import sqlite_vec

    conn = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")
    conn.enable_load_extension(True)
    sqlite_vec.load(conn)
    conn.enable_load_extension(False)
    return conn


def open_disposable_sqlite_connection(
    path: str | Path,
) -> tuple[Disposable, sqlite3.Connection]:
    """Open a WAL-mode SQLite connection and return (disposable, connection).

    The disposable closes the connection when disposed.
    """
    conn = open_sqlite_connection(path)
    return Disposable(lambda: conn.close()), conn
