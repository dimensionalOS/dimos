# Copyright 2025-2026 Dimensional Inc.
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
"""PostgreSQL backend for TimeSeriesStore."""

from collections.abc import Iterator
import pickle
import re

import psycopg2  # type: ignore[import-not-found]
import psycopg2.extensions  # type: ignore[import-not-found]

from dimos.core.resource import Resource
from dimos.memory.timeseries.base import T, TimeSeriesStore

# Valid SQL identifier: alphanumeric and underscores, not starting with digit
_VALID_IDENTIFIER = re.compile(r"^[a-zA-Z_][a-zA-Z0-9_]*$")


def _validate_identifier(name: str) -> str:
    """Validate SQL identifier to prevent injection."""
    if not _VALID_IDENTIFIER.match(name):
        raise ValueError(
            f"Invalid identifier '{name}': must be alphanumeric/underscore, not start with digit"
        )
    if len(name) > 128:
        raise ValueError(f"Identifier too long: {len(name)} > 128")
    return name


class PostgresStore(TimeSeriesStore[T], Resource):
    """PostgreSQL backend for sensor data.

    Multiple stores can share the same database with different tables.
    Implements Resource for lifecycle management (start/stop/dispose).

    Usage:
        # Create store
        store = PostgresStore("lidar")
        store.start()  # open connection

        # Use store
        store.save(data)  # saves using data.ts
        data = store.find_closest_seek(10.0)

        # Cleanup
        store.stop()  # close connection

        # Multiple sensors in same db
        lidar = PostgresStore("lidar")
        images = PostgresStore("images")

        # Manual run management via table naming
        run1_lidar = PostgresStore("run1_lidar")
    """

    def __init__(
        self,
        table: str,
        db: str = "dimensional",
        host: str = "localhost",
        port: int = 5432,
        user: str | None = None,
    ) -> None:
        """
        Args:
            table: Table name for this sensor's data (alphanumeric/underscore only).
            db: Database name (alphanumeric/underscore only).
            host: PostgreSQL host.
            port: PostgreSQL port.
            user: PostgreSQL user. Defaults to current system user.
        """
        self._table = _validate_identifier(table)
        self._db = _validate_identifier(db)
        self._host = host
        self._port = port
        self._user = user
        self._conn: psycopg2.extensions.connection | None = None
        self._table_created = False

    def start(self) -> None:
        """Open database connection."""
        if self._conn is not None:
            return
        self._conn = psycopg2.connect(
            dbname=self._db,
            host=self._host,
            port=self._port,
            user=self._user,
        )

    def stop(self) -> None:
        """Close database connection."""
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def _get_conn(self) -> psycopg2.extensions.connection:
        """Get connection, starting if needed."""
        if self._conn is None:
            self.start()
        assert self._conn is not None
        return self._conn

    def _ensure_table(self) -> None:
        """Create table if it doesn't exist."""
        if self._table_created:
            return
        conn = self._get_conn()
        with conn.cursor() as cur:
            cur.execute(f"""
                CREATE TABLE IF NOT EXISTS {self._table} (
                    timestamp DOUBLE PRECISION PRIMARY KEY,
                    data BYTEA NOT NULL
                )
            """)
            cur.execute(f"""
                CREATE INDEX IF NOT EXISTS idx_{self._table}_ts
                ON {self._table}(timestamp)
            """)
        conn.commit()
        self._table_created = True

    def _save(self, timestamp: float, data: T) -> None:
        self._ensure_table()
        conn = self._get_conn()
        blob = pickle.dumps(data)
        with conn.cursor() as cur:
            cur.execute(
                f"""
                INSERT INTO {self._table} (timestamp, data) VALUES (%s, %s)
                ON CONFLICT (timestamp) DO UPDATE SET data = EXCLUDED.data
                """,
                (timestamp, psycopg2.Binary(blob)),
            )
        conn.commit()

    def _load(self, timestamp: float) -> T | None:
        self._ensure_table()
        conn = self._get_conn()
        with conn.cursor() as cur:
            cur.execute(f"SELECT data FROM {self._table} WHERE timestamp = %s", (timestamp,))
            row = cur.fetchone()
        if row is None:
            return None
        data: T = pickle.loads(row[0])
        return data

    def _delete(self, timestamp: float) -> T | None:
        data = self._load(timestamp)
        if data is not None:
            conn = self._get_conn()
            with conn.cursor() as cur:
                cur.execute(f"DELETE FROM {self._table} WHERE timestamp = %s", (timestamp,))
            conn.commit()
        return data

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        self._ensure_table()
        conn = self._get_conn()

        query = f"SELECT timestamp, data FROM {self._table}"
        params: list[float] = []
        conditions = []

        if start is not None:
            conditions.append("timestamp >= %s")
            params.append(start)
        if end is not None:
            conditions.append("timestamp < %s")
            params.append(end)

        if conditions:
            query += " WHERE " + " AND ".join(conditions)
        query += " ORDER BY timestamp"

        with conn.cursor() as cur:
            cur.execute(query, params)
            for row in cur:
                ts: float = row[0]
                data: T = pickle.loads(row[1])
                yield (ts, data)

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        self._ensure_table()
        conn = self._get_conn()

        with conn.cursor() as cur:
            # Get closest timestamp <= target
            cur.execute(
                f"""
                SELECT timestamp FROM {self._table}
                WHERE timestamp <= %s
                ORDER BY timestamp DESC LIMIT 1
                """,
                (timestamp,),
            )
            before = cur.fetchone()

            # Get closest timestamp >= target
            cur.execute(
                f"""
                SELECT timestamp FROM {self._table}
                WHERE timestamp >= %s
                ORDER BY timestamp ASC LIMIT 1
                """,
                (timestamp,),
            )
            after = cur.fetchone()

        candidates: list[float] = []
        if before:
            candidates.append(before[0])
        if after:
            candidates.append(after[0])

        if not candidates:
            return None

        closest = min(candidates, key=lambda ts: abs(ts - timestamp))

        if tolerance is not None and abs(closest - timestamp) > tolerance:
            return None

        return closest

    def _count(self) -> int:
        self._ensure_table()
        conn = self._get_conn()
        with conn.cursor() as cur:
            cur.execute(f"SELECT COUNT(*) FROM {self._table}")
            row = cur.fetchone()
        return row[0] if row else 0  # type: ignore[no-any-return]

    def _last_timestamp(self) -> float | None:
        self._ensure_table()
        conn = self._get_conn()
        with conn.cursor() as cur:
            cur.execute(f"SELECT MAX(timestamp) FROM {self._table}")
            row = cur.fetchone()
        if row is None or row[0] is None:
            return None
        return row[0]  # type: ignore[no-any-return]

    def _find_before(self, timestamp: float) -> tuple[float, T] | None:
        self._ensure_table()
        conn = self._get_conn()
        with conn.cursor() as cur:
            cur.execute(
                f"SELECT timestamp, data FROM {self._table} WHERE timestamp < %s ORDER BY timestamp DESC LIMIT 1",
                (timestamp,),
            )
            row = cur.fetchone()
        if row is None:
            return None
        return (row[0], pickle.loads(row[1]))

    def _find_after(self, timestamp: float) -> tuple[float, T] | None:
        self._ensure_table()
        conn = self._get_conn()
        with conn.cursor() as cur:
            cur.execute(
                f"SELECT timestamp, data FROM {self._table} WHERE timestamp > %s ORDER BY timestamp ASC LIMIT 1",
                (timestamp,),
            )
            row = cur.fetchone()
        if row is None:
            return None
        return (row[0], pickle.loads(row[1]))


def reset_db(db: str = "dimensional", host: str = "localhost", port: int = 5432) -> None:
    """Drop and recreate database. Simple migration strategy.

    WARNING: This deletes all data in the database!

    Args:
        db: Database name to reset (alphanumeric/underscore only).
        host: PostgreSQL host.
        port: PostgreSQL port.
    """
    db = _validate_identifier(db)
    # Connect to 'postgres' database to drop/create
    conn = psycopg2.connect(dbname="postgres", host=host, port=port)
    conn.autocommit = True
    with conn.cursor() as cur:
        # Terminate existing connections
        cur.execute(
            """
            SELECT pg_terminate_backend(pid)
            FROM pg_stat_activity
            WHERE datname = %s AND pid <> pg_backend_pid()
        """,
            (db,),
        )
        cur.execute(f"DROP DATABASE IF EXISTS {db}")
        cur.execute(f"CREATE DATABASE {db}")
    conn.close()
