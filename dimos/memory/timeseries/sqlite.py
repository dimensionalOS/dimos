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
"""SQLite backend for TimeSeriesStore."""

from collections.abc import Iterator
from pathlib import Path
import pickle
import re
import sqlite3

from dimos.memory.timeseries.base import T, TimeSeriesStore
from dimos.utils.data import get_data, get_data_dir

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


class SqliteStore(TimeSeriesStore[T]):
    """SQLite backend for sensor data. Good for indexed queries and single-file storage.

    Data is stored as pickled BLOBs with timestamp as indexed column.

    Usage:
        # Named store (uses data/ directory, auto-downloads from LFS if needed)
        store = SqliteStore("recordings/lidar")  # -> data/recordings/lidar.db
        store.save(data)  # saves using data.ts

        # Absolute path
        store = SqliteStore("/path/to/sensors.db")

        # In-memory (for testing)
        store = SqliteStore(":memory:")

        # Multiple tables in one DB
        store = SqliteStore("recordings/sensors", table="lidar")
    """

    def __init__(self, name: str | Path, table: str = "sensor_data") -> None:
        """
        Args:
            name: Data name (e.g. "recordings/lidar") resolved via get_data,
                  absolute path, or ":memory:" for in-memory.
            table: Table name for this sensor's data (alphanumeric/underscore only).
        """
        self._name = str(name)
        self._table = _validate_identifier(table)
        self._db_path: str | None = None
        self._conn: sqlite3.Connection | None = None

    def _get_db_path(self, for_write: bool = False) -> str:
        """Get database path, resolving via get_data if needed."""
        if self._db_path is not None:
            return self._db_path

        # Special case for in-memory
        if self._name == ":memory:":
            self._db_path = ":memory:"
            return self._db_path

        # If absolute path, use directly
        if Path(self._name).is_absolute():
            self._db_path = self._name
        elif for_write:
            # For writing: use get_data_dir
            db_file = get_data_dir(self._name + ".db")
            db_file.parent.mkdir(parents=True, exist_ok=True)
            self._db_path = str(db_file)
        else:
            # For reading: use get_data (handles LFS download)
            # Try with .db extension first
            try:
                db_file = get_data(self._name + ".db")
                self._db_path = str(db_file)
            except FileNotFoundError:
                # Fall back to get_data_dir for new databases
                db_file = get_data_dir(self._name + ".db")
                db_file.parent.mkdir(parents=True, exist_ok=True)
                self._db_path = str(db_file)

        return self._db_path

    def _get_conn(self) -> sqlite3.Connection:
        """Get or create database connection."""
        if self._conn is None:
            db_path = self._get_db_path(for_write=True)
            self._conn = sqlite3.connect(db_path, check_same_thread=False)
            self._create_table()
        return self._conn

    def _create_table(self) -> None:
        """Create table if it doesn't exist."""
        conn = self._conn
        assert conn is not None
        conn.execute(f"""
            CREATE TABLE IF NOT EXISTS {self._table} (
                timestamp REAL PRIMARY KEY,
                data BLOB NOT NULL
            )
        """)
        conn.execute(f"""
            CREATE INDEX IF NOT EXISTS idx_{self._table}_timestamp
            ON {self._table}(timestamp)
        """)
        conn.commit()

    def _save(self, timestamp: float, data: T) -> None:
        conn = self._get_conn()
        blob = pickle.dumps(data)
        conn.execute(
            f"INSERT OR REPLACE INTO {self._table} (timestamp, data) VALUES (?, ?)",
            (timestamp, blob),
        )
        conn.commit()

    def _load(self, timestamp: float) -> T | None:
        conn = self._get_conn()
        cursor = conn.execute(f"SELECT data FROM {self._table} WHERE timestamp = ?", (timestamp,))
        row = cursor.fetchone()
        if row is None:
            return None
        data: T = pickle.loads(row[0])
        return data

    def _delete(self, timestamp: float) -> T | None:
        data = self._load(timestamp)
        if data is not None:
            conn = self._get_conn()
            conn.execute(f"DELETE FROM {self._table} WHERE timestamp = ?", (timestamp,))
            conn.commit()
        return data

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        conn = self._get_conn()

        # Build query with optional range filters
        query = f"SELECT timestamp, data FROM {self._table}"
        params: list[float] = []
        conditions = []

        if start is not None:
            conditions.append("timestamp >= ?")
            params.append(start)
        if end is not None:
            conditions.append("timestamp < ?")
            params.append(end)

        if conditions:
            query += " WHERE " + " AND ".join(conditions)
        query += " ORDER BY timestamp"

        cursor = conn.execute(query, params)
        for row in cursor:
            ts: float = row[0]
            data: T = pickle.loads(row[1])
            yield (ts, data)

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        conn = self._get_conn()

        # Find closest timestamp using SQL
        # Get the closest timestamp <= target
        cursor = conn.execute(
            f"""
            SELECT timestamp FROM {self._table}
            WHERE timestamp <= ?
            ORDER BY timestamp DESC LIMIT 1
            """,
            (timestamp,),
        )
        before = cursor.fetchone()

        # Get the closest timestamp >= target
        cursor = conn.execute(
            f"""
            SELECT timestamp FROM {self._table}
            WHERE timestamp >= ?
            ORDER BY timestamp ASC LIMIT 1
            """,
            (timestamp,),
        )
        after = cursor.fetchone()

        # Find the closest of the two
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
        conn = self._get_conn()
        cursor = conn.execute(f"SELECT COUNT(*) FROM {self._table}")
        return cursor.fetchone()[0]  # type: ignore[no-any-return]

    def _last_timestamp(self) -> float | None:
        conn = self._get_conn()
        cursor = conn.execute(f"SELECT MAX(timestamp) FROM {self._table}")
        row = cursor.fetchone()
        if row is None or row[0] is None:
            return None
        return row[0]  # type: ignore[no-any-return]

    def _find_before(self, timestamp: float) -> tuple[float, T] | None:
        conn = self._get_conn()
        cursor = conn.execute(
            f"SELECT timestamp, data FROM {self._table} WHERE timestamp < ? ORDER BY timestamp DESC LIMIT 1",
            (timestamp,),
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return (row[0], pickle.loads(row[1]))

    def _find_after(self, timestamp: float) -> tuple[float, T] | None:
        conn = self._get_conn()
        cursor = conn.execute(
            f"SELECT timestamp, data FROM {self._table} WHERE timestamp > ? ORDER BY timestamp ASC LIMIT 1",
            (timestamp,),
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return (row[0], pickle.loads(row[1]))

    def close(self) -> None:
        """Close the database connection."""
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def __del__(self) -> None:
        self.close()
