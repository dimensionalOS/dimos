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
"""SQLite backend for SensorStore."""

from collections.abc import Iterator
from pathlib import Path
import pickle
import sqlite3

from dimos.memory.sensor.base import SensorStore, T


class SqliteStore(SensorStore[T]):
    """SQLite backend for sensor data. Good for indexed queries and single-file storage.

    Data is stored as pickled BLOBs with timestamp as indexed column.

    Usage:
        # File-based (persistent)
        store = SqliteStore("sensors.db", table="lidar")
        store.save(data, timestamp)

        # In-memory (for testing)
        store = SqliteStore(":memory:")

        # Query by timestamp
        data = store.find_closest(seek=10.0)
    """

    def __init__(self, db_path: str | Path, table: str = "sensor_data") -> None:
        """
        Args:
            db_path: Path to SQLite database file, or ":memory:" for in-memory.
            table: Table name for this sensor's data.
        """
        self._db_path = str(db_path)
        self._table = table
        self._conn: sqlite3.Connection | None = None

    def _get_conn(self) -> sqlite3.Connection:
        """Get or create database connection."""
        if self._conn is None:
            # Create parent directory if needed (for file-based DBs)
            if self._db_path != ":memory:":
                Path(self._db_path).parent.mkdir(parents=True, exist_ok=True)

            self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
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

    def close(self) -> None:
        """Close the database connection."""
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def __del__(self) -> None:
        self.close()
