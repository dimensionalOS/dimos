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

"""Persistent scorer totals. SQLite writes run on the lobby thread, not physics."""

import queue
import sqlite3
from pathlib import Path
from uuid import uuid4


class ScorerLedger:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.pending = queue.SimpleQueue()
        with sqlite3.connect(path) as db:
            db.execute(
                "CREATE TABLE IF NOT EXISTS goals "
                "(id TEXT PRIMARY KEY, user_id TEXT, handle TEXT, scored_at REAL)"
            )
            self.rows = self._read(db)

    @staticmethod
    def _read(db):
        return [
            dict(handle=handle, goals=count)
            for handle, count in db.execute(
                "SELECT (SELECT handle FROM goals newer WHERE newer.user_id=g.user_id "
                "ORDER BY scored_at DESC, rowid DESC LIMIT 1), COUNT(*) "
                "FROM goals g GROUP BY user_id ORDER BY COUNT(*) DESC, g.user_id LIMIT 20"
            )
        ]

    def record(self, identity, scored_at):
        self.pending.put((str(uuid4()), identity["userId"], identity["handle"], scored_at))

    def flush(self):
        batch = []
        while not self.pending.empty():
            batch.append(self.pending.get_nowait())
        if not batch:
            return
        try:
            with sqlite3.connect(self.path) as db:
                db.executemany("INSERT OR IGNORE INTO goals VALUES (?,?,?,?)", batch)
                rows = self._read(db)
            self.rows = rows
        except Exception:
            for item in batch:
                self.pending.put(item)
            raise
