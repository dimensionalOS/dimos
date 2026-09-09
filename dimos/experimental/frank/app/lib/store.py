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

"""SQLite storage for FRANK: people, messages, agent events, wake tasks."""

import base64
import os
from pathlib import Path
import re
import sqlite3
import threading
import time
from typing import Any
import uuid

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PEOPLE_DIR = DATA_DIR / "people"
DB_PATH = DATA_DIR / "frank.db"

SCHEMA = """
CREATE TABLE IF NOT EXISTS people (
    person_id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    created_at REAL NOT NULL,
    last_seen_ts REAL,
    last_seen_pose TEXT
);
CREATE TABLE IF NOT EXISTS messages (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    person_id TEXT NOT NULL,
    sender TEXT NOT NULL,
    text TEXT NOT NULL,
    ts REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS messages_by_person ON messages (person_id, id);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    type TEXT NOT NULL,
    payload TEXT NOT NULL,
    ts REAL NOT NULL,
    delivered INTEGER NOT NULL DEFAULT 0
);
CREATE TABLE IF NOT EXISTS sightings_last (
    person_id TEXT PRIMARY KEY,
    x REAL,
    y REAL,
    bearing_deg REAL,
    range_m REAL,
    face_px INTEGER,
    last_seen_ts REAL
);
CREATE TABLE IF NOT EXISTS tasks (
    task_id TEXT PRIMARY KEY,
    task TEXT NOT NULL,
    person_id TEXT,
    created_at REAL NOT NULL,
    closed_at REAL,
    outcome TEXT,
    note TEXT
);
"""


class Store:
    def __init__(self, db_path: Path = DB_PATH) -> None:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        PEOPLE_DIR.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self.db = sqlite3.connect(db_path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA)
        self.db.commit()

    def query(self, sql: str, args: tuple = ()) -> list[sqlite3.Row]:
        with self._lock:
            return list(self.db.execute(sql, args))

    def one(self, sql: str, args: tuple = ()) -> sqlite3.Row | None:
        rows = self.query(sql, args)
        return rows[0] if rows else None

    def write(self, sql: str, args: tuple = ()) -> int:
        with self._lock:
            cur = self.db.execute(sql, args)
            self.db.commit()
            return cur.lastrowid or 0

    # --- people ---------------------------------------------------------

    def add_person(self, name: str, selfie_b64: str | None = None) -> dict[str, Any]:
        person_id = "p_" + uuid.uuid4().hex[:4]
        now = time.time()
        self.write(
            "INSERT INTO people (person_id, name, created_at) VALUES (?, ?, ?)",
            (person_id, name, now),
        )
        if selfie_b64:
            save_selfie(person_id, selfie_b64)
        return {"person_id": person_id, "name": name, "created_at": now}

    def get_person(self, person_id: str) -> sqlite3.Row | None:
        return self.one("SELECT * FROM people WHERE person_id = ?", (person_id,))

    def forget(self, person_id: str) -> None:
        self.write("DELETE FROM messages WHERE person_id = ?", (person_id,))
        self.write("DELETE FROM people WHERE person_id = ?", (person_id,))
        self.write("DELETE FROM tasks WHERE person_id = ?", (person_id,))
        self.write("DELETE FROM sightings_last WHERE person_id = ?", (person_id,))
        # selfie plus whatever the face matcher parked next to it (embeddings)
        for path in PEOPLE_DIR.glob(f"{person_id}.*"):
            path.unlink(missing_ok=True)

    def people_overview(self, follow_up_window_s: float = 86400) -> list[dict[str, Any]]:
        cutoff = time.time() - follow_up_window_s
        out = []
        for p in self.query("SELECT * FROM people ORDER BY created_at"):
            last_chat = self.one(
                "SELECT MAX(ts) AS ts FROM messages WHERE person_id = ?", (p["person_id"],)
            )
            done = self.one(
                "SELECT COUNT(*) AS n FROM tasks WHERE person_id = ? AND created_at > ?",
                (p["person_id"], cutoff),
            )
            out.append(
                {
                    "person_id": p["person_id"],
                    "name": p["name"],
                    "created_at": p["created_at"],
                    "last_chat_ts": last_chat["ts"] if last_chat else None,
                    "last_seen_ts": p["last_seen_ts"],
                    "last_seen_pose": parse_pose(p["last_seen_pose"]),
                    "follow_ups_today": done["n"] if done else 0,
                }
            )
        return out

    def save_last_sighting(self, person_id: str, row: dict[str, Any]) -> None:
        """Write-through of the world's latest sighting, so it survives a restart."""
        self.write(
            "INSERT INTO sightings_last (person_id, x, y, bearing_deg, range_m, face_px,"
            " last_seen_ts) VALUES (?, ?, ?, ?, ?, ?, ?)"
            " ON CONFLICT(person_id) DO UPDATE SET x = excluded.x, y = excluded.y,"
            " bearing_deg = excluded.bearing_deg, range_m = excluded.range_m,"
            " face_px = excluded.face_px, last_seen_ts = excluded.last_seen_ts",
            (
                person_id,
                row.get("x"),
                row.get("y"),
                row.get("bearing_deg"),
                row.get("range_m"),
                row.get("face_px"),
                row.get("last_seen_ts"),
            ),
        )

    def last_sightings(self) -> dict[str, dict[str, Any]]:
        """Everything `save_last_sighting` kept, for reloading the world on startup."""
        out: dict[str, dict[str, Any]] = {}
        for r in self.query("SELECT * FROM sightings_last"):
            row = {k: r[k] for k in r.keys() if k != "person_id" and r[k] is not None}
            row["in_view"] = False  # nobody is in view until the watcher says so again
            out[r["person_id"]] = row
        return out

    def record_sighting(self, person_id: str, pose: dict[str, float] | None) -> None:
        self.write(
            "UPDATE people SET last_seen_ts = ?, last_seen_pose = ? WHERE person_id = ?",
            (time.time(), pose_to_text(pose), person_id),
        )

    # --- messages -------------------------------------------------------

    def add_message(self, person_id: str, sender: str, text: str) -> dict[str, Any]:
        ts = time.time()
        msg_id = self.write(
            "INSERT INTO messages (person_id, sender, text, ts) VALUES (?, ?, ?, ?)",
            (person_id, sender, text, ts),
        )
        return {"id": msg_id, "person_id": person_id, "from": sender, "text": text, "ts": ts}

    def messages_after(self, person_id: str, after: int) -> list[dict[str, Any]]:
        rows = self.query(
            "SELECT * FROM messages WHERE person_id = ? AND id > ? ORDER BY id",
            (person_id, after),
        )
        return [as_message(r) for r in rows]

    def history_since(self, person_id: str, since_minutes: float) -> list[dict[str, Any]]:
        rows = self.query(
            "SELECT * FROM messages WHERE person_id = ? AND ts > ? ORDER BY id",
            (person_id, time.time() - since_minutes * 60),
        )
        return [as_message(r) for r in rows]

    def last_messages(self, person_id: str, limit: int) -> list[dict[str, Any]]:
        rows = self.query(
            "SELECT * FROM messages WHERE person_id = ? ORDER BY id DESC LIMIT ?",
            (person_id, limit),
        )
        return [as_message(r) for r in reversed(rows)]


def as_message(row: sqlite3.Row) -> dict[str, Any]:
    return {
        "id": row["id"],
        "person_id": row["person_id"],
        "from": row["sender"],
        "text": row["text"],
        "ts": row["ts"],
    }


def pose_to_text(pose: dict[str, float] | None) -> str | None:
    if not pose:
        return None
    return f"{pose.get('x', 0.0)},{pose.get('y', 0.0)},{pose.get('yaw', 0.0)}"


def parse_pose(text: str | None) -> dict[str, float] | None:
    if not text:
        return None
    x, y, yaw = (float(v) for v in text.split(","))
    return {"x": x, "y": y, "yaw": yaw}


def save_selfie(person_id: str, selfie_b64: str) -> Path:
    """Accepts a bare base64 JPEG or a `data:image/jpeg;base64,...` URL."""
    raw = re.sub(r"^data:[^;]*;base64,", "", selfie_b64.strip())
    data = base64.b64decode(raw, validate=False)
    path = PEOPLE_DIR / f"{person_id}.jpg"
    path.write_bytes(data)
    return path


def selfie_path(person_id: str) -> Path:
    return PEOPLE_DIR / f"{person_id}.jpg"


def db_path_from_env() -> Path:
    return Path(os.environ.get("FRANK_DB", str(DB_PATH)))
