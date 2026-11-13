"""Transactional local Learning persistence; no platform or robot dependencies."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import json
import os
from pathlib import Path
import sqlite3
from typing import Any


class Store:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)

    @contextmanager
    def transaction(self) -> Iterator[sqlite3.Connection]:
        db = sqlite3.connect(self.path, timeout=30, isolation_level=None)
        db.execute("PRAGMA busy_timeout=30000")
        db.execute("PRAGMA foreign_keys=ON")
        try:
            db.execute("BEGIN IMMEDIATE")
            yield db
            db.commit()
        except BaseException:
            db.rollback()
            raise
        finally:
            db.close()

    @contextmanager
    def read(self) -> Iterator[sqlite3.Connection]:
        db = sqlite3.connect(self.path, timeout=30)
        try:
            yield db
        finally:
            db.close()

    def migrate(self) -> None:
        with self.transaction() as db:
            db.execute("CREATE TABLE IF NOT EXISTS migrations(version INTEGER PRIMARY KEY)")
            for path in sorted((Path(__file__).parent / "migrations").glob("*.sql")):
                version = int(path.stem.split("_")[0])
                if not db.execute(
                    "SELECT 1 FROM migrations WHERE version=?", (version,)
                ).fetchone():
                    pending = ""
                    for line in path.read_text().splitlines(keepends=True):
                        pending += line
                        if sqlite3.complete_statement(pending):
                            db.execute(pending)
                            pending = ""
                    if pending.strip():
                        raise ValueError(f"Incomplete migration: {path}")
                    db.execute("INSERT INTO migrations VALUES(?)", (version,))
        with sqlite3.connect(self.path) as db:
            db.execute("PRAGMA journal_mode=WAL")

    @staticmethod
    def get(db: sqlite3.Connection, kind: str, key: str) -> dict[str, Any] | None:
        row = db.execute("SELECT body FROM documents WHERE kind=? AND id=?", (kind, key)).fetchone()
        return json.loads(row[0]) if row else None

    @staticmethod
    def put(db: sqlite3.Connection, kind: str, key: str, body: dict[str, Any]) -> None:
        db.execute(
            "INSERT INTO documents(kind,id,body) VALUES(?,?,?) ON CONFLICT(kind,id) DO UPDATE SET body=excluded.body",
            (kind, key, json.dumps(body, allow_nan=False)),
        )

    @staticmethod
    def all(db: sqlite3.Connection, kind: str) -> list[dict[str, Any]]:
        return [
            json.loads(row[0])
            for row in db.execute("SELECT body FROM documents WHERE kind=? ORDER BY rowid", (kind,))
        ]


def default_store() -> Store:
    return Store(
        Path(os.environ.get("DIMOS_LEARNING_STATE_DIR", ".learning-state")) / "learning.sqlite3"
    )


if __name__ == "__main__":
    default_store().migrate()
    print("Learning migrations applied")
