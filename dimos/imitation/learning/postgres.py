"""Dedicated-schema PostgreSQL Store, with explicit opt-in migrations only.

A schema-scoped transaction advisory lock preserves SQLite BEGIN IMMEDIATE's
single-writer semantics for CAS, lease claims and immutable revision insertion.
This is intentionally conservative, not a high-throughput queue implementation.
"""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import re
from typing import Any

from dimos.imitation.learning.store import Store


class _Connection:
    def __init__(self, connection: Any):
        self.connection = connection

    def execute(self, statement: str, parameters: tuple = ()) -> Any:
        # Store's fixed SQL uses qmark parameters; never rewrite user SQL here.
        return self.connection.execute(statement.replace("?", "%s"), parameters)


class PostgresStore(Store):
    def __init__(self, dsn: str, schema: str, state_dir: Path):
        if not re.fullmatch(r"dimos_learning(?:_[a-z0-9_]+)?", schema) or len(schema) > 63:
            raise ValueError("Explicit dedicated dimos_learning[_suffix] schema required")
        self.dsn = dsn
        self.schema = schema
        # Local media/artifact workspace is separate from DB selection.
        self.path = state_dir / "learning.postgres"
        state_dir.mkdir(parents=True, exist_ok=True)

    def _connect(self):
        import psycopg

        return psycopg.connect(self.dsn)

    def _scope(self, db):
        from psycopg import sql

        db.execute(
            sql.SQL("SET LOCAL search_path TO {}, pg_catalog").format(sql.Identifier(self.schema))
        )

    @contextmanager
    def transaction(self):
        with self._connect() as db:
            self._scope(db)
            db.execute("SELECT pg_advisory_xact_lock(hashtextextended(%s, 0))", (self.schema,))
            yield _Connection(db)

    @contextmanager
    def read(self):
        with self._connect() as db:
            db.execute("SET TRANSACTION READ ONLY")
            self._scope(db)
            yield _Connection(db)

    def migrate(self):
        from psycopg import sql

        with self._connect() as db:
            db.execute("SELECT pg_advisory_xact_lock(hashtextextended(%s, 0))", (self.schema,))
            db.execute(
                sql.SQL("CREATE SCHEMA IF NOT EXISTS {}").format(sql.Identifier(self.schema))
            )
            self._scope(db)
            db.execute("CREATE TABLE IF NOT EXISTS migrations(version INTEGER PRIMARY KEY)")
            for path in sorted((Path(__file__).parent / "migrations/postgres").glob("*.sql")):
                version = int(path.stem.split("_")[0])
                if not db.execute(
                    "SELECT 1 FROM migrations WHERE version=%s", (version,)
                ).fetchone():
                    db.execute(path.read_text())
                    db.execute("INSERT INTO migrations VALUES (%s)", (version,))
