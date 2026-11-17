"""Real disposable PostgreSQL checks, enabled only with explicit test DSN."""

from concurrent.futures import ThreadPoolExecutor
import os
from uuid import uuid4

import pytest

from dimos.imitation.learning.postgres import PostgresStore


@pytest.mark.parametrize(
    "schema",
    ["public", "orgs", "dimos_learning;DROP TABLE documents", "dimos_learning.", "DIMOS_LEARNING"],
)
def test_schema_requires_dedicated_namespace(tmp_path, schema):
    with pytest.raises(ValueError, match="dedicated"):
        PostgresStore("unused", schema, tmp_path)


def test_postgres_migrations_rollback_order_immutability(tmp_path):
    dsn = os.environ.get("DIMOS_LEARNING_TEST_POSTGRES_DSN")
    if not dsn:
        pytest.skip("Explicit disposable DIMOS_LEARNING_TEST_POSTGRES_DSN required")
    import psycopg
    from psycopg import sql

    schema = "dimos_learning_test_" + uuid4().hex
    store = PostgresStore(dsn, schema, tmp_path)
    try:
        # Constructors/read contexts do not migrate, even when schema is absent.
        with pytest.raises(psycopg.errors.UndefinedTable), store.read() as db:
            store.all(db, "record")
        with ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(lambda _: store.migrate(), range(2)))
        with store.transaction() as db:
            store.put(db, "record", "z", {"v": 1})
            store.put(db, "record", "a", {"v": 2})
            store.put(db, "revision", "immutable", {"v": 1})
        with pytest.raises(RuntimeError), store.transaction() as db:
            store.put(db, "record", "z", {"v": 999})
            raise RuntimeError("rollback")
        for query in (
            "UPDATE documents SET body='{}' WHERE kind='revision'",
            "DELETE FROM documents WHERE kind='revision'",
        ):
            with (
                pytest.raises(psycopg.errors.RaiseException, match="immutable"),
                store.transaction() as db,
            ):
                db.execute(query)
        reopened = PostgresStore(dsn, schema, tmp_path)
        with reopened.read() as db:
            assert reopened.all(db, "record") == [{"v": 1}, {"v": 2}]
            assert db.execute("SELECT COUNT(*) FROM migrations").fetchone()[0] == 2
        with pytest.raises(psycopg.errors.ReadOnlySqlTransaction), store.read() as db:
            store.put(db, "record", "write-denied", {})
    finally:
        with psycopg.connect(dsn) as db:
            db.execute(sql.SQL("DROP SCHEMA IF EXISTS {} CASCADE").format(sql.Identifier(schema)))
