import pytest

from dimos.imitation.learning.store import Store


def test_migrate_reopen_and_rollback(tmp_path):
    store = Store(tmp_path / "db.sqlite3")
    store.migrate()
    store.migrate()
    with store.transaction() as db:
        store.put(db, "record", "id", {"value": 1})
    with pytest.raises(RuntimeError), store.transaction() as db:
        store.put(db, "record", "id", {"value": 2})
        raise RuntimeError("abort")
    reopened = Store(store.path)
    with reopened.read() as db:
        assert reopened.get(db, "record", "id") == {"value": 1}
        assert db.execute("SELECT COUNT(*) FROM migrations").fetchone()[0] == 2
        assert db.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
