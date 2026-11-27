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

"""Read-only SQLite store access."""

from pathlib import Path
import sqlite3

import pytest

from dimos.memory.store.sqlite import SqliteStore


def test_read_only_store_reads_data_and_rejects_writes(tmp_path: Path) -> None:
    path = tmp_path / "map ?#.db"
    with SqliteStore(path=path) as store:
        store.stream("values", int).append(42, ts=1.0)
    before = path.read_bytes()

    with SqliteStore(path=path, read_only=True) as store:
        stream = store.stream("values", int)
        assert stream.first().data == 42
        with pytest.raises(sqlite3.OperationalError, match="readonly"):
            stream.append(43, ts=2.0)
        with pytest.raises(sqlite3.OperationalError, match="readonly"):
            store.stream("new_stream", int)
        assert store.list_streams() == ["values"]

    assert path.read_bytes() == before


def test_read_only_store_does_not_create_missing_file(tmp_path: Path) -> None:
    path = tmp_path / "missing" / "map.db"
    with pytest.raises(FileNotFoundError):
        SqliteStore(path=path, read_only=True)
    assert not path.parent.exists()


def test_packaged_recording_has_no_sidecars_and_rejects_writes(tmp_path, monkeypatch) -> None:
    path = tmp_path / "run #1.db"
    with SqliteStore(path=path) as store:
        store.stream("values", int).append(42, ts=1.0)
    original = path.read_bytes()
    monkeypatch.setenv("DIMOS_DATA_DIR", str(tmp_path))

    with SqliteStore(path=path) as store:
        stream = store.stream("values", int)
        assert stream.first().data == 42
        with pytest.raises(sqlite3.OperationalError, match="readonly"):
            stream.append(43, ts=2.0)
        assert list(tmp_path.iterdir()) == [path]

    with pytest.raises(FileNotFoundError):
        SqliteStore(path=tmp_path / "missing" / "output.db")
    assert list(tmp_path.iterdir()) == [path]
    assert path.read_bytes() == original


def test_packaged_recording_with_outstanding_wal_is_rejected(tmp_path, monkeypatch) -> None:
    path = tmp_path / "live.db"
    with SqliteStore(path=path) as writer:
        writer.stream("values", int).append(1, ts=1.0)
        monkeypatch.setenv("DIMOS_DATA_DIR", str(tmp_path))
        with pytest.raises(sqlite3.OperationalError, match="Checkpoint"):
            SqliteStore(path=path)
