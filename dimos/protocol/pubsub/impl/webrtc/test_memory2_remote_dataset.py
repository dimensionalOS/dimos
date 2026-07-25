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

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import sys
from types import ModuleType

import pytest
from typer.testing import CliRunner

from dimos.memory2 import remote_dataset
from dimos.memory2.cli import app as memory2_cli
from dimos.memory2.remote_dataset import (
    index_memory2_snapshot,
    snapshot_sqlite_database,
    upload_memory2_dataset,
)
from dimos.protocol.pubsub.impl.webrtc.replay_repository import ReplayObject, sha256_file


def _make_memory2_database(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(path)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
    connection.execute("INSERT INTO _streams VALUES ('camera', '{}')")
    connection.execute(
        "CREATE TABLE camera (id INTEGER PRIMARY KEY, ts REAL NOT NULL, value NUMERIC)"
    )
    connection.execute("INSERT INTO camera VALUES (1, 10.5, 1)")
    connection.commit()
    return connection


def _replay_object(path: Path, *, owner: str = "alice", repository: str = "go2") -> ReplayObject:
    digest = sha256_file(path)
    return ReplayObject(
        owner=owner,
        repository=repository,
        object_id=digest,
        filename=path.name,
        size_bytes=path.stat().st_size,
        sha256=digest,
        content_type="application/octet-stream",
        created_at=datetime.now(timezone.utc).isoformat(),
    )


def test_snapshot_includes_committed_wal_data(tmp_path: Path) -> None:
    source = tmp_path / "live.db"
    writer = _make_memory2_database(source)
    writer.execute("INSERT INTO camera VALUES (2, 11.5, 2)")
    writer.commit()

    snapshot = snapshot_sqlite_database(source, tmp_path / "snapshot.db")
    writer.close()

    with sqlite3.connect(snapshot) as connection:
        assert connection.execute("PRAGMA integrity_check").fetchone() == ("ok",)
        assert connection.execute("SELECT COUNT(*) FROM camera").fetchone() == (2,)
    assert not snapshot.with_name(f"{snapshot.name}-wal").exists()


def test_index_contains_stream_ranges_and_checksum(tmp_path: Path) -> None:
    source = tmp_path / "go2.db"
    writer = _make_memory2_database(source)
    writer.execute("INSERT INTO camera VALUES (2, 12.0, 2)")
    writer.commit()
    writer.close()

    index = index_memory2_snapshot(source)

    assert index.dataset == "go2"
    assert index.sha256 == sha256_file(source)
    assert index.size_bytes == source.stat().st_size
    assert len(index.streams) == 1
    assert index.streams[0].to_dict() == {
        "name": "camera",
        "items": 2,
        "start_time": 10.5,
        "end_time": 12.0,
    }


def test_upload_publishes_snapshot_then_completed_index(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "go2.db"
    writer = _make_memory2_database(source)
    writer.close()
    uploads: list[tuple[str, bytes]] = []

    def fake_upload_file(**kwargs: object) -> ReplayObject:
        path = Path(str(kwargs["path"]))
        uploads.append((path.name, path.read_bytes()))
        return _replay_object(path)

    monkeypatch.setattr(remote_dataset, "upload_file", fake_upload_file)

    result = upload_memory2_dataset(
        path=source,
        server_url="https://replays.example",
        owner="alice",
        repository="go2",
        dataset="field-test",
    )

    assert [name for name, _ in uploads] == ["go2.db", "field-test.memory2.json"]
    uploaded_index = json.loads(uploads[1][1])
    assert uploaded_index["kind"] == "dimos-memory2-sqlite"
    assert uploaded_index["dataset"] == "field-test"
    assert uploaded_index["object"]["object_id"] == result.dataset_object.object_id
    assert result.index_object.filename == "field-test.memory2.json"


def test_rejects_non_memory2_database(tmp_path: Path) -> None:
    source = tmp_path / "plain.db"
    with sqlite3.connect(source) as connection:
        connection.execute("CREATE TABLE ordinary (value TEXT)")

    try:
        index_memory2_snapshot(source)
    except ValueError as exc:
        assert "not a memory2" in str(exc)
    else:
        raise AssertionError("expected a non-memory2 database to be rejected")


def test_rejects_dataset_name_with_directories(tmp_path: Path) -> None:
    source = tmp_path / "go2.db"
    writer = _make_memory2_database(source)
    writer.close()

    with pytest.raises(ValueError, match="plain name"):
        index_memory2_snapshot(source, dataset="../escape")


def test_memory2_upload_cli(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "go2.db"
    writer = _make_memory2_database(source)
    writer.close()

    dataset_module = ModuleType("dimos.memory2.cli.dataset")
    dataset_module.resolve_dataset = lambda _: source  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.memory2.cli.dataset", dataset_module)
    dataset_object = _replay_object(source)
    index_path = tmp_path / "go2.memory2.json"
    index_path.write_text("{}")
    index_object = _replay_object(index_path)
    index = index_memory2_snapshot(source, remote_object=dataset_object)
    monkeypatch.setattr(
        remote_dataset,
        "upload_memory2_dataset",
        lambda **_: remote_dataset.Memory2Upload(
            index=index,
            dataset_object=dataset_object,
            index_object=index_object,
        ),
    )

    result = CliRunner().invoke(
        memory2_cli.mem_app,
        [
            "upload",
            "go2",
            "--owner",
            "alice",
            "--repo",
            "robot-bugs",
            "--server-url",
            "https://replays.example",
        ],
    )

    assert result.exit_code == 0
    payload = json.loads(result.stdout)
    assert payload["dataset"]["dataset"] == "go2"
    assert payload["dataset_object"]["object_id"] == dataset_object.object_id
