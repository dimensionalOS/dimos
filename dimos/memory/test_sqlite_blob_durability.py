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

from contextlib import closing
from pathlib import Path
import sqlite3
from typing import Any

import pytest

from dimos.memory.blobstore.sqlite import SqliteBlobStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def _rows(path: Path, table: str) -> list[tuple[Any, ...]]:
    with closing(sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)) as reader:
        return reader.execute(f'SELECT * FROM "{table}" ORDER BY id').fetchall()


@pytest.mark.parametrize("operation", ["put", "replace", "delete"])
@pytest.mark.parametrize("after_stop", [False, True], ids=["live-reader", "reopen"])
def test_owned_connection_persists_changes(
    tmp_path: Path, operation: str, after_stop: bool
) -> None:
    path = tmp_path / "payloads.db"
    with closing(sqlite3.connect(path)) as seed:
        seed.execute("CREATE TABLE pose_blob (id INTEGER PRIMARY KEY, data BLOB NOT NULL)")
        seed.execute("INSERT INTO pose_blob VALUES (1, ?)", (b"old",))
        seed.commit()
    with SqliteBlobStore(path=str(path)) as blob:
        if operation == "put":
            blob.put("pose", 2, b"new")
            expected = [(1, b"old"), (2, b"new")]
        elif operation == "replace":
            blob.put("pose", 1, b"new")
            expected = [(1, b"new")]
        else:
            blob.delete("pose", 1)
            expected = []
        if after_stop:
            blob.stop()
        assert _rows(path, "pose_blob") == expected


@pytest.mark.parametrize("isolation_level", ["", None], ids=["implicit", "autocommit"])
@pytest.mark.parametrize("finish", ["commit", "rollback", "error"])
def test_borrowed_connection_preserves_explicit_transaction(
    tmp_path: Path, isolation_level: str | None, finish: str
) -> None:
    path = tmp_path / "shared.db"
    with closing(sqlite3.connect(path, isolation_level=isolation_level)) as caller:
        caller.execute("CREATE TABLE metadata (id INTEGER PRIMARY KEY)")
        caller.execute("CREATE TABLE pose_blob (id INTEGER PRIMARY KEY, data BLOB NOT NULL)")
        caller.commit()
        caller.execute("BEGIN")
        caller.execute("INSERT INTO metadata VALUES (1)")
        # Starting and stopping a borrowed blob resource must leave this pending.
        with SqliteBlobStore(conn=caller) as blob:
            blob.put("pose", 1, b"pending")
        assert caller.isolation_level == isolation_level
        assert caller.in_transaction
        assert _rows(path, "metadata") == []
        assert _rows(path, "pose_blob") == []
        if finish == "error":
            # A real SQLite statement failure exits the resource context. The
            # caller still owns rollback and must retain a usable connection.
            with pytest.raises(sqlite3.IntegrityError, match="NOT NULL"):
                with SqliteBlobStore(conn=caller) as blob:
                    blob.put("pose", 2, b"also pending")
                    caller.execute("INSERT INTO pose_blob VALUES (3, NULL)")
            assert caller.in_transaction
            assert _rows(path, "metadata") == []
            assert _rows(path, "pose_blob") == []
            caller.rollback()
        elif finish == "commit":
            caller.commit()
        else:
            caller.rollback()
        expected_blob = [(1, b"pending")] if finish == "commit" else []
        expected_metadata = [(1,)] if finish == "commit" else []
        assert _rows(path, "pose_blob") == expected_blob
        assert _rows(path, "metadata") == expected_metadata
        assert caller.execute("SELECT * FROM pose_blob").fetchall() == expected_blob
        # A second explicit transaction succeeds after either rollback path.
        caller.execute("BEGIN")
        with SqliteBlobStore(conn=caller) as recovered:
            recovered.put("pose", 4, b"recovered")
        caller.commit()
        assert _rows(path, "pose_blob") == [*expected_blob, (4, b"recovered")]


@pytest.mark.parametrize("error", [False, True], ids=["normal", "exception"])
def test_owned_connection_closes_on_resource_exit(tmp_path: Path, error: bool) -> None:
    blob = SqliteBlobStore(path=str(tmp_path / "owned.db"))
    if error:
        with pytest.raises(RuntimeError, match="caller failure"):
            with blob:
                connection = blob._conn
                blob.put("pose", 1, b"written")
                raise RuntimeError("caller failure")
    else:
        with blob:
            connection = blob._conn
            blob.put("pose", 1, b"written")
    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        connection.execute("SELECT 1")
    blob.stop()  # Repeated resource cleanup remains safe.


@pytest.mark.parametrize("eager", [False, True], ids=["lazy", "eager"])
def test_external_pose_append_reopen_preserves_encoded_payloads(
    tmp_path: Path, eager: bool
) -> None:
    metadata = tmp_path / "recording.db"
    payloads = tmp_path / "payloads.db"
    expected = [PoseStamped(ts=100 + i / 8, position=(i, i / 2, -i)) for i in range(6)]
    with SqliteStore(path=metadata) as store:
        stream = store.stream(
            "pose", PoseStamped, blob_store=SqliteBlobStore(path=str(payloads)), eager_blobs=eager
        )
        for value in expected[:4]:
            stream.append(value, ts=value.ts)
    with SqliteStore(path=metadata, must_exist=True) as store:
        stream = store.stream("pose")
        for value in expected[4:]:
            stream.append(value, ts=value.ts)
    with SqliteStore(path=metadata, must_exist=True) as store:
        actual = list(store.replay().streams.pose.iterate())
    assert [p.lcm_encode() for p in actual] == [p.lcm_encode() for p in expected]
    rows = _rows(payloads, "pose_blob")
    assert [key for key, _ in rows] == list(range(1, 7))
    assert [PoseStamped.lcm_decode(data).lcm_encode() for _, data in rows] == [
        p.lcm_encode() for p in expected
    ]


@pytest.mark.parametrize("eager", [False, True], ids=["lazy", "eager"])
def test_default_shared_commit_error_rolls_back_both_tables(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, eager: bool
) -> None:
    path = tmp_path / "default.db"
    with SqliteStore(path=path) as store:
        stream = store.stream("payload", bytes, eager_blobs=eager)
        stream.append(b"first", ts=1.0)
        metadata_before = _rows(path, "payload")
        blobs_before = _rows(path, "payload_blob")

        def fail_commit() -> None:
            # Blob.put must not have committed the shared connection prematurely.
            assert _rows(path, "payload") == metadata_before
            assert _rows(path, "payload_blob") == blobs_before
            raise sqlite3.OperationalError("injected commit failure")

        with monkeypatch.context() as patch:
            patch.setattr(stream._source.metadata_store, "commit", fail_commit)
            with pytest.raises(sqlite3.OperationalError, match="injected commit failure"):
                stream.append(b"failed", ts=2.0)
        assert _rows(path, "payload") == metadata_before
        assert _rows(path, "payload_blob") == blobs_before
        stream.append(b"recovered", ts=3.0)
    with SqliteStore(path=path, must_exist=True) as store:
        assert list(store.replay().streams.payload.iterate()) == [b"first", b"recovered"]
