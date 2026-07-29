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

"""Tests for the hosted-data filesystem repository."""

from __future__ import annotations

from dataclasses import replace
import hashlib
from io import BytesIO
import json
from pathlib import Path

import pytest

from dimos.hosted_data.repository import (
    ReplayObject,
    ReplayRepository,
    RepositoryError,
    sha256_file,
)


def _put(
    repository: ReplayRepository,
    payload: bytes = b"replay-data",
    *,
    owner: str = "alice",
    name: str = "go2",
    filename: str = "recording.db",
    expected_sha256: str | None = None,
) -> ReplayObject:
    return repository.put_stream(
        owner=owner,
        repository=name,
        filename=filename,
        source=BytesIO(payload),
        size_bytes=len(payload),
        content_type="application/x-sqlite3",
        expected_sha256=expected_sha256,
    )


def test_put_open_and_list_round_trip(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    payload = b"memory2" * 100_000

    stored = _put(repository, payload)
    metadata, body = repository.open("alice", "go2", stored.object_id)
    try:
        downloaded = body.read()
    finally:
        body.close()

    assert metadata == stored
    assert downloaded == payload
    assert stored.object_id == hashlib.sha256(payload).hexdigest()
    assert repository.list("alice", "go2") == [stored]


def test_repositories_are_scoped_by_owner_and_name(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    stored = _put(repository)

    assert repository.list("alice", "go2") == [stored]
    assert repository.list("bob", "go2") == []
    assert repository.list("alice", "another") == []
    with pytest.raises(FileNotFoundError):
        repository.get("bob", "go2", stored.object_id)


def test_same_content_is_idempotent(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    first = _put(repository, filename="first.db")
    second = _put(repository, filename="second.db")

    assert second == first
    assert repository.list("alice", "go2") == [first]


def test_expected_digest_is_checked_before_publication(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")

    with pytest.raises(RepositoryError, match="SHA-256 mismatch"):
        _put(repository, expected_sha256="0" * 64)

    assert repository.list("alice", "go2") == []
    assert list((tmp_path / "objects").rglob("*.part")) == []


def test_short_stream_is_rejected_and_cleaned_up(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")

    with pytest.raises(RepositoryError, match="still expected"):
        repository.put_stream(
            owner="alice",
            repository="go2",
            filename="short.db",
            source=BytesIO(b"short"),
            size_bytes=10,
            content_type="application/x-sqlite3",
        )

    assert repository.list("alice", "go2") == []
    assert list((tmp_path / "objects").rglob("*.part")) == []


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("owner", "../alice"),
        ("name", "go2/office"),
        ("filename", "../recording.db"),
        ("filename", "folder\\recording.db"),
    ],
)
def test_paths_and_names_are_validated(
    tmp_path: Path,
    field: str,
    value: str,
) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    arguments = {
        "owner": "alice",
        "name": "go2",
        "filename": "recording.db",
    }
    arguments[field] = value

    with pytest.raises(RepositoryError):
        _put(repository, **arguments)


@pytest.mark.parametrize("size_bytes", [-1, True, "1"])
def test_metadata_rejects_invalid_size_types(size_bytes: object) -> None:
    digest = hashlib.sha256(b"data").hexdigest()
    data = {
        "owner": "alice",
        "repository": "go2",
        "object_id": digest,
        "filename": "recording.db",
        "size_bytes": size_bytes,
        "sha256": digest,
        "content_type": "application/x-sqlite3",
        "created_at": "2026-01-01T00:00:00+00:00",
    }

    with pytest.raises(RepositoryError):
        ReplayObject.from_dict(data)


def test_metadata_digest_must_match_object_id() -> None:
    digest = hashlib.sha256(b"data").hexdigest()
    metadata = ReplayObject(
        owner="alice",
        repository="go2",
        object_id=digest,
        filename="recording.db",
        size_bytes=4,
        sha256="0" * 64,
        content_type="application/x-sqlite3",
        created_at="2026-01-01T00:00:00+00:00",
    )

    with pytest.raises(RepositoryError, match="sha256 must match"):
        ReplayObject.from_dict(metadata.to_dict())


def test_get_rejects_metadata_from_another_repository(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    stored = _put(repository)
    _, metadata_path = repository._paths("alice", "go2", stored.object_id)
    metadata_path.write_text(
        json.dumps(replace(stored, owner="bob").to_dict()),
        encoding="utf-8",
    )

    with pytest.raises(RepositoryError, match="does not match"):
        repository.get("alice", "go2", stored.object_id)


def test_get_rejects_truncated_blob(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    stored = _put(repository)
    _, object_path = repository.get("alice", "go2", stored.object_id)
    object_path.write_bytes(b"short")

    with pytest.raises(RepositoryError, match="size does not match"):
        repository.get("alice", "go2", stored.object_id)


def test_recover_incomplete_uploads(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    first = repository.root / "alice" / "go2" / ".upload-a.part"
    second = repository.root / "bob" / "go2" / ".upload-b.part"
    first.parent.mkdir(parents=True)
    second.parent.mkdir(parents=True)
    first.write_bytes(b"partial")
    second.write_bytes(b"partial")

    assert repository.recover_incomplete_uploads() == 2
    assert not first.exists()
    assert not second.exists()


def test_sha256_file_streams_file_contents(tmp_path: Path) -> None:
    path = tmp_path / "recording.db"
    path.write_bytes(b"memory2" * 200_000)

    assert sha256_file(path) == hashlib.sha256(path.read_bytes()).hexdigest()
