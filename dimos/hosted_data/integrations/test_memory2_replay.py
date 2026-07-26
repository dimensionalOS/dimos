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

"""Tests for remote memory2 replay integration."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import closing, contextmanager
import os
from pathlib import Path
import sqlite3
import threading

import pytest

from dimos.hosted_data.integrations import memory2_replay
from dimos.hosted_data.integrations.memory2_replay import (
    RemoteReplayReference,
    is_remote_replay,
    resolve_replay_dataset,
)
from dimos.hosted_data.repository import (
    ReplayRepository,
    ReplayRepositoryServer,
    sha256_file,
    upload_file,
)


def _memory2_database(path: Path) -> Path:
    with sqlite3.connect(path) as connection:
        connection.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
    return path


def _reference(path: Path) -> RemoteReplayReference:
    return RemoteReplayReference(
        server_url="https://replays.example",
        owner="alice",
        repository="go2-debug",
        object_id=sha256_file(path),
    )


@contextmanager
def _running_server(root: Path) -> Iterator[str]:
    server = ReplayRepositoryServer(
        ("127.0.0.1", 0),
        ReplayRepository(root),
        token="test-token",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address[:2]
        host_text = host.decode() if isinstance(host, bytes) else host
        yield f"http://{host_text}:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5.0)


def test_remote_replay_reference_round_trip(tmp_path: Path) -> None:
    source = _memory2_database(tmp_path / "go2.db")
    reference = _reference(source)

    assert RemoteReplayReference.parse(reference.to_uri()) == reference
    assert is_remote_replay(reference.to_uri())
    assert not is_remote_replay(source)
    assert "replays.example" in reference.to_uri()


@pytest.mark.parametrize(
    "value",
    [
        "https://replays.example/object",
        "dimos-replay://alice/repo/not-a-digest?server=https%3A%2F%2Freplays.example",
        f"dimos-replay://alice/repo/{'a' * 64}",
        f"dimos-replay://alice/repo/{'a' * 64}?server=file%3A%2F%2Ftmp",
    ],
)
def test_invalid_remote_replay_reference(value: str) -> None:
    with pytest.raises(ValueError):
        RemoteReplayReference.parse(value)


def test_resolve_remote_replay_downloads_and_reuses_cache(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _memory2_database(tmp_path / "source.db")
    reference = _reference(source)
    calls: list[dict[str, object]] = []
    monkeypatch.setenv("DIMOS_REPLAY_SERVER_URL", reference.server_url)

    def fake_download(**kwargs: object) -> Path:
        calls.append(kwargs)
        destination = Path(str(kwargs["output"]))
        destination.write_bytes(source.read_bytes())
        return destination

    monkeypatch.setattr(memory2_replay, "download_object", fake_download)

    first = resolve_replay_dataset(
        reference.to_uri(),
        token="secret",
        cache_dir=tmp_path / "cache",
    )
    second = resolve_replay_dataset(
        reference.to_uri(),
        token="secret",
        cache_dir=tmp_path / "cache",
    )

    assert first == second
    assert first.name == f"{reference.object_id}.db"
    assert sha256_file(first) == reference.object_id
    assert len(calls) == 1
    assert calls[0]["token"] == "secret"


def test_resolve_remote_replay_recovers_stale_cache_lock(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _memory2_database(tmp_path / "source.db")
    reference = _reference(source)
    monkeypatch.setenv("DIMOS_REPLAY_SERVER_URL", reference.server_url)
    destination = (
        tmp_path / "cache" / reference.owner / reference.repository / f"{reference.object_id}.db"
    )
    destination.parent.mkdir(parents=True)
    lock = destination.with_suffix(".lock")
    lock.write_text("stale")
    os.utime(lock, (0, 0))

    def fake_download(**kwargs: object) -> Path:
        target = Path(str(kwargs["output"]))
        target.write_bytes(source.read_bytes())
        return target

    monkeypatch.setattr(memory2_replay, "download_object", fake_download)

    assert resolve_replay_dataset(
        reference.to_uri(),
        cache_dir=tmp_path / "cache",
    ).is_file()
    assert not lock.exists()


def test_cache_lock_release_does_not_remove_a_new_owner(tmp_path: Path) -> None:
    lock_path = tmp_path / "cache.lock"
    lock = memory2_replay._acquire_cache_lock(lock_path)
    lock_path.write_text("replacement-owner", encoding="ascii")

    memory2_replay._release_cache_lock(lock)

    assert lock_path.read_text(encoding="ascii") == "replacement-owner"


def test_remote_memory2_http_round_trip(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _memory2_database(tmp_path / "go2.db")
    with _running_server(tmp_path / "objects") as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2-debug",
            path=source,
            token="test-token",
        )
        reference = RemoteReplayReference(
            server_url=server_url,
            owner=uploaded.owner,
            repository=uploaded.repository,
            object_id=uploaded.object_id,
        )
        monkeypatch.setenv("DIMOS_REPLAY_SERVER_URL", server_url)
        resolved = resolve_replay_dataset(
            reference.to_uri(),
            token="test-token",
            cache_dir=tmp_path / "cache",
        )

    assert resolved.is_file()
    assert sha256_file(resolved) == uploaded.object_id
    with closing(sqlite3.connect(resolved)) as connection:
        assert connection.execute("SELECT COUNT(*) FROM _streams").fetchone() == (0,)


def test_resolve_remote_replay_rejects_non_memory2_object(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "not-memory2.db"
    source.write_bytes(b"not sqlite")
    reference = _reference(source)
    monkeypatch.setenv("DIMOS_REPLAY_SERVER_URL", reference.server_url)

    def fake_download(**kwargs: object) -> Path:
        destination = Path(str(kwargs["output"]))
        destination.write_bytes(source.read_bytes())
        return destination

    monkeypatch.setattr(memory2_replay, "download_object", fake_download)

    with pytest.raises(ValueError, match="SQLite"):
        resolve_replay_dataset(reference.to_uri(), cache_dir=tmp_path / "cache")
    assert not tuple((tmp_path / "cache").rglob("*.db"))


def test_remote_replay_refuses_an_untrusted_uri_server(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _memory2_database(tmp_path / "source.db")
    reference = _reference(source)
    monkeypatch.setenv("DIMOS_REPLAY_SERVER_URL", "https://trusted.example")
    monkeypatch.setenv("DIMOS_REPLAY_REPOSITORY_TOKEN", "must-not-leak")
    monkeypatch.setattr(
        memory2_replay,
        "download_object",
        lambda **_: pytest.fail("untrusted server must not receive a request"),
    )

    with pytest.raises(ValueError, match="not trusted"):
        resolve_replay_dataset(reference.to_uri(), cache_dir=tmp_path / "cache")


def test_local_replay_uses_existing_resolver(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    expected = tmp_path / "local.db"
    monkeypatch.setattr(memory2_replay, "_resolve_local_replay", lambda _: expected)

    assert resolve_replay_dataset("local") == expected
