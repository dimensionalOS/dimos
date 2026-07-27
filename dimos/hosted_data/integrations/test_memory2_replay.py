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
import sys
import threading
from types import ModuleType

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


@pytest.mark.parametrize(
    "value",
    [
        f"dimos-replay://bad owner/repo/{'a' * 64}?server=https%3A%2F%2Freplays.example",
        f"dimos-replay://alice/bad%20repo/{'a' * 64}?server=https%3A%2F%2Freplays.example",
        f"dimos-replay://alice/repo/{'a' * 64}?server=https%3A%2F%2Fuser%40example.test",
        f"dimos-replay://alice/repo/{'a' * 64}?server=https%3A%2F%2Fexample.test%2F%3Fx%3D1",
        f"dimos-replay://alice/repo/{'a' * 64}?server=https%3A%2F%2Fexample.test#fragment",
        f"dimos-replay://alice/extra/repo/{'a' * 64}?server=https%3A%2F%2Fexample.test",
    ],
)
def test_remote_reference_rejects_unsafe_identity_and_server(value: str) -> None:
    with pytest.raises(ValueError):
        RemoteReplayReference.parse(value)


def test_configured_nodes_are_trusted(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("DIMOS_REPLAY_SERVER_URL", raising=False)
    monkeypatch.setenv(
        "DIMOS_REPLAY_NODES",
        '[{"name":"cn","url":"https://cn.example/","region":"china"}]',
    )

    assert memory2_replay._configured_server_urls() == {"https://cn.example"}


def test_memory2_validation_rejects_sqlite_without_streams(tmp_path: Path) -> None:
    database = tmp_path / "ordinary.db"
    with sqlite3.connect(database) as connection:
        connection.execute("CREATE TABLE events (id INTEGER)")

    with pytest.raises(ValueError, match="not a memory2"):
        memory2_replay._validate_memory2_database(database)


def test_cache_lock_timeout_and_ownership_fail_closed(tmp_path: Path) -> None:
    lock_path = tmp_path / "cache.lock"
    lock_path.write_text("other-owner", encoding="ascii")

    assert not memory2_replay._lock_is_owned(lock_path, "expected-owner")
    assert not memory2_replay._lock_is_owned(tmp_path / "missing.lock", "owner")
    with pytest.raises(TimeoutError, match="timed out waiting"):
        memory2_replay._acquire_cache_lock(lock_path, timeout_seconds=0)


def test_cache_lock_heartbeat_stops_after_ownership_changes(tmp_path: Path) -> None:
    class ImmediateTick:
        def wait(self, _: float) -> bool:
            return False

    lock_path = tmp_path / "cache.lock"
    lock_path.write_text("new-owner", encoding="ascii")

    memory2_replay._heartbeat_cache_lock(
        lock_path,
        "old-owner",
        ImmediateTick(),  # type: ignore[arg-type]
    )
    assert lock_path.read_text(encoding="ascii") == "new-owner"


def test_cache_lock_release_removes_the_current_owner(tmp_path: Path) -> None:
    lock_path = tmp_path / "cache.lock"
    lock = memory2_replay._acquire_cache_lock(lock_path)

    memory2_replay._release_cache_lock(lock)

    assert not lock_path.exists()


def test_cache_lock_creation_cleans_up_after_initialization_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lock_path = tmp_path / "cache.lock"
    monkeypatch.setattr(
        memory2_replay.os,
        "write",
        lambda *_: (_ for _ in ()).throw(OSError("disk unavailable")),
    )

    with pytest.raises(OSError, match="disk unavailable"):
        memory2_replay._acquire_cache_lock(lock_path)
    assert not lock_path.exists()


def test_resolver_rechecks_cache_after_acquiring_lock(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _memory2_database(tmp_path / "source.db")
    reference = _reference(source)
    monkeypatch.setenv("DIMOS_REPLAY_SERVER_URL", reference.server_url)
    destination = (
        tmp_path / "cache" / reference.owner / reference.repository / f"{reference.object_id}.db"
    )
    real_acquire = memory2_replay._acquire_cache_lock

    def populate_then_lock(path: Path) -> memory2_replay._CacheLock:
        destination.write_bytes(source.read_bytes())
        return real_acquire(path)

    monkeypatch.setattr(memory2_replay, "_acquire_cache_lock", populate_then_lock)
    monkeypatch.setattr(
        memory2_replay,
        "download_object",
        lambda **_: pytest.fail("valid cache must avoid a second download"),
    )

    assert resolve_replay_dataset(reference.to_uri(), cache_dir=tmp_path / "cache") == destination


def test_cache_lock_waits_once_then_acquires(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lock_path = tmp_path / "cache.lock"
    lock_path.write_text("current-owner", encoding="ascii")
    monkeypatch.setattr(memory2_replay.time, "sleep", lambda _: lock_path.unlink())

    lock = memory2_replay._acquire_cache_lock(lock_path, timeout_seconds=1)
    memory2_replay._release_cache_lock(lock)

    assert not lock_path.exists()


def test_cache_heartbeat_stops_when_touch_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class ImmediateTick:
        def wait(self, _: float) -> bool:
            return False

    lock_path = tmp_path / "cache.lock"
    monkeypatch.setattr(memory2_replay, "_lock_is_owned", lambda *_: True)
    monkeypatch.setattr(Path, "touch", lambda *_: (_ for _ in ()).throw(OSError("disk")))

    memory2_replay._heartbeat_cache_lock(
        lock_path,
        "owner",
        ImmediateTick(),  # type: ignore[arg-type]
    )


def test_local_resolver_delegates_to_memory2(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    expected = tmp_path / "local.db"
    replay_module = ModuleType("dimos.memory2.replay")
    replay_module.resolve_db_path = lambda dataset: expected  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.memory2.replay", replay_module)

    assert memory2_replay._resolve_local_replay("local") == expected
