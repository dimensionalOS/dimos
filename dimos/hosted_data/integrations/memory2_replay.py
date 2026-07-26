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

"""Resolve immutable hosted objects into local memory2 replay datasets."""

from __future__ import annotations

from contextlib import closing
from dataclasses import dataclass
import os
from pathlib import Path
import re
import sqlite3
from threading import Event, Thread
import time
from urllib.parse import parse_qs, urlencode, urlsplit, urlunsplit
from uuid import uuid4

from dimos.constants import CACHE_DIR
from dimos.hosted_data.repository import (
    download_object,
    sha256_file,
)

_REMOTE_REPLAY_SCHEME = "dimos-replay"
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_OBJECT_ID_RE = re.compile(r"^[0-9a-f]{64}$")
_STALE_LOCK_SECONDS = 300.0
_LOCK_HEARTBEAT_SECONDS = 30.0


def _validate_name(value: str, label: str) -> str:
    if not _NAME_RE.fullmatch(value):
        raise ValueError(f"invalid remote replay {label}: {value!r}")
    return value


def _validate_object_id(value: str) -> str:
    if not _OBJECT_ID_RE.fullmatch(value):
        raise ValueError("remote replay object id must be a lowercase SHA-256 digest")
    return value


def _validate_server_url(value: str) -> str:
    parsed = urlsplit(value)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("remote replay server must be an absolute HTTP(S) URL")
    if parsed.username is not None or parsed.password is not None:
        raise ValueError("remote replay server URL must not contain credentials")
    if parsed.query or parsed.fragment:
        raise ValueError("remote replay server URL must not contain a query or fragment")
    return value.rstrip("/")


def _configured_server_urls() -> set[str]:
    """Return server URLs explicitly trusted by the local environment."""
    urls: set[str] = set()
    configured_server = os.environ.get("DIMOS_REPLAY_SERVER_URL")
    if configured_server:
        urls.add(_validate_server_url(configured_server))
    configured_nodes = os.environ.get("DIMOS_REPLAY_NODES")
    if configured_nodes:
        from dimos.hosted_data.nodes import load_replay_nodes

        urls.update(node.url for node in load_replay_nodes(configured_nodes))
    return urls


def _require_trusted_server(server_url: str) -> None:
    if server_url in _configured_server_urls():
        return
    raise ValueError(
        "remote replay server is not trusted; set DIMOS_REPLAY_SERVER_URL "
        "or include it in DIMOS_REPLAY_NODES before opening a shared reference"
    )


@dataclass(frozen=True)
class RemoteReplayReference:
    """Address of one immutable memory2 object in a replay repository."""

    server_url: str
    owner: str
    repository: str
    object_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "server_url", _validate_server_url(self.server_url))
        object.__setattr__(self, "owner", _validate_name(self.owner, "owner"))
        object.__setattr__(
            self,
            "repository",
            _validate_name(self.repository, "repository"),
        )
        object.__setattr__(self, "object_id", _validate_object_id(self.object_id))

    def to_uri(self) -> str:
        return urlunsplit(
            (
                _REMOTE_REPLAY_SCHEME,
                self.owner,
                f"/{self.repository}/{self.object_id}",
                urlencode({"server": self.server_url}),
                "",
            )
        )

    @classmethod
    def parse(cls, value: str) -> RemoteReplayReference:
        parsed = urlsplit(value)
        if parsed.scheme != _REMOTE_REPLAY_SCHEME:
            raise ValueError(f"remote replay URI must use {_REMOTE_REPLAY_SCHEME}://")
        if parsed.fragment:
            raise ValueError("remote replay URI must not contain a fragment")
        parts = tuple(part for part in parsed.path.split("/") if part)
        if len(parts) != 2:
            raise ValueError("remote replay URI path must be /repository/object-id")
        query = parse_qs(parsed.query, strict_parsing=True)
        if set(query) != {"server"} or len(query["server"]) != 1:
            raise ValueError("remote replay URI requires exactly one server query parameter")
        return cls(
            server_url=query["server"][0],
            owner=parsed.netloc,
            repository=parts[0],
            object_id=parts[1],
        )


def is_remote_replay(value: str | Path) -> bool:
    return str(value).startswith(f"{_REMOTE_REPLAY_SCHEME}://")


def _validate_memory2_database(path: Path) -> None:
    try:
        with closing(sqlite3.connect(f"{path.resolve().as_uri()}?mode=ro", uri=True)) as connection:
            row = connection.execute(
                "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = '_streams'"
            ).fetchone()
    except sqlite3.DatabaseError as exc:
        raise ValueError("remote replay object is not a valid SQLite database") from exc
    if row is None:
        raise ValueError("remote replay object is not a memory2 SQLite database")


@dataclass(frozen=True)
class _CacheLock:
    path: Path
    file_descriptor: int
    owner: str
    stop_event: Event
    heartbeat_thread: Thread


def _lock_is_owned(path: Path, owner: str) -> bool:
    try:
        return path.read_text(encoding="ascii") == owner
    except (FileNotFoundError, OSError, UnicodeError):
        return False


def _heartbeat_cache_lock(path: Path, owner: str, stop_event: Event) -> None:
    while not stop_event.wait(_LOCK_HEARTBEAT_SECONDS):
        if not _lock_is_owned(path, owner):
            return
        try:
            path.touch()
        except OSError:
            return


def _acquire_cache_lock(path: Path, *, timeout_seconds: float = 60.0) -> _CacheLock:
    deadline = time.monotonic() + timeout_seconds
    while True:
        try:
            file_descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        except FileExistsError:
            try:
                lock_age = time.time() - path.stat().st_mtime
            except FileNotFoundError:
                continue
            if lock_age >= _STALE_LOCK_SECONDS:
                path.unlink(missing_ok=True)
                continue
            if time.monotonic() >= deadline:
                raise TimeoutError(f"timed out waiting for replay cache lock {path}") from None
            time.sleep(0.1)
            continue
        try:
            owner = f"{os.getpid()}:{uuid4().hex}"
            os.write(file_descriptor, owner.encode("ascii"))
            os.fsync(file_descriptor)
            stop_event = Event()
            heartbeat_thread = Thread(
                target=_heartbeat_cache_lock,
                args=(path, owner, stop_event),
                daemon=True,
                name="dimos-replay-cache-lock",
            )
            heartbeat_thread.start()
            return _CacheLock(
                path=path,
                file_descriptor=file_descriptor,
                owner=owner,
                stop_event=stop_event,
                heartbeat_thread=heartbeat_thread,
            )
        except BaseException:
            os.close(file_descriptor)
            path.unlink(missing_ok=True)
            raise


def _release_cache_lock(lock: _CacheLock) -> None:
    lock.stop_event.set()
    lock.heartbeat_thread.join(timeout=1.0)
    os.close(lock.file_descriptor)
    if _lock_is_owned(lock.path, lock.owner):
        lock.path.unlink(missing_ok=True)


def _resolve_local_replay(dataset: str | Path) -> Path:
    from dimos.memory2.replay import resolve_db_path

    return Path(resolve_db_path(dataset))


def resolve_replay_dataset(
    dataset: str | Path,
    *,
    token: str | None = None,
    cache_dir: str | Path | None = None,
) -> Path:
    """Resolve a local dataset or download and cache a remote memory2 object."""
    if not is_remote_replay(dataset):
        return _resolve_local_replay(dataset)

    reference = RemoteReplayReference.parse(str(dataset))
    _require_trusted_server(reference.server_url)
    root = Path(cache_dir) if cache_dir is not None else CACHE_DIR / "replays"
    destination = root / reference.owner / reference.repository / f"{reference.object_id}.db"
    if destination.is_file() and sha256_file(destination) == reference.object_id:
        _validate_memory2_database(destination)
        return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    lock_path = destination.with_suffix(".lock")
    cache_lock = _acquire_cache_lock(lock_path)
    try:
        if destination.is_file() and sha256_file(destination) == reference.object_id:
            _validate_memory2_database(destination)
            return destination
        download_object(
            server_url=reference.server_url,
            owner=reference.owner,
            repository=reference.repository,
            object_id=reference.object_id,
            output=destination,
            token=token or os.environ.get("DIMOS_REPLAY_REPOSITORY_TOKEN"),
            overwrite=True,
        )
        _validate_memory2_database(destination)
        return destination
    except Exception:
        destination.unlink(missing_ok=True)
        raise
    finally:
        _release_cache_lock(cache_lock)
