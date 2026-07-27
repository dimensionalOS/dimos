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

"""Persistent sequential multipart uploads for large replay objects."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import os
from pathlib import Path
import tempfile
from threading import Lock
from typing import Any
from uuid import uuid4

from dimos.hosted_data.repository import (
    BinaryReader,
    ReplayObject,
    ReplayRepositoryBackend,
    RepositoryError,
    _validate_filename,
    _validate_name,
    _validate_object_id,
)

_COPY_CHUNK_SIZE = 1024 * 1024


@dataclass(frozen=True)
class UploadSession:
    """Durable server-side state for one resumable upload."""

    upload_id: str
    owner: str
    repository: str
    filename: str
    size_bytes: int
    content_type: str
    expected_sha256: str
    received_bytes: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> UploadSession:
        try:
            upload_id = str(data["upload_id"])
            if len(upload_id) != 32 or any(
                character not in "0123456789abcdef" for character in upload_id
            ):
                raise ValueError("invalid upload id")
            owner = _validate_name(str(data["owner"]), "owner")
            repository = _validate_name(str(data["repository"]), "repository")
            filename = _validate_filename(str(data["filename"]))
            size_bytes = int(data["size_bytes"])
            received_bytes = int(data.get("received_bytes", 0))
            content_type = str(data["content_type"])
            expected_sha256 = _validate_object_id(str(data["expected_sha256"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise RepositoryError("invalid resumable upload metadata") from exc
        if size_bytes < 0 or received_bytes < 0 or received_bytes > size_bytes:
            raise RepositoryError("invalid resumable upload byte counts")
        return cls(
            upload_id=upload_id,
            owner=owner,
            repository=repository,
            filename=filename,
            size_bytes=size_bytes,
            content_type=content_type,
            expected_sha256=expected_sha256,
            received_bytes=received_bytes,
        )


class ResumableUploadManager:
    """Persist upload state so transfers can continue after process restarts."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._lock = Lock()

    def _paths(self, upload_id: str) -> tuple[Path, Path]:
        if len(upload_id) != 32 or any(
            character not in "0123456789abcdef" for character in upload_id
        ):
            raise RepositoryError("invalid upload id")
        return self.root / f"{upload_id}.json", self.root / f"{upload_id}.part"

    def _write(self, session: UploadSession) -> None:
        metadata_path, _ = self._paths(session.upload_id)
        self.root.mkdir(parents=True, exist_ok=True)
        temporary: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                prefix=f".{session.upload_id}.",
                suffix=".json.part",
                dir=self.root,
                delete=False,
            ) as target:
                temporary = Path(target.name)
                json.dump(session.to_dict(), target, sort_keys=True)
            os.replace(temporary, metadata_path)
            temporary = None
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def create(
        self,
        *,
        owner: str,
        repository: str,
        filename: str,
        size_bytes: int,
        content_type: str,
        expected_sha256: str,
    ) -> UploadSession:
        if size_bytes < 0:
            raise RepositoryError("size_bytes cannot be negative")
        session = UploadSession(
            upload_id=uuid4().hex,
            owner=_validate_name(owner, "owner"),
            repository=_validate_name(repository, "repository"),
            filename=_validate_filename(filename),
            size_bytes=size_bytes,
            content_type=content_type or "application/octet-stream",
            expected_sha256=_validate_object_id(expected_sha256),
        )
        with self._lock:
            self._write(session)
            _, part_path = self._paths(session.upload_id)
            part_path.touch(exist_ok=False)
        return session

    def get(self, upload_id: str, *, owner: str, repository: str) -> UploadSession:
        metadata_path, part_path = self._paths(upload_id)
        if not metadata_path.is_file() or not part_path.is_file():
            raise FileNotFoundError(upload_id)
        session = UploadSession.from_dict(json.loads(metadata_path.read_text(encoding="utf-8")))
        if session.owner != owner or session.repository != repository:
            raise FileNotFoundError(upload_id)
        actual_size = part_path.stat().st_size
        if actual_size != session.received_bytes:
            raise RepositoryError("resumable upload state does not match its partial file")
        return session

    def append(
        self,
        upload_id: str,
        *,
        owner: str,
        repository: str,
        offset: int,
        source: BinaryReader,
        size_bytes: int,
    ) -> UploadSession:
        if size_bytes < 1:
            raise RepositoryError("upload chunk cannot be empty")
        with self._lock:
            session = self.get(upload_id, owner=owner, repository=repository)
            if offset != session.received_bytes:
                raise RepositoryError(
                    f"chunk offset {offset} does not match received bytes {session.received_bytes}"
                )
            if offset + size_bytes > session.size_bytes:
                raise RepositoryError("upload chunk exceeds declared object size")
            _, part_path = self._paths(upload_id)
            remaining = size_bytes
            with part_path.open("ab") as target:
                while remaining:
                    chunk = source.read(min(_COPY_CHUNK_SIZE, remaining))
                    if not chunk:
                        raise RepositoryError(f"chunk ended with {remaining} bytes still expected")
                    target.write(chunk)
                    remaining -= len(chunk)
                target.flush()
                os.fsync(target.fileno())
            updated = replace(session, received_bytes=offset + size_bytes)
            self._write(updated)
            return updated

    def complete(
        self,
        upload_id: str,
        *,
        owner: str,
        repository: str,
        backend: ReplayRepositoryBackend,
    ) -> ReplayObject:
        with self._lock:
            session = self.get(upload_id, owner=owner, repository=repository)
            if session.received_bytes != session.size_bytes:
                raise RepositoryError(
                    f"upload has {session.received_bytes} of {session.size_bytes} bytes"
                )
            metadata_path, part_path = self._paths(upload_id)
            with part_path.open("rb") as source:
                result = backend.put_stream(
                    owner=owner,
                    repository=repository,
                    filename=session.filename,
                    source=source,
                    size_bytes=session.size_bytes,
                    content_type=session.content_type,
                    expected_sha256=session.expected_sha256,
                )
            metadata_path.unlink(missing_ok=True)
            part_path.unlink(missing_ok=True)
            return result

    def cancel(self, upload_id: str, *, owner: str, repository: str) -> None:
        with self._lock:
            self.get(upload_id, owner=owner, repository=repository)
            metadata_path, part_path = self._paths(upload_id)
            metadata_path.unlink(missing_ok=True)
            part_path.unlink(missing_ok=True)
