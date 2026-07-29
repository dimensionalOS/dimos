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

"""Content-addressed filesystem storage for hosted DimOS replay data."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Protocol

_CHUNK_SIZE = 1024 * 1024
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_OBJECT_ID_RE = re.compile(r"^[0-9a-f]{64}$")


class RepositoryError(ValueError):
    """Repository data or input is invalid."""


class BinaryReader(Protocol):
    """Minimum input interface required by the repository."""

    def read(self, size: int = -1) -> bytes: ...


class ClosableBinaryReader(BinaryReader, Protocol):
    """Seekable object body that releases its underlying resource."""

    def seek(self, offset: int, whence: int = 0) -> int: ...

    def close(self) -> None: ...


def _validate_name(value: str, label: str) -> str:
    if not _NAME_RE.fullmatch(value):
        raise RepositoryError(
            f"{label} must start with an ASCII letter or number and contain at most "
            "64 letters, numbers, dots, underscores, or hyphens"
        )
    return value


def _validate_object_id(value: str) -> str:
    if not _OBJECT_ID_RE.fullmatch(value):
        raise RepositoryError("object_id must be a lowercase SHA-256 hex digest")
    return value


def _validate_filename(value: str) -> str:
    if (
        not value
        or Path(value).name != value
        or value in {".", ".."}
        or "/" in value
        or "\\" in value
    ):
        raise RepositoryError("filename must be a plain file name without directories")
    return value


def sha256_file(path: str | Path) -> str:
    """Return a file's SHA-256 digest without loading it into memory."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        while chunk := source.read(_CHUNK_SIZE):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class ReplayObject:
    """Metadata for one immutable replay object."""

    owner: str
    repository: str
    object_id: str
    filename: str
    size_bytes: int
    sha256: str
    content_type: str
    created_at: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ReplayObject:
        try:
            strings = {
                field: data[field]
                for field in (
                    "owner",
                    "repository",
                    "object_id",
                    "filename",
                    "sha256",
                    "content_type",
                    "created_at",
                )
            }
            if any(not isinstance(value, str) for value in strings.values()):
                raise TypeError("string field has the wrong type")
            size_bytes = data["size_bytes"]
            if not isinstance(size_bytes, int) or isinstance(size_bytes, bool):
                raise TypeError("size_bytes must be an integer")
            owner = _validate_name(strings["owner"], "owner")
            repository = _validate_name(strings["repository"], "repository")
            object_id = _validate_object_id(strings["object_id"])
            filename = _validate_filename(strings["filename"])
            sha256 = _validate_object_id(strings["sha256"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RepositoryError("invalid replay object metadata") from exc
        if size_bytes < 0:
            raise RepositoryError("size_bytes cannot be negative")
        if sha256 != object_id:
            raise RepositoryError("object sha256 must match object_id")
        if not strings["content_type"] or not strings["created_at"]:
            raise RepositoryError("content_type and created_at cannot be empty")
        return cls(
            owner=owner,
            repository=repository,
            object_id=object_id,
            filename=filename,
            size_bytes=size_bytes,
            sha256=sha256,
            content_type=strings["content_type"],
            created_at=strings["created_at"],
        )


class ReplayRepositoryBackend(Protocol):
    """Storage contract used by later transfer and replay integrations."""

    def put_stream(
        self,
        *,
        owner: str,
        repository: str,
        filename: str,
        source: BinaryReader,
        size_bytes: int,
        content_type: str,
        expected_sha256: str | None = None,
    ) -> ReplayObject: ...

    def open(
        self,
        owner: str,
        repository: str,
        object_id: str,
    ) -> tuple[ReplayObject, ClosableBinaryReader]: ...

    def list(self, owner: str, repository: str) -> list[ReplayObject]: ...


class ReplayRepository:
    """Filesystem-backed immutable object repository."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)

    def _repository_dir(self, owner: str, repository: str) -> Path:
        return self.root / _validate_name(owner, "owner") / _validate_name(repository, "repository")

    def _paths(self, owner: str, repository: str, object_id: str) -> tuple[Path, Path]:
        directory = self._repository_dir(owner, repository)
        object_id = _validate_object_id(object_id)
        return directory / f"{object_id}.blob", directory / f"{object_id}.json"

    @staticmethod
    def _read_metadata(path: Path) -> ReplayObject:
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise RepositoryError(f"invalid object metadata: {path.name}") from exc
        if not isinstance(raw, dict):
            raise RepositoryError(f"invalid object metadata: {path.name}")
        return ReplayObject.from_dict(raw)

    def put_stream(
        self,
        *,
        owner: str,
        repository: str,
        filename: str,
        source: BinaryReader,
        size_bytes: int,
        content_type: str,
        expected_sha256: str | None = None,
    ) -> ReplayObject:
        """Atomically store exactly ``size_bytes`` bytes."""
        if size_bytes < 0:
            raise RepositoryError("size_bytes cannot be negative")
        if expected_sha256 is not None:
            _validate_object_id(expected_sha256)
        filename = _validate_filename(filename)
        directory = self._repository_dir(owner, repository)
        directory.mkdir(parents=True, exist_ok=True)

        digest = hashlib.sha256()
        temporary_path: Path | None = None
        remaining = size_bytes
        try:
            with tempfile.NamedTemporaryFile(
                mode="wb",
                prefix=".upload-",
                suffix=".part",
                dir=directory,
                delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                while remaining:
                    chunk = source.read(min(_CHUNK_SIZE, remaining))
                    if not chunk:
                        raise RepositoryError(f"stream ended with {remaining} bytes still expected")
                    if len(chunk) > remaining:
                        raise RepositoryError("stream returned more bytes than requested")
                    temporary.write(chunk)
                    digest.update(chunk)
                    remaining -= len(chunk)

            object_id = digest.hexdigest()
            if expected_sha256 is not None and object_id != expected_sha256:
                raise RepositoryError(
                    f"SHA-256 mismatch: expected {expected_sha256}, received {object_id}"
                )
            object_path, metadata_path = self._paths(owner, repository, object_id)
            if object_path.is_file() and metadata_path.is_file():
                return self.get(owner, repository, object_id)[0]

            metadata = ReplayObject(
                owner=owner,
                repository=repository,
                object_id=object_id,
                filename=filename,
                size_bytes=size_bytes,
                sha256=object_id,
                content_type=content_type or "application/octet-stream",
                created_at=datetime.now(timezone.utc).isoformat(),
            )
            os.replace(temporary_path, object_path)
            temporary_path = None
            metadata_part = metadata_path.with_name(f".{metadata_path.name}.{os.getpid()}.part")
            try:
                metadata_part.write_text(
                    json.dumps(metadata.to_dict(), sort_keys=True),
                    encoding="utf-8",
                )
                os.replace(metadata_part, metadata_path)
            finally:
                metadata_part.unlink(missing_ok=True)
            return metadata
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def get(self, owner: str, repository: str, object_id: str) -> tuple[ReplayObject, Path]:
        """Return verified metadata and the local object path."""
        object_path, metadata_path = self._paths(owner, repository, object_id)
        if not object_path.is_file() or not metadata_path.is_file():
            raise FileNotFoundError(object_id)
        metadata = self._read_metadata(metadata_path)
        if (
            metadata.owner != owner
            or metadata.repository != repository
            or metadata.object_id != object_id
        ):
            raise RepositoryError("object metadata does not match its repository path")
        if object_path.stat().st_size != metadata.size_bytes:
            raise RepositoryError("object size does not match its metadata")
        return metadata, object_path

    def open(
        self,
        owner: str,
        repository: str,
        object_id: str,
    ) -> tuple[ReplayObject, ClosableBinaryReader]:
        """Open one object for streaming."""
        metadata, object_path = self.get(owner, repository, object_id)
        return metadata, object_path.open("rb")

    def list(self, owner: str, repository: str) -> list[ReplayObject]:
        """List complete objects in newest-first order."""
        directory = self._repository_dir(owner, repository)
        if not directory.is_dir():
            return []
        objects = [self.get(owner, repository, path.stem)[0] for path in directory.glob("*.json")]
        return sorted(objects, key=lambda item: item.created_at, reverse=True)

    def recover_incomplete_uploads(self) -> int:
        """Remove temporary files left by interrupted writes."""
        if not self.root.exists():
            return 0
        parts = [path for path in self.root.rglob("*.part") if path.is_file()]
        for path in parts:
            path.unlink(missing_ok=True)
        return len(parts)
