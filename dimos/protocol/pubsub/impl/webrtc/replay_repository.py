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

"""Small raw-object repository used to validate replay upload and download."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import hmac
from html import escape
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import re
import tempfile
import time
from typing import Any, Protocol, TypeVar, cast
from urllib.parse import parse_qs, quote, unquote, urlsplit

import requests

_CHUNK_SIZE = 1024 * 1024
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_OBJECT_ID_RE = re.compile(r"^[0-9a-f]{64}$")
_T = TypeVar("_T")


class RepositoryError(ValueError):
    """A repository request is invalid."""


class BinaryReader(Protocol):
    """Minimum interface needed for a streaming request body."""

    def read(self, size: int = -1) -> bytes: ...


class ClosableBinaryReader(BinaryReader, Protocol):
    """Streaming object body that can release its transport resources."""

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
    filename = Path(value).name
    if not filename or filename != value or filename in {".", ".."}:
        raise RepositoryError("filename must be a plain file name without directories")
    return filename


def _with_retries(
    operation: Callable[[], _T],
    *,
    retries: int,
    backoff_seconds: float,
) -> _T:
    if retries < 0:
        raise RepositoryError("retries cannot be negative")
    if backoff_seconds < 0:
        raise RepositoryError("backoff_seconds cannot be negative")
    for attempt in range(retries + 1):
        try:
            return operation()
        except (requests.RequestException, RuntimeError) as exc:
            response = exc.response if isinstance(exc, requests.HTTPError) else None
            status = response.status_code if response is not None else None
            retryable_status = status is None or status in {408, 429} or status >= 500
            if attempt >= retries or not retryable_status:
                raise
            time.sleep(min(backoff_seconds * (2**attempt), 30.0))
    raise AssertionError("retry loop returned without a result")


def sha256_file(path: str | Path) -> str:
    """Return the SHA-256 digest of a file without loading it into memory."""
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
            owner = _validate_name(str(data["owner"]), "owner")
            repository = _validate_name(str(data["repository"]), "repository")
            object_id = _validate_object_id(str(data["object_id"]))
            filename = _validate_filename(str(data["filename"]))
            size_bytes = int(data["size_bytes"])
            sha256 = _validate_object_id(str(data["sha256"]))
            content_type = str(data["content_type"])
            created_at = str(data["created_at"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RepositoryError("invalid replay object metadata") from exc
        if size_bytes < 0:
            raise RepositoryError("object size_bytes cannot be negative")
        if sha256 != object_id:
            raise RepositoryError("object sha256 must match object_id")
        if not content_type:
            raise RepositoryError("object content_type cannot be empty")
        if not created_at:
            raise RepositoryError("object created_at cannot be empty")
        return cls(
            owner=owner,
            repository=repository,
            object_id=object_id,
            filename=filename,
            size_bytes=size_bytes,
            sha256=sha256,
            content_type=content_type,
            created_at=created_at,
        )


@dataclass(frozen=True)
class ReplayManifest:
    """A stable batch transfer manifest."""

    owner: str
    repository: str
    objects: tuple[ReplayObject, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "owner": self.owner,
            "repository": self.repository,
            "objects": [item.to_dict() for item in self.objects],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ReplayManifest:
        if data.get("version") != 1:
            raise RepositoryError("manifest version must be 1")
        raw_objects = data.get("objects")
        if not isinstance(raw_objects, list):
            raise RepositoryError("manifest objects must be a list")
        if any(not isinstance(item, dict) for item in raw_objects):
            raise RepositoryError("every manifest object must be a JSON object")
        try:
            owner = _validate_name(str(data["owner"]), "owner")
            repository = _validate_name(str(data["repository"]), "repository")
        except (KeyError, TypeError, ValueError) as exc:
            raise RepositoryError("invalid manifest repository identity") from exc
        objects = tuple(ReplayObject.from_dict(item) for item in raw_objects)
        if any(item.owner != owner or item.repository != repository for item in objects):
            raise RepositoryError("manifest objects must belong to the manifest repository")
        return cls(owner=owner, repository=repository, objects=objects)


class BatchTransferError(RuntimeError):
    """One or more items in a batch transfer failed."""

    def __init__(self, failures: dict[str, str]) -> None:
        self.failures = failures
        details = "; ".join(f"{name}: {message}" for name, message in failures.items())
        super().__init__(f"{len(failures)} batch item(s) failed: {details}")


class ReplayRepositoryBackend(Protocol):
    """Storage contract shared by filesystem and object-storage backends."""

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
        owner = _validate_name(owner, "owner")
        repository = _validate_name(repository, "repository")
        return self.root / owner / repository

    def _paths(self, owner: str, repository: str, object_id: str) -> tuple[Path, Path]:
        directory = self._repository_dir(owner, repository)
        object_id = _validate_object_id(object_id)
        return directory / f"{object_id}.blob", directory / f"{object_id}.json"

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
        """Store exactly ``size_bytes`` bytes and return immutable metadata."""
        if size_bytes < 0:
            raise RepositoryError("size_bytes cannot be negative")
        if expected_sha256 is not None:
            _validate_object_id(expected_sha256)
        filename = _validate_filename(filename)
        directory = self._repository_dir(owner, repository)
        directory.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256()
        remaining = size_bytes
        temporary_path: Path | None = None
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
                        raise RepositoryError(
                            f"request body ended with {remaining} bytes still expected"
                        )
                    temporary.write(chunk)
                    digest.update(chunk)
                    remaining -= len(chunk)

            object_id = digest.hexdigest()
            if expected_sha256 is not None and object_id != expected_sha256:
                raise RepositoryError(
                    f"SHA-256 mismatch: expected {expected_sha256}, received {object_id}"
                )
            object_path, metadata_path = self._paths(owner, repository, object_id)
            if metadata_path.exists() and object_path.exists():
                return ReplayObject.from_dict(json.loads(metadata_path.read_text(encoding="utf-8")))

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
            metadata_temporary = metadata_path.with_suffix(".json.part")
            metadata_temporary.write_text(
                json.dumps(metadata.to_dict(), sort_keys=True),
                encoding="utf-8",
            )
            os.replace(metadata_temporary, metadata_path)
            return metadata
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def get(self, owner: str, repository: str, object_id: str) -> tuple[ReplayObject, Path]:
        object_path, metadata_path = self._paths(owner, repository, object_id)
        if not object_path.is_file() or not metadata_path.is_file():
            raise FileNotFoundError(object_id)
        metadata = ReplayObject.from_dict(json.loads(metadata_path.read_text(encoding="utf-8")))
        return metadata, object_path

    def open(
        self,
        owner: str,
        repository: str,
        object_id: str,
    ) -> tuple[ReplayObject, ClosableBinaryReader]:
        """Open one immutable object for streaming."""
        metadata, object_path = self.get(owner, repository, object_id)
        return metadata, object_path.open("rb")

    def list(self, owner: str, repository: str) -> list[ReplayObject]:
        directory = self._repository_dir(owner, repository)
        if not directory.is_dir():
            return []
        objects = [
            ReplayObject.from_dict(json.loads(path.read_text(encoding="utf-8")))
            for path in directory.glob("*.json")
        ]
        return sorted(objects, key=lambda item: item.created_at, reverse=True)


class ReplayRepositoryServer(ThreadingHTTPServer):
    """Threaded HTTP server carrying repository state."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        repository: ReplayRepositoryBackend,
        *,
        token: str | None,
        public_read: bool = False,
    ) -> None:
        self.repository = repository
        self.token = token
        self.public_read = public_read
        super().__init__(address, ReplayRepositoryRequestHandler)


class ReplayRepositoryRequestHandler(BaseHTTPRequestHandler):
    """Raw streaming REST API for replay objects."""

    protocol_version = "HTTP/1.1"
    server_version = "DimOSReplayRepository/0.1"

    def _repository_server(self) -> ReplayRepositoryServer:
        return cast("ReplayRepositoryServer", self.server)

    def _authorized(self, *, write: bool) -> bool:
        if not write and self._repository_server().public_read:
            return True
        expected = self._repository_server().token
        if expected is None:
            return True
        supplied = self.headers.get("Authorization", "")
        return hmac.compare_digest(supplied, f"Bearer {expected}")

    def _route(self) -> tuple[str, str, str | None]:
        parts = [unquote(part) for part in urlsplit(self.path).path.split("/") if part]
        if len(parts) not in {6, 7} or parts[:3] != ["api", "v1", "repositories"]:
            raise RepositoryError("unknown repository route")
        if parts[5] != "objects":
            raise RepositoryError("unknown repository route")
        owner = _validate_name(parts[3], "owner")
        repository = _validate_name(parts[4], "repository")
        object_id = _validate_object_id(parts[6]) if len(parts) == 7 else None
        return owner, repository, object_id

    def _send_json(self, status: HTTPStatus, data: dict[str, Any]) -> None:
        payload = json.dumps(data, sort_keys=True).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(payload)
        self.close_connection = True

    def _send_error_json(self, status: HTTPStatus, message: str) -> None:
        self._send_json(status, {"error": message})

    def _send_html(self, status: HTTPStatus, page: str) -> None:
        payload = page.encode()
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(payload)
        self.close_connection = True

    def _check_authorized(self, *, write: bool) -> bool:
        if self._authorized(write=write):
            return True
        self._send_error_json(HTTPStatus.UNAUTHORIZED, "invalid bearer token")
        return False

    def _serve_repository_page(self) -> None:
        parts = [unquote(part) for part in urlsplit(self.path).path.split("/") if part]
        if len(parts) != 3 or parts[0] != "r":
            raise RepositoryError("unknown repository page")
        owner = _validate_name(parts[1], "owner")
        repository_name = _validate_name(parts[2], "repository")
        objects = self._repository_server().repository.list(owner, repository_name)
        cards: list[str] = []
        for item in objects:
            object_url = (
                f"/api/v1/repositories/{quote(owner, safe='')}/"
                f"{quote(repository_name, safe='')}/objects/{item.object_id}"
            )
            preview = ""
            if item.content_type.startswith("video/"):
                preview = f'<video controls preload="metadata" src="{object_url}"></video>'
            cards.append(
                "<article>"
                f"<h2>{escape(item.filename)}</h2>"
                f"{preview}"
                f"<p>{item.size_bytes:,} bytes · SHA-256 {escape(item.sha256)}</p>"
                f'<a href="{object_url}?download=1">Download</a>'
                "</article>"
            )
        body = "".join(cards) or "<p>No objects have been uploaded yet.</p>"
        page = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>{escape(owner)}/{escape(repository_name)} replay repository</title>
  <style>
    body {{ max-width: 900px; margin: 2rem auto; padding: 0 1rem;
            font: 16px system-ui, sans-serif; background: #101114; color: #f5f5f5; }}
    article {{ padding: 1rem; margin: 1rem 0; border: 1px solid #333; border-radius: 12px; }}
    video {{ display: block; width: 100%; max-height: 520px; background: #000; }}
    a {{ color: #70b7ff; }}
    p {{ color: #bbb; overflow-wrap: anywhere; }}
  </style>
</head>
<body>
  <h1>{escape(owner)}/{escape(repository_name)}</h1>
  {body}
</body>
</html>"""
        self._send_html(HTTPStatus.OK, page)

    def do_POST(self) -> None:
        if not self._check_authorized(write=True):
            return
        try:
            owner, repository, object_id = self._route()
            if object_id is not None:
                raise RepositoryError("upload requests must target the objects collection")
            raw_length = self.headers.get("Content-Length")
            if raw_length is None:
                raise RepositoryError("Content-Length is required")
            size_bytes = int(raw_length)
            filename = self.headers.get("X-Dimos-Filename")
            if filename is None:
                raise RepositoryError("X-Dimos-Filename is required")
            metadata = self._repository_server().repository.put_stream(
                owner=owner,
                repository=repository,
                filename=unquote(filename),
                source=self.rfile,
                size_bytes=size_bytes,
                content_type=self.headers.get("Content-Type", "application/octet-stream"),
                expected_sha256=self.headers.get("X-Dimos-Sha256"),
            )
            self._send_json(HTTPStatus.CREATED, metadata.to_dict())
        except (RepositoryError, ValueError) as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
        except OSError as exc:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

    def do_GET(self) -> None:
        if not self._check_authorized(write=False):
            return
        try:
            if urlsplit(self.path).path.startswith("/r/"):
                self._serve_repository_page()
                return
            owner, repository, object_id = self._route()
            if object_id is None:
                objects = self._repository_server().repository.list(owner, repository)
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "owner": owner,
                        "repository": repository,
                        "objects": [o.to_dict() for o in objects],
                    },
                )
                return
            metadata, source = self._repository_server().repository.open(
                owner,
                repository,
                object_id,
            )
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", metadata.content_type)
            self.send_header("Content-Length", str(metadata.size_bytes))
            self.send_header("X-Dimos-Sha256", metadata.sha256)
            self.send_header("X-Dimos-Filename", quote(metadata.filename, safe=""))
            disposition = (
                "attachment"
                if parse_qs(urlsplit(self.path).query).get("download") == ["1"]
                else "inline"
            )
            self.send_header(
                "Content-Disposition",
                f"{disposition}; filename*=UTF-8''{quote(metadata.filename, safe='')}",
            )
            self.send_header("Connection", "close")
            self.end_headers()
            try:
                while chunk := source.read(_CHUNK_SIZE):
                    self.wfile.write(chunk)
            finally:
                source.close()
            self.close_connection = True
        except RepositoryError as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
        except FileNotFoundError:
            self._send_error_json(HTTPStatus.NOT_FOUND, "object not found")
        except OSError as exc:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

    def log_message(self, format: str, *args: Any) -> None:
        return


def serve_repository(
    *,
    root: str | Path,
    host: str,
    port: int,
    token: str | None,
    public_read: bool = False,
    repository: ReplayRepositoryBackend | None = None,
) -> None:
    """Serve until interrupted."""
    with ReplayRepositoryServer(
        (host, port),
        repository or ReplayRepository(root),
        token=token,
        public_read=public_read,
    ) as server:
        server.serve_forever()


def _api_headers(token: str | None) -> dict[str, str]:
    return {"Authorization": f"Bearer {token}"} if token else {}


def _objects_url(server_url: str, owner: str, repository: str) -> str:
    owner = _validate_name(owner, "owner")
    repository = _validate_name(repository, "repository")
    return (
        f"{server_url.rstrip('/')}/api/v1/repositories/"
        f"{quote(owner, safe='')}/{quote(repository, safe='')}/objects"
    )


def _upload_file_once(
    *,
    server_url: str,
    owner: str,
    repository: str,
    path: str | Path,
    token: str | None = None,
) -> ReplayObject:
    """Upload one raw file and return its repository metadata."""
    source_path = Path(path)
    size_bytes = source_path.stat().st_size
    digest = sha256_file(source_path)
    headers = {
        **_api_headers(token),
        "Content-Length": str(size_bytes),
        "Content-Type": mimetypes.guess_type(source_path.name)[0] or "application/octet-stream",
        "X-Dimos-Filename": quote(source_path.name, safe=""),
        "X-Dimos-Sha256": digest,
    }
    with source_path.open("rb") as source:
        response = requests.post(
            _objects_url(server_url, owner, repository),
            data=source,
            headers=headers,
            timeout=(30.0, 3600.0),
        )
    response.raise_for_status()
    data = response.json()
    if not isinstance(data, dict):
        raise RuntimeError("repository returned non-object JSON")
    return ReplayObject.from_dict(data)


def upload_file(
    *,
    server_url: str,
    owner: str,
    repository: str,
    path: str | Path,
    token: str | None = None,
    retries: int = 3,
    backoff_seconds: float = 0.5,
) -> ReplayObject:
    """Upload one raw file with bounded retry and checksum verification."""
    return _with_retries(
        lambda: _upload_file_once(
            server_url=server_url,
            owner=owner,
            repository=repository,
            path=path,
            token=token,
        ),
        retries=retries,
        backoff_seconds=backoff_seconds,
    )


def list_objects(
    *,
    server_url: str,
    owner: str,
    repository: str,
    token: str | None = None,
) -> list[ReplayObject]:
    """List objects in one developer repository."""
    response = requests.get(
        _objects_url(server_url, owner, repository),
        headers=_api_headers(token),
        timeout=30.0,
    )
    response.raise_for_status()
    data = response.json()
    raw_objects = data.get("objects") if isinstance(data, dict) else None
    if not isinstance(raw_objects, list):
        raise RuntimeError("repository returned an invalid object listing")
    return [ReplayObject.from_dict(item) for item in raw_objects if isinstance(item, dict)]


def _download_object_once(
    *,
    server_url: str,
    owner: str,
    repository: str,
    object_id: str,
    output: str | Path | None = None,
    token: str | None = None,
    overwrite: bool = False,
) -> Path:
    """Download one object atomically and verify its SHA-256 digest."""
    object_id = _validate_object_id(object_id)
    url = f"{_objects_url(server_url, owner, repository)}/{object_id}"
    with requests.get(
        url,
        headers=_api_headers(token),
        stream=True,
        timeout=(30.0, 3600.0),
    ) as response:
        response.raise_for_status()
        encoded_filename = response.headers.get("X-Dimos-Filename", object_id)
        destination = Path(output) if output is not None else Path(unquote(encoded_filename))
        if destination.exists() and not overwrite:
            raise FileExistsError(
                f"{destination} already exists; pass overwrite=True to replace it"
            )
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.{object_id[:12]}.part")
        digest = hashlib.sha256()
        try:
            with temporary.open("wb") as target:
                for chunk in response.iter_content(chunk_size=_CHUNK_SIZE):
                    if chunk:
                        target.write(chunk)
                        digest.update(chunk)
            expected = response.headers.get("X-Dimos-Sha256", object_id)
            received = digest.hexdigest()
            if received != expected:
                raise RuntimeError(
                    f"download SHA-256 mismatch: expected {expected}, received {received}"
                )
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)
    return destination


def download_object(
    *,
    server_url: str,
    owner: str,
    repository: str,
    object_id: str,
    output: str | Path | None = None,
    token: str | None = None,
    overwrite: bool = False,
    retries: int = 3,
    backoff_seconds: float = 0.5,
) -> Path:
    """Download one object with atomic writes, retries, and SHA verification."""
    return _with_retries(
        lambda: _download_object_once(
            server_url=server_url,
            owner=owner,
            repository=repository,
            object_id=object_id,
            output=output,
            token=token,
            overwrite=overwrite,
        ),
        retries=retries,
        backoff_seconds=backoff_seconds,
    )


def upload_files(
    *,
    server_url: str,
    owner: str,
    repository: str,
    paths: Iterable[str | Path],
    token: str | None = None,
    workers: int = 4,
    retries: int = 3,
    backoff_seconds: float = 0.5,
) -> ReplayManifest:
    """Upload many files concurrently and return a deterministic manifest."""
    source_paths = tuple(Path(path) for path in paths)
    if workers < 1:
        raise RepositoryError("workers must be at least 1")
    if not source_paths:
        raise RepositoryError("at least one source file is required")
    if any(not path.is_file() for path in source_paths):
        missing = [str(path) for path in source_paths if not path.is_file()]
        raise FileNotFoundError(", ".join(missing))

    results: list[ReplayObject | None] = [None] * len(source_paths)
    failures: dict[str, str] = {}
    with ThreadPoolExecutor(max_workers=min(workers, len(source_paths))) as executor:
        future_to_index = {
            executor.submit(
                upload_file,
                server_url=server_url,
                owner=owner,
                repository=repository,
                path=path,
                token=token,
                retries=retries,
                backoff_seconds=backoff_seconds,
            ): index
            for index, path in enumerate(source_paths)
        }
        for future in as_completed(future_to_index):
            index = future_to_index[future]
            try:
                results[index] = future.result()
            except Exception as exc:
                failures[str(source_paths[index])] = str(exc)
    if failures:
        raise BatchTransferError(failures)
    return ReplayManifest(
        owner=_validate_name(owner, "owner"),
        repository=_validate_name(repository, "repository"),
        objects=tuple(item for item in results if item is not None),
    )


def download_objects(
    *,
    server_url: str,
    manifest: ReplayManifest,
    output_dir: str | Path,
    token: str | None = None,
    workers: int = 4,
    retries: int = 3,
    backoff_seconds: float = 0.5,
    overwrite: bool = False,
) -> tuple[Path, ...]:
    """Download every object in a manifest concurrently and atomically."""
    if workers < 1:
        raise RepositoryError("workers must be at least 1")
    destination_dir = Path(output_dir)
    destination_dir.mkdir(parents=True, exist_ok=True)
    destinations: list[Path] = []
    used_names: set[str] = set()
    for item in manifest.objects:
        filename = item.filename
        if filename in used_names:
            filename = f"{item.object_id[:12]}-{filename}"
        used_names.add(filename)
        destinations.append(destination_dir / filename)

    results: list[Path | None] = [None] * len(manifest.objects)
    failures: dict[str, str] = {}
    with ThreadPoolExecutor(max_workers=min(workers, max(1, len(manifest.objects)))) as executor:
        future_to_index = {
            executor.submit(
                download_object,
                server_url=server_url,
                owner=manifest.owner,
                repository=manifest.repository,
                object_id=item.object_id,
                output=destinations[index],
                token=token,
                overwrite=overwrite,
                retries=retries,
                backoff_seconds=backoff_seconds,
            ): index
            for index, item in enumerate(manifest.objects)
        }
        for future in as_completed(future_to_index):
            index = future_to_index[future]
            try:
                results[index] = future.result()
            except Exception as exc:
                failures[manifest.objects[index].object_id] = str(exc)
    if failures:
        raise BatchTransferError(failures)
    return tuple(item for item in results if item is not None)


def write_manifest(path: str | Path, manifest: ReplayManifest) -> Path:
    """Write a manifest atomically so a partial batch is never advertised."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f".{destination.name}.",
            suffix=".part",
            dir=destination.parent,
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(manifest.to_dict(), temporary, indent=2, sort_keys=True)
            temporary.write("\n")
        os.replace(temporary_path, destination)
        temporary_path = None
        return destination
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def read_manifest(path: str | Path) -> ReplayManifest:
    """Read and validate a previously completed batch manifest."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RepositoryError("manifest must contain a JSON object")
    return ReplayManifest.from_dict(data)


def iter_repository_paths(
    repository: ReplayRepository,
    owner: str,
    name: str,
) -> Iterator[Path]:
    """Yield stored blob paths for diagnostics and tests."""
    for item in repository.list(owner, name):
        yield repository.get(owner, name, item.object_id)[1]
