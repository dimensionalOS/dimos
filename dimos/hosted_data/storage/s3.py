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

"""S3-compatible hosted-data storage for AWS S3, Cloudflare R2, and MinIO."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import tempfile
from typing import Any, BinaryIO, Protocol, cast

from dimos.hosted_data.repository import (
    BinaryReader,
    ClosableBinaryReader,
    ReplayObject,
    RepositoryError,
    _validate_filename,
    _validate_name,
    _validate_object_id,
)

_CHUNK_SIZE = 1024 * 1024
_SPOOL_MEMORY_LIMIT = 64 * 1024 * 1024
_NOT_FOUND_CODES = {"404", "NoSuchKey", "NotFound"}


class S3Client(Protocol):
    """Subset of the boto3 S3 client used by the repository."""

    def get_object(self, **kwargs: Any) -> dict[str, Any]: ...

    def put_object(self, **kwargs: Any) -> dict[str, Any]: ...`n`n    def delete_object(self, **kwargs: Any) -> dict[str, Any]: ...

    def upload_fileobj(
        self,
        Fileobj: BinaryIO,
        Bucket: str,
        Key: str,
        ExtraArgs: dict[str, str] | None = None,
    ) -> None: ...

    def list_objects_v2(self, **kwargs: Any) -> dict[str, Any]: ...


def create_s3_client(
    *,
    endpoint_url: str | None = None,
    region_name: str | None = None,
    access_key_id: str | None = None,
    secret_access_key: str | None = None,
    session_token: str | None = None,
    addressing_style: str = "auto",
) -> S3Client:
    """Create a boto3 S3 client without making boto3 a core dependency."""
    try:
        import boto3  # type: ignore[import-not-found,import-untyped]
        from botocore.config import Config  # type: ignore[import-not-found,import-untyped]
    except ImportError as exc:
        raise RuntimeError(
            "S3 replay storage requires `pip install 'dimos[cloud-storage]'`"
        ) from exc
    config = Config(s3={"addressing_style": addressing_style})
    return cast(
        "S3Client",
        boto3.client(
            "s3",
            endpoint_url=endpoint_url,
            region_name=region_name,
            aws_access_key_id=access_key_id,
            aws_secret_access_key=secret_access_key,
            aws_session_token=session_token,
            config=config,
        ),
    )


def _is_not_found(exc: Exception) -> bool:
    response = getattr(exc, "response", None)
    if not isinstance(response, dict):
        return False
    error = response.get("Error")
    return isinstance(error, dict) and str(error.get("Code")) in _NOT_FOUND_CODES


def _read_response_body(response: dict[str, Any]) -> bytes:
    body = response.get("Body")
    if body is None or not hasattr(body, "read"):
        raise RuntimeError("S3 response did not contain a readable Body")
    try:
        payload = body.read()
    finally:
        close = getattr(body, "close", None)
        if callable(close):
            close()
    if not isinstance(payload, bytes):
        raise RuntimeError("S3 response Body did not return bytes")
    return payload


class S3ReplayRepository:
    """Immutable replay objects stored through an S3-compatible API."""

    def __init__(
        self,
        *,
        bucket: str,
        prefix: str = "dimos-replays",
        client: S3Client | None = None,
        endpoint_url: str | None = None,
        region_name: str | None = None,
        access_key_id: str | None = None,
        secret_access_key: str | None = None,
        session_token: str | None = None,
        addressing_style: str = "auto",
    ) -> None:
        if not bucket:
            raise RepositoryError("S3 bucket cannot be empty")
        if addressing_style not in {"auto", "path", "virtual"}:
            raise RepositoryError("S3 addressing style must be auto, path, or virtual")
        self.bucket = bucket
        self.prefix = prefix.strip("/")
        self.client = client or create_s3_client(
            endpoint_url=endpoint_url,
            region_name=region_name,
            access_key_id=access_key_id,
            secret_access_key=secret_access_key,
            session_token=session_token,
            addressing_style=addressing_style,
        )

    def _base_key(self, owner: str, repository: str) -> str:
        owner = _validate_name(owner, "owner")
        repository = _validate_name(repository, "repository")
        relative = f"{owner}/{repository}/objects"
        return f"{self.prefix}/{relative}" if self.prefix else relative

    def _keys(
        self,
        owner: str,
        repository: str,
        object_id: str,
    ) -> tuple[str, str]:
        object_id = _validate_object_id(object_id)
        base = self._base_key(owner, repository)
        return f"{base}/{object_id}.blob", f"{base}/{object_id}.json"

    def _read_metadata(
        self,
        owner: str,
        repository: str,
        object_id: str,
    ) -> ReplayObject:
        _, metadata_key = self._keys(owner, repository, object_id)
        try:
            response = self.client.get_object(Bucket=self.bucket, Key=metadata_key)
        except Exception as exc:
            if _is_not_found(exc):
                raise FileNotFoundError(object_id) from exc
            raise
        payload = _read_response_body(response)
        data = json.loads(payload)
        if not isinstance(data, dict):
            raise RepositoryError("S3 replay metadata must be a JSON object")
        metadata = ReplayObject.from_dict(data)
        if (
            metadata.owner != owner
            or metadata.repository != repository
            or metadata.object_id != object_id
        ):
            raise RepositoryError("S3 replay metadata does not match its object key")
        return metadata

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
        """Hash one request body, then atomically advertise it through metadata."""
        if size_bytes < 0:
            raise RepositoryError("size_bytes cannot be negative")
        owner = _validate_name(owner, "owner")
        repository = _validate_name(repository, "repository")
        filename = _validate_filename(filename)
        if expected_sha256 is not None:
            _validate_object_id(expected_sha256)
        digest = hashlib.sha256()
        remaining = size_bytes
        with tempfile.SpooledTemporaryFile(
            max_size=_SPOOL_MEMORY_LIMIT,
            mode="w+b",
        ) as temporary:
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
            try:
                return self._read_metadata(owner, repository, object_id)
            except FileNotFoundError:
                pass

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
            object_key, metadata_key = self._keys(owner, repository, object_id)
            temporary.seek(0)
            self.client.upload_fileobj(
                cast("BinaryIO", temporary),
                self.bucket,
                object_key,
                ExtraArgs={"ContentType": metadata.content_type},
            )
            self.client.put_object(
                Bucket=self.bucket,
                Key=metadata_key,
                Body=json.dumps(metadata.to_dict(), sort_keys=True).encode(),
                ContentType="application/json",
            )
            return metadata

    def open(
        self,
        owner: str,
        repository: str,
        object_id: str,
    ) -> tuple[ReplayObject, ClosableBinaryReader]:
        """Open an S3 object body without downloading it to local disk."""
        metadata = self._read_metadata(owner, repository, object_id)
        object_key, _ = self._keys(owner, repository, object_id)
        try:
            response = self.client.get_object(Bucket=self.bucket, Key=object_key)
        except Exception as exc:
            if _is_not_found(exc):
                raise FileNotFoundError(object_id) from exc
            raise
        body = response.get("Body")
        if body is None or not hasattr(body, "read") or not hasattr(body, "close"):
            raise RuntimeError("S3 object response did not contain a closable Body")
        return metadata, cast("ClosableBinaryReader", body)

    def delete(self, owner: str, repository: str, object_id: str) -> ReplayObject:
        """Delete metadata first so partially deleted blobs stay invisible."""
        metadata = self._read_metadata(owner, repository, object_id)
        object_key, metadata_key = self._keys(owner, repository, object_id)
        self.client.delete_object(Bucket=self.bucket, Key=metadata_key)
        self.client.delete_object(Bucket=self.bucket, Key=object_key)
        return metadata

    def list(self, owner: str, repository: str) -> list[ReplayObject]:
        """List completed objects; orphan blobs are intentionally invisible."""
        base = self._base_key(owner, repository)
        prefix = f"{base}/"
        continuation_token: str | None = None
        metadata: list[ReplayObject] = []
        while True:
            request: dict[str, Any] = {
                "Bucket": self.bucket,
                "Prefix": prefix,
            }
            if continuation_token is not None:
                request["ContinuationToken"] = continuation_token
            response = self.client.list_objects_v2(**request)
            contents = response.get("Contents", [])
            if not isinstance(contents, list):
                raise RuntimeError("S3 list response Contents must be a list")
            for item in contents:
                if not isinstance(item, dict):
                    continue
                key = str(item.get("Key", ""))
                if not key.endswith(".json"):
                    continue
                object_id = key.removesuffix(".json").rsplit("/", 1)[-1]
                metadata.append(self._read_metadata(owner, repository, object_id))
            if not response.get("IsTruncated"):
                break
            continuation_token = str(response.get("NextContinuationToken", ""))
            if not continuation_token:
                raise RuntimeError("truncated S3 list response omitted continuation token")
        return sorted(metadata, key=lambda item: item.created_at, reverse=True)
