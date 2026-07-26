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

"""Tests for S3-compatible hosted-data storage."""

from __future__ import annotations

import hashlib
from io import BytesIO
from pathlib import Path
import threading
from typing import Any, BinaryIO

import pytest

from dimos.hosted_data.repository import (
    ReplayRepositoryServer,
    RepositoryError,
    download_object,
    list_objects,
    upload_file,
)
from dimos.hosted_data.storage.s3 import S3ReplayRepository


class _MissingKeyError(Exception):
    response = {"Error": {"Code": "NoSuchKey"}}


class _FakeS3Client:
    def __init__(self, *, page_size: int = 1000) -> None:
        self.objects: dict[tuple[str, str], bytes] = {}
        self.page_size = page_size
        self.uploads = 0

    def get_object(self, **kwargs: Any) -> dict[str, Any]:
        key = (str(kwargs["Bucket"]), str(kwargs["Key"]))
        try:
            payload = self.objects[key]
        except KeyError as exc:
            raise _MissingKeyError from exc
        return {"Body": BytesIO(payload), "ContentLength": len(payload)}

    def put_object(self, **kwargs: Any) -> dict[str, Any]:
        body = kwargs["Body"]
        assert isinstance(body, bytes)
        self.objects[(str(kwargs["Bucket"]), str(kwargs["Key"]))] = body
        return {}

    def upload_fileobj(
        self,
        Fileobj: BinaryIO,
        Bucket: str,
        Key: str,
        ExtraArgs: dict[str, str] | None = None,
    ) -> None:
        self.uploads += 1
        self.objects[(Bucket, Key)] = Fileobj.read()

    def list_objects_v2(self, **kwargs: Any) -> dict[str, Any]:
        bucket = str(kwargs["Bucket"])
        prefix = str(kwargs["Prefix"])
        keys = sorted(
            key
            for stored_bucket, key in self.objects
            if stored_bucket == bucket and key.startswith(prefix)
        )
        offset = int(kwargs.get("ContinuationToken", 0))
        page = keys[offset : offset + self.page_size]
        next_offset = offset + len(page)
        truncated = next_offset < len(keys)
        response: dict[str, Any] = {
            "Contents": [{"Key": key} for key in page],
            "IsTruncated": truncated,
        }
        if truncated:
            response["NextContinuationToken"] = str(next_offset)
        return response


def test_s3_repository_upload_list_and_stream_download() -> None:
    client = _FakeS3Client(page_size=1)
    repository = S3ReplayRepository(
        bucket="replays",
        prefix="tenant-a",
        client=client,
    )
    payload = b"raw-h264-data" * 100
    digest = hashlib.sha256(payload).hexdigest()

    uploaded = repository.put_stream(
        owner="alice",
        repository="go2",
        filename="capture.h264",
        source=BytesIO(payload),
        size_bytes=len(payload),
        content_type="video/h264",
        expected_sha256=digest,
    )
    listed = repository.list("alice", "go2")
    metadata, body = repository.open("alice", "go2", uploaded.object_id)
    try:
        downloaded = body.read()
    finally:
        body.close()

    assert uploaded.object_id == digest
    assert listed == [uploaded]
    assert metadata == uploaded
    assert downloaded == payload


def test_s3_repository_deduplicates_by_digest() -> None:
    client = _FakeS3Client()
    repository = S3ReplayRepository(bucket="replays", client=client)
    payload = b"same-content"

    first = repository.put_stream(
        owner="alice",
        repository="go2",
        filename="first.mp4",
        source=BytesIO(payload),
        size_bytes=len(payload),
        content_type="video/mp4",
    )
    second = repository.put_stream(
        owner="alice",
        repository="go2",
        filename="second.mp4",
        source=BytesIO(payload),
        size_bytes=len(payload),
        content_type="video/mp4",
    )

    assert second == first
    assert client.uploads == 1


def test_s3_repository_rejects_checksum_mismatch() -> None:
    client = _FakeS3Client()
    repository = S3ReplayRepository(bucket="replays", client=client)

    with pytest.raises(RepositoryError, match="SHA-256 mismatch"):
        repository.put_stream(
            owner="alice",
            repository="go2",
            filename="capture.mp4",
            source=BytesIO(b"payload"),
            size_bytes=7,
            content_type="video/mp4",
            expected_sha256="0" * 64,
        )

    assert client.objects == {}


def test_s3_repository_rejects_invalid_configuration() -> None:
    client = _FakeS3Client()

    with pytest.raises(RepositoryError, match="bucket cannot be empty"):
        S3ReplayRepository(bucket="", client=client)
    with pytest.raises(RepositoryError, match="addressing style"):
        S3ReplayRepository(
            bucket="replays",
            client=client,
            addressing_style="invalid",
        )


def test_s3_repository_serves_the_standard_http_contract(tmp_path: Path) -> None:
    client = _FakeS3Client()
    repository = S3ReplayRepository(bucket="replays", client=client)
    server = ReplayRepositoryServer(
        ("127.0.0.1", 0),
        repository,
        token="test-token",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    source = tmp_path / "capture.mp4"
    source.write_bytes(b"video-through-s3")
    output = tmp_path / "restored.mp4"
    try:
        host, port = server.server_address[:2]
        host_text = host.decode() if isinstance(host, bytes) else host
        server_url = f"http://{host_text}:{port}"
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=source,
            token="test-token",
        )
        listed = list_objects(
            server_url=server_url,
            owner="alice",
            repository="go2",
            token="test-token",
        )
        restored = download_object(
            server_url=server_url,
            owner="alice",
            repository="go2",
            object_id=uploaded.object_id,
            output=output,
            token="test-token",
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5.0)

    assert listed == [uploaded]
    assert restored.read_bytes() == source.read_bytes()
