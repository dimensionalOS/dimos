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

"""End-to-end tests for the hosted replay HTTP API."""

from __future__ import annotations

import hashlib
from pathlib import Path

from fastapi.testclient import TestClient

from dimos.hosted_data.api import create_app
from dimos.hosted_data.repository import ReplayRepository


def _client(tmp_path: Path) -> TestClient:
    return TestClient(create_app(ReplayRepository(tmp_path / "objects")))


def _upload(client: TestClient, payload: bytes, *, owner: str = "alice"):
    return client.put(
        f"/v1/repositories/{owner}/go2/objects",
        params={"filename": "capture.mp4"},
        content=payload,
        headers={"Content-Type": "video/mp4", "Content-Length": str(len(payload))},
    )


def test_upload_list_head_and_download(tmp_path: Path) -> None:
    payload = b"\x00\x00\x00\x18ftypmp42" + b"video-frame" * 100_000
    digest = hashlib.sha256(payload).hexdigest()

    with _client(tmp_path) as client:
        upload = client.put(
            "/v1/repositories/alice/go2/objects",
            params={"filename": "capture.mp4"},
            content=payload,
            headers={
                "Content-Type": "video/mp4",
                "Content-Length": str(len(payload)),
                "X-Content-SHA256": digest,
            },
        )
        assert upload.status_code == 201
        assert upload.json()["object_id"] == digest

        listing = client.get("/v1/repositories/alice/go2/objects")
        assert listing.status_code == 200
        assert listing.json() == [upload.json()]

        head = client.head(f"/v1/repositories/alice/go2/objects/{digest}")
        assert head.status_code == 200
        assert head.headers["content-length"] == str(len(payload))
        assert head.headers["accept-ranges"] == "bytes"
        assert head.headers["content-type"] == "video/mp4"

        download = client.get(f"/v1/repositories/alice/go2/objects/{digest}")
        assert download.status_code == 200
        assert download.content == payload
        assert download.headers["etag"] == f'"{digest}"'


def test_download_supports_browser_byte_ranges(tmp_path: Path) -> None:
    payload = bytes(range(100))

    with _client(tmp_path) as client:
        uploaded = _upload(client, payload).json()
        url = f"/v1/repositories/alice/go2/objects/{uploaded['object_id']}"

        middle = client.get(url, headers={"Range": "bytes=10-19"})
        assert middle.status_code == 206
        assert middle.content == payload[10:20]
        assert middle.headers["content-range"] == "bytes 10-19/100"
        assert middle.headers["content-length"] == "10"

        suffix = client.get(url, headers={"Range": "bytes=-5"})
        assert suffix.status_code == 206
        assert suffix.content == payload[-5:]
        assert suffix.headers["content-range"] == "bytes 95-99/100"


def test_invalid_range_and_missing_object_return_http_errors(tmp_path: Path) -> None:
    digest = "0" * 64

    with _client(tmp_path) as client:
        missing = client.get(f"/v1/repositories/alice/go2/objects/{digest}")
        assert missing.status_code == 404

        uploaded = _upload(client, b"small-video").json()
        invalid = client.get(
            f"/v1/repositories/alice/go2/objects/{uploaded['object_id']}",
            headers={"Range": "bytes=999-1000"},
        )
        assert invalid.status_code == 416
        assert invalid.headers["content-range"] == "bytes */11"


def test_upload_rejects_digest_mismatch_and_unsafe_names(tmp_path: Path) -> None:
    with _client(tmp_path) as client:
        mismatch = client.put(
            "/v1/repositories/alice/go2/objects",
            params={"filename": "clip.mp4"},
            content=b"video",
            headers={"Content-Length": "5", "X-Content-SHA256": "0" * 64},
        )
        assert mismatch.status_code == 400

        unsafe = _upload(client, b"video", owner="bad!owner")
        assert unsafe.status_code == 400


def test_upload_requires_content_length(tmp_path: Path) -> None:
    with _client(tmp_path) as client:
        response = client.put(
            "/v1/repositories/alice/go2/objects",
            params={"filename": "clip.mp4"},
            content=b"",
            headers={"Content-Length": ""},
        )
        assert response.status_code in {411, 422}


def test_health(tmp_path: Path) -> None:
    with _client(tmp_path) as client:
        assert client.get("/healthz").json() == {"status": "ok"}
