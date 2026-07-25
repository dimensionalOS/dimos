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

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
import threading

import pytest
import requests

from dimos.protocol.pubsub.impl.webrtc.replay_repository import (
    RepositoryError,
    ReplayManifest,
    ReplayObject,
    ReplayRepository,
    ReplayRepositoryServer,
    download_object,
    download_objects,
    list_objects,
    read_manifest,
    sha256_file,
    upload_file,
    upload_files,
    write_manifest,
)


@contextmanager
def _running_server(
    root: Path,
    *,
    token: str | None = "test-token",
    public_read: bool = False,
) -> Iterator[str]:
    server = ReplayRepositoryServer(
        ("127.0.0.1", 0),
        ReplayRepository(root),
        token=token,
        public_read=public_read,
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


def test_raw_video_upload_list_download_round_trip(tmp_path: Path) -> None:
    source = tmp_path / "source.mp4"
    source.write_bytes(b"\x00\x00\x00\x18ftypmp42" + bytes(range(256)) * 8192)
    download = tmp_path / "downloads" / "copy.mp4"

    with _running_server(tmp_path / "objects") as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2-office",
            path=source,
            token="test-token",
        )
        listed = list_objects(
            server_url=server_url,
            owner="alice",
            repository="go2-office",
            token="test-token",
        )
        result = download_object(
            server_url=server_url,
            owner="alice",
            repository="go2-office",
            object_id=uploaded.object_id,
            output=download,
            token="test-token",
        )

    assert uploaded.filename == "source.mp4"
    assert uploaded.size_bytes == source.stat().st_size
    assert uploaded.sha256 == sha256_file(source)
    assert listed == [uploaded]
    assert result == download
    assert download.read_bytes() == source.read_bytes()
    assert sha256_file(download) == uploaded.sha256


def test_repository_is_scoped_by_owner_and_name(tmp_path: Path) -> None:
    source = tmp_path / "sample.h264"
    source.write_bytes(b"h264-test-data")

    with _running_server(tmp_path / "objects") as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=source,
            token="test-token",
        )
        assert list_objects(
            server_url=server_url,
            owner="alice",
            repository="go2",
            token="test-token",
        ) == [uploaded]
        assert (
            list_objects(
                server_url=server_url,
                owner="bob",
                repository="go2",
                token="test-token",
            )
            == []
        )
        assert (
            list_objects(
                server_url=server_url,
                owner="alice",
                repository="another",
                token="test-token",
            )
            == []
        )


def test_repository_rejects_invalid_token(tmp_path: Path) -> None:
    with _running_server(tmp_path / "objects") as server_url:
        with pytest.raises(requests.HTTPError, match="401"):
            list_objects(
                server_url=server_url,
                owner="alice",
                repository="go2",
                token="wrong-token",
            )


def test_repository_rejects_upload_checksum_mismatch(tmp_path: Path) -> None:
    payload = b"not-the-claimed-object"
    headers = {
        "Authorization": "Bearer test-token",
        "Content-Length": str(len(payload)),
        "Content-Type": "video/mp4",
        "X-Dimos-Filename": "sample.mp4",
        "X-Dimos-Sha256": "0" * 64,
    }
    with _running_server(tmp_path / "objects") as server_url:
        response = requests.post(
            f"{server_url}/api/v1/repositories/alice/go2/objects",
            data=payload,
            headers=headers,
            timeout=5.0,
        )
        assert response.status_code == 400
        assert "SHA-256 mismatch" in response.json()["error"]
        assert (
            list_objects(
                server_url=server_url,
                owner="alice",
                repository="go2",
                token="test-token",
            )
            == []
        )


def test_public_repository_page_plays_and_downloads_video(tmp_path: Path) -> None:
    source = tmp_path / "share.mp4"
    source.write_bytes(b"\x00\x00\x00\x18ftypmp42" + b"video-payload")

    with _running_server(tmp_path / "objects", public_read=True) as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=source,
            token="test-token",
        )
        page = requests.get(f"{server_url}/r/alice/go2", timeout=5.0)
        object_url = f"{server_url}/api/v1/repositories/alice/go2/objects/{uploaded.object_id}"
        video = requests.get(object_url, timeout=5.0)
        download = requests.get(f"{object_url}?download=1", timeout=5.0)

    assert page.status_code == 200
    assert "<video controls" in page.text
    assert "share.mp4" in page.text
    assert video.content == source.read_bytes()
    assert video.headers["Content-Disposition"].startswith("inline;")
    assert download.content == source.read_bytes()
    assert download.headers["Content-Disposition"].startswith("attachment;")


def test_batch_upload_manifest_and_download(tmp_path: Path) -> None:
    source_dir = tmp_path / "capture"
    nested_dir = source_dir / "nested"
    nested_dir.mkdir(parents=True)
    sources = {
        source_dir / "frame-01.mp4": b"video-one",
        source_dir / "frame-02.h264": b"video-two",
        nested_dir / "telemetry.json": b'{"ts": 1}',
    }
    for path, payload in sources.items():
        path.write_bytes(payload)
    manifest_path = tmp_path / "manifest.json"
    output_dir = tmp_path / "restored"

    with _running_server(tmp_path / "objects") as server_url:
        manifest = upload_files(
            server_url=server_url,
            owner="alice",
            repository="batch",
            paths=sorted(sources),
            token="test-token",
            workers=2,
            retries=1,
        )
        write_manifest(manifest_path, manifest)
        restored_manifest = read_manifest(manifest_path)
        restored = download_objects(
            server_url=server_url,
            manifest=restored_manifest,
            output_dir=output_dir,
            token="test-token",
            workers=2,
            retries=1,
        )

    assert len(manifest.objects) == 3
    assert [path.name for path in restored] == [
        "frame-01.mp4",
        "frame-02.h264",
        "telemetry.json",
    ]
    assert {path.name: path.read_bytes() for path in restored} == {
        path.name: payload for path, payload in sources.items()
    }
    assert all(
        sha256_file(path) == item.sha256
        for path, item in zip(restored, manifest.objects, strict=True)
    )


def test_manifest_rejects_unknown_version_and_invalid_entries() -> None:
    valid_object = ReplayObject(
        owner="alice",
        repository="go2",
        object_id="a" * 64,
        filename="capture.db",
        size_bytes=10,
        sha256="a" * 64,
        content_type="application/octet-stream",
        created_at="2026-07-25T00:00:00+00:00",
    ).to_dict()

    with pytest.raises(RepositoryError, match="version must be 1"):
        ReplayManifest.from_dict(
            {
                "version": 2,
                "owner": "alice",
                "repository": "go2",
                "objects": [],
            }
        )
    with pytest.raises(RepositoryError, match="every manifest object"):
        ReplayManifest.from_dict(
            {
                "version": 1,
                "owner": "alice",
                "repository": "go2",
                "objects": [valid_object, "invalid"],
            }
        )


def test_manifest_rejects_cross_repository_objects() -> None:
    item = ReplayObject(
        owner="mallory",
        repository="other",
        object_id="b" * 64,
        filename="capture.db",
        size_bytes=10,
        sha256="b" * 64,
        content_type="application/octet-stream",
        created_at="2026-07-25T00:00:00+00:00",
    )

    with pytest.raises(RepositoryError, match="must belong"):
        ReplayManifest.from_dict(
            {
                "version": 1,
                "owner": "alice",
                "repository": "go2",
                "objects": [item.to_dict()],
            }
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("object_id", "not-a-digest", "invalid replay object metadata"),
        ("sha256", "c" * 64, "object sha256 must match"),
        ("filename", "../capture.db", "invalid replay object metadata"),
        ("size_bytes", -1, "size_bytes cannot be negative"),
        ("content_type", "", "content_type cannot be empty"),
        ("created_at", "", "created_at cannot be empty"),
    ],
)
def test_replay_object_metadata_validation(
    field: str,
    value: object,
    message: str,
) -> None:
    data = ReplayObject(
        owner="alice",
        repository="go2",
        object_id="d" * 64,
        filename="capture.db",
        size_bytes=10,
        sha256="d" * 64,
        content_type="application/octet-stream",
        created_at="2026-07-25T00:00:00+00:00",
    ).to_dict()
    data[field] = value

    with pytest.raises(RepositoryError, match=message):
        ReplayObject.from_dict(data)
