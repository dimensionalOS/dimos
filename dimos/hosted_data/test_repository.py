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

"""Tests for the hosted-data repository."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import hashlib
from http import HTTPStatus
from io import BytesIO
from pathlib import Path
import threading
from typing import TYPE_CHECKING

import pytest
import requests

from dimos.hosted_data import repository as repository_module
from dimos.hosted_data.repository import (
    BatchTransferError,
    ReplayManifest,
    ReplayObject,
    ReplayRepository,
    ReplayRepositoryServer,
    RepositoryError,
    download_object,
    download_objects,
    iter_repository_paths,
    list_objects,
    read_manifest,
    sha256_file,
    upload_file,
    upload_files,
    write_manifest,
)

if TYPE_CHECKING:
    from typing import Any


@contextmanager
def _running_server(
    root: Path,
    *,
    token: str | None = "test-token",
    public_read: bool = False,
    **server_options: Any,
) -> Iterator[str]:
    server = ReplayRepositoryServer(
        ("127.0.0.1", 0),
        ReplayRepository(root),
        token=token,
        public_read=public_read,
        **server_options,
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


def test_public_read_never_allows_anonymous_upload(tmp_path: Path) -> None:
    payload = b""
    digest = hashlib.sha256(payload).hexdigest()
    headers = {
        "Content-Length": str(len(payload)),
        "Content-Type": "video/mp4",
        "X-Dimos-Filename": "anonymous.mp4",
        "X-Dimos-Sha256": digest,
    }

    with _running_server(tmp_path / "objects", public_read=True) as server_url:
        response = requests.post(
            f"{server_url}/api/v1/repositories/alice/go2/objects",
            data=payload,
            headers=headers,
            timeout=5.0,
        )
        objects = list_objects(
            server_url=server_url,
            owner="alice",
            repository="go2",
        )

    assert response.status_code == 401
    assert objects == []


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
    assert page.headers["X-Content-Type-Options"] == "nosniff"
    assert "default-src 'none'" in page.headers["Content-Security-Policy"]
    assert video.content == source.read_bytes()
    assert video.headers["X-Content-Type-Options"] == "nosniff"
    assert video.headers["Content-Disposition"].startswith("inline;")
    assert download.content == source.read_bytes()
    assert download.headers["Content-Disposition"].startswith("attachment;")


def test_repository_enforces_object_and_repository_quotas(tmp_path: Path) -> None:
    too_large = tmp_path / "too-large.mp4"
    too_large.write_bytes(b"123456")
    with _running_server(
        tmp_path / "object-limit",
        max_object_bytes=5,
    ) as server_url:
        with pytest.raises(requests.HTTPError, match="413"):
            upload_file(
                server_url=server_url,
                owner="alice",
                repository="go2",
                path=too_large,
                token="test-token",
                retries=0,
            )

    first = tmp_path / "first.mp4"
    second = tmp_path / "second.mp4"
    first.write_bytes(b"first!")
    second.write_bytes(b"second")
    with _running_server(
        tmp_path / "repository-limit",
        max_repository_bytes=10,
    ) as server_url:
        upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=first,
            token="test-token",
        )
        with pytest.raises(requests.HTTPError, match="507"):
            upload_file(
                server_url=server_url,
                owner="alice",
                repository="go2",
                path=second,
                token="test-token",
                retries=0,
            )


def test_metrics_count_completed_transfer_bytes(tmp_path: Path) -> None:
    source = tmp_path / "metrics.mp4"
    source.write_bytes(b"metrics-video")
    download = tmp_path / "metrics-copy.mp4"

    with _running_server(tmp_path / "objects") as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=source,
            token="test-token",
        )
        download_object(
            server_url=server_url,
            owner="alice",
            repository="go2",
            object_id=uploaded.object_id,
            output=download,
            token="test-token",
        )
        metrics = requests.get(
            f"{server_url}/metrics",
            headers={"Authorization": "Bearer test-token"},
            timeout=5.0,
        )

    assert metrics.status_code == 200
    assert "dimos_replay_uploads_total 1" in metrics.text
    assert "dimos_replay_downloads_total 1" in metrics.text
    assert f"dimos_replay_uploaded_bytes_total {source.stat().st_size}" in metrics.text
    assert f"dimos_replay_downloaded_bytes_total {source.stat().st_size}" in metrics.text


def test_cdn_base_url_is_used_by_repository_page(tmp_path: Path) -> None:
    source = tmp_path / "cdn.mp4"
    source.write_bytes(b"cdn-video")

    with _running_server(
        tmp_path / "objects",
        public_read=True,
        cdn_base_url="https://cdn.example",
    ) as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=source,
            token="test-token",
        )
        page = requests.get(f"{server_url}/r/alice/go2", timeout=5.0)

    assert page.status_code == 200
    assert (
        f"https://cdn.example/api/v1/repositories/alice/go2/objects/{uploaded.object_id}"
        in page.text
    )
    assert (
        page.headers["Content-Security-Policy"] == "default-src 'none'; style-src 'unsafe-inline'; "
        "media-src 'self' https://cdn.example"
    )


def test_server_recovers_interrupted_upload_parts(tmp_path: Path) -> None:
    root = tmp_path / "objects"
    part = root / "alice" / "go2" / ".upload-interrupted.part"
    part.parent.mkdir(parents=True)
    part.write_bytes(b"incomplete")

    with _running_server(root):
        assert not part.exists()


def test_non_video_objects_are_always_downloaded(tmp_path: Path) -> None:
    source = tmp_path / "untrusted.html"
    source.write_text("<script>alert('unsafe')</script>", encoding="utf-8")

    with _running_server(tmp_path / "objects", public_read=True) as server_url:
        uploaded = upload_file(
            server_url=server_url,
            owner="alice",
            repository="go2",
            path=source,
            token="test-token",
        )
        response = requests.get(
            f"{server_url}/api/v1/repositories/alice/go2/objects/{uploaded.object_id}",
            timeout=5.0,
        )

    assert response.status_code == 200
    assert response.headers["Content-Type"] == "text/html"
    assert response.headers["X-Content-Type-Options"] == "nosniff"
    assert response.headers["Content-Disposition"].startswith("attachment;")


def test_upload_rejects_mismatched_response_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "capture.db"
    source.write_bytes(b"memory2-data")
    digest = sha256_file(source)

    class Response:
        def raise_for_status(self) -> None:
            return None

        def json(self) -> dict[str, Any]:
            return ReplayObject(
                owner="mallory",
                repository="go2",
                object_id=digest,
                filename=source.name,
                size_bytes=source.stat().st_size,
                sha256=digest,
                content_type="application/octet-stream",
                created_at="2026-07-25T00:00:00+00:00",
            ).to_dict()

        status_code = HTTPStatus.NO_CONTENT

    monkeypatch.setattr(requests, "head", lambda *args, **kwargs: Response())
    monkeypatch.setattr(requests, "post", lambda *args, **kwargs: Response())

    with pytest.raises(RuntimeError, match="metadata does not match"):
        upload_file(
            server_url="https://replays.example",
            owner="alice",
            repository="go2",
            path=source,
            retries=0,
        )


@pytest.mark.parametrize(
    "encoded_filename",
    [
        "../escaped.db",
        "..%2Fescaped.db",
        r"..\escaped.db",
        r"nested\escaped.db",
    ],
)
def test_download_rejects_server_filename_path_traversal(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    encoded_filename: str,
) -> None:
    payload = b"payload"
    object_id = hashlib.sha256(payload).hexdigest()

    class Response:
        headers = {
            "X-Dimos-Filename": encoded_filename,
            "X-Dimos-Sha256": object_id,
        }

        def __enter__(self) -> Response:
            return self

        def __exit__(self, *args: object) -> None:
            return None

        def raise_for_status(self) -> None:
            return None

        def iter_content(self, chunk_size: int) -> Iterator[bytes]:
            del chunk_size
            yield payload

    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())
    download_root = tmp_path / "downloads"
    download_root.mkdir()
    monkeypatch.chdir(download_root)

    with pytest.raises(RepositoryError, match="plain file name"):
        download_object(
            server_url="https://replays.example",
            owner="alice",
            repository="go2",
            object_id=object_id,
            retries=0,
        )

    assert list(download_root.iterdir()) == []
    assert not (tmp_path / "escaped.db").exists()


def test_download_digest_is_bound_to_requested_object_id(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = b"attacker-selected-payload"
    payload_digest = hashlib.sha256(payload).hexdigest()
    requested_object_id = "a" * 64

    class Response:
        headers = {
            "X-Dimos-Filename": "capture.db",
            "X-Dimos-Sha256": payload_digest,
        }

        def __enter__(self) -> Response:
            return self

        def __exit__(self, *args: object) -> None:
            return None

        def raise_for_status(self) -> None:
            return None

        def iter_content(self, chunk_size: int) -> Iterator[bytes]:
            del chunk_size
            yield payload

    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())
    output = tmp_path / "capture.db"

    with pytest.raises(RuntimeError, match="requested object id"):
        download_object(
            server_url="https://replays.example",
            owner="alice",
            repository="go2",
            object_id=requested_object_id,
            output=output,
            retries=0,
        )

    assert not output.exists()
    assert not list(tmp_path.glob("*.part"))


def test_download_rejects_body_digest_mismatch(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = b"tampered-response-body"
    requested_object_id = "a" * 64

    class Response:
        headers = {
            "X-Dimos-Filename": "capture.db",
            "X-Dimos-Sha256": requested_object_id,
        }

        def __enter__(self) -> Response:
            return self

        def __exit__(self, *args: object) -> None:
            return None

        def raise_for_status(self) -> None:
            return None

        def iter_content(self, chunk_size: int) -> Iterator[bytes]:
            del chunk_size
            yield payload

    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())
    output = tmp_path / "capture.db"

    with pytest.raises(RuntimeError, match="download SHA-256 mismatch"):
        download_object(
            server_url="https://replays.example",
            owner="alice",
            repository="go2",
            object_id=requested_object_id,
            output=output,
            retries=0,
        )

    assert not output.exists()
    assert not list(tmp_path.glob("*.part"))


@pytest.mark.parametrize("invalid_entry", ["garbage", 123, None])
def test_list_rejects_non_object_entries(
    monkeypatch: pytest.MonkeyPatch,
    invalid_entry: object,
) -> None:
    class Response:
        def raise_for_status(self) -> None:
            return None

        def json(self) -> dict[str, Any]:
            return {"objects": [invalid_entry]}

    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())

    with pytest.raises(RuntimeError, match="non-object list entry"):
        list_objects(
            server_url="https://replays.example",
            owner="alice",
            repository="go2",
        )


def test_list_rejects_cross_repository_metadata(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    item = ReplayObject(
        owner="mallory",
        repository="other",
        object_id="a" * 64,
        filename="capture.db",
        size_bytes=10,
        sha256="a" * 64,
        content_type="application/octet-stream",
        created_at="2026-07-25T00:00:00+00:00",
    )

    class Response:
        def raise_for_status(self) -> None:
            return None

        def json(self) -> dict[str, Any]:
            return {"objects": [item.to_dict()]}

    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())

    with pytest.raises(RuntimeError, match="different repository"):
        list_objects(
            server_url="https://replays.example",
            owner="alice",
            repository="go2",
        )


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


def test_batch_preserves_names_for_duplicate_content(tmp_path: Path) -> None:
    source_dir = tmp_path / "capture"
    source_dir.mkdir()
    first = source_dir / "camera-left.mp4"
    second = source_dir / "camera-right.mp4"
    first.write_bytes(b"same-video")
    second.write_bytes(b"same-video")
    output_dir = tmp_path / "restored"

    with _running_server(tmp_path / "objects") as server_url:
        manifest = upload_files(
            server_url=server_url,
            owner="alice",
            repository="duplicates",
            paths=(first, second),
            token="test-token",
            workers=2,
        )
        restored = download_objects(
            server_url=server_url,
            manifest=manifest,
            output_dir=output_dir,
            token="test-token",
            workers=2,
        )
        listed = list_objects(
            server_url=server_url,
            owner="alice",
            repository="duplicates",
            token="test-token",
        )

    assert [item.filename for item in manifest.objects] == [
        "camera-left.mp4",
        "camera-right.mp4",
    ]
    assert len({item.object_id for item in manifest.objects}) == 1
    assert len(listed) == 1
    assert [path.name for path in restored] == ["camera-left.mp4", "camera-right.mp4"]
    assert all(path.read_bytes() == b"same-video" for path in restored)


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


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("version", True, "version must be 1"),
        ("owner", None, "invalid manifest repository identity"),
        ("repository", 7, "invalid manifest repository identity"),
    ],
)
def test_manifest_rejects_coerced_json_types(
    field: str,
    value: object,
    message: str,
) -> None:
    data: dict[str, object] = {
        "version": 1,
        "owner": "alice",
        "repository": "go2",
        "objects": [],
    }
    data[field] = value

    with pytest.raises(RepositoryError, match=message):
        ReplayManifest.from_dict(data)


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
        ("filename", r"..\capture.db", "invalid replay object metadata"),
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


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("owner", None),
        ("repository", None),
        ("object_id", None),
        ("filename", 7),
        ("size_bytes", True),
        ("size_bytes", "10"),
        ("sha256", None),
        ("content_type", None),
        ("created_at", None),
    ],
)
def test_replay_object_rejects_coerced_json_types(
    field: str,
    value: object,
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

    with pytest.raises(RepositoryError, match="invalid replay object metadata"):
        ReplayObject.from_dict(data)


def test_retry_policy_validates_input_and_retries_transient_failures(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with pytest.raises(RepositoryError, match="retries cannot be negative"):
        repository_module._with_retries(lambda: None, retries=-1, backoff_seconds=0)
    with pytest.raises(RepositoryError, match="backoff_seconds cannot be negative"):
        repository_module._with_retries(lambda: None, retries=0, backoff_seconds=-1)

    attempts = 0
    sleeps: list[float] = []

    def operation() -> str:
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            raise requests.ConnectionError("temporary")
        return "ok"

    monkeypatch.setattr(repository_module.time, "sleep", sleeps.append)
    assert (
        repository_module._with_retries(
            operation,
            retries=2,
            backoff_seconds=0.25,
        )
        == "ok"
    )
    assert attempts == 3
    assert sleeps == [0.25, 0.5]

    response = requests.Response()
    response.status_code = HTTPStatus.BAD_REQUEST
    with pytest.raises(requests.HTTPError):
        repository_module._with_retries(
            lambda: (_ for _ in ()).throw(requests.HTTPError(response=response)),
            retries=3,
            backoff_seconds=0,
        )


@pytest.mark.parametrize(
    ("options", "message"),
    [
        ({"region": "moon"}, "region must be"),
        ({"max_object_bytes": 0}, "max_object_bytes must be positive"),
        ({"max_repository_bytes": 0}, "max_repository_bytes must be positive"),
        ({"cdn_base_url": "relative/path"}, "absolute HTTP"),
        ({"cdn_base_url": "https://user@example.test"}, "must not contain"),
        ({"cdn_base_url": "https://example.test:bad"}, "invalid port"),
    ],
)
def test_server_rejects_invalid_runtime_configuration(
    tmp_path: Path,
    options: dict[str, object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        ReplayRepositoryServer(
            ("127.0.0.1", 0),
            ReplayRepository(tmp_path),
            token=None,
            **options,  # type: ignore[arg-type]
        )


def test_server_tracks_pending_repository_quota(tmp_path: Path) -> None:
    server = ReplayRepositoryServer(
        ("127.0.0.1", 0),
        ReplayRepository(tmp_path),
        token=None,
        max_repository_bytes=10,
        cdn_base_url="https://cdn.example.test:8443/base/",
    )
    try:
        assert server.cdn_origin == "https://cdn.example.test:8443"
        assert server.reserve_upload("alice", "demo", 6, None) == 6
        with pytest.raises(repository_module.RepositoryQuotaError):
            server.reserve_upload("alice", "demo", 5, None)
        server.release_upload("alice", "demo", 2)
        assert server._pending_bytes[("alice", "demo")] == 4
        server.release_upload("alice", "demo", 4)
        assert ("alice", "demo") not in server._pending_bytes
        server.release_upload("alice", "demo", 0)
    finally:
        server.server_close()


def test_batch_validation_and_failure_details(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "capture.h264"
    source.write_bytes(b"video")
    with pytest.raises(RepositoryError, match="workers must be at least 1"):
        upload_files(
            server_url="http://unused",
            owner="alice",
            repository="demo",
            paths=[source],
            workers=0,
        )
    with pytest.raises(RepositoryError, match="at least one source"):
        upload_files(
            server_url="http://unused",
            owner="alice",
            repository="demo",
            paths=[],
        )
    with pytest.raises(FileNotFoundError):
        upload_files(
            server_url="http://unused",
            owner="alice",
            repository="demo",
            paths=[tmp_path / "missing.mp4"],
        )

    monkeypatch.setattr(
        repository_module,
        "upload_file",
        lambda **_: (_ for _ in ()).throw(RuntimeError("provider offline")),
    )
    with pytest.raises(BatchTransferError, match="provider offline") as exc_info:
        upload_files(
            server_url="http://unused",
            owner="alice",
            repository="demo",
            paths=[source],
        )
    assert str(source) in exc_info.value.failures


def test_manifest_io_and_repository_path_diagnostics(tmp_path: Path) -> None:
    repository = ReplayRepository(tmp_path / "objects")
    payload = b"replay"
    item = repository.put_stream(
        owner="alice",
        repository="demo",
        filename="capture.h264",
        source=BytesIO(payload),
        size_bytes=len(payload),
        content_type="video/h264",
    )
    manifest = ReplayManifest(owner="alice", repository="demo", objects=(item,))
    path = write_manifest(tmp_path / "manifest.json", manifest)

    assert read_manifest(path) == manifest
    assert list(iter_repository_paths(repository, "alice", "demo")) == [
        repository.get("alice", "demo", item.object_id)[1]
    ]
    path.write_text("[]", encoding="utf-8")
    with pytest.raises(RepositoryError, match="JSON object"):
        read_manifest(path)
