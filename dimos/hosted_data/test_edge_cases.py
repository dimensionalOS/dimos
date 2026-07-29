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

"""Failure-path tests for hosted replay transfer and resolution."""

from __future__ import annotations

import hashlib
import io
from pathlib import Path
from typing import Any

from fastapi.testclient import TestClient
import pytest
import requests
from typer.testing import CliRunner

from dimos.hosted_data import cli, reference
from dimos.hosted_data.api import _parse_range, _stream, create_app
from dimos.hosted_data.repository import ReplayRepository, RepositoryError


class _Response:
    def __init__(
        self,
        payload: Any,
        *,
        body: bytes = b"",
        error: requests.RequestException | None = None,
        text: str = "",
    ) -> None:
        self._payload = payload
        self._body = body
        self._error = error
        self.text = text

    def raise_for_status(self) -> None:
        if self._error is not None:
            raise self._error

    def json(self) -> Any:
        return self._payload

    def iter_content(self, chunk_size: int):
        yield from (
            self._body[index : index + chunk_size]
            for index in range(0, len(self._body), chunk_size)
        )

    def close(self) -> None:
        return None


def _metadata(payload: bytes, filename: str = "go2.db") -> dict[str, Any]:
    object_id = hashlib.sha256(payload).hexdigest()
    return {
        "owner": "alice",
        "repository": "go2",
        "object_id": object_id,
        "filename": filename,
        "size_bytes": len(payload),
        "sha256": object_id,
        "content_type": "application/x-sqlite3",
        "created_at": "2026-07-29T00:00:00+00:00",
    }


def test_head_missing_object_and_range_variants(tmp_path: Path) -> None:
    client = TestClient(create_app(ReplayRepository(tmp_path / "objects")))
    digest = "0" * 64
    assert client.head(f"/v1/repositories/alice/go2/objects/{digest}").status_code == 404

    uploaded = client.put(
        "/v1/repositories/alice/go2/objects",
        params={"filename": "capture.mp4"},
        content=b"0123456789",
        headers={"Content-Length": "10"},
    ).json()
    url = f"/v1/repositories/alice/go2/objects/{uploaded['object_id']}"
    open_ended = client.get(url, headers={"Range": "bytes=7-"})
    assert open_ended.status_code == 206
    assert open_ended.content == b"789"

    for value in ("bytes=-0", "bytes=8-7", "items=0-1", "bytes=-"):
        response = client.get(url, headers={"Range": value})
        assert response.status_code == 416
        assert response.headers["content-range"] == "bytes */10"


def test_empty_object_rejects_range(tmp_path: Path) -> None:
    client = TestClient(create_app(ReplayRepository(tmp_path / "objects")))
    uploaded = client.put(
        "/v1/repositories/alice/go2/objects",
        params={"filename": "empty.db"},
        content=b"",
        headers={"Content-Length": "0"},
    ).json()
    response = client.get(
        f"/v1/repositories/alice/go2/objects/{uploaded['object_id']}",
        headers={"Range": "bytes=0-"},
    )
    assert response.status_code == 416


def test_stream_detects_truncated_repository_object() -> None:
    body = io.BytesIO(b"short")
    stream = _stream(body, remaining=10)
    assert next(stream) == b"short"
    with pytest.raises(RepositoryError, match="ended before"):
        next(stream)
    assert body.closed


@pytest.mark.parametrize(
    ("value", "size", "message"),
    [
        ("bytes=20-21", 10, "starts beyond"),
        ("bytes=8-7", 10, "ends before"),
        ("bytes=-0", 10, "must be positive"),
        ("garbage", 10, "invalid"),
    ],
)
def test_parse_range_errors(value: str, size: int, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _parse_range(value, size)


@pytest.mark.parametrize(
    ("uri", "message"),
    [
        ("dimos-replay:///go2/" + "a" * 64, "must be"),
        ("dimos-replay://alice/go2", "must be"),
        ("dimos-replay://alice/go2/" + "A" * 64, "lowercase SHA-256"),
        ("dimos-replay://alice/go2/" + "a" * 64 + "?download=1", "must be"),
    ],
)
def test_rejects_malformed_hosted_reference(uri: str, message: str) -> None:
    with pytest.raises(reference.HostedReplayError, match=message):
        reference.HostedReplayReference.parse(uri)


def test_reports_missing_object_and_metadata_request_failure(monkeypatch, tmp_path: Path) -> None:
    object_id = "a" * 64
    uri = f"dimos-replay://alice/go2/{object_id}"
    monkeypatch.setattr(reference.requests, "get", lambda *args, **kwargs: _Response([]))
    with pytest.raises(reference.HostedReplayError, match="object not found"):
        reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)

    monkeypatch.setattr(
        reference.requests,
        "get",
        lambda *args, **kwargs: _Response([], error=requests.ConnectionError("offline")),
    )
    with pytest.raises(reference.HostedReplayError, match="failed to list"):
        reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)


@pytest.mark.parametrize(
    ("body", "message"),
    [(b"too-short", "size does not match"), (b"x" * 16, "SHA-256 verification")],
)
def test_rejects_corrupt_download(
    monkeypatch,
    tmp_path: Path,
    body: bytes,
    message: str,
) -> None:
    payload = b"expected-content"
    metadata = _metadata(payload)

    def fake_get(url: str, **kwargs):
        if kwargs.get("stream"):
            return _Response({}, body=body)
        return _Response([metadata])

    monkeypatch.setattr(reference.requests, "get", fake_get)
    uri = f"dimos-replay://alice/go2/{metadata['object_id']}"
    with pytest.raises(reference.HostedReplayError, match=message):
        reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)
    assert not list(tmp_path.glob("*.part"))


def test_reports_download_request_failure(monkeypatch, tmp_path: Path) -> None:
    payload = b"expected-content"
    metadata = _metadata(payload)

    def fake_get(url: str, **kwargs):
        if kwargs.get("stream"):
            return _Response({}, error=requests.ConnectionError("offline"))
        return _Response([metadata])

    monkeypatch.setattr(reference.requests, "get", fake_get)
    uri = f"dimos-replay://alice/go2/{metadata['object_id']}"
    with pytest.raises(reference.HostedReplayError, match="failed to download"):
        reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)


def test_list_supports_json_and_empty_output(monkeypatch) -> None:
    responses = iter([_Response([]), _Response([{"object_id": "a" * 64}])])
    monkeypatch.setattr(cli.requests, "get", lambda *args, **kwargs: next(responses))
    runner = CliRunner()
    empty = runner.invoke(cli.data_app, ["list"])
    json_output = runner.invoke(cli.data_app, ["list", "--json"])
    assert empty.exit_code == 0
    assert "No objects found." in empty.stdout
    assert json_output.exit_code == 0
    assert '"object_id"' in json_output.stdout


def test_list_reports_network_and_http_errors(monkeypatch) -> None:
    def offline(*args, **kwargs):
        raise requests.ConnectionError("offline")

    monkeypatch.setattr(cli.requests, "get", offline)
    network = CliRunner().invoke(cli.data_app, ["list"])
    assert network.exit_code == 1
    assert "offline" in network.stderr

    monkeypatch.setattr(
        cli.requests,
        "get",
        lambda *args, **kwargs: _Response(
            [], error=requests.HTTPError("bad gateway"), text="repository unavailable"
        ),
    )
    http = CliRunner().invoke(cli.data_app, ["list"])
    assert http.exit_code == 1
    assert "repository unavailable" in http.stderr


def test_download_rejects_missing_existing_and_corrupt_objects(monkeypatch, tmp_path: Path) -> None:
    runner = CliRunner()
    monkeypatch.setattr(cli.requests, "get", lambda *args, **kwargs: _Response([]))
    missing = runner.invoke(cli.data_app, ["download", "a" * 64])
    assert missing.exit_code == 1
    assert "object not found" in missing.stderr

    payload = b"expected"
    object_id = hashlib.sha256(payload).hexdigest()
    item = {"object_id": object_id, "size_bytes": len(payload), "filename": "capture.mp4"}
    output = tmp_path / "capture.mp4"
    output.write_bytes(b"existing")
    monkeypatch.setattr(cli.requests, "get", lambda *args, **kwargs: _Response([item]))
    existing = runner.invoke(cli.data_app, ["download", object_id, "-o", str(output)])
    assert existing.exit_code == 1
    assert "already exists" in existing.stderr

    def corrupt_get(url, **kwargs):
        return _Response([item] if not kwargs.get("stream") else {}, body=b"corrupt")

    monkeypatch.setattr(cli.requests, "get", corrupt_get)
    corrupt = runner.invoke(
        cli.data_app,
        ["download", object_id, "-o", str(output), "--force"],
    )
    assert corrupt.exit_code == 1
    assert "SHA-256 verification" in corrupt.stderr
    assert output.read_bytes() == b"existing"


def test_upload_reports_network_error(monkeypatch, tmp_path: Path) -> None:
    source = tmp_path / "capture.mp4"
    source.write_bytes(b"video")

    def offline(*args, **kwargs):
        raise requests.ConnectionError("offline")

    monkeypatch.setattr(cli.requests, "put", offline)
    result = CliRunner().invoke(cli.data_app, ["upload", str(source)])
    assert result.exit_code == 1
    assert "offline" in result.stderr
