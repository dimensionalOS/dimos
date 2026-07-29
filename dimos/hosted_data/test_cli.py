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

"""Tests for ``dimos data`` commands."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

from typer.testing import CliRunner

from dimos.hosted_data import cli


class _Response:
    def __init__(self, payload: Any, *, body: bytes = b"") -> None:
        self._payload = payload
        self._body = body
        self.status_code = 200
        self.text = ""

    def raise_for_status(self) -> None:
        return None

    def json(self) -> Any:
        return self._payload

    def iter_content(self, chunk_size: int):
        yield from (self._body[index : index + chunk_size] for index in range(0, len(self._body), chunk_size))

    def close(self) -> None:
        return None


def test_upload_streams_file_and_digest(monkeypatch, tmp_path: Path) -> None:
    payload = b"video" * 1000
    source = tmp_path / "capture.mp4"
    source.write_bytes(payload)
    captured: dict[str, Any] = {}

    def fake_put(url, **kwargs):
        captured["url"] = url
        captured["params"] = kwargs["params"]
        captured["headers"] = kwargs["headers"]
        captured["body"] = kwargs["data"].read()
        return _Response(
            {
                "filename": source.name,
                "size_bytes": len(payload),
                "object_id": hashlib.sha256(payload).hexdigest(),
            }
        )

    monkeypatch.setattr(cli.requests, "put", fake_put)
    result = CliRunner().invoke(cli.data_app, ["upload", str(source)])

    assert result.exit_code == 0
    assert captured["body"] == payload
    assert captured["params"] == {"filename": "capture.mp4"}
    assert captured["headers"]["X-Content-SHA256"] == hashlib.sha256(payload).hexdigest()


def test_list_prints_remote_objects(monkeypatch) -> None:
    item = {"object_id": "a" * 64, "size_bytes": 42, "filename": "capture.mp4"}
    monkeypatch.setattr(cli.requests, "get", lambda *args, **kwargs: _Response([item]))

    result = CliRunner().invoke(cli.data_app, ["list"])

    assert result.exit_code == 0
    assert "capture.mp4" in result.stdout
    assert "a" * 64 in result.stdout


def test_download_streams_and_verifies(monkeypatch, tmp_path: Path) -> None:
    payload = b"downloaded-video"
    object_id = hashlib.sha256(payload).hexdigest()
    item = {"object_id": object_id, "size_bytes": len(payload), "filename": "capture.mp4"}

    def fake_get(url, **kwargs):
        return _Response(item if False else ([item] if not kwargs.get("stream") else {}), body=payload)

    monkeypatch.setattr(cli.requests, "get", fake_get)
    output = tmp_path / "result.mp4"
    result = CliRunner().invoke(
        cli.data_app,
        ["download", object_id, "--output", str(output)],
    )

    assert result.exit_code == 0
    assert output.read_bytes() == payload
