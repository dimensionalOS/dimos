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

"""Tests for hosted replay reference resolution and caching."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import pytest

from dimos.hosted_data import reference


class _Response:
    def __init__(self, payload: Any, *, body: bytes = b"") -> None:
        self._payload = payload
        self._body = body

    def raise_for_status(self) -> None:
        return None

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


def test_parse_hosted_reference() -> None:
    object_id = "a" * 64
    parsed = reference.HostedReplayReference.parse(
        f"dimos-replay://alice/go2/{object_id}"
    )
    assert parsed == reference.HostedReplayReference("alice", "go2", object_id)
    assert reference.HostedReplayReference.parse("go2_short") is None


def test_downloads_and_reuses_verified_cache(monkeypatch, tmp_path: Path) -> None:
    payload = b"sqlite-replay" * 1000
    metadata = _metadata(payload)
    calls: list[str] = []

    def fake_get(url: str, **kwargs):
        calls.append(url)
        if kwargs.get("stream"):
            return _Response({}, body=payload)
        return _Response([metadata])

    monkeypatch.setattr(reference.requests, "get", fake_get)
    uri = f"dimos-replay://alice/go2/{metadata['object_id']}"

    first = reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)
    second = reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)

    assert first == second
    assert first is not None and first.read_bytes() == payload
    assert len(calls) == 2


def test_rejects_video_as_memory2_replay(monkeypatch, tmp_path: Path) -> None:
    payload = b"video"
    metadata = _metadata(payload, filename="capture.mp4")
    monkeypatch.setattr(
        reference.requests,
        "get",
        lambda *args, **kwargs: _Response([metadata]),
    )
    uri = f"dimos-replay://alice/go2/{metadata['object_id']}"

    with pytest.raises(reference.HostedReplayError, match="not a memory2 SQLite replay"):
        reference.resolve_hosted_replay(uri, server_url="http://data", cache_root=tmp_path)
