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

import hashlib
import io
from pathlib import Path
import tarfile

import pytest
from pytest_mock import MockerFixture

from dimos.evals.suites.lib.space import data


@pytest.fixture
def payload(monkeypatch: pytest.MonkeyPatch) -> bytes:
    value = b'[{"question":"Synthetic test question","answer":2}]'
    monkeypatch.setattr(data, "DATA_BYTES", len(value))
    monkeypatch.setattr(data, "DATA_SHA256", hashlib.sha256(value).hexdigest())
    return value


def archive_bytes(payload: bytes, *, name: str = data.DATA_MEMBER, link: bool = False) -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        member = tarfile.TarInfo(name)
        member.size = len(payload)
        if link:
            member.type = tarfile.SYMTYPE
            member.linkname = "/outside-cache"
        archive.addfile(member, io.BytesIO(payload))
    return buffer.getvalue()


def test_read_member_preserves_verified_bytes(payload: bytes) -> None:
    assert data.read_member(io.BytesIO(archive_bytes(payload))) == payload


def test_read_member_rejects_corruption(payload: bytes) -> None:
    with pytest.raises(ValueError, match="SHA-256"):
        data.read_member(io.BytesIO(archive_bytes(payload.replace(b"2", b"1"))))


@pytest.mark.parametrize("link", [False, True])
def test_read_member_rejects_nonmatching_or_linked_member(payload: bytes, link: bool) -> None:
    name = data.DATA_MEMBER if link else "../../escape"
    with pytest.raises(ValueError, match="type or size|does not contain"):
        data.read_member(io.BytesIO(archive_bytes(payload, name=name, link=link)))


def test_compressed_reader_enforces_limit_and_deadline(mocker: MockerFixture) -> None:
    reader = data.BoundedReader(io.BytesIO(b"abcdef"), limit=3, timeout_s=10)
    assert reader.read(3) == b"abc"
    with pytest.raises(ValueError, match="download limit"):
        reader.read(1)
    mocker.patch.object(data.time, "monotonic", side_effect=[0.0, 2.0])
    expired = data.BoundedReader(io.BytesIO(b"abc"), limit=3, timeout_s=1)
    with pytest.raises(TimeoutError, match="deadline"):
        expired.read(1)


def test_download_publishes_only_verified_data(
    payload: bytes, tmp_path: Path, mocker: MockerFixture
) -> None:
    response = mocker.MagicMock()
    response.__enter__.return_value = response
    response.raw = io.BytesIO(archive_bytes(payload))
    get = mocker.patch.object(data.requests, "get", return_value=response)
    destination = tmp_path / "external-cache" / "qas.json"

    assert data.acquire_data(destination) > 0
    assert destination.read_bytes() == payload
    assert list(destination.parent.iterdir()) == [destination]
    response.__exit__.assert_called_once()
    assert data.acquire_data(destination) == 0
    assert get.call_count == 1


def test_failed_download_does_not_publish(
    payload: bytes, tmp_path: Path, mocker: MockerFixture
) -> None:
    response = mocker.MagicMock()
    response.__enter__.return_value = response
    response.raw = io.BytesIO(archive_bytes(payload.replace(b"2", b"1")))
    mocker.patch.object(data.requests, "get", return_value=response)
    destination = tmp_path / "qas.json"
    with pytest.raises(ValueError, match="SHA-256"):
        data.acquire_data(destination)
    assert not destination.exists()
    response.__exit__.assert_called_once()


def test_missing_or_corrupt_data_fails_before_selection(tmp_path: Path) -> None:
    paths = data.SpacePaths(tmp_path)
    with pytest.raises(FileNotFoundError, match="commands setup"):
        data.load_examples(paths)
    paths.questions.parent.mkdir(parents=True)
    paths.questions.write_text("[]")
    with pytest.raises(ValueError, match="SHA-256"):
        data.load_examples(paths)
