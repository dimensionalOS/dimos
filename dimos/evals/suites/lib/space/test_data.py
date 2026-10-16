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
from concurrent.futures import ThreadPoolExecutor
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
from pathlib import Path
import tarfile
import threading
from typing import Any

from filelock import FileLock
import pytest
from pytest_mock import MockerFixture
import requests

from dimos.evals.suites.lib.space import data
from dimos.evals.suites.lib.space.constants import (
    DATA_MEMBER,
    DOWNLOAD_TIMEOUT_S,
)


@pytest.fixture
def payload(monkeypatch: pytest.MonkeyPatch) -> bytes:
    value = b'[{"question":"Synthetic test question","answer":2}]'
    monkeypatch.setattr(data, "DATA_BYTES", len(value))
    monkeypatch.setattr(data, "DATA_SHA256", hashlib.sha256(value).hexdigest())
    return value


def archive_bytes(payload: bytes, *, name: str = DATA_MEMBER, link: bool = False) -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        member = tarfile.TarInfo(name)
        member.size = len(payload)
        if link:
            member.type = tarfile.SYMTYPE
            member.linkname = "/outside-cache"
        archive.addfile(member, io.BytesIO(payload))
    return buffer.getvalue()


@pytest.mark.parametrize("link", [False, True])
def test_read_member_rejects_nonmatching_or_linked_member(payload: bytes, link: bool) -> None:
    name = DATA_MEMBER if link else "../../escape"
    with pytest.raises(ValueError, match="type or size|does not contain"):
        data.read_member(io.BytesIO(archive_bytes(payload, name=name, link=link)))


def test_compressed_reader_enforces_limit_and_deadline(mocker: MockerFixture) -> None:
    reader = data.BoundedReader(io.BytesIO(b"abcdef").read1, limit=3, timeout_s=10)
    assert reader.read(3) == b"abc"
    with pytest.raises(ValueError, match="download limit"):
        reader.read(1)
    mocker.patch.object(data, "time").monotonic.side_effect = [0.0, 2.0]
    expired = data.BoundedReader(io.BytesIO(b"abc").read1, limit=3, timeout_s=1)
    with pytest.raises(TimeoutError, match="deadline"):
        expired.read(1)


@pytest.fixture
def stalled_response() -> Iterator[requests.Response]:
    release = threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"\x1f")
            self.wfile.flush()
            release.wait(5)

    with HTTPServer(("127.0.0.1", 0), Handler) as server:
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        try:
            with requests.Session() as session:
                session.trust_env = False
                with session.get(
                    f"http://127.0.0.1:{server.server_port}", stream=True, timeout=(5, 1)
                ) as response:
                    yield response
        finally:
            release.set()
            server.shutdown()
            worker.join(timeout=5)
            assert not worker.is_alive()


def test_download_checks_deadline_after_partial_response(
    stalled_response: requests.Response, tmp_path: Path, mocker: MockerFixture
) -> None:
    mocker.patch.object(requests, "get", return_value=stalled_response)
    mocker.patch.object(data, "time").monotonic.side_effect = [0, 0, DOWNLOAD_TIMEOUT_S + 1]
    destination = tmp_path / "qas.json"
    with pytest.raises(TimeoutError, match="deadline"):
        data.acquire_data(destination)
    assert not destination.exists()


def test_download_publishes_only_verified_data(
    payload: bytes, tmp_path: Path, mocker: MockerFixture
) -> None:
    response = mocker.MagicMock()
    response.__enter__.return_value = response
    response.raw = io.BytesIO(archive_bytes(payload))
    get = mocker.patch.object(requests, "get", return_value=response)
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
    mocker.patch.object(requests, "get", return_value=response)
    destination = tmp_path / "qas.json"
    with pytest.raises(ValueError, match="SHA-256"):
        data.acquire_data(destination)
    assert not destination.exists()
    response.__exit__.assert_called_once()


def test_missing_or_corrupt_data_fails_before_selection(tmp_path: Path) -> None:
    paths = data.SpacePaths(tmp_path)
    with pytest.raises(FileNotFoundError, match="dimos evals setup space"):
        data.load_examples(paths)
    paths.questions.parent.mkdir(parents=True)
    paths.questions.write_text("[]")
    with pytest.raises(ValueError, match="SHA-256"):
        data.load_examples(paths)


@pytest.fixture
def setup_workers() -> Iterator[ThreadPoolExecutor]:
    with ThreadPoolExecutor(max_workers=2) as workers:
        yield workers


def test_concurrent_setup_downloads_data_once(
    payload: bytes, tmp_path: Path, mocker: MockerFixture, setup_workers: ThreadPoolExecutor
) -> None:
    paths = data.SpacePaths(tmp_path / "cache")
    downloading = threading.Event()
    second_caller = threading.Event()
    lock_count = 0

    def setup_lock(*args: Any, **kwargs: Any) -> FileLock:
        nonlocal lock_count
        lock_count += 1
        if lock_count == 2:
            second_caller.set()
        return FileLock(*args, **kwargs)

    def download(*args: Any, **kwargs: Any) -> Any:
        downloading.set()
        assert second_caller.wait(5), "second setup never reached the cache"
        response = mocker.MagicMock()
        response.__enter__.return_value = response
        response.raw = io.BytesIO(archive_bytes(payload))
        return response

    mocker.patch.object(data, "FileLock", side_effect=setup_lock)
    mocker.patch.object(data, "verify_package")
    mocker.patch.object(data, "load_examples", return_value=())
    get = mocker.patch.object(requests, "get", side_effect=download)
    first = setup_workers.submit(data.setup, paths)
    assert downloading.wait(5), "first setup never started downloading"
    second = setup_workers.submit(data.setup, paths)
    results = [first.result(timeout=10), second.result(timeout=10)]
    assert [result["downloaded_compressed_bytes"] for result in results] == [
        len(archive_bytes(payload)),
        0,
    ]
    assert get.call_count == 1
    assert paths.questions.read_bytes() == payload


def test_setup_checks_package_before_creating_cache(tmp_path: Path, mocker: MockerFixture) -> None:
    paths = data.SpacePaths(tmp_path / "cache")
    mocker.patch.object(data, "verify_package", side_effect=RuntimeError("Missing package"))
    acquire = mocker.patch.object(data, "acquire_data")
    with pytest.raises(RuntimeError, match="Missing package"):
        data.setup(paths)
    assert not paths.root.exists()
    acquire.assert_not_called()


def test_failed_download_releases_setup_lock(
    payload: bytes, tmp_path: Path, mocker: MockerFixture, setup_workers: ThreadPoolExecutor
) -> None:
    paths = data.SpacePaths(tmp_path / "cache")
    response = mocker.MagicMock()
    response.__enter__.return_value = response
    response.raw = io.BytesIO(archive_bytes(payload))
    mocker.patch.object(data, "verify_package")
    mocker.patch.object(data, "load_examples", return_value=())
    mocker.patch.object(requests, "get", side_effect=[requests.ConnectionError("failed"), response])
    with pytest.raises(requests.ConnectionError, match="failed"):
        data.setup(paths)
    assert not paths.questions.exists()
    result = setup_workers.submit(data.setup, paths).result(timeout=10)
    assert result["downloaded_compressed_bytes"] > 0
    assert paths.questions.read_bytes() == payload
