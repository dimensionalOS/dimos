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

"""Reading a release, on fixtures shaped like one and written here.

Every question below is invented. The real release is Apple's to distribute,
so nothing from it — question, answer or transcript — is committed anywhere in
this repository. The download tests answer the request themselves: they never
reach Apple, and the archive they hash is a few kilobytes built in-process.
"""

from collections.abc import Iterator
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile
from typing import Any

import pytest
import requests

import dimos.benchmark.space_qa.data as data_module
from dimos.benchmark.space_qa.data import (
    DOWNLOAD_CONNECT_TIMEOUT_SECONDS,
    DOWNLOAD_READ_TIMEOUT_SECONDS,
    PROVENANCE_NAME,
    RELEASE_DIR_NAME,
    SPACE_DATA_ENV,
    SPACE_DATA_URL,
    download_release,
    load_task_rows,
    release_sha256,
    resolve_space_data,
)
from dimos.benchmark.space_qa.tasks import SpaceTextTask

TASK = SpaceTextTask(name="FAKE_text", groups=3)
OTHER_DIGEST = "0" * 64


def _rows(count: int = TASK.expected_rows) -> list[dict[str, Any]]:
    return [
        {"question": f"Where is marker {ordinal}?", "answer": ordinal % 4 + 1, "task": "FAKE"}
        for ordinal in range(count)
    ]


def _release(root: Path, rows: list[Any]) -> Path:
    path = root / TASK.qas_relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rows), encoding="utf-8")
    return root


def _archive(rows: list[Any], root: str = RELEASE_DIR_NAME) -> bytes:
    """A few-kilobyte stand-in for the 3.6 GB release, shaped the same way."""
    payload = json.dumps(rows).encode("utf-8")
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
        entry = tarfile.TarInfo(f"{root}/{TASK.qas_relative_path}")
        entry.size = len(payload)
        tar.addfile(entry, io.BytesIO(payload))
    return buffer.getvalue()


class _Response:
    """The slice of ``requests.Response`` the downloader touches."""

    def __init__(
        self, body: bytes, refusal: Exception | None = None, stall: Exception | None = None
    ) -> None:
        self._body = body
        self._refusal = refusal
        self._stall = stall

    def __enter__(self) -> "_Response":
        return self

    def __exit__(self, *_closing: Any) -> None:
        return None

    def raise_for_status(self) -> None:
        if self._refusal is not None:
            raise self._refusal

    def iter_content(self, chunk_size: int) -> Iterator[bytes]:
        for start in range(0, len(self._body), chunk_size):
            yield self._body[start : start + chunk_size]
        if self._stall is not None:
            raise self._stall


def _serve(monkeypatch, response: _Response) -> list[dict[str, Any]]:
    """Answer the one GET the downloader makes, and record how it asked."""
    asked: list[dict[str, Any]] = []

    def get(url: str, **kwargs: Any) -> _Response:
        asked.append({"url": url, **kwargs})
        return response

    monkeypatch.setattr(data_module.requests, "get", get)
    return asked


def _pin(monkeypatch, body: bytes) -> str:
    digest = hashlib.sha256(body).hexdigest()
    monkeypatch.setattr(data_module, "SPACE_DATA_SHA256", digest)
    return digest


def _cache(root: Path, monkeypatch, record: str | bytes | None) -> Path:
    """A cache directory as a run would find it: a release, and what it claims to be."""
    release = root / "data" / RELEASE_DIR_NAME
    release.mkdir(parents=True)
    if record is not None:
        (root / "data" / PROVENANCE_NAME).write_bytes(
            record.encode("utf-8") if isinstance(record, str) else record
        )
    monkeypatch.delenv(SPACE_DATA_ENV, raising=False)
    monkeypatch.setattr(data_module, "space_cache_root", lambda: root)
    monkeypatch.setattr(
        data_module, "download_release", lambda _target: pytest.fail("downloaded over a cache")
    )
    return release


def _staging_left_behind(target: Path) -> list[Path]:
    return [path for path in target.iterdir() if path.name.startswith(".space-data-")]


def test_a_task_reads_back_every_row_in_upstream_order(tmp_path) -> None:
    rows = _rows()

    assert load_task_rows(TASK, _release(tmp_path, rows)).rows == tuple(rows)


def test_a_task_hashes_the_bytes_it_read_its_questions_from(tmp_path) -> None:
    """The release digest repeats a claim; this one is taken off the file itself."""
    root = _release(tmp_path, _rows())
    path = root / TASK.qas_relative_path

    assert load_task_rows(TASK, root).sha256 == hashlib.sha256(path.read_bytes()).hexdigest()


def test_a_question_file_edited_after_it_was_unpacked_hashes_differently(tmp_path) -> None:
    """Nothing here refuses the edit; what it cannot do is report the digest of the file it lost."""
    root = _release(tmp_path, _rows())
    before = load_task_rows(TASK, root).sha256
    tampered = _rows()
    tampered[3]["question"] = "Where is the marker, really?"
    _release(root, tampered)

    after = load_task_rows(TASK, root)

    assert after.sha256 != before
    assert after.sha256 == hashlib.sha256((root / TASK.qas_relative_path).read_bytes()).hexdigest()


def test_a_file_that_changed_length_fails_the_run(tmp_path) -> None:
    """Row position is the only address a question has, so a resize invalidates every index."""
    root = _release(tmp_path, _rows(TASK.expected_rows - 4))

    with pytest.raises(ValueError, match=f"holds {TASK.expected_rows - 4} rows"):
        load_task_rows(TASK, root)


def test_a_missing_task_directory_names_the_path_it_looked_at(tmp_path) -> None:
    with pytest.raises(FileNotFoundError, match=TASK.name):
        load_task_rows(TASK, tmp_path)


@pytest.mark.parametrize(
    ("row", "complaint"),
    [
        ("just a string", "expected an object"),
        ({"answer": 1}, "no question text"),
        ({"question": "", "answer": 1}, "no question text"),
        ({"question": "Where?", "answer": "1"}, "non-integer answer"),
        ({"question": "Where?", "answer": True}, "non-integer answer"),
        ({"question": "Where?"}, "non-integer answer"),
    ],
)
def test_a_row_this_adapter_cannot_ask_is_refused(tmp_path, row: Any, complaint: str) -> None:
    rows: list[Any] = list(_rows())
    rows[5] = row

    with pytest.raises(ValueError, match=complaint):
        load_task_rows(TASK, _release(tmp_path, rows))


def test_a_file_holding_something_other_than_rows_is_refused(tmp_path) -> None:
    with pytest.raises(ValueError, match="expected a list of rows"):
        load_task_rows(TASK, _release(tmp_path, {"question": "Where?"}))


def test_an_override_points_at_an_already_extracted_release(tmp_path, monkeypatch) -> None:
    release = tmp_path / RELEASE_DIR_NAME
    release.mkdir()
    monkeypatch.setenv(SPACE_DATA_ENV, str(release))

    assert resolve_space_data() == release.resolve()


def test_an_override_that_is_not_a_directory_is_refused(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv(SPACE_DATA_ENV, str(tmp_path / "nowhere"))

    with pytest.raises(FileNotFoundError, match=SPACE_DATA_ENV):
        resolve_space_data()


def test_a_cached_release_is_used_without_downloading(tmp_path, monkeypatch) -> None:
    release = tmp_path / "data" / RELEASE_DIR_NAME
    release.mkdir(parents=True)
    (tmp_path / "data" / PROVENANCE_NAME).write_text(
        json.dumps({"sha256": data_module.SPACE_DATA_SHA256}), encoding="utf-8"
    )
    monkeypatch.delenv(SPACE_DATA_ENV, raising=False)
    monkeypatch.setattr(data_module, "space_cache_root", lambda: tmp_path)
    monkeypatch.setattr(
        data_module, "download_release", lambda _target: pytest.fail("downloaded a cached release")
    )

    assert resolve_space_data() == release


def test_a_release_without_a_provenance_record_reports_no_digest(tmp_path) -> None:
    """An overridden release was never verified here and must not borrow the pin."""
    release = tmp_path / RELEASE_DIR_NAME
    release.mkdir()
    assert release_sha256(release) is None

    (tmp_path / PROVENANCE_NAME).write_text(json.dumps({"sha256": "abc123"}), encoding="utf-8")
    assert release_sha256(release) == "abc123"


def test_a_cache_that_records_another_release_is_refused(tmp_path, monkeypatch) -> None:
    """The pin is checked on every run, not only on the run that filled the cache."""
    release = _cache(tmp_path, monkeypatch, json.dumps({"sha256": OTHER_DIGEST}))

    with pytest.raises(RuntimeError) as raised:
        resolve_space_data()

    complaint = str(raised.value)
    assert OTHER_DIGEST in complaint
    assert data_module.SPACE_DATA_SHA256 in complaint
    # Both ways out: throw the cache away, or say out loud that it is unverified.
    assert str(release.parent) in complaint
    assert f"{SPACE_DATA_ENV}={release}" in complaint


def test_a_cache_with_no_record_of_what_it_is_is_refused(tmp_path, monkeypatch) -> None:
    """A hand-unpacked cache reads exactly like a verified one; only the record separates them."""
    _cache(tmp_path, monkeypatch, None)

    with pytest.raises(RuntimeError, match=PROVENANCE_NAME):
        resolve_space_data()


@pytest.mark.parametrize("record", ["{ truncated", b"\xff\xfe not utf-8"])
def test_a_cache_whose_record_will_not_parse_is_refused(
    tmp_path, monkeypatch, record: str | bytes
) -> None:
    _cache(tmp_path, monkeypatch, record)

    with pytest.raises(RuntimeError, match=PROVENANCE_NAME):
        resolve_space_data()


@pytest.mark.parametrize("record", ["null", "[]", '"fd4cc896"', "3"])
def test_a_record_that_parses_but_is_no_record_is_refused_with_the_same_guidance(
    tmp_path, monkeypatch, record: str
) -> None:
    """Valid JSON that is not an object says as little as a truncated file, and has to read so."""
    release = _cache(tmp_path, monkeypatch, record)

    with pytest.raises(RuntimeError) as raised:
        resolve_space_data()

    complaint = str(raised.value)
    assert PROVENANCE_NAME in complaint
    assert str(release.parent) in complaint
    assert f"{SPACE_DATA_ENV}={release}" in complaint


def test_download_is_refused_when_the_interpreter_cannot_filter_extraction(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setattr(
        data_module.requests, "get", lambda *_a, **_k: pytest.fail("fetched before the refusal")
    )
    monkeypatch.delattr(data_module.tarfile, "data_filter")

    with pytest.raises(RuntimeError) as raised:
        download_release(tmp_path)

    message = str(raised.value)
    assert "3.10.12" in message
    assert "3.11.4" in message
    assert list(tmp_path.iterdir()) == []


def test_a_download_is_verified_unpacked_and_published(tmp_path, monkeypatch) -> None:
    rows = _rows()
    body = _archive(rows)
    digest = _pin(monkeypatch, body)
    asked = _serve(monkeypatch, _Response(body))

    release = download_release(tmp_path)

    assert release == tmp_path / RELEASE_DIR_NAME
    assert load_task_rows(TASK, release).rows == tuple(rows)
    assert release_sha256(release) == digest
    assert _staging_left_behind(tmp_path) == []
    # Streamed, and bounded at both ends: a hung handshake and a stalled body
    # are the two ways a 3.6 GB fetch waits forever.
    assert asked == [
        {
            "url": SPACE_DATA_URL,
            "stream": True,
            "timeout": (DOWNLOAD_CONNECT_TIMEOUT_SECONDS, DOWNLOAD_READ_TIMEOUT_SECONDS),
        }
    ]


def test_an_archive_that_is_not_the_pinned_release_is_refused(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(data_module, "SPACE_DATA_SHA256", OTHER_DIGEST)
    _serve(monkeypatch, _Response(_archive(_rows())))

    with pytest.raises(RuntimeError, match="the pin has to be re-reviewed"):
        download_release(tmp_path)

    assert not (tmp_path / RELEASE_DIR_NAME).exists()
    assert not (tmp_path / PROVENANCE_NAME).exists()
    assert _staging_left_behind(tmp_path) == []


def test_an_archive_without_the_release_directory_is_refused(tmp_path, monkeypatch) -> None:
    body = _archive(_rows(), root="some_other_release")
    _pin(monkeypatch, body)
    _serve(monkeypatch, _Response(body))

    with pytest.raises(RuntimeError, match=f"no {RELEASE_DIR_NAME}/ directory"):
        download_release(tmp_path)

    assert not (tmp_path / PROVENANCE_NAME).exists()
    assert _staging_left_behind(tmp_path) == []


def test_a_refused_download_reaches_the_caller(tmp_path, monkeypatch) -> None:
    _serve(monkeypatch, _Response(b"", refusal=requests.HTTPError("403 Forbidden")))

    with pytest.raises(requests.HTTPError):
        download_release(tmp_path)

    assert not (tmp_path / RELEASE_DIR_NAME).exists()
    assert _staging_left_behind(tmp_path) == []


def test_a_stalled_download_fails_rather_than_hanging(tmp_path, monkeypatch) -> None:
    """A body that stops arriving ends the read; the partial file never outlives the attempt."""
    body = _archive(_rows())
    _pin(monkeypatch, body)
    _serve(monkeypatch, _Response(body, stall=requests.ReadTimeout("no bytes for 120s")))

    with pytest.raises(requests.ReadTimeout):
        download_release(tmp_path)

    assert not (tmp_path / RELEASE_DIR_NAME).exists()
    assert _staging_left_behind(tmp_path) == []


def test_provenance_record_is_published_before_the_release(tmp_path, monkeypatch) -> None:
    """Interrupted the other way round, a cache is unusable forever; this way it heals itself."""
    body = _archive(_rows())
    digest = _pin(monkeypatch, body)
    _serve(monkeypatch, _Response(body))
    published = os.replace
    interrupted = [False]

    def replace(source: Any, destination: Any) -> None:
        if not interrupted[0] and Path(destination).name == RELEASE_DIR_NAME:
            interrupted[0] = True
            raise OSError("the machine went down mid-publish")
        published(source, destination)

    monkeypatch.setattr(data_module.os, "replace", replace)

    with pytest.raises(OSError, match="mid-publish"):
        download_release(tmp_path)
    assert (tmp_path / PROVENANCE_NAME).is_file()
    assert not (tmp_path / RELEASE_DIR_NAME).exists()

    # A record with no release beside it is not even reachable: the next run
    # sees no cache, downloads again, and overwrites the record it left.
    assert download_release(tmp_path) == tmp_path / RELEASE_DIR_NAME
    assert release_sha256(tmp_path / RELEASE_DIR_NAME) == digest
