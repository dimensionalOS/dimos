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
this repository.
"""

import json
from pathlib import Path
from typing import Any

import pytest

import dimos.benchmark.space_qa.data as data_module
from dimos.benchmark.space_qa.data import (
    PROVENANCE_NAME,
    RELEASE_DIR_NAME,
    SPACE_DATA_ENV,
    load_task_rows,
    release_sha256,
    resolve_space_data,
)
from dimos.benchmark.space_qa.tasks import SpaceTextTask

TASK = SpaceTextTask(name="FAKE_text", groups=3)


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


def test_a_task_reads_back_every_row_in_upstream_order(tmp_path) -> None:
    rows = _rows()

    assert load_task_rows(TASK, _release(tmp_path, rows)) == rows


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
    monkeypatch.delenv(SPACE_DATA_ENV, raising=False)
    monkeypatch.setattr(data_module, "space_cache_root", lambda: tmp_path)
    monkeypatch.setattr(
        data_module, "download_release", lambda _target: pytest.fail("downloaded a cached release")
    )

    assert resolve_space_data() == release


def test_only_a_release_we_unpacked_ourselves_reports_a_digest(tmp_path) -> None:
    """An overridden release was never verified here and must not borrow the pin."""
    release = tmp_path / RELEASE_DIR_NAME
    release.mkdir()
    assert release_sha256(release) is None

    (tmp_path / PROVENANCE_NAME).write_text(json.dumps({"sha256": "abc123"}), encoding="utf-8")
    assert release_sha256(release) == "abc123"
