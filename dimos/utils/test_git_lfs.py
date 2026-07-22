# Copyright 2025-2026 Dimensional Inc.
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

import json
from pathlib import Path
import subprocess
from unittest.mock import Mock

import pytest

from dimos.utils import _git_lfs


def test_get_committed_file_sha256_reads_matching_entry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    oid = "abcdef01" * 8
    relative_name = "data/.lfs/archive.tar.gz"
    archive = tmp_path / relative_name
    run_mock = Mock(
        return_value=Mock(
            stdout=json.dumps(
                {"files": [{"name": relative_name, "oid_type": "sha256", "oid": oid}]}
            )
        )
    )
    monkeypatch.setattr(_git_lfs.subprocess, "run", run_mock)

    assert _git_lfs.get_committed_file_sha256(archive, tmp_path) == oid
    run_mock.assert_called_once_with(
        [
            "git",
            "lfs",
            "ls-files",
            "--json",
            "--include=data/.lfs/archive.tar.gz",
            "HEAD",
        ],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )


def test_get_committed_file_sha256_reports_command_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    failure = subprocess.CalledProcessError(128, ["git", "lfs"], stderr="missing file")
    monkeypatch.setattr(_git_lfs.subprocess, "run", Mock(side_effect=failure))

    with pytest.raises(RuntimeError, match="Failed to list committed Git LFS file"):
        _git_lfs.get_committed_file_sha256(tmp_path / "archive.tar.gz", tmp_path)


@pytest.mark.parametrize(
    ("response", "message"),
    [
        ("not json", "Malformed Git LFS JSON response"),
        ({"files": []}, "exactly one"),
        (
            {"files": [{"name": "other.tar.gz", "oid_type": "sha256", "oid": "a" * 64}]},
            "name mismatch",
        ),
        (
            {"files": [{"name": "archive.tar.gz", "oid_type": "sha1", "oid": "a" * 64}]},
            "unsupported oid_type",
        ),
        (
            {"files": [{"name": "archive.tar.gz", "oid_type": "sha256", "oid": "not-an-oid"}]},
            "invalid sha256 oid",
        ),
        ({"files": [{"name": "archive.tar.gz"}]}, "Malformed Git LFS JSON response"),
    ],
)
def test_get_committed_file_sha256_reports_invalid_response(
    response: str | dict[str, object],
    message: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        _git_lfs.subprocess,
        "run",
        Mock(
            return_value=Mock(
                stdout=response if isinstance(response, str) else json.dumps(response)
            )
        ),
    )

    with pytest.raises(RuntimeError, match=message):
        _git_lfs.get_committed_file_sha256(tmp_path / "archive.tar.gz", tmp_path)
