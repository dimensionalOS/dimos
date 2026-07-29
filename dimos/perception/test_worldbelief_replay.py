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

from pathlib import Path

import pytest

from dimos.perception.worldbelief_replay import _resolve_recording_path


def test_resolve_explicit_worldbelief_db(tmp_path: Path) -> None:
    recording = tmp_path / "recording.db"
    recording.touch()

    assert _resolve_recording_path(recording) == recording


def test_resolve_worldbelief_dataset_directory(tmp_path: Path) -> None:
    recording = tmp_path / "session.db"
    recording.touch()

    assert _resolve_recording_path(tmp_path) == recording


@pytest.mark.parametrize("count", [0, 2])
def test_worldbelief_dataset_directory_requires_one_db(tmp_path: Path, count: int) -> None:
    for index in range(count):
        (tmp_path / f"session_{index}.db").touch()

    with pytest.raises(ValueError, match="exactly one .db"):
        _resolve_recording_path(tmp_path)


def test_worldbelief_replay_rejects_non_db_file(tmp_path: Path) -> None:
    recording = tmp_path / "recording.mcap"
    recording.touch()

    with pytest.raises(ValueError, match=r"expects a Memory2 \.db recording"):
        _resolve_recording_path(recording)
