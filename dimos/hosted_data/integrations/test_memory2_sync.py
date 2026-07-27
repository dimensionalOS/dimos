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

"""Tests for continuous memory2 snapshot publication."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.hosted_data.integrations import memory2_sync
from dimos.hosted_data.integrations.memory2_sync import ContinuousMemory2Publisher


def test_continuous_publisher_skips_unchanged_database(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database = tmp_path / "go2.db"
    database.write_bytes(b"sqlite")
    calls: list[Path] = []

    class Result:
        class Object:
            def to_dict(self) -> dict[str, str]:
                return {"object_id": "a" * 64}

        dataset_object = Object()
        index_object = Object()

    def fake_upload(**kwargs: object) -> Result:
        calls.append(Path(str(kwargs["path"])))
        return Result()

    monkeypatch.setattr(memory2_sync, "upload_memory2_dataset", fake_upload)
    publisher = ContinuousMemory2Publisher(
        path=database,
        server_url="https://replay.example",
        owner="alice",
        repository="go2",
        state_path=tmp_path / "state.json",
    )

    assert publisher.publish_if_changed() is not None
    assert publisher.publish_if_changed() is None
    database.write_bytes(b"changed")
    assert publisher.publish_if_changed() is not None
    assert calls == [database, database]


def test_continuous_publisher_validates_interval(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="at least"):
        ContinuousMemory2Publisher(
            path=tmp_path / "go2.db",
            server_url="https://replay.example",
            owner="alice",
            repository="go2",
            interval_seconds=0,
        )
