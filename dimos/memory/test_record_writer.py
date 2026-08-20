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

from typing import Any, cast

import pytest

from dimos.memory.backend import Backend, PreparedAppend
from dimos.memory.record_writer import RecordWriter
from dimos.memory.recorder_queue import RecorderFailedError
from dimos.memory.type.observation import Observation


class _Backend:
    def __init__(self, events: list[str], *, fail: bool = False) -> None:
        self.events = events
        self.fail = fail

    @property
    def name(self) -> str:
        return "camera"

    def persist_prepared(self, prepared: PreparedAppend[Any]) -> Observation[Any]:
        self.events.append(f"persist-{prepared.observation.ts}")
        if self.fail:
            raise OSError("database unavailable")
        return prepared.observation

    def commit(self) -> None:
        self.events.append("commit")

    def rollback(self) -> None:
        self.events.append("rollback")

    def notify(self, observation: Observation[Any]) -> None:
        self.events.append(f"notify-{observation.ts}")


def _prepared(ts: float, encoded: bytes | None = None) -> PreparedAppend[Any]:
    return PreparedAppend(Observation(ts=ts, _data=ts), encoded)


def test_writer_groups_rows_and_notifies_only_after_commit() -> None:
    events: list[str] = []
    backend = cast("Backend[Any]", _Backend(events))
    writer = RecordWriter(max_rows=3, max_delay_s=1.0)

    for ts in (1.0, 2.0, 3.0):
        writer.submit(backend, _prepared(ts, b"data"))
    writer.close(timeout_s=1.0)

    assert events == [
        "persist-1.0",
        "persist-2.0",
        "persist-3.0",
        "commit",
        "notify-1.0",
        "notify-2.0",
        "notify-3.0",
    ]


def test_writer_rolls_back_and_reports_failure() -> None:
    events: list[str] = []
    backend = cast("Backend[Any]", _Backend(events, fail=True))
    writer = RecordWriter(max_rows=1, max_delay_s=0.01)
    writer.submit(backend, _prepared(1.0))

    with pytest.raises(RecorderFailedError, match="failed"):
        writer.close(timeout_s=1.0)

    assert events == ["persist-1.0", "rollback"]
