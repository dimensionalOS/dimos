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

import pytest

from dimos.memory.store.base import Store
from dimos.memory.utils.time_window import RecordingWindow


@pytest.mark.parametrize(
    ("seek", "duration", "expected_early", "expected_late"),
    [
        (1.0, 0.5, [101.0, 101.25], [101.25]),
        (0.0, 1.0, [100.0, 100.5], []),
        (1.5, None, [101.5, 102.0], [101.5, 102.0]),
        (0.0, 0.0, [], []),
        (3.0, 1.0, [], []),
    ],
)
def test_window_is_measured_from_the_recording_start(
    session: Store,
    seek: float,
    duration: float | None,
    expected_early: list[float],
    expected_late: list[float],
) -> None:
    early = session.stream("early", str)
    late = session.stream("late", str)
    for ts in (100.0, 100.5, 101.0, 101.25, 101.5, 102.0):
        early.append("early", ts=ts)
    for ts in (101.25, 101.5, 102.0):
        late.append("late", ts=ts)

    window = RecordingWindow.from_store(session, seek, duration)

    assert [obs.ts for obs in window.apply(early)] == expected_early
    assert [obs.ts for obs in window.apply(late)] == expected_late


def test_no_seek_or_duration_leaves_the_stream_untouched(session: Store) -> None:
    stream = session.stream("lidar", str)
    stream.append("scan", ts=100.0)

    assert RecordingWindow.from_store(session, 0.0, None).apply(stream) is stream


def test_empty_recording_has_no_window(session: Store) -> None:
    stream = session.stream("lidar", str)

    assert RecordingWindow.from_store(session, 1.0, 2.0).apply(stream) is stream
