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
    ("seek", "duration", "expected_target", "expected_late"),
    [
        (0.25, 0.1, [100.25], []),
        (1.0, 0.5, [101.0, 101.25], [101.125, 101.25]),
        (0.0, 1.0, [100.25], []),
        (1.5, None, [101.5, 102.0], [101.5, 102.0]),
        (0.0, 0.0, [], []),
        (3.0, 1.0, [], []),
    ],
)
def test_window_uses_earliest_timestamp_across_streams(
    session: Store,
    seek: float,
    duration: float | None,
    expected_target: list[float],
    expected_late: list[float],
) -> None:
    store = session
    store.stream("empty", str)
    # Append out of order so insertion order cannot define the recording start.
    reference = store.stream("reference", str)
    reference.append("last", ts=200.0)
    reference.append("first", ts=100.0)
    target = store.stream("target", str)
    late = store.stream("late", str)
    for ts in (102.0, 100.25, 101.0, 101.25, 101.5):
        target.append(f"target-{ts}", ts=ts)
    for ts in (102.0, 101.125, 101.25, 101.5):
        late.append(f"late-{ts}", ts=ts)

    window = RecordingWindow.from_store(store, seek=seek, duration=duration)

    assert [obs.ts for obs in window.apply(target)] == expected_target
    assert [obs.ts for obs in window.apply(late)] == expected_late
    assert window.apply(store.stream("empty")).to_list() == []
