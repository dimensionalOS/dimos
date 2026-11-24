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

"""One seek/duration window shared by every stream of a recording."""

from __future__ import annotations

from dataclasses import dataclass
from math import inf
from typing import TYPE_CHECKING, TypeVar

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

T = TypeVar("T")


@dataclass(frozen=True)
class RecordingWindow:
    """Absolute interval [start, end) that every stream of a recording is cut to."""

    start: float = -inf
    end: float = inf

    @classmethod
    def from_store(cls, store: Store, seek: float, duration: float | None) -> RecordingWindow:
        """Measure seek and duration from the recording start, as ``Store.replay`` does."""
        if not seek and duration is None:
            return cls()
        first_ts = store.replay().first_ts()
        if first_ts is None:
            return cls()
        start = first_ts + seek
        return cls(start, inf if duration is None else start + duration)

    def apply(self, stream: Stream[T]) -> Stream[T]:
        if self == RecordingWindow():
            return stream
        # time_range includes both ends. before() makes the end exclusive.
        return stream.time_range(self.start, self.end).before(self.end)
