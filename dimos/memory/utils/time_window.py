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

"""Resolve one recording-relative window for all streams in a dataset."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, TypeVar

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

T = TypeVar("T")


@dataclass(frozen=True)
class RecordingWindow:
    """Absolute half-open interval shared by all streams in a recording."""

    start: float
    end: float

    @classmethod
    def from_store(
        cls, store: Store, *, seek: float = 0.0, duration: float | None = None
    ) -> RecordingWindow:
        """Resolve bounds from the earliest timestamp, including unselected streams."""
        if seek == 0.0 and duration is None:
            return cls(float("-inf"), float("inf"))
        first_ts = store.first_ts()
        start = (first_ts if first_ts is not None else 0.0) + seek
        end = start + duration if duration is not None else float("inf")
        return cls(start, end)

    def apply(self, stream: Stream[T]) -> Stream[T]:
        """Include the start timestamp and exclude the end without moving either bound."""
        # time_range includes both endpoints, while before excludes the end.
        return stream.time_range(self.start, self.end).before(self.end).order_by("ts")
