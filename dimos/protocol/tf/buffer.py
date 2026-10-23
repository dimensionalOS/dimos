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


"""Bounded timestamp index for generated transforms; frame search stays in TF."""

from copy import deepcopy
import math
from typing import cast

from sortedcontainers import SortedDict  # type: ignore[import-untyped]

from dimos.message_utils.time import time_from_seconds, to_nanoseconds, to_seconds
from dimos.messages.geometry_msgs.msg import TransformStamped
from dimos.types.timestamped import to_human_readable


class TBuffer:
    """A bounded time index of generated transforms, keyed by integer nanoseconds."""

    def __init__(self, buffer_size: float = 10.0) -> None:
        if math.isnan(buffer_size) or buffer_size <= 0:
            raise ValueError("buffer_size must be positive")
        self.buffer_size = buffer_size
        self._entries: SortedDict = SortedDict()

    def __len__(self) -> int:
        return len(self._entries)

    def add(self, transform: TransformStamped) -> None:
        stamp = to_nanoseconds(transform.header.stamp)
        self._entries[stamp] = deepcopy(transform)
        if math.isfinite(self.buffer_size):
            oldest = self._entries.peekitem(-1)[0] - round(self.buffer_size * 1_000_000_000)
            while self._entries and self._entries.peekitem(0)[0] < oldest:
                self._entries.popitem(0)

    def first(self) -> TransformStamped | None:
        return deepcopy(self._entries.peekitem(0)[1]) if self._entries else None

    def last(self) -> TransformStamped | None:
        return deepcopy(self._entries.peekitem(-1)[1]) if self._entries else None

    def get(
        self, time_point: float | None = None, time_tolerance: float = 1.0
    ) -> TransformStamped | None:
        if time_point is None:
            return self.last()
        stamp = to_nanoseconds(time_from_seconds(time_point))
        if math.isnan(time_tolerance) or time_tolerance < 0:
            raise ValueError("time_tolerance must be nonnegative")
        index = self._entries.bisect_left(stamp)
        candidates = [
            self._entries.peekitem(i) for i in (index - 1, index) if 0 <= i < len(self._entries)
        ]
        if not candidates:
            return None
        nearest, result = min(candidates, key=lambda item: (abs(item[0] - stamp), -item[0]))
        if abs(nearest - stamp) / 1_000_000_000 > time_tolerance:
            return None
        return deepcopy(cast("TransformStamped", result))

    def __str__(self) -> str:
        first, last = self.first(), self.last()
        if first is None or last is None:
            return "TBuffer(empty)"
        start, end = to_seconds(first.header.stamp), to_seconds(last.header.stamp)
        return (
            f"TBuffer({first.header.frame_id} -> {first.child_frame_id}, "
            f"{len(self)} msgs, {end - start:.2f}s "
            f"[{to_human_readable(start)} - {to_human_readable(end)}])"
        )
