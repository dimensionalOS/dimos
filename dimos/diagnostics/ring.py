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

"""Bounded compressed trace chunks, used only by the observer worker."""

from collections import deque
from collections.abc import Iterator
import zlib


class TraceRing:
    def __init__(self, limit: int) -> None:
        self.limit = limit
        self.chunks: deque[tuple[float, float, bytes, int]] = deque()
        self.pending = bytearray()
        self.first = self.last = 0.0
        self.compressed_bytes = 0
        self.raw_bytes = 0
        self.evictions = 0
        self.capacity_lost_until = 0.0

    @property
    def size(self) -> int:
        # Include conservative allocation/tuple/deque overhead, not only payload bytes.
        return self.compressed_bytes + 256 * len(self.chunks) + 2 * len(self.pending)

    def append(self, at: float, row: bytes) -> None:
        if self.pending and (len(self.pending) + len(row) > 65536 or at - self.first >= 0.1):
            self.flush()
        if not self.pending:
            self.first = at
        self.last = at
        self.pending.extend(row)
        self.raw_bytes += len(row)

    def flush(self) -> None:
        if self.pending:
            packed = zlib.compress(self.pending, 1)
            self.chunks.append((self.first, self.last, packed, len(self.pending)))
            self.compressed_bytes += len(packed)
            self.pending.clear()

    def trim(self, cutoff: float) -> None:
        if self.size > self.limit:
            self.flush()
        while self.chunks and (self.size > self.limit or self.chunks[0][1] < cutoff):
            capacity = self.size > self.limit
            _, last, row, raw = self.chunks.popleft()
            self.compressed_bytes -= len(row)
            self.raw_bytes -= raw
            self.evictions += 1
            if capacity:
                self.capacity_lost_until = last

    def rows(self) -> Iterator[bytes]:
        for _, _, packed, _ in self.chunks:
            yield zlib.decompress(packed)
        if self.pending:
            yield bytes(self.pending)

    def clear(self) -> None:
        self.chunks.clear()
        self.pending.clear()
        self.compressed_bytes = self.raw_bytes = 0
