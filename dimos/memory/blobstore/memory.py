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

from collections import OrderedDict
import threading
from typing import Any

from dimos.memory.blobstore.base import BlobStore, BlobStoreConfig


class MemoryBlobStoreConfig(BlobStoreConfig):
    max_items: int | None = None


class MemoryBlobStore(BlobStore):
    """Encoded blobs held in RAM, one insertion-ordered map per stream.

    ``max_items`` keeps a rolling window of the newest blobs per stream, the
    counterpart of ``ListObservationStore.max_size``: observation ids are
    monotonic, so both evict the same oldest rows.
    """

    config: MemoryBlobStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._blobs: dict[str, OrderedDict[int, bytes]] = {}
        self._lock = threading.Lock()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def put(self, stream_name: str, key: int, data: bytes) -> None:
        with self._lock:
            blobs = self._blobs.setdefault(stream_name, OrderedDict())
            blobs[key] = data
            blobs.move_to_end(key)
            limit = self.config.max_items
            while limit is not None and len(blobs) > limit:
                blobs.popitem(last=False)

    def get(self, stream_name: str, key: int) -> bytes:
        with self._lock:
            try:
                return self._blobs[stream_name][key]
            except KeyError:
                raise KeyError(f"No blob for stream={stream_name!r}, key={key}") from None

    def delete(self, stream_name: str, key: int) -> None:
        with self._lock:
            try:
                del self._blobs[stream_name][key]
            except KeyError:
                raise KeyError(f"No blob for stream={stream_name!r}, key={key}") from None

    def size_bytes(self, stream_name: str) -> int | None:
        with self._lock:
            return sum(len(b) for b in self._blobs.get(stream_name, {}).values())
