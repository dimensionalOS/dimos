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

"""Read-only Memory2 view bounded by one frozen timestamp."""

from __future__ import annotations

import math
from typing import Any, TypeVar, cast

from dimos.core.resource import CompositeResource
from dimos.memory2.store.base import Store, StreamAccessor
from dimos.memory2.stream import Stream

T = TypeVar("T")


class FrozenMemoryStore(CompositeResource):
    """Overlay source and derived stores while hiding observations after a cutoff."""

    def __init__(
        self,
        source: Store,
        *,
        through_timestamp: float,
        derived: Store | None = None,
    ) -> None:
        super().__init__()
        self.through_timestamp = through_timestamp
        self._source = self.register_disposable(source)
        self._derived = self.register_disposable(derived) if derived is not None else None
        self._streams: dict[str, Stream[Any]] = {}
        source_names = set(source.list_streams())
        derived_names = set(derived.list_streams()) if derived is not None else set()
        collisions = source_names & derived_names
        if collisions:
            names = ", ".join(sorted(collisions))
            raise ValueError(f"Source and derived stores contain overlapping streams: {names}")
        self._source_names = source_names
        self._derived_names = derived_names

    @property
    def streams(self) -> StreamAccessor[Stream[Any]]:
        return StreamAccessor(self)

    def list_streams(self) -> list[str]:
        return sorted(self._source_names | self._derived_names)

    def summary(self) -> str:
        """Describe only observations visible through the frozen boundary."""
        return "\n".join(stream.summary() for _, stream in self.streams.items())

    def stream(self, name: str, payload_type: type[T] | None = None) -> Stream[T]:
        if payload_type is not None:
            raise TypeError("Frozen memory streams cannot be created or retyped")
        if name not in self.list_streams():
            raise KeyError(f"No stream {name!r}. Available: {self.list_streams()}")
        if name not in self._streams:
            store = self._source if name in self._source_names else self._derived
            assert store is not None
            self._streams[name] = store.stream(name).time_range(-math.inf, self.through_timestamp)
        return cast("Stream[T]", self._streams[name])

    def delete_stream(self, name: str) -> None:
        raise PermissionError("Cannot delete streams from frozen memory")

    def stop(self) -> None:
        for stream in self._streams.values():
            stream.stop()
        super().stop()
