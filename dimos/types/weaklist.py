# Copyright 2025 Dimensional Inc.
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

"""Weak reference list implementation that automatically removes dead references."""

from collections.abc import Iterator
from typing import Any
import weakref


class WeakList:
    """A list that holds weak references to objects.

    Objects are automatically removed when garbage collected.
    Supports iteration, append, remove, and length operations.
    """

    def __init__(self) -> None:
        self._refs = []  # type: ignore[var-annotated]

    def append(self, obj: Any) -> None:
        """Add an object to the list (stored as weak reference)."""

        def _cleanup(ref) -> None:  # type: ignore[no-untyped-def]
            try:
                self._refs.remove(ref)
            except ValueError:
                pass

        self._refs.append(weakref.ref(obj, _cleanup))

    def remove(self, obj: Any) -> None:
        """Remove an object from the list."""
        for i, ref in enumerate(self._refs):
            if ref() is obj:
                del self._refs[i]
                return
        raise ValueError(f"{obj} not in WeakList")

    def discard(self, obj: Any) -> None:
        """Remove an object from the list if present, otherwise do nothing."""
        try:
            self.remove(obj)
        except ValueError:
            pass

    def __iter__(self) -> Iterator[Any]:
        """Iterate over live objects, skipping dead references."""
        # Create a copy to avoid modification during iteration
        for ref in self._refs[:]:
            obj = ref()
            if obj is not None:
                yield obj

    def __len__(self) -> int:
        """Return count of live objects."""
        return sum(1 for _ in self)

    def __contains__(self, obj: Any) -> bool:
        """Check if object is in the list."""
        return any(ref() is obj for ref in self._refs)

    def clear(self) -> None:
        """Remove all references."""
        self._refs.clear()

    def __getitem__(self, index: int) -> Any:
        """Get object at index (only counting live objects)."""
        for i, obj in enumerate(self):
            if i == index:
                return obj
        raise IndexError("WeakList index out of range")

    def __repr__(self) -> str:
        return f"WeakList({list(self)})"
