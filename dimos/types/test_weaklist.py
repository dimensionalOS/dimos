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

"""Tests for WeakList implementation."""

import gc

import pytest

from dimos.types.weaklist import WeakList


class SampleObject:
    """Simple test object."""

    def __init__(self, value) -> None:
        self.value = value

    def __repr__(self) -> str:
        return f"SampleObject({self.value})"


def test_weaklist_basic_operations() -> None:
    """Test basic append, iterate, and length operations."""
    wl = WeakList()

    # Add objects
    obj1 = SampleObject(1)
    obj2 = SampleObject(2)
    obj3 = SampleObject(3)

    wl.append(obj1)
    wl.append(obj2)
    wl.append(obj3)

    # Check length and iteration
    assert len(wl) == 3
    assert list(wl) == [obj1, obj2, obj3]

    # Check contains
    assert obj1 in wl
    assert obj2 in wl
    assert SampleObject(4) not in wl


def test_weaklist_auto_removal() -> None:
    """Test that objects are automatically removed when garbage collected."""
    wl = WeakList()

    obj1 = SampleObject(1)
    obj2 = SampleObject(2)
    obj3 = SampleObject(3)

    wl.append(obj1)
    wl.append(obj2)
    wl.append(obj3)

    assert len(wl) == 3

    # Delete one object and force garbage collection
    del obj2
    gc.collect()

    # Should only have 2 objects now
    assert len(wl) == 2
    assert list(wl) == [obj1, obj3]


def test_weaklist_explicit_remove() -> None:
    """Test explicit removal of objects."""
    wl = WeakList()

    obj1 = SampleObject(1)
    obj2 = SampleObject(2)

    wl.append(obj1)
    wl.append(obj2)

    # Remove obj1
    wl.remove(obj1)
    assert len(wl) == 1
    assert obj1 not in wl
    assert obj2 in wl

    # Try to remove non-existent object
    with pytest.raises(ValueError):
        wl.remove(SampleObject(3))


def test_weaklist_indexing() -> None:
    """Test index access."""
    wl = WeakList()

    obj1 = SampleObject(1)
    obj2 = SampleObject(2)
    obj3 = SampleObject(3)

    wl.append(obj1)
    wl.append(obj2)
    wl.append(obj3)

    assert wl[0] is obj1
    assert wl[1] is obj2
    assert wl[2] is obj3

    # Test index out of range
    with pytest.raises(IndexError):
        _ = wl[3]


def test_weaklist_clear() -> None:
    """Test clearing the list."""
    wl = WeakList()

    obj1 = SampleObject(1)
    obj2 = SampleObject(2)

    wl.append(obj1)
    wl.append(obj2)

    assert len(wl) == 2

    wl.clear()
    assert len(wl) == 0
    assert obj1 not in wl


def test_weaklist_iteration_during_modification() -> None:
    """Test that iteration works even if objects are deleted during iteration."""
    wl = WeakList()

    objects = [SampleObject(i) for i in range(5)]
    for obj in objects:
        wl.append(obj)

    # Verify initial state
    assert len(wl) == 5

    # Iterate and check that we can safely delete objects
    seen_values = []
    for obj in wl:
        seen_values.append(obj.value)
        if obj.value == 2:
            # Delete another object (not the current one)
            del objects[3]  # Delete SampleObject(3)
            gc.collect()

    # The object with value 3 gets garbage collected during iteration
    # so we might not see it (depends on timing)
    assert len(seen_values) in [4, 5]
    assert all(v in [0, 1, 2, 3, 4] for v in seen_values)

    # After iteration, the list should have 4 objects (one was deleted)
    assert len(wl) == 4
