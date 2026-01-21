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
"""Tests for SensorStore implementations."""

from pathlib import Path
import tempfile

import pytest

from dimos.memory.sensor.base import InMemoryStore, SensorStore
from dimos.memory.sensor.pickledir import PickleDirStore
from dimos.memory.sensor.sqlite import SqliteStore


@pytest.fixture
def temp_dir():
    """Create a temporary directory for file-based store tests."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield tmpdir


def make_in_memory_store() -> SensorStore[str]:
    return InMemoryStore[str]()


def make_pickle_dir_store(tmpdir: str) -> SensorStore[str]:
    return PickleDirStore[str](tmpdir)


def make_sqlite_store(tmpdir: str) -> SensorStore[str]:
    return SqliteStore[str](Path(tmpdir) / "test.db")


@pytest.mark.parametrize(
    "store_factory,store_name",
    [
        (lambda _: make_in_memory_store(), "InMemoryStore"),
        (lambda tmpdir: make_pickle_dir_store(tmpdir), "PickleDirStore"),
        (lambda tmpdir: make_sqlite_store(tmpdir), "SqliteStore"),
    ],
)
class TestSensorStore:
    """Parametrized tests for all SensorStore implementations."""

    def test_save_and_load(self, store_factory, store_name, temp_dir):
        store = store_factory(temp_dir)
        store.save("data_at_1", 1.0)
        store.save("data_at_2", 2.0)

        assert store.load(1.0) == "data_at_1"
        assert store.load(2.0) == "data_at_2"
        assert store.load(3.0) is None

    def test_find_closest_timestamp(self, store_factory, store_name, temp_dir):
        store = store_factory(temp_dir)
        store.save("a", 1.0)
        store.save("b", 2.0)
        store.save("c", 3.0)

        # Exact match
        assert store._find_closest_timestamp(2.0) == 2.0

        # Closest to 1.4 is 1.0
        assert store._find_closest_timestamp(1.4) == 1.0

        # Closest to 1.6 is 2.0
        assert store._find_closest_timestamp(1.6) == 2.0

        # With tolerance
        assert store._find_closest_timestamp(1.4, tolerance=0.5) == 1.0
        assert store._find_closest_timestamp(1.4, tolerance=0.3) is None

    def test_iter_items(self, store_factory, store_name, temp_dir):
        store = store_factory(temp_dir)
        store.save("a", 1.0)
        store.save("c", 3.0)
        store.save("b", 2.0)

        # Should iterate in timestamp order
        items = list(store._iter_items())
        assert items == [(1.0, "a"), (2.0, "b"), (3.0, "c")]

    def test_iter_items_with_range(self, store_factory, store_name, temp_dir):
        store = store_factory(temp_dir)
        store.save("a", 1.0)
        store.save("b", 2.0)
        store.save("c", 3.0)
        store.save("d", 4.0)

        # Start only
        items = list(store._iter_items(start=2.0))
        assert items == [(2.0, "b"), (3.0, "c"), (4.0, "d")]

        # End only
        items = list(store._iter_items(end=3.0))
        assert items == [(1.0, "a"), (2.0, "b")]

        # Both
        items = list(store._iter_items(start=2.0, end=4.0))
        assert items == [(2.0, "b"), (3.0, "c")]

    def test_empty_store(self, store_factory, store_name, temp_dir):
        store = store_factory(temp_dir)

        assert store.load(1.0) is None
        assert store._find_closest_timestamp(1.0) is None
        assert list(store._iter_items()) == []
