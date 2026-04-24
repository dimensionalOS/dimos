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

"""Tests for NullStore and max_size=0 discard behavior."""

from __future__ import annotations

from dimos.memory2.store.null import NullStore


def test_max_size_zero_monotonic_ids() -> None:
    """NullStore assigns monotonically increasing IDs despite discarding data."""
    store = NullStore()
    with store:
        stream = store.stream("test", str)
        obs0 = stream.append("hello")
        obs1 = stream.append("world")
        obs2 = stream.append("!")

        assert obs0.id == 0
        assert obs1.id == 1
        assert obs2.id == 2


def test_max_size_zero_empty_query() -> None:
    """NullStore queries always return empty."""
    store = NullStore()
    with store:
        stream = store.stream("test", str)
        stream.append("data")
        assert stream.count() == 0
        assert stream.to_list() == []


def test_null_store_discards_history() -> None:
    """NullStore discards history but still supports live streaming."""
    store = NullStore()
    with store:
        stream = store.stream("test", int)
        stream.append(1)
        stream.append(2)
        stream.append(3)

        assert stream.count() == 0
        assert stream.to_list() == []
