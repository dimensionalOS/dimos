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

"""Tests for backpressure buffers."""

from __future__ import annotations

import threading
import time

import pytest

from dimos.memory2.buffer import Bounded, ClosedError, DropNew, KeepLast, Unbounded


class TestBackpressureBuffers:
    """Thread-safe buffers bridging push sources to pull consumers."""

    def test_keep_last_overwrites(self):
        buf = KeepLast[int]()
        buf.put(1)
        buf.put(2)
        buf.put(3)
        assert buf.take() == 3
        assert len(buf) == 0

    def test_bounded_drops_oldest(self):
        buf = Bounded[int](maxlen=2)
        buf.put(1)
        buf.put(2)
        buf.put(3)  # drops 1
        assert buf.take() == 2
        assert buf.take() == 3

    def test_drop_new_rejects(self):
        buf = DropNew[int](maxlen=2)
        assert buf.put(1) is True
        assert buf.put(2) is True
        assert buf.put(3) is False  # rejected
        assert buf.take() == 1
        assert buf.take() == 2

    def test_unbounded_keeps_all(self):
        buf = Unbounded[int]()
        for i in range(100):
            buf.put(i)
        assert len(buf) == 100

    def test_close_signals_end(self):
        buf = KeepLast[int]()
        buf.close()
        with pytest.raises(ClosedError):
            buf.take()

    def test_buffer_is_iterable(self):
        """Iterating a buffer yields items until closed."""
        buf = Unbounded[int]()
        buf.put(1)
        buf.put(2)
        buf.close()
        assert list(buf) == [1, 2]

    def test_take_blocks_until_put(self):
        buf = KeepLast[int]()
        result = []

        def producer():
            time.sleep(0.05)
            buf.put(42)

        t = threading.Thread(target=producer)
        t.start()
        result.append(buf.take(timeout=2.0))
        t.join()
        assert result == [42]
