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

import time

import pytest

from dimos.utils.decorators import LatestAccumulator, RollingAverageAccumulator, limit


def test_limit():
    """Test limit decorator with keyword arguments."""
    calls = []

    @limit(20)  # 20 Hz
    def process(msg: str, keyword: int = 0):
        calls.append((msg, keyword))
        return f"{msg}:{keyword}"

    # First call goes through
    result1 = process("first", keyword=1)
    assert result1 == "first:1"
    assert calls == [("first", 1)]

    # Quick calls get accumulated
    result2 = process("second", keyword=2)
    assert result2 is None

    result3 = process("third", keyword=3)
    assert result3 is None

    # Wait for interval, expect to be called after it passes
    time.sleep(0.6)

    result4 = process("fourth")
    assert result4 == "fourth:0"

    assert calls == [("first", 1), ("third", 3), ("fourth", 0)]


def test_latest_rolling_average():
    """Test RollingAverageAccumulator with limit decorator."""
    calls = []

    accumulator = RollingAverageAccumulator()

    @limit(20, accumulator=accumulator)  # 20 Hz
    def process(value: float, label: str = ""):
        calls.append((value, label))
        return f"{value}:{label}"

    # First call goes through
    result1 = process(10.0, label="first")
    assert result1 == "10.0:first"
    assert calls == [(10.0, "first")]

    # Quick calls get accumulated
    result2 = process(20.0, label="second")
    assert result2 is None

    result3 = process(30.0, label="third")
    assert result3 is None

    # Wait for interval
    time.sleep(0.6)

    # Should see the average of accumulated values
    assert calls == [(10.0, "first"), (25.0, "third")]  # (20+30)/2 = 25
