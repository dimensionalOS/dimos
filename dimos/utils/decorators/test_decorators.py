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

from dimos.utils.decorators import RollingAverageAccumulator, limit, retry, simple_mcache


def test_limit() -> None:
    """Test limit decorator with keyword arguments."""
    calls = []

    @limit(20)  # 20 Hz
    def process(msg: str, keyword: int = 0) -> str:
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


def test_latest_rolling_average() -> None:
    """Test RollingAverageAccumulator with limit decorator."""
    calls = []

    accumulator = RollingAverageAccumulator()

    @limit(20, accumulator=accumulator)  # 20 Hz
    def process(value: float, label: str = "") -> str:
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


def test_retry_success_after_failures() -> None:
    """Test that retry decorator retries on failure and eventually succeeds."""
    attempts = []

    @retry(max_retries=3)
    def flaky_function(fail_times: int = 2) -> str:
        attempts.append(len(attempts))
        if len(attempts) <= fail_times:
            raise ValueError(f"Attempt {len(attempts)} failed")
        return "success"

    result = flaky_function()
    assert result == "success"
    assert len(attempts) == 3  # Failed twice, succeeded on third attempt


def test_retry_exhausted() -> None:
    """Test that retry decorator raises exception when retries are exhausted."""
    attempts = []

    @retry(max_retries=2)
    def always_fails():
        attempts.append(len(attempts))
        raise RuntimeError(f"Attempt {len(attempts)} failed")

    with pytest.raises(RuntimeError) as exc_info:
        always_fails()

    assert "Attempt 3 failed" in str(exc_info.value)
    assert len(attempts) == 3  # Initial attempt + 2 retries


def test_retry_specific_exception() -> None:
    """Test that retry only catches specified exception types."""
    attempts = []

    @retry(max_retries=3, on_exception=ValueError)
    def raises_different_exceptions() -> str:
        attempts.append(len(attempts))
        if len(attempts) == 1:
            raise ValueError("First attempt")
        elif len(attempts) == 2:
            raise TypeError("Second attempt - should not be retried")
        return "success"

    # Should fail on TypeError (not retried)
    with pytest.raises(TypeError) as exc_info:
        raises_different_exceptions()

    assert "Second attempt" in str(exc_info.value)
    assert len(attempts) == 2  # First attempt with ValueError, second with TypeError


def test_retry_no_failures() -> None:
    """Test that retry decorator works when function succeeds immediately."""
    attempts = []

    @retry(max_retries=5)
    def always_succeeds() -> str:
        attempts.append(len(attempts))
        return "immediate success"

    result = always_succeeds()
    assert result == "immediate success"
    assert len(attempts) == 1  # Only one attempt needed


def test_retry_with_delay() -> None:
    """Test that retry decorator applies delay between attempts."""
    attempts = []
    times = []

    @retry(max_retries=2, delay=0.1)
    def delayed_failures() -> str:
        times.append(time.time())
        attempts.append(len(attempts))
        if len(attempts) < 2:
            raise ValueError(f"Attempt {len(attempts)}")
        return "success"

    start = time.time()
    result = delayed_failures()
    duration = time.time() - start

    assert result == "success"
    assert len(attempts) == 2
    assert duration >= 0.1  # At least one delay occurred

    # Check that delays were applied
    if len(times) >= 2:
        assert times[1] - times[0] >= 0.1


def test_retry_zero_retries() -> None:
    """Test retry with max_retries=0 (no retries, just one attempt)."""
    attempts = []

    @retry(max_retries=0)
    def single_attempt():
        attempts.append(len(attempts))
        raise ValueError("Failed")

    with pytest.raises(ValueError):
        single_attempt()

    assert len(attempts) == 1  # Only the initial attempt


def test_retry_invalid_parameters() -> None:
    """Test that retry decorator validates parameters."""
    with pytest.raises(ValueError):

        @retry(max_retries=-1)
        def invalid_retries() -> None:
            pass

    with pytest.raises(ValueError):

        @retry(delay=-0.5)
        def invalid_delay() -> None:
            pass


def test_retry_with_methods() -> None:
    """Test that retry decorator works with class methods, instance methods, and static methods."""

    class TestClass:
        def __init__(self) -> None:
            self.instance_attempts = []
            self.instance_value = 42

        @retry(max_retries=3)
        def instance_method(self, fail_times: int = 2) -> str:
            """Test retry on instance method."""
            self.instance_attempts.append(len(self.instance_attempts))
            if len(self.instance_attempts) <= fail_times:
                raise ValueError(f"Instance attempt {len(self.instance_attempts)} failed")
            return f"instance success with value {self.instance_value}"

        @classmethod
        @retry(max_retries=2)
        def class_method(cls, attempts_list, fail_times: int = 1) -> str:
            """Test retry on class method."""
            attempts_list.append(len(attempts_list))
            if len(attempts_list) <= fail_times:
                raise ValueError(f"Class attempt {len(attempts_list)} failed")
            return f"class success from {cls.__name__}"

        @staticmethod
        @retry(max_retries=2)
        def static_method(attempts_list, fail_times: int = 1) -> str:
            """Test retry on static method."""
            attempts_list.append(len(attempts_list))
            if len(attempts_list) <= fail_times:
                raise ValueError(f"Static attempt {len(attempts_list)} failed")
            return "static success"

    # Test instance method
    obj = TestClass()
    result = obj.instance_method()
    assert result == "instance success with value 42"
    assert len(obj.instance_attempts) == 3  # Failed twice, succeeded on third

    # Test class method
    class_attempts = []
    result = TestClass.class_method(class_attempts)
    assert result == "class success from TestClass"
    assert len(class_attempts) == 2  # Failed once, succeeded on second

    # Test static method
    static_attempts = []
    result = TestClass.static_method(static_attempts)
    assert result == "static success"
    assert len(static_attempts) == 2  # Failed once, succeeded on second

    # Test that self is properly maintained across retries
    obj2 = TestClass()
    obj2.instance_value = 100
    result = obj2.instance_method()
    assert result == "instance success with value 100"
    assert len(obj2.instance_attempts) == 3


def test_simple_mcache() -> None:
    """Test simple_mcache decorator caches and can be invalidated."""
    call_count = 0

    class Counter:
        @simple_mcache
        def expensive(self) -> int:
            nonlocal call_count
            call_count += 1
            return call_count

    obj = Counter()

    # First call computes
    assert obj.expensive() == 1
    assert call_count == 1

    # Second call returns cached
    assert obj.expensive() == 1
    assert call_count == 1

    # Invalidate and call again
    obj.expensive.invalidate_cache(obj)
    assert obj.expensive() == 2
    assert call_count == 2

    # Cached again
    assert obj.expensive() == 2
    assert call_count == 2


def test_simple_mcache_separate_instances() -> None:
    """Test that simple_mcache caches per instance."""
    call_count = 0

    class Counter:
        @simple_mcache
        def expensive(self) -> int:
            nonlocal call_count
            call_count += 1
            return call_count

    obj1 = Counter()
    obj2 = Counter()

    assert obj1.expensive() == 1
    assert obj2.expensive() == 2  # separate cache
    assert obj1.expensive() == 1  # still cached
    assert call_count == 2

    # Invalidating one doesn't affect the other
    obj1.expensive.invalidate_cache(obj1)
    assert obj1.expensive() == 3
    assert obj2.expensive() == 2  # still cached
