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

import threading

import pytest

from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map


def test_empty_returns_empty() -> None:
    assert safe_thread_map([], lambda x: x) == []


def test_all_succeed_preserves_order() -> None:
    items = [3, 1, 2]
    result = safe_thread_map(items, lambda x: x * 10)
    assert result == [30, 10, 20]


def test_all_succeed_runs_every_item() -> None:
    seen: list[int] = []
    safe_thread_map([1, 2, 3], lambda x: seen.append(x))
    assert sorted(seen) == [1, 2, 3]


def test_single_failure_raises_exceptiongroup() -> None:
    def boom(x: int) -> int:
        raise ValueError("bad")

    with pytest.raises(ExceptionGroup):
        safe_thread_map([1, 2, 3], boom)


def test_failure_invokes_on_errors_with_outcomes() -> None:
    def boom(x: int) -> int:
        if x == 2:
            raise RuntimeError("two")
        return x * 10

    captured: dict[str, object] = {}

    def on_errors(outcomes, successes, errors):
        captured["outcomes"] = outcomes
        captured["successes"] = successes
        captured["errors"] = errors
        return "handled"

    result = safe_thread_map([1, 2, 3], boom, on_errors=on_errors)
    assert result == "handled"
    # successes is collected in completion order, not input order, so compare unordered.
    assert sorted(captured["successes"]) == [10, 30]  # type: ignore[misc]
    errors = captured["errors"]
    assert isinstance(errors, list)
    assert len(errors) == 1
    assert isinstance(errors[0], RuntimeError)
    assert [item[0] for item in captured["outcomes"]] == [1, 2, 3]  # type: ignore[union-attr]
    assert isinstance(captured["outcomes"][1][1], RuntimeError)  # type: ignore[union-attr]


def test_multiple_failures_exceptiongroup_contains_all() -> None:
    def boom(x: int) -> int:
        raise ValueError(str(x))

    with pytest.raises(ExceptionGroup) as exc:
        safe_thread_map([1, 2, 3], boom)
    messages = {str(error) for error in exc.value.exceptions}
    assert messages == {"1", "2", "3"}


def test_on_errors_return_value_propagates() -> None:
    def on_errors(outcomes, successes, errors):
        return ("fallback",)

    result = safe_thread_map([0], lambda x: 1 / x, on_errors=on_errors)
    assert result == ("fallback",)


def test_worker_concurrency() -> None:
    # Use a start/release barrier so both workers are provably active at the
    # same time. Without real concurrency the peak would only ever reach 1.
    started = threading.Event()
    release = threading.Event()
    active: dict[str, int] = {"n": 0}
    peak: dict[str, int] = {"n": 0}
    lock = threading.Lock()

    def work(x: int) -> int:
        with lock:
            active["n"] += 1
            if active["n"] > peak["n"]:
                peak["n"] = active["n"]
        if x == 0:
            started.set()  # first worker has entered
            release.wait(5.0)  # block until the other worker releases it
        elif x == 1:
            started.wait(5.0)  # ensure the first worker entered first
            release.set()  # then release both at once
        with lock:
            active["n"] -= 1
        return x

    out = safe_thread_map([0, 1], work)
    assert out == [0, 1]
    # Both workers were simultaneously active -> true concurrency.
    assert peak["n"] == 2
    assert active["n"] == 0
