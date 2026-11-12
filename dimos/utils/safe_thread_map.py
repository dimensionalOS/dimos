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
from __future__ import annotations

from collections.abc import Callable, Sequence
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
import sys
from typing import Any, TypeVar

if sys.version_info < (3, 11):

    class ExceptionGroup(Exception):  # noqa: N818
        """Minimal ExceptionGroup polyfill for Python 3.10."""

        exceptions: tuple[BaseException, ...]

        def __init__(self, message: str, exceptions: Sequence[BaseException]) -> None:
            super().__init__(message)
            self.exceptions = tuple(exceptions)
else:
    import builtins

    ExceptionGroup = builtins.ExceptionGroup

T = TypeVar("T")
R = TypeVar("R")


def safe_thread_map(
    items: Sequence[T],
    fn: Callable[[T], R],
    on_errors: Callable[[list[tuple[T, R | Exception]], list[R], list[Exception]], Any]
    | None = None,
) -> list[R]:
    """Thread-pool map that waits for all items to finish before raising and a cleanup handler

    - Empty *items* → returns ``[]`` immediately.
    - All succeed → returns results in input order.
    - Any fail → calls ``on_errors(outcomes, successes, errors)`` where
      *outcomes* is a list of ``(input, result_or_exception)`` pairs in input
      order, *successes* is the list of successful results, and *errors* is
      the list of exceptions. If *on_errors* raises, that exception propagates.
      If *on_errors* returns normally, its return value is returned from
      ``safe_thread_map``. If *on_errors* is ``None``, raises an
      ``ExceptionGroup``.
    """
    if not items:
        return []

    outcomes: dict[int, R | Exception] = {}

    with ThreadPoolExecutor(max_workers=len(items)) as pool:
        futures: dict[Future[R], int] = {pool.submit(fn, item): i for i, item in enumerate(items)}
        for fut in as_completed(futures):
            idx = futures[fut]
            try:
                outcomes[idx] = fut.result()
            except Exception as e:
                outcomes[idx] = e

    successes: list[R] = []
    errors: list[Exception] = []
    for v in outcomes.values():
        if isinstance(v, Exception):
            errors.append(v)
        else:
            successes.append(v)

    if errors:
        if on_errors is not None:
            zipped = [(items[i], outcomes[i]) for i in range(len(items))]
            return on_errors(zipped, successes, errors)  # type: ignore[no-any-return]
        raise ExceptionGroup("safe_thread_map failed", errors)

    return [outcomes[i] for i in range(len(items))]  # type: ignore[misc]
