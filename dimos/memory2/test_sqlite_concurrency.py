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

"""Regression tests for SqliteStore concurrent access (issue #2233)."""

from __future__ import annotations

from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile

import pytest

from dimos.memory2.store.sqlite import SqliteStore


@pytest.fixture
def populated_store() -> SqliteStore:
    """Single-threaded write of N obs, returned for concurrent reading."""
    tmp = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
    tmp.close()
    store = SqliteStore(path=tmp.name)
    s = store.stream("color_image", bytes)
    for i in range(855):  # mirror the issue's go2_short.db count
        s.append(b"frame", ts=float(i))
    yield store
    store.stop()
    Path(tmp.name).unlink(missing_ok=True)


def test_concurrent_count_is_consistent(populated_store: SqliteStore) -> None:
    """Many threads hammering count() on a shared conn must all see the same total.

    Regression for #2233 — without per-connection locking, concurrent count()
    calls on a single shared sqlite3.Connection returned 0 or raised TypeError.
    """
    stream = populated_store.stream("color_image")
    expected = stream.count()
    assert expected == 855

    threads, calls = 16, 250

    def worker() -> Counter[str]:
        tally: Counter[str] = Counter()
        for _ in range(calls):
            try:
                n = stream.count()
                tally["ok" if n == expected else f"wrong({n})"] += 1
            except Exception as e:
                tally[f"error:{type(e).__name__}"] += 1
        return tally

    total: Counter[str] = Counter()
    with ThreadPoolExecutor(threads) as ex:
        for f in [ex.submit(worker) for _ in range(threads)]:
            total += f.result()

    assert total == Counter(ok=threads * calls), total


def test_concurrent_iterate_and_count(populated_store: SqliteStore) -> None:
    """Mixed iterate / count workload across threads."""
    stream = populated_store.stream("color_image")
    expected = stream.count()

    def count_worker() -> int:
        return stream.count()

    def iterate_worker() -> int:
        return sum(1 for _ in stream)

    with ThreadPoolExecutor(16) as ex:
        futs = []
        for i in range(64):
            futs.append(ex.submit(count_worker if i % 2 else iterate_worker))
        results = [f.result() for f in futs]

    assert all(r == expected for r in results), Counter(results)
