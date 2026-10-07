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
import threading
import tracemalloc

import pytest
import sqlite_vec

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.utils.sqlite import _locks, open_sqlite_connection


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


def test_eager_blob_query_streams_rows(tmp_path: Path) -> None:
    """Reading one observation must not load every joined blob into memory."""
    blob = b"x" * 256 * 1024
    total = 40
    with SqliteStore(path=str(tmp_path / "eager.db")) as store:
        stream = store.stream("blobs", bytes, eager_blobs=True)
        for i in range(total):
            stream.append(blob, ts=float(i))
        tracemalloc.start()
        try:
            next(iter(stream))
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()

    assert peak < len(blob) * total // 4, peak


def test_repeated_stop_does_not_leak_connection_locks(tmp_path: Path) -> None:
    before = len(_locks)
    store = SqliteStore(path=str(tmp_path / "stop.db"))
    store.stop()
    store.stop()

    assert len(_locks) == before


def _race_failing_append(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, *, b_finishes_first: bool
) -> list[float]:
    """Append A (ts=1) fails in its blob write while append B (ts=2) hits the same stream.

    Returns the timestamps left in the store. B only reaches the store between A's
    insert and A's failure if appends are not serialized for their whole transaction.
    """
    wait = 0.2
    with SqliteStore(path=str(tmp_path / "race.db")) as store:
        stream = store.stream("race", bytes)
        backend = stream._source
        a_inserted, b_inserted, a_done, b_done = (threading.Event() for _ in range(4))
        insert, put = backend.metadata_store.insert, backend.blob_store.put

        ts_by_row: dict[int, float] = {}

        def racing_insert(obs):
            if obs.ts == 2.0:
                a_inserted.wait(wait)
            row_id = insert(obs)
            ts_by_row[row_id] = obs.ts
            (a_inserted if obs.ts == 1.0 else b_inserted).set()
            return row_id

        def racing_put(name, key, data):
            if ts_by_row[key] == 1.0:
                (b_done if b_finishes_first else b_inserted).wait(wait)
                raise RuntimeError("blob write failed")
            if not b_finishes_first:
                a_done.wait(wait)
            put(name, key, data)

        monkeypatch.setattr(backend.metadata_store, "insert", racing_insert)
        monkeypatch.setattr(backend.blob_store, "put", racing_put)

        def append(payload: bytes, ts: float, done: threading.Event) -> None:
            try:
                stream.append(payload, ts=ts)
            finally:
                done.set()

        with ThreadPoolExecutor(2) as ex:
            fa = ex.submit(append, b"A", 1.0, a_done)
            fb = ex.submit(append, b"B", 2.0, b_done)
            with pytest.raises(RuntimeError, match="blob write failed"):
                fa.result(timeout=10)
            fb.result(timeout=10)

        return [obs.ts for obs in stream]


def test_failed_append_does_not_roll_back_concurrent_append(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    assert _race_failing_append(tmp_path, monkeypatch, b_finishes_first=False) == [2.0]


def test_failed_append_is_not_committed_by_concurrent_append(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    assert _race_failing_append(tmp_path, monkeypatch, b_finishes_first=True) == [2.0]


def test_concurrent_size_bytes_is_consistent(populated_store: SqliteStore) -> None:
    stream = populated_store.stream("color_image")
    expected = stream.size_bytes()
    assert expected

    def worker() -> list[int | None]:
        return [stream.size_bytes() for _ in range(250)]

    with ThreadPoolExecutor(16) as ex:
        results = [r for f in [ex.submit(worker) for _ in range(16)] for r in f.result()]

    assert Counter(results) == Counter({expected: 16 * 250})


def test_failed_open_does_not_leak_connection_lock(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fail(conn: object) -> None:
        raise RuntimeError("extension load failed")

    monkeypatch.setattr(sqlite_vec, "load", fail)
    before = len(_locks)
    with pytest.raises(RuntimeError, match="extension load failed"):
        open_sqlite_connection(tmp_path / "failed.db")

    assert len(_locks) == before
