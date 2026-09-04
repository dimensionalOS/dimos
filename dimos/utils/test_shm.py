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

"""Regression tests for the SHM creation race (shm_open before ftruncate)."""

from multiprocessing.shared_memory import SharedMemory
import os
import threading
import time
import uuid

import pytest

from dimos.utils.shm import ShmNotReadyError, attach_shm, create_or_attach_shm

SIZE = 1 << 16


@pytest.fixture
def name() -> str:
    n = f"dimos_test_{uuid.uuid4().hex[:12]}"
    yield n
    try:
        SharedMemory(name=n).unlink()
    except (FileNotFoundError, ValueError):
        pass


@pytest.fixture
def slow_ftruncate(monkeypatch):
    """Widen the shm_open->ftruncate window so the race is deterministic."""
    real = os.ftruncate

    def delayed(fd: int, length: int) -> None:
        time.sleep(0.2)
        return real(fd, length)

    monkeypatch.setattr(os, "ftruncate", delayed)
    return delayed


def test_bare_attach_hits_the_race(name, slow_ftruncate):
    """Without the fix, attaching inside the window raises the reported error."""
    seen: list[BaseException] = []

    def hammer() -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not seen:
            try:
                SharedMemory(name=name).close()
                return
            except FileNotFoundError:
                continue
            except ValueError as exc:
                seen.append(exc)

    t = threading.Thread(target=hammer, daemon=True)
    t.start()
    time.sleep(0.01)
    owner = SharedMemory(create=True, size=SIZE, name=name)
    t.join(timeout=3)
    owner.close()

    assert seen, "expected the creation window to be observable"
    assert "cannot mmap an empty file" in str(seen[0])


@pytest.mark.flaky(reruns=2)
def test_attach_shm_waits_out_the_race(name, slow_ftruncate):
    """attach_shm blocks through the window and returns a fully sized segment."""
    result: list[object] = []

    def attacher() -> None:
        try:
            shm = attach_shm(name, timeout=5.0)
            result.append(shm.size)
            shm.close()
        except BaseException as exc:
            result.append(exc)

    t = threading.Thread(target=attacher, daemon=True)
    t.start()
    time.sleep(0.01)
    owner = SharedMemory(create=True, size=SIZE, name=name)
    t.join(timeout=6)
    owner.close()

    assert result == [SIZE], f"attacher did not survive the window: {result}"


def test_attach_shm_retries_a_zero_sized_handle(name, monkeypatch, mocker):
    """CPython may expose the creation window as a size-zero handle."""
    empty = mocker.Mock(size=0)
    ready = mocker.Mock(size=SIZE)
    shared_memory = mocker.patch(
        "dimos.utils.shm.SharedMemory",
        side_effect=[empty, ready],
    )
    mocker.patch("dimos.utils.shm.resource_tracker.unregister")
    monkeypatch.setattr("dimos.utils.shm._POLL_S", 0)

    attached = attach_shm(name, timeout=1.0)

    assert attached is ready
    empty.close.assert_called_once_with()
    assert shared_memory.call_count == 2


def test_attach_shm_times_out_instead_of_hanging(name):
    started = time.monotonic()
    with pytest.raises(ShmNotReadyError) as excinfo:
        attach_shm(name, timeout=0.3)
    elapsed = time.monotonic() - started
    assert 0.3 <= elapsed < 3.0
    assert "does not exist" in str(excinfo.value)


def test_timeout_error_is_still_a_file_not_found_error(name):
    """Callers written against the old attach path keep working mid-deploy."""
    with pytest.raises(FileNotFoundError):
        attach_shm(name, timeout=0.05)


def test_a_wrongly_sized_segment_attaches_immediately(name):
    """A layout mismatch is a permanent error for the caller to raise, not a race."""
    owner = SharedMemory(create=True, size=SIZE, name=name)
    try:
        shm = attach_shm(name, timeout=0.2)
        assert shm.size == SIZE
        shm.close()
    finally:
        owner.close()


def test_create_or_attach_splits_owner_and_reader(name, slow_ftruncate):
    """Concurrent symmetric callers: exactly one owns, the other waits and attaches."""
    out: list[tuple[bool, int]] = []
    lock = threading.Lock()
    barrier = threading.Barrier(2)

    def racer() -> None:
        barrier.wait()
        shm, owner = create_or_attach_shm(name, SIZE, timeout=5.0)
        with lock:
            out.append((owner, shm.size))
        shm.close()

    threads = [threading.Thread(target=racer, daemon=True) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=8)

    assert len(out) == 2, f"a racer did not finish: {out}"
    assert sorted(owner for owner, _ in out) == [False, True]
    assert {size for _, size in out} == {SIZE}
