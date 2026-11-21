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

import errno
import json
from multiprocessing.shared_memory import SharedMemory
import os
import subprocess
import sys
import threading
import time
import uuid

import pytest

from dimos.utils.shm import ShmNotReadyError, attach_shm

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
            except (ValueError, OSError) as exc:
                seen.append(exc)

    t = threading.Thread(target=hammer, daemon=True)
    t.start()
    time.sleep(0.01)
    owner = SharedMemory(create=True, size=SIZE, name=name)
    t.join(timeout=3)
    owner.close()

    assert seen, "expected the creation window to be observable"
    # Linux fails the empty-file mmap inside Python; darwin's shm fds are not
    # regular files, so the kernel rejects the zero-length mmap with EINVAL.
    err = seen[0]
    assert isinstance(err, ValueError) or (isinstance(err, OSError) and err.errno == errno.EINVAL)


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


def _read_sim_segment(layout: str, key: str) -> None:
    from dimos.simulation.engines.mujoco_shm import ManipShmReader
    from dimos.simulation.mujoco.shared_memory import ShmReader

    reader = ManipShmReader(key) if layout == "arm" else ShmReader(json.loads(key))
    try:
        assert reader.shm.as_list()[0].buf[0] == 42
    finally:
        reader.cleanup()


def _run_sim_owner(layout: str) -> None:
    from dimos.simulation.engines.mujoco_shm import ManipShmWriter
    from dimos.simulation.mujoco.shared_memory import ShmWriter

    owner: ManipShmWriter | ShmWriter
    if layout == "arm":
        key = uuid.uuid4().hex[:12]
        # Leave the previous generation named, as after an interrupted shutdown.
        previous = ManipShmWriter(key)
        for segment in previous.shm.as_list():
            segment.close()
        owner = ManipShmWriter(key)
    else:
        owner = ShmWriter()
        key = json.dumps(owner.shm.to_names())
    try:
        owner.shm.as_list()[0].buf[0] = 42
        # An independent reader must leave the names available to a later attach.
        subprocess.run(
            [
                sys.executable,
                "-c",
                "import sys; from dimos.utils.test_shm import _read_sim_segment; "
                "_read_sim_segment(*sys.argv[1:])",
                layout,
                key,
            ],
            check=True,
            timeout=30,
        )
        # This reader shares the owner's tracker and must leave its registration intact.
        _read_sim_segment(layout, key)
    finally:
        owner.cleanup()


@pytest.mark.parametrize("layout", ["arm", "unitree"])
def test_only_the_sim_owner_tracks_its_segments(layout: str) -> None:
    # The tracker reports asynchronous errors on stderr when this process exits.
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; from dimos.utils.test_shm import _run_sim_owner; "
            "_run_sim_owner(sys.argv[1])",
            layout,
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stderr
    assert "resource_tracker" not in result.stderr, result.stderr
