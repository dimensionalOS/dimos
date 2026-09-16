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

from collections.abc import Callable, Iterator
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any

import pytest

from dimos.deps.lease import LEASE_FD_ENV, EnvironmentBusyError, EnvironmentLease, is_held

WAIT_S = 10.0
CHILD = """
import os, sys
from pathlib import Path
from dimos.deps.lease import EnvironmentLease
lease = EnvironmentLease.adopt(os.environ, Path(sys.argv[1]))
print("adopted" if lease else "none", flush=True)
sys.stdin.readline()
"""


@pytest.fixture
def leases() -> Iterator[list[EnvironmentLease]]:
    held: list[EnvironmentLease] = []
    yield held
    for lease in held:
        lease.release()


@pytest.fixture
def spawn() -> Iterator[Callable[..., subprocess.Popen[str]]]:
    children: list[subprocess.Popen[str]] = []

    def start(*args: Any, **kwargs: Any) -> subprocess.Popen[str]:
        child: subprocess.Popen[str] = subprocess.Popen(*args, **kwargs)
        children.append(child)
        return child

    yield start
    for child in children:
        if child.poll() is None:
            child.kill()
        child.wait()


def shared(path: Path) -> EnvironmentLease:
    return EnvironmentLease.acquire(path, shared=True, blocking=False, busy="env is busy")


def exclusive(path: Path) -> EnvironmentLease:
    return EnvironmentLease.acquire(path, shared=False, blocking=False, busy="env is busy")


def _poll(condition: Callable[[], bool]) -> None:
    deadline = time.monotonic() + WAIT_S
    while not condition():
        assert time.monotonic() < deadline, "condition not met in time"
        time.sleep(0.01)


def test_shared_leases_coexist_and_exclude_exclusive_ones(
    tmp_path: Path, leases: list[EnvironmentLease]
) -> None:
    path = tmp_path / ".locks" / "env.lock"
    assert not is_held(path)
    first, second = shared(path), shared(path)
    leases += [first, second]
    assert is_held(path)
    with pytest.raises(EnvironmentBusyError, match="env is busy"):
        exclusive(path)
    first.release()
    second.release()
    assert not is_held(path)
    leases.append(exclusive(path))
    with pytest.raises(EnvironmentBusyError):
        shared(path)


def test_blocking_acquire_waits_for_the_holder(
    tmp_path: Path, leases: list[EnvironmentLease]
) -> None:
    path = tmp_path / "env.lock"
    holder = exclusive(path)
    leases.append(holder)
    messages: list[str] = []
    acquired: list[EnvironmentLease] = []

    def wait_for_it() -> None:
        acquired.append(
            EnvironmentLease.acquire(
                path, shared=True, blocking=True, busy="env is busy", echo=messages.append
            )
        )

    waiter = threading.Thread(target=wait_for_it)
    waiter.start()
    _poll(lambda: bool(messages))
    assert messages == ["env is busy; waiting..."] and not acquired
    holder.release()
    waiter.join(timeout=WAIT_S)
    assert not waiter.is_alive() and len(acquired) == 1
    leases.extend(acquired)


def test_export_and_adopt_round_trip(tmp_path: Path, leases: list[EnvironmentLease]) -> None:
    path = tmp_path / "env.lock"
    lease = shared(path)
    leases.append(lease)
    env: dict[str, str] = {}
    lease.export(env)
    assert env == {LEASE_FD_ENV: str(lease.fd)} and os.get_inheritable(lease.fd)
    adopted = EnvironmentLease.adopt(env, path)
    assert adopted is not None and adopted.fd == lease.fd and env == {}
    assert not os.get_inheritable(lease.fd)
    assert EnvironmentLease.adopt({}, path) is None
    other = tmp_path / "other.lock"
    other.write_text("")
    assert EnvironmentLease.adopt({LEASE_FD_ENV: str(lease.fd)}, other) is None
    closed = os.open(path, os.O_RDWR)
    os.close(closed)
    assert EnvironmentLease.adopt({LEASE_FD_ENV: str(closed)}, path) is None


def test_lease_survives_exec_into_a_child(
    tmp_path: Path,
    leases: list[EnvironmentLease],
    spawn: Callable[..., subprocess.Popen[str]],
) -> None:
    path = tmp_path / "env.lock"
    lease = shared(path)
    leases.append(lease)
    env = dict(os.environ)
    lease.export(env)
    child = spawn(
        [sys.executable, "-c", CHILD, str(path)],
        env=env,
        pass_fds=(lease.fd,),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    assert child.stdout is not None and child.stdin is not None
    assert child.stdout.readline().strip() == "adopted"
    lease.release()
    assert is_held(path)
    child.stdin.close()
    child.wait(timeout=WAIT_S)
    assert not is_held(path)
