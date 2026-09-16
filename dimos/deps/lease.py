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

"""Ownership leases for managed environments.

Two ``flock`` files per environment live in ``envs_dir()/.locks/``, outside the
environment directory, so removing an environment never deletes a held lock:

* ``<name>.prepare.lock`` is a mutex. Preparation, validation and deletion hold
  it exclusively; a candidate that finds it taken waits for one preparation.
* ``<name>.lock`` is the use lease. Every process running from the environment
  holds it shared for its whole life; the install step and deletion take it
  exclusively and never wait for it.

One file could not do both jobs: a waiter blocked on an exclusive lock would
sleep for as long as a run keeps the shared one. The lock order is always
prepare, then use.

A lease is released by closing its descriptor, never with ``LOCK_UN``: after a
fork both processes share the open file description, so unlocking in the
exiting launcher would also drop the daemon's lease. Across ``execve`` the
descriptor is made inheritable and its number travels in ``DIMOS_ENV_LEASE_FD``
so the new image adopts it instead of reopening the file.
"""

from __future__ import annotations

from collections.abc import Callable, MutableMapping
import fcntl
import os
from pathlib import Path
import sys

LEASE_FD_ENV = "DIMOS_ENV_LEASE_FD"
LOCKS_DIR = ".locks"
Echo = Callable[[str], None]


class EnvironmentBusyError(RuntimeError):
    """A conflicting lease is held elsewhere; the message names the environment."""


def use_lock(env_path: Path) -> Path:
    return env_path.parent / LOCKS_DIR / f"{env_path.name}.lock"


def prepare_lock(env_path: Path) -> Path:
    return env_path.parent / LOCKS_DIR / f"{env_path.name}.prepare.lock"


def is_held(path: Path) -> bool:
    """Whether some process holds ``path`` right now; for display only."""
    try:
        fd = os.open(path, os.O_RDWR)
    except FileNotFoundError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return True
    finally:
        os.close(fd)
    return False


def _echo(message: str) -> None:
    print(message, file=sys.stderr)


class EnvironmentLease:
    """A ``flock`` on one lock file, held until the descriptor is closed."""

    def __init__(self, fd: int, path: Path) -> None:
        self.fd = fd
        self.path = path

    @classmethod
    def acquire(
        cls, path: Path, *, shared: bool, blocking: bool, busy: str, echo: Echo = _echo
    ) -> EnvironmentLease:
        """Lock ``path``; ``busy`` says who holds it when the lock is already taken."""
        path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        operation = fcntl.LOCK_SH if shared else fcntl.LOCK_EX
        try:
            fcntl.flock(fd, operation | fcntl.LOCK_NB)
        except BlockingIOError:
            if not blocking:
                os.close(fd)
                raise EnvironmentBusyError(busy) from None
            echo(f"{busy}; waiting...")
            try:
                fcntl.flock(fd, operation)
            except BaseException:
                os.close(fd)
                raise
        return cls(fd, path)

    @classmethod
    def adopt(cls, environ: MutableMapping[str, str], path: Path) -> EnvironmentLease | None:
        """The lease on ``path`` that a parent image exported, or ``None`` when there is none."""
        exported = environ.pop(LEASE_FD_ENV, None)
        if exported is None:
            return None
        fd = int(exported)
        try:
            held, expected = os.fstat(fd), os.stat(path)
        except OSError:
            return None
        if (held.st_dev, held.st_ino) != (expected.st_dev, expected.st_ino):
            return None
        os.set_inheritable(fd, False)
        return cls(fd, path)

    def export(self, env: MutableMapping[str, str]) -> None:
        """Let the image that ``execve`` loads with ``env`` adopt this lease."""
        os.set_inheritable(self.fd, True)
        env[LEASE_FD_ENV] = str(self.fd)

    def release(self) -> None:
        """Close the descriptor; a forked child that shares it keeps the lease."""
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self) -> EnvironmentLease:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.release()
