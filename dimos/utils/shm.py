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

"""Race-free attachment to named POSIX shared-memory segments.

``SharedMemory(create=True, size=N)`` runs ``shm_open`` and *then* ``ftruncate``.
Between the two the segment exists at size 0, so a concurrent attacher mmaps an
empty file and dies with ``ValueError: cannot mmap an empty file``.

Readiness here is "the segment exists and maps at a non-zero size". ``ftruncate``
moves the segment from 0 to its full size in one step and the creator never shrinks
it, so a segment that maps at all is a segment the creator has finished sizing.
Attachers poll that predicate against a deadline and raise :class:`ShmNotReadyError`
when it is not met, rather than retrying blindly and hoping the next mmap lands.

Readiness deliberately stops at "sized": a segment that maps at the *wrong* size was
built by a peer with a different layout, which is a permanent error its caller must
raise, not a race to wait out.
"""

import mmap
from multiprocessing.shared_memory import SharedMemory
import os
import sys
import time

if sys.platform != "win32":
    import _posixshmem

DEFAULT_ATTACH_TIMEOUT_S = float(os.getenv("DIMOS_SHM_ATTACH_TIMEOUT_S", "10"))
_POLL_S = float(os.getenv("DIMOS_SHM_ATTACH_POLL_S", "0.0005"))


class ShmNotReadyError(TimeoutError, FileNotFoundError):
    """A segment did not become mappable at its layout size before the deadline.

    Subclasses ``FileNotFoundError`` so callers written against the pre-readiness
    attach path keep working across a rolling deploy.
    """


class AttachedShm:
    """A POSIX mapping borrowed from its creator, outside the Python resource tracker."""

    def __init__(self, name: str, fd: int, size: int) -> None:
        self.name = name.removeprefix("/")
        self.size = size
        self._mapping = mmap.mmap(fd, size)
        self.buf = memoryview(self._mapping)

    def close(self) -> None:
        self.buf.release()
        self._mapping.close()

    def unlink(self) -> None:
        unlink_shm(self.name)


ShmBuffer = SharedMemory | AttachedShm


def unlink_shm(name: str) -> None:
    """Discard a stale name without enrolling it in this process's tracker."""
    if sys.platform == "win32":
        # Windows releases the name when its final handle closes.
        SharedMemory(name=name).close()
    else:
        _posixshmem.shm_unlink("/" + name.removeprefix("/"))


def attach_shm(name: str, *, timeout: float | None = None) -> ShmBuffer:
    """Attach to ``name`` once its creator has finished sizing it."""
    limit = DEFAULT_ATTACH_TIMEOUT_S if timeout is None else timeout
    deadline = time.monotonic() + limit
    while True:
        shm, reason = _try_attach(name)
        if shm is not None:
            return shm
        if time.monotonic() >= deadline:
            raise ShmNotReadyError(f"SHM {name!r} not ready after {limit:g}s: {reason}")
        time.sleep(_POLL_S)


def _try_attach(name: str) -> tuple[ShmBuffer | None, str]:
    try:
        if sys.platform == "win32":
            return SharedMemory(name=name), ""
        fd = _posixshmem.shm_open("/" + name.removeprefix("/"), os.O_RDWR, mode=0)
    except FileNotFoundError:
        return None, "segment does not exist"
    try:
        size = os.fstat(fd).st_size
        if size == 0:
            return None, "creator has not sized the segment yet"
        # Map the descriptor we checked. SharedMemory(name=...) would register
        # the creator's name even in an independent reader's tracker.
        return AttachedShm(name, fd, size), ""
    finally:
        os.close(fd)
