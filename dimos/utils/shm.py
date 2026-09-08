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

"""Race-free creation and attachment of named POSIX shared-memory segments.

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

from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
import os
import time

DEFAULT_ATTACH_TIMEOUT_S = float(os.getenv("DIMOS_SHM_ATTACH_TIMEOUT_S", "10"))
_POLL_S = float(os.getenv("DIMOS_SHM_ATTACH_POLL_S", "0.0005"))


class ShmNotReadyError(TimeoutError, FileNotFoundError):
    """A segment did not become mappable at its layout size before the deadline.

    Subclasses ``FileNotFoundError`` so callers written against the pre-readiness
    attach path keep working across a rolling deploy.
    """


def unregister(shm: SharedMemory) -> SharedMemory:
    """Drop ``shm`` from the resource tracker; lifecycle is managed explicitly."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass
    return shm


def attach_shm(name: str, *, timeout: float | None = None) -> SharedMemory:
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


def create_or_attach_shm(
    name: str, size: int, *, timeout: float | None = None
) -> tuple[SharedMemory, bool]:
    """Create ``name`` at ``size`` (owner) or wait out its creator and attach.

    Returns ``(segment, is_owner)``. Owners stay registered with the resource
    tracker so a crash still cleans up; attachers do not, since they only close.
    """
    try:
        return SharedMemory(create=True, size=size, name=name), True
    except FileExistsError:
        return attach_shm(name, timeout=timeout), False


def _try_attach(name: str) -> tuple[SharedMemory | None, str]:
    try:
        return unregister(SharedMemory(name=name)), ""
    except FileNotFoundError:
        return None, "segment does not exist"
    except ValueError:
        # shm_open has landed but ftruncate has not: the creation window.
        return None, "creator has not sized the segment yet"
