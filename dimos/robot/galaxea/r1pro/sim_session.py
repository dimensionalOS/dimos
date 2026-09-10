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

"""Reserve the messaging and shared-memory resources of one local R1Pro demo."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
import hashlib
import os
from pathlib import Path
import tempfile

from filelock import FileLock, Timeout


class DemoSessionInUseError(RuntimeError):
    """Another local demo owns resources this run would otherwise share."""


@contextmanager
def reserve_demo_session(
    scout_address: str, output: Path, *, lock_directory: Path | None = None
) -> Iterator[None]:
    """Reject overlapping runs before creating a scene or connecting any streams.

    Messaging addresses scope streams and RPCs. The resolved output directory
    determines the scene path and therefore the motor shared-memory namespace.
    Hold both OS-backed locks until the coordinator and its workers shut down.
    """
    directory = lock_directory or (
        Path(tempfile.gettempdir()) / f"dimos-r1pro-sessions-{os.getuid()}"
    )
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    resources = (
        ("messaging address", scout_address.strip()),
        ("output directory", str(output.expanduser().resolve())),
    )
    with ExitStack() as stack:
        for label, value in resources:
            digest = hashlib.sha256(f"{label}:{value}".encode()).hexdigest()
            lock_path = directory / f"{digest}.lock"
            owner_path = lock_path.with_suffix(".pid")
            try:
                stack.enter_context(FileLock(lock_path, timeout=0))
            except Timeout as error:
                try:
                    owner = f" (PID {int(owner_path.read_text())})"
                except (OSError, ValueError):
                    owner = ""
                raise DemoSessionInUseError(
                    f"An R1Pro demo{owner} is already using {label} {value!r}. "
                    "Stop it with Ctrl-C in its terminal or close its MuJoCo window, "
                    "then run this command again. Concurrent demos need different "
                    "messaging addresses and output directories."
                ) from error
            owner_path.write_text(f"{os.getpid()}\n")
            stack.callback(owner_path.unlink, missing_ok=True)
        yield
