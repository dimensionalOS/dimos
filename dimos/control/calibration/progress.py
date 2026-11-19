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

"""Terminal progress for the calibration scripts: no wait is ever silent.

Plain carriage-return lines (tqdm is not a declared dependency). ``countdown``
runs a blocking call in a worker thread and shows elapsed/remaining seconds
until it returns; ``ProgressBar`` ticks over a known number of items;
``confirm`` is the drivers' prompt (Enter continue, ``s`` skip, ``p`` park).
"""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import sys
import time
from typing import IO, TypeVar

T = TypeVar("T")

_REFRESH_S = 0.1
CONFIRM_HINT = "[Enter continue, s skip, p park and quit]"


class ParkRequestedError(Exception):
    """The operator asked to park the arms and end the run."""


def confirm(prompt: str, enabled: bool, *, read: Callable[[str], str] = input) -> bool:
    """Ask before a motion: True to run it, False to skip it, ``ParkRequestedError`` on ``p``.

    ``q`` is accepted as a synonym for ``p`` so an old habit still ends safely.
    """
    if not enabled:
        return True
    answer = read(f"{prompt} {CONFIRM_HINT} ").strip().lower()
    if answer in ("p", "q"):
        raise ParkRequestedError("park requested at the prompt")
    return answer != "s"


def countdown(
    label: str,
    expected_s: float,
    call: Callable[[], T],
    *,
    stream: IO[str] = sys.stdout,
) -> T:
    """Run ``call`` in a thread, updating one line with elapsed and remaining seconds."""
    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix="countdown") as pool:
        future = pool.submit(call)
        while True:
            elapsed = time.monotonic() - start
            remaining = max(expected_s - elapsed, 0.0)
            stream.write(f"\r{label}  {elapsed:5.1f} s elapsed, {remaining:5.1f} s remaining")
            stream.flush()
            try:
                result = future.result(timeout=_REFRESH_S)
            except TimeoutError:
                continue
            stream.write(f"\r{label}  done in {time.monotonic() - start:.1f} s" + " " * 20 + "\n")
            stream.flush()
            return result


class ProgressBar:
    """``[####....] 3/12 text`` on one line; ``close`` ends the line."""

    def __init__(self, total: int, label: str = "", *, stream: IO[str] = sys.stdout) -> None:
        self._total = max(total, 1)
        self._label = label
        self._stream = stream
        self._count = 0
        self._start = time.monotonic()
        self.update(0)

    def update(self, count: int | None = None, text: str = "") -> None:
        self._count = self._count + 1 if count is None else count
        filled = round(20 * self._count / self._total)
        bar = "#" * filled + "." * (20 - filled)
        elapsed = time.monotonic() - self._start
        line = f"\r{self._label} [{bar}] {self._count}/{self._total} {elapsed:5.1f} s {text}"
        self._stream.write(line.ljust(100))
        self._stream.flush()

    def close(self) -> None:
        self.update(self._total)
        self._stream.write("\n")
        self._stream.flush()
