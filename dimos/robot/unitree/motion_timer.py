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


from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import threading


class MotionTimer:
    """Serialize a velocity command and the expiry that belongs to it."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None

    @contextmanager
    def command(self) -> Iterator[MotionTimer]:
        """Hold the command lock while sending a velocity and updating its deadline."""
        with self._lock:
            yield self

    def cancel(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def is_current(self, timer: threading.Timer) -> bool:
        return self._timer is timer

    def arm(self, duration: float, stop: Callable[[], None]) -> threading.Timer:
        self.cancel()

        def expire() -> None:
            with self._lock:
                # cancel() cannot recall a callback that is already running.
                if self.is_current(timer):
                    self._timer = None
                    stop()

        timer = threading.Timer(duration, expire)
        self._timer = timer
        timer.daemon = True
        timer.start()
        return timer
