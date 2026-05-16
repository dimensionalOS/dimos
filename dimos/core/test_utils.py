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

"""Shared test helpers that must remain import-safe in minimal environments."""

from __future__ import annotations

from collections.abc import Callable
import threading


def retry_until(
    event: threading.Event,
    action: Callable[[], None],
    timeout: float = 2.0,
    interval: float = 0.01,
) -> None:
    """Retry an action until an Event fires.

    Useful for async test paths where the producer may need to retry briefly
    before the subscriber is fully ready.
    """
    deadline = threading.Event()
    timer = threading.Timer(timeout, deadline.set)
    timer.start()
    try:
        while not event.is_set() and not deadline.is_set():
            action()
            event.wait(interval)
    finally:
        timer.cancel()
    assert event.is_set(), f"Timed out after {timeout}s waiting for event"
