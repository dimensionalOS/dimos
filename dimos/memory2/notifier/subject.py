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

"""In-memory fan-out notifier (same-process, thread-safe)."""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any, TypeVar

from reactivex.disposable import Disposable

from dimos.memory2.notifier.base import Notifier, NotifierConfig

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.type.observation import Observation

T = TypeVar("T")


class SubjectNotifierConfig(NotifierConfig):
    pass


class SubjectNotifier(Notifier[T]):
    """In-memory fan-out notifier for same-process live notification.

    Thread-safe.  ``notify()`` copies the subscriber list under the lock,
    then iterates outside the lock to avoid deadlocks with slow consumers.
    """

    config: SubjectNotifierConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._subscribers: list[BackpressureBuffer[Observation[T]]] = []
        self._lock = threading.Lock()

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        with self._lock:
            self._subscribers.append(buf)

        def _unsubscribe() -> None:
            with self._lock:
                try:
                    self._subscribers.remove(buf)
                except ValueError:
                    pass

        return Disposable(action=_unsubscribe)

    def notify(self, obs: Observation[T]) -> None:
        with self._lock:
            subs = list(self._subscribers)
        for buf in subs:
            buf.put(obs)

    def stop(self) -> None:
        """Close all subscribed buffers, unblocking any live iterators."""
        with self._lock:
            subs = list(self._subscribers)
            self._subscribers.clear()
        for buf in subs:
            buf.close()
