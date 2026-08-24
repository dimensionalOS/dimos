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

"""Pair the two imagers of a stereo stream by stamp."""

from __future__ import annotations

import threading
from typing import Any

import reactivex as rx
from reactivex.disposable import Disposable

STEREO_PAIR_TOLERANCE = 0.001
"""Matches dimSLAM's own 1 ms stereo pair gate."""


def stamp_matched_pairs(left: Any, right: Any) -> Any:
    """The imagers subscribe an instant apart, so an ordinal zip pairs mismatched stamps."""

    def subscribe(observer: Any, scheduler: Any = None) -> Any:
        lock = threading.Lock()
        pending: dict[str, Any] = {"left": None, "right": None}
        completed: set[str] = set()

        def on_frame(side: str, other: str, frame: Any) -> None:
            emit = None
            with lock:
                held = pending[other]
                if held is not None and abs(held.ts - frame.ts) <= STEREO_PAIR_TOLERANCE:
                    pending[other] = None
                    emit = (held, frame) if side == "right" else (frame, held)
                elif held is not None and held.ts > frame.ts:
                    pass  # partner dropped; the held frame still waits
                else:
                    # Any held older frame lost its partner; keep only the newest.
                    pending[other] = None
                    pending[side] = frame
            if emit is not None:
                observer.on_next(emit)

        def on_done(side: str) -> None:
            with lock:
                completed.add(side)
                done = len(completed) == 2
            if done:
                observer.on_completed()

        subscriptions = [
            left.subscribe(
                on_next=lambda f: on_frame("left", "right", f),
                on_completed=lambda: on_done("left"),
            ),
            right.subscribe(
                on_next=lambda f: on_frame("right", "left", f),
                on_completed=lambda: on_done("right"),
            ),
        ]

        def dispose() -> None:
            for subscription in subscriptions:
                subscription.dispose()

        return Disposable(dispose)

    return rx.create(subscribe)
