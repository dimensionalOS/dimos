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

"""timed_playback's own scheduling, without recorded data (runs everywhere, not only self-hosted)."""

import threading

from reactivex.disposable import Disposable
from reactivex.scheduler import ImmediateScheduler

from dimos.utils.testing.replay import timed_playback


def test_timed_playback_survives_a_timer_that_fires_before_scheduling_returns() -> None:
    """A busy machine runs behind, so a frame's delay is ~0 and its timer can fire (and schedule
    the next frame) before schedule_relative returns. Replaying that ordering: the first timer
    runs at once, the rest wait in a queue; every frame must still come out."""
    queued: list[tuple[object, list[bool]]] = []

    class FiresFirstTimerAtOnce(ImmediateScheduler):
        def schedule_relative(self, duetime, action, state=None):  # type: ignore[no-untyped-def,override]
            cancelled = [False]
            if not queued and not getattr(self, "fired", False):
                self.fired = True
                worker = threading.Thread(target=action, args=(self, state))
                worker.start()
                worker.join()
            else:
                queued.append((action, cancelled))
            return Disposable(lambda: cancelled.__setitem__(0, True))

    scheduler = FiresFirstTimerAtOnce()
    frames = [(float(index), index) for index in range(6)]
    seen: list[int] = []
    timed_playback(lambda: iter(frames), speed=1000.0).subscribe(seen.append, scheduler=scheduler)
    while queued:
        action, cancelled = queued.pop(0)
        if not cancelled[0]:
            action(scheduler, None)
    assert seen == [0, 1, 2, 3, 4, 5]
