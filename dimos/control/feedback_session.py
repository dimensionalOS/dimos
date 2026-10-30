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

"""One coherent control session: sequencing, reset, retransmission and lifecycle.

The coordinator supplies the control step; this object never inspects tasks,
robot models, simulator state or hardware drivers.
"""

from collections.abc import Callable
import math
import threading
import time
from typing import Any

from dimos.control.feedback import ControlFeedback, MotorControl
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class FeedbackSession:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.ready = threading.Event()
        self.current: ControlFeedback | None = None
        self._reply: MotorControl | None = None
        self._retired: set[str] = set()
        self._stopped = False
        self._restart = False
        self._max_compute_ms = 0.0
        self._error = ""

    def run(
        self,
        sample: ControlFeedback,
        reset: Callable[[], None],
        compute: Callable[[ControlFeedback], None],
        publish: Callable[[MotorControl], None],
    ) -> bool:
        with self.lock:
            if not self.ready.is_set() or self._stopped or sample.epoch in self._retired:
                return False
            previous = self.current
            if previous is not None and sample.epoch == previous.epoch:
                if sample.tick < previous.tick:
                    return False
                if sample.tick == previous.tick and not self._restart:
                    if self._reply is not None:
                        publish(self._reply)
                    return False
                if sample.tick > previous.tick and sample.time <= previous.time:
                    raise ValueError("feedback time must advance with its tick")
            if (
                sample.tick < 0
                or not math.isfinite(sample.time)
                or not math.isfinite(sample.dt)
                or sample.dt <= 0
            ):
                raise ValueError("invalid control clock")
            if previous is None or sample.epoch != previous.epoch or self._restart:
                if previous is not None and sample.epoch != previous.epoch:
                    self._retired.add(previous.epoch)
                reset()
            self.current, self._reply = sample, None
            self._restart, self._error = False, ""
            started = time.monotonic()
            try:
                compute(sample)
            except Exception as error:
                self._reply = None
                self._error = repr(error)
                raise
            self._max_compute_ms = max(self._max_compute_ms, (time.monotonic() - started) * 1000)
            if self._reply is not None:
                publish(self._reply)
            return True

    def set_command(self, joints: tuple[str, ...], motors: MotorCommandArray) -> None:
        # Called synchronously by compute, while run owns the session lock.
        assert self.current is not None
        self._reply = MotorControl(self.current.epoch, self.current.tick, joints, motors)

    def set_stopped(self, stopped: bool) -> None:
        with self.lock:
            if self._stopped and not stopped:
                self._restart, self._reply = True, None
            self._stopped = stopped

    def close(self) -> None:
        self.ready.clear()
        with self.lock:
            self.current, self._reply = None, None
            self._retired.clear()
            self._restart = False

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "epoch": self.current.epoch if self.current else None,
                "tick": self.current.tick if self.current else None,
                "max_compute_ms": self._max_compute_ms,
                "error": self._error,
            }
