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

"""Acquisition cadence in the source clock; duplicates never resample noise."""

import math

from dimos_lcm.common_msgs import SampleHeader


class Acquisition:
    def __init__(self, hz: float) -> None:
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("sensor rate must be finite and positive")
        self.period_ns = round(1e9 / hz)
        self.clock_id: str | None = None
        self.epoch = -1
        self.last_ns = -1
        self.next_ns = 0

    def due(self, h: SampleHeader) -> bool:
        if h.time_ns < 0:
            raise ValueError("negative acquisition time")
        if self.clock_id is not None and h.clock_id != self.clock_id:
            raise ValueError("source clock changed; rebind the sensor")
        if h.epoch < self.epoch:
            return False
        if h.epoch > self.epoch:
            self.epoch, self.last_ns, self.next_ns = h.epoch, -1, 0
        self.clock_id = h.clock_id
        if h.time_ns <= self.last_ns:
            return False
        self.last_ns = h.time_ns
        if h.time_ns < self.next_ns:
            return False
        self.next_ns = (h.time_ns // self.period_ns + 1) * self.period_ns
        return True


def same_acquisition(a: SampleHeader, b: SampleHeader | None) -> bool:
    return b is not None and (a.clock_id, a.epoch, a.time_ns) == (b.clock_id, b.epoch, b.time_ns)
