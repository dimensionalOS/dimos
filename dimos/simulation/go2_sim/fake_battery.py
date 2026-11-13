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

"""A battery stand-in for the simulated Go2."""

import threading

from reactivex import interval

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Float32 import Float32


class FakeBatteryConfig(ModuleConfig):
    start_pct: float = 32.0
    drain_per_s: float = 0.5
    charge_per_s: float = 20.0
    period_s: float = 0.25


class FakeBattery(Module):
    """Drains at a fixed rate, and charges while it is told to."""

    config: FakeBatteryConfig
    percent: Out[Float32]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._level = self.config.start_pct
        self._charging = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(interval(self.config.period_s).subscribe(lambda _: self._tick()))

    @rpc
    def start_charging(self) -> None:
        self._charging.set()

    @rpc
    def stop_charging(self) -> None:
        self._charging.clear()

    def _tick(self) -> None:
        rate = self.config.charge_per_s if self._charging.is_set() else -self.config.drain_per_s
        self._level = min(100.0, max(0.0, self._level + rate * self.config.period_s))
        self.percent.publish(Float32(self._level))
