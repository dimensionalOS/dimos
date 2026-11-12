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

"""The fake base, battery and estop switch that `patrol.toml` runs against."""

import threading
import time

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from reactivex import interval

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32

FAILING_GOAL = 2


class FakeBaseConfig(ModuleConfig):
    arrive_s: float = 1.0
    spin_s: float = 0.5


class FakeBase(Module):
    """Reaches every goal after a delay. Its second goal fails."""

    config: FakeBaseConfig
    status: Out[GoalStatus]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._goals = 0
        self._arrival: threading.Timer | None = None
        self._halted = threading.Event()

    @rpc
    def stop(self) -> None:
        self.halt()
        super().stop()

    @rpc
    def enable(self) -> None:
        pass

    @rpc
    def disable(self) -> None:
        pass

    @rpc
    def go_to(self, name: str) -> str:
        """Set off for a named place. Returns the id that its status message carries."""
        with self._lock:
            self._goals += 1
            if self._goals == FAILING_GOAL:
                raise RuntimeError(f"no path to {name}")
            goal = f"goal-{self._goals}"
            reached = GoalStatus(goal_id=GoalID(id=goal), status=GoalStatus.SUCCEEDED)
            self._arrival = threading.Timer(self.config.arrive_s, self.status.publish, (reached,))
            self._arrival.daemon = True
            self._arrival.start()
        return goal

    @rpc
    def spin(self, turns: float) -> None:
        """Turn in place. Returns once the turns are done or the base is halted."""
        self._halted.clear()
        self._halted.wait(turns * self.config.spin_s)

    @rpc
    def halt(self) -> None:
        with self._lock:
            if self._arrival is not None:
                self._arrival.cancel()
        self._halted.set()


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


class FakeEstopConfig(ModuleConfig):
    press_at_s: float = 40.0
    press_for_s: float = 3.0
    period_s: float = 0.25


class FakeEstop(Module):
    """A switch that is pressed once, for a few seconds."""

    config: FakeEstopConfig
    pressed: Out[Bool]

    @rpc
    def start(self) -> None:
        super().start()
        began = time.monotonic()
        self.register_disposable(
            interval(self.config.period_s).subscribe(lambda _: self._tick(began))
        )

    def _tick(self, began: float) -> None:
        held_for = time.monotonic() - began - self.config.press_at_s
        self.pressed.publish(Bool(0.0 <= held_for < self.config.press_for_s))
