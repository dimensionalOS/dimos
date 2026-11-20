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

"""A robot with no hardware, for orchestrator demos: each task takes a while, and some fail on cue."""

import threading
import time

from reactivex import interval

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.std_msgs.Bool import Bool


class ToyRobotConfig(ModuleConfig):
    drive_s: float = 3.0
    pick_s: float = 2.0
    place_s: float = 1.5
    scan_s: float = 2.5
    speak_s: float = 1.0
    calibrate_s: float = 2.0
    # The first this many calls of each task raise.
    drive_failures: int = 0
    pick_failures: int = 0
    scan_failures: int = 0
    # Picks fail again in a run of `pick_failures` every this many picks (0: only the first run).
    pick_failures_every: int = 0
    battery_start_pct: float = 80.0
    drain_per_s: float = 1.0
    charge_per_s: float = 12.0
    # Seconds after start at which a Bool stream (`obstacle`, `estop`, `gripper_ok`) is published.
    script: list[tuple[float, str, bool]] = []
    # The `release_arm` call (counting from 1) that hangs instead of returning, to show the fault a late
    # cancel causes. 0 never hangs.
    jam_on_release: int = 0
    period_s: float = 0.25


class ToyRobot(Module):
    """Drives, picks, places, scans and charges, each taking its configured time."""

    config: ToyRobotConfig
    battery: Out[BatteryState]
    obstacle: Out[Bool]
    estop: Out[Bool]
    gripper_ok: Out[Bool]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._level = self.config.battery_start_pct
        self._charging = threading.Event()
        self._driving = threading.Event()
        self._stops = {task: threading.Event() for task in ("drive", "arm", "scan")}
        self._calls = {task: 0 for task in ("drive", "pick", "scan", "release")}
        self._lock = threading.Lock()
        self._started_at = 0.0
        self._script = sorted(self.config.script)

    @rpc
    def start(self) -> None:
        super().start()
        self._started_at = time.monotonic()
        self.register_disposable(interval(self.config.period_s).subscribe(lambda _: self._tick()))

    def _tick(self) -> None:
        rate = 0.0
        if self._charging.is_set():
            rate = self.config.charge_per_s
        elif self._driving.is_set():
            rate = -self.config.drain_per_s
        self._level = min(100.0, max(0.0, self._level + rate * self.config.period_s))
        self.battery.publish(BatteryState(percentage=self._level / 100.0))
        elapsed = time.monotonic() - self._started_at
        while self._script and self._script[0][0] <= elapsed:
            _, stream, value = self._script.pop(0)
            getattr(self, stream).publish(Bool(data=value))

    def _work(self, stop: str, seconds: float) -> None:
        """Wait out a task, or until its cancel comes."""
        event = self._stops[stop]
        event.clear()
        event.wait(seconds)

    def _count(self, task: str) -> int:
        with self._lock:
            self._calls[task] += 1
            return self._calls[task]

    @rpc
    def drive_to(self, place: str) -> str:
        """Drive to a named place. Returns the place once there."""
        self._driving.set()
        try:
            self._work("drive", self.config.drive_s)
        finally:
            self._driving.clear()
        if self._count("drive") <= self.config.drive_failures:
            raise RuntimeError(f"could not reach {place}")
        return place

    @rpc
    def stop_driving(self) -> None:
        self._stops["drive"].set()

    @rpc
    def pick(self, item: str) -> str:
        """Pick up an item. Returns the item once it is held."""
        self._work("arm", self.config.pick_s)
        picks = self._count("pick") - 1
        if self.config.pick_failures_every:
            picks %= self.config.pick_failures_every
        if picks < self.config.pick_failures:
            raise RuntimeError(f"{item} slipped")
        return item

    @rpc
    def place_item(self, item: str) -> None:
        self._work("arm", self.config.place_s)

    @rpc
    def release_arm(self) -> None:
        if self._count("release") == self.config.jam_on_release:
            time.sleep(3600.0)
        self._stops["arm"].set()

    @rpc
    def scan(self, area: str) -> int:
        """Scan an area. Returns how many objects it found, two more on each scan."""
        self._work("scan", self.config.scan_s)
        scans = self._count("scan")
        if scans <= self.config.scan_failures:
            raise RuntimeError(f"the camera lost {area}")
        return 2 * (scans - self.config.scan_failures)

    @rpc
    def stop_scan(self) -> None:
        self._stops["scan"].set()

    @rpc
    def speak(self, text: str) -> None:
        time.sleep(self.config.speak_s)

    @rpc
    def calibrate(self) -> None:
        time.sleep(self.config.calibrate_s)

    @rpc
    def start_charging(self) -> None:
        self._charging.set()

    @rpc
    def stop_charging(self) -> None:
        self._charging.clear()
