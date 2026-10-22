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

"""Measurement acquisition and delayed delivery without a DimOS worker or transport."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Generic, TypeVar

from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.reader import WorldReader

SensorT = TypeVar("SensorT", bound=Sensor)


@dataclass(frozen=True)
class Measurement:
    port: str
    value: Any
    episode: int
    acquisition_start: float
    acquisition_end: float
    available_at: float
    available_timestamp: float


class SensorDevice(ABC, Generic[SensorT]):
    def __init__(
        self,
        reader: WorldReader,
        sensor: SensorT,
        robot_id: str | None = None,
        root_body: str | None = None,
    ) -> None:
        self.reader, self.sensor = reader, sensor
        self.model_prefix = f"{robot_id}/" if robot_id is not None else ""
        self.frame_prefix = f"{robot_id or 'world'}/"
        self.root_body = root_body
        self._episode = -1
        self._indices: dict[str, int] = {}
        self._pending: list[Measurement] = []
        self._error_lock = threading.RLock()
        self._status: dict[str, Any] = {
            "dropped_acquisitions": 0,
            "published": 0,
            "max_lateness_seconds": 0.0,
        }
        self.open()

    def open(self) -> None:
        pass

    def reset(self) -> None:
        """Reset device-specific acquisition state after the world changes epoch."""

    def poll(self) -> list[Measurement]:
        with self._error_lock:
            self.reader.update()
            if self.reader.sequence < 0 or self.reader.channel.lifecycle != "ready":
                return []
            now = float(self.reader.data.time)
            if self._episode != self.reader.episode:
                self._episode = self.reader.episode
                self._indices.clear()
                self._pending.clear()
                self.reset()
            self.capture()
            if self.reader.channel.episode_id != self._episode:
                self._pending.clear()
                return []
            ready = [m for m in self._pending if m.available_at <= now + 1e-9]
            self._pending = [m for m in self._pending if m.available_at > now + 1e-9]
            return sorted(ready, key=lambda m: m.available_at)

    def sample_time(self, rate: float, key: str = "sample") -> float | None:
        """Acquire the newest due sample; report skipped work instead of changing its clock."""
        index = math.floor((float(self.reader.data.time) + 1e-9) * rate)
        previous = self._indices.get(key, index - 1)
        if index <= previous:
            return None
        self._indices[key] = index
        self._status["dropped_acquisitions"] += max(0, index - previous - 1)
        timestamp = index / rate
        if not self.reader.history(timestamp, timestamp):
            self._status["dropped_acquisitions"] += 1
            return None
        self.reader.restore_at(timestamp)
        return timestamp

    def emit(
        self,
        port: str,
        value: Any,
        *,
        start: float | None = None,
        end: float | None = None,
        latency: float | None = None,
    ) -> None:
        end = float(self.reader.data.time) if end is None else end
        start = end if start is None else start
        latency = self.sensor.latency if latency is None else latency
        self._pending.append(
            Measurement(
                port,
                value,
                self._episode,
                start,
                end,
                end + latency,
                self.reader.timestamp + (end - float(self.reader.data.time)) + latency,
            )
        )

    def published(self, measurement: Measurement, at: float | None = None) -> None:
        """Record actual host publication separately from the modeled simulation deadline."""
        at = time.time() if at is None else at
        lateness = max(0.0, at - measurement.available_timestamp)
        with self._error_lock:
            self._status.update(
                published=self._status["published"] + 1,
                acquisition_start=measurement.acquisition_start,
                acquisition_end=measurement.acquisition_end,
                available_at=measurement.available_at,
                available_timestamp=measurement.available_timestamp,
                published_at=at,
                lateness_seconds=lateness,
                max_lateness_seconds=max(self._status["max_lateness_seconds"], lateness),
            )

    @abstractmethod
    def capture(self) -> None: ...

    def status(self) -> dict[str, Any]:
        with self._error_lock:
            return {
                **self._status,
                "pending_deliveries": len(self._pending),
                "episode": self._episode,
            }

    def close(self) -> None:
        self._pending.clear()
