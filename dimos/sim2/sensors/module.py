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

"""Sensor scheduling runs independently from physics and reports capture failures."""

from __future__ import annotations

import queue
import threading
import time
from typing import Any

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.sim2.module import SimulationSpec
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.device import SensorDevice
from dimos.sim2.sensors.reader import WorldReader
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SensorModuleConfig(ModuleConfig):
    robot_id: str | None = None
    sensor: Sensor
    # Worker polling is a deployment setting, not the sensor acquisition clock.
    rate_hz: float | None = Field(default=None, gt=0, allow_inf_nan=False)

    @property
    def model_prefix(self) -> str:
        return f"{self.robot_id}/" if self.robot_id is not None else ""

    @property
    def frame_prefix(self) -> str:
        return f"{self.robot_id or 'world'}/"


class SensorModule(Module):
    config: SensorModuleConfig
    _simulation: SimulationSpec
    dedicated_worker = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ready: queue.Queue[Exception | None] = queue.Queue(maxsize=1)
        self._error: str | None = None
        self._error_lock = threading.Lock()
        self.reader: WorldReader
        self.device: SensorDevice[Any]

    @rpc
    def start(self) -> None:
        super().start()
        description = self._simulation.describe()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(description,), daemon=True)
        self._thread.start()
        error = self._ready.get(timeout=60)
        if error is not None:
            raise error

    def _run(self, description: dict[str, Any]) -> None:
        opened = False
        reader_created = False
        try:
            self.reader = WorldReader(description)
            reader_created = True
            self.open()
            opened = True
            self._ready.put(None)
            rate = self.config.rate_hz or self.config.sensor.acquisition.sample_hz
            while not self._stop.is_set():
                before = time.monotonic()
                self.capture()
                self._stop.wait(max(0.0, 1.0 / rate - (time.monotonic() - before)))
        except Exception as error:
            with self._error_lock:
                self._error = str(error)
            if not opened:
                self._ready.put(error)
            logger.exception("sim2 sensor failed", module=type(self).__name__)
        finally:
            self.close_sensor()
            if reader_created:
                self.reader.close()

    def open(self) -> None:
        raise NotImplementedError("sensor modules construct their measurement device")

    def capture(self) -> None:
        for measurement in self.device.poll():
            if self.reader.channel.episode_id != measurement.episode:
                break
            getattr(self, measurement.port).publish(measurement.value)
            self.device.published(measurement)

    def close_sensor(self) -> None:
        device = getattr(self, "device", None)
        if device is not None:
            device.close()

    @rpc
    def sensor_status(self) -> dict[str, Any]:
        with self._error_lock:
            device = getattr(self, "device", None)
            return {"error": self._error, **(device.status() if device is not None else {})}

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("sensor worker did not stop")
        else:
            self.close_sensor()
        super().stop()
