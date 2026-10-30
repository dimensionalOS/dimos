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

"""Engine-independent observation frames and typed device-stream bridges.

Acquisition remains with the backend. These bridges preserve sampling timestamps
and publish each new sample once.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
import threading
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.sim2.messages import FrameMetadata
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.composition import Sensor, observable_sensors
from dimos.sim2.sensors.imu.spec import Imu


@dataclass(frozen=True)
class ObservationFrame:
    """One sensor's latest upstream sample, with its acquisition timestamp."""

    metadata: FrameMetadata
    robot_id: str | None
    sensor_name: str
    timestamp: float
    values: dict[str, Any]


def observation_key(robot_id: str | None, sensor: Camera | Imu) -> str:
    return f"{robot_id or 'world'}/{sensor.name}"


def collect_observations(
    values: Mapping[str, Any],
    sensors: tuple[tuple[str | None, Sensor], ...],
    metadata: FrameMetadata,
    clock_origin: float,
    previous: dict[str, tuple[int, float]],
) -> list[ObservationFrame]:
    """Emit each newly acquired latest value once, without re-dating held samples."""
    frames = []
    for robot_id, sensor in observable_sensors(sensors):
        key = observation_key(robot_id, sensor)
        names = (
            ("image", "depth")
            if isinstance(sensor, Camera) and sensor.depth
            else (("image",) if isinstance(sensor, Camera) else ("imu",))
        )
        payload: dict[str, Any] = {}
        changed = []
        timestamps = {}
        for name in names:
            meta_name = "depth_metadata" if name == "depth" else "metadata"
            raw_metadata = values.get(f"{key}/{meta_name}")
            if raw_metadata is None or f"{key}/{name}" not in values:
                continue
            tagged = np.atleast_1d(raw_metadata)
            # Observable initializes values to zero before its first sample.
            if tagged[0] != 1 or not math.isfinite(float(tagged[1])):
                continue
            sample_time = float(tagged[1])
            identity = metadata.episode_id, sample_time
            payload[name] = values[f"{key}/{name}"]
            payload[meta_name] = tagged[1:].copy()
            timestamps[name] = clock_origin + sample_time
            if previous.get(f"{key}/{name}") != identity:
                previous[f"{key}/{name}"] = identity
                changed.append(name)
        if not changed:
            continue
        for name in names:
            if name in payload:
                payload[name] = np.array(payload[name], copy=True)
        frames.append(
            ObservationFrame(
                metadata,
                robot_id,
                sensor.name,
                max(timestamps[name] for name in changed),
                {**payload, "changed": tuple(changed), "timestamps": timestamps},
            )
        )
    return frames


class ObservationModuleConfig(ModuleConfig):
    robot_id: str | None = None
    sensor: Camera | Imu


class ObservationModule(Module):
    """Publish an upstream sensor sample on ordinary typed device ports."""

    config: ObservationModuleConfig
    observations: In[ObservationFrame]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._observation_lock = threading.RLock()
        self._last_observation = (-1, -1)
        self._published = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.observations.subscribe(self._receive)))

    def _receive(self, frame: ObservationFrame) -> None:
        if frame.robot_id != self.config.robot_id or frame.sensor_name != self.config.sensor.name:
            return
        identity = frame.metadata.episode_id, frame.metadata.sequence
        with self._observation_lock:
            if identity <= self._last_observation:
                return
            self._last_observation = identity
            self.publish_observation(frame)
            self._published += 1

    def publish_observation(self, frame: ObservationFrame) -> None:
        raise NotImplementedError

    @rpc
    def sensor_status(self) -> dict[str, Any]:
        with self._observation_lock:
            return {
                "error": None,
                "fidelity": "backend_observation",
                "published": self._published,
                "episode": self._last_observation[0],
            }
