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

"""Sensor families accepted by world composition and their physical attachments."""

from __future__ import annotations

from typing import TypeAlias

from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.spec import Lidar

Sensor: TypeAlias = Camera | Lidar | Imu


def mounted_sensors(sensors: tuple[Sensor, ...]) -> tuple[Sensor, ...]:
    """Include the physical IMU attachment of an integrated lidar device."""
    return tuple(
        child
        for sensor in sensors
        for child in (
            (sensor, sensor.imu)
            if isinstance(sensor, Lidar) and sensor.imu is not None
            else (sensor,)
        )
    )
