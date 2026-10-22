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

"""Publish independently acquired IMU measurements as DimOS streams."""

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.sim2.sensors.imu.device import ImuDevice
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.module import SensorModule, SensorModuleConfig


class ImuModuleConfig(SensorModuleConfig):
    sensor: Imu


class SimImuModule(SensorModule):
    config: ImuModuleConfig
    imu: Out[ImuMessage]

    def open(self) -> None:
        self.device = ImuDevice(self.reader, self.config.sensor, self.config.robot_id)
