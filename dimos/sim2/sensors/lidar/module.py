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

"""Typed stream/lifecycle wrapper for lidar measurement acquisition."""

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.sensors.lidar.device import LidarDevice
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.sensors.module import SensorModule, SensorModuleConfig


class LidarModuleConfig(SensorModuleConfig):
    sensor: Lidar
    root_body: str | None = None


class LidarModule(SensorModule):
    """A lidar worker with a real driver's ports.

    ``lidar`` is the sensor-frame scan with per-point offsets and channels, and
    ``imu`` the device's own IMU, exactly what a Mid360 driver publishes.
    ``pointcloud`` is the deskewed world-frame cloud and ``tf`` the true sensor
    pose, both ground truth a real device cannot give; ``Lidar.truth_outputs``
    turns them off when an estimator is under test.
    """

    config: LidarModuleConfig
    lidar: Out[PointCloud2]
    imu: Out[Imu]
    pointcloud: Out[PointCloud2]
    tf: Out[TFMessage]

    def open(self) -> None:
        self.device = LidarDevice(
            self.reader, self.config.sensor, self.config.robot_id, self.config.root_body
        )
