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

"""Ideal mounted IMU acquisition."""

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.sim2.sensors.device import SensorDevice
from dimos.sim2.sensors.imu.spec import Imu


class ImuDevice(SensorDevice[Imu]):
    def open(self) -> None:
        name = f"{self.model_prefix}sensor/{self.sensor.name}"
        self._gyro = self.reader.model.sensor(f"{name}/gyro").id
        self._accel = self.reader.model.sensor(f"{name}/accel").id
        self._status["fidelity"] = "ideal_instantaneous"

    def capture(self) -> None:
        if self.sample_time(self.sensor.rate_hz) is None:
            return
        self.emit(
            "imu",
            ImuMessage(
                angular_velocity=Vector3(*self.reader.data.sensor(self._gyro).data),
                linear_acceleration=Vector3(*self.reader.data.sensor(self._accel).data),
                orientation_covariance=[-1.0, 0, 0, 0, 0, 0, 0, 0, 0],
                frame_id=self.frame_prefix + self.sensor.name,
                ts=self.reader.timestamp,
            ),
        )
