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

"""Convert an upstream IMU observation to a DimOS message."""

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu as ImuMessage
from dimos.sim2.sensors.observations import ObservationFrame


def imu_message(frame: ObservationFrame) -> ImuMessage:
    value = frame.values["imu"]
    return ImuMessage(
        angular_velocity=Vector3(*value[:3]),
        linear_acceleration=Vector3(*value[3:]),
        orientation_covariance=[-1.0, 0, 0, 0, 0, 0, 0, 0, 0],
        frame_id=f"{frame.robot_id or 'world'}/{frame.sensor_name}",
        ts=frame.timestamp,
    )
