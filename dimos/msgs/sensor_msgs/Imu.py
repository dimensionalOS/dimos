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

from __future__ import annotations

import time

from dimos_lcm.sensor_msgs.Imu import Imu as LCMImu

from dimos.msgs.geometry_msgs import Quaternion, Vector3
from dimos.types.timestamped import Timestamped


class Imu(Timestamped):
    """IMU sensor message mirroring ROS sensor_msgs/Imu.

    Contains orientation, angular velocity, and linear acceleration
    with optional covariance matrices (3x3 row-major as flat 9-element lists).
    """

    msg_name = "sensor_msgs.Imu"

    def __init__(
        self,
        angular_velocity: Vector3 | None = None,
        linear_acceleration: Vector3 | None = None,
        orientation: Quaternion | None = None,
        orientation_covariance: list[float] | None = None,
        angular_velocity_covariance: list[float] | None = None,
        linear_acceleration_covariance: list[float] | None = None,
        frame_id: str = "imu_link",
        ts: float | None = None,
    ) -> None:
        self.ts = ts if ts is not None else time.time()  # type: ignore[assignment]
        self.frame_id = frame_id
        self.angular_velocity = angular_velocity or Vector3(0.0, 0.0, 0.0)
        self.linear_acceleration = linear_acceleration or Vector3(0.0, 0.0, 0.0)
        self.orientation = orientation or Quaternion(0.0, 0.0, 0.0, 1.0)
        self.orientation_covariance = orientation_covariance or [0.0] * 9
        self.angular_velocity_covariance = angular_velocity_covariance or [0.0] * 9
        self.linear_acceleration_covariance = linear_acceleration_covariance or [0.0] * 9

    def lcm_encode(self) -> bytes:
        msg = LCMImu()
        [msg.header.stamp.sec, msg.header.stamp.nsec] = self.ros_timestamp()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = self.orientation.x
        msg.orientation.y = self.orientation.y
        msg.orientation.z = self.orientation.z
        msg.orientation.w = self.orientation.w
        msg.orientation_covariance = self.orientation_covariance

        msg.angular_velocity.x = self.angular_velocity.x
        msg.angular_velocity.y = self.angular_velocity.y
        msg.angular_velocity.z = self.angular_velocity.z
        msg.angular_velocity_covariance = self.angular_velocity_covariance

        msg.linear_acceleration.x = self.linear_acceleration.x
        msg.linear_acceleration.y = self.linear_acceleration.y
        msg.linear_acceleration.z = self.linear_acceleration.z
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> Imu:
        msg = LCMImu.lcm_decode(data)
        ts = msg.header.stamp.sec + (msg.header.stamp.nsec / 1_000_000_000)
        return cls(
            angular_velocity=Vector3(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
            linear_acceleration=Vector3(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            orientation=Quaternion(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ),
            orientation_covariance=list(msg.orientation_covariance),
            angular_velocity_covariance=list(msg.angular_velocity_covariance),
            linear_acceleration_covariance=list(msg.linear_acceleration_covariance),
            frame_id=msg.header.frame_id,
            ts=ts,
        )

    def __str__(self) -> str:
        return (
            f"Imu(frame_id='{self.frame_id}', "
            f"gyro=({self.angular_velocity.x:.3f}, {self.angular_velocity.y:.3f}, {self.angular_velocity.z:.3f}), "
            f"accel=({self.linear_acceleration.x:.3f}, {self.linear_acceleration.y:.3f}, {self.linear_acceleration.z:.3f}))"
        )

    def __repr__(self) -> str:
        return (
            f"Imu(ts={self.ts}, frame_id='{self.frame_id}', "
            f"angular_velocity={self.angular_velocity}, "
            f"linear_acceleration={self.linear_acceleration}, "
            f"orientation={self.orientation})"
        )
