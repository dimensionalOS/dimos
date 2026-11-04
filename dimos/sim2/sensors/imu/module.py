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

"""Mounted IMU measurement from physical motion, independent of the physics engine."""

from dataclasses import dataclass
from typing import Any, Literal

from dimos_lcm.sim_msgs import BodyMotion
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.sim2.sensors.sampling import Acquisition, measurement_header, same_acquisition


@dataclass(frozen=True)
class ImuConfig:
    frame_id: str
    source_id: str = "imu"
    hz: float = 200
    orientation: Literal["none", "ideal"] = "none"
    gyro_bias: tuple[float, float, float] = (0, 0, 0)
    accel_bias: tuple[float, float, float] = (0, 0, 0)
    gyro_std: float = 0
    accel_std: float = 0
    seed: int = 0


class ImuModuleConfig(ModuleConfig):
    sensor: ImuConfig


class SimImuModule(Module):
    config: ImuModuleConfig
    body_motion: In[BodyMotion]
    imu: Out[Imu]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sampling = Acquisition(self.config.sensor.hz)
        self._last: Imu | None = None
        c = self.config.sensor
        if (
            min(c.gyro_std, c.accel_std, c.seed) < 0
            or not np.isfinite((*c.gyro_bias, *c.accel_bias, c.gyro_std, c.accel_std)).all()
        ):
            raise ValueError("invalid IMU noise/calibration")

    def measure(self, truth: BodyMotion) -> Imu | None:
        c = self.config.sensor
        if truth.header.frame_id != c.frame_id:
            return None
        if self._last is not None and same_acquisition(truth.header, self._last.sample_header):
            return self._last
        if not self._sampling.due(truth.header):
            return None
        q = truth.pose_world.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        gyro = rotation.T @ truth.angular_velocity_world
        accel = rotation.T @ (np.asarray(truth.linear_acceleration_world) - truth.gravity_world)
        rng = np.random.default_rng([c.seed, truth.header.epoch, truth.header.time_ns])
        gyro += np.asarray(c.gyro_bias) + rng.normal(0, c.gyro_std, 3)
        accel += np.asarray(c.accel_bias) + rng.normal(0, c.accel_std, 3)
        sample = Imu(
            angular_velocity=Vector3(*gyro),
            linear_acceleration=Vector3(*accel),
            orientation=Quaternion(q.x, q.y, q.z, q.w),
            frame_id=c.frame_id,
            orientation_covariance=([-1.0] + [0.0] * 8) if c.orientation == "none" else [0.0] * 9,
            angular_velocity_covariance=(np.eye(3) * c.gyro_std**2).ravel().tolist(),
            linear_acceleration_covariance=(np.eye(3) * c.accel_std**2).ravel().tolist(),
            ts=truth.header.time_ns / 1e9,
        )
        self._last = sample.with_sample_header(
            measurement_header(truth.header, c.source_id, c.frame_id)
        )
        return self._last

    async def handle_body_motion(self, truth: BodyMotion) -> None:
        sample = self.measure(truth)
        if sample is not None:
            self.imu.publish(sample)
