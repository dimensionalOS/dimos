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

"""dimSLAM's lidar-inertial odometry with no cuVSLAM: any number of lidars and IMUs (and other
odometry sources) fused in one filter, Point-LIO's estimator. Builds on boards with no CUDA
(the M20's RK3588). The camera path stays in ``dimos.mapping.dim_slam``."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field, model_validator

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

# [x, y, z, roll, pitch, yaw] (or [vx, vy, vz, wx, wy, wz]): <0 the message's covariance, 0 drops the axis, >0 fixed.
Variances = tuple[float, float, float, float, float, float]


class ImuConfig(BaseModel):
    """One IMU, identified by the frame_id its samples carry; tf places it relative to the output frame. Defaults are Point-LIO's."""

    frame_id: str
    # rad^2/s^2 and m^2/s^4: variance of one gyro / accelerometer reading.
    gyro_variance: float = 0.01
    accel_variance: float = 0.01
    # rad/s and g: readings within 1% of these are clipped and say nothing; 0 trusts every reading.
    gyro_clip: float = 35.0
    accel_clip_g: float = 3.0
    # Random-walk variances of the gyro and accelerometer biases, per s^2.
    gyro_bias_random_walk: float = 0.0001
    accel_bias_random_walk: float = 0.0001


class LidarConfig(BaseModel):
    """One lidar, identified by the frame_id its scans carry; tf places it relative to the output frame."""

    frame_id: str
    # m^2: variance of one point's distance to its plane (Point-LIO's default).
    point_variance: float = 0.01


class SourceConfig(BaseModel):
    """An external odometry source, identified by the transform its estimates carry."""

    parent_frame_id: str
    child_frame_id: str
    # pose: a non-drifting world of its own; twist: body velocity only; pose_delta: a drifting pose, fused by its motion.
    kind: Literal["pose", "twist", "pose_delta"] = "pose_delta"
    pose_variances: Variances = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    twist_variances: Variances = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # Zero-velocity update: a twist at or under these speeds is fused as zero with still_variance; 0 disables.
    still_variance: float = 0.0
    still_speed: float = 0.0
    still_angular_speed: float = 0.0


class ProcessNoise(BaseModel):
    """Random-walk variances, per s^2, of velocity, angular velocity and specific force (Point-LIO's defaults)."""

    velocity: float = 20.0
    angular_velocity: float = 1000.0
    specific_force: float = 500.0


class DimSlamLioConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/mapping/dim_slam_lio/rust"
    executable: str = "result/bin/dim_slam_lio"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # Frame the odometry is stamped in (its parent), and the body the filter estimates (its child).
    odom_frame_id: str = "odom"
    output_frame_id: str = "base_link"
    # Off when something downstream owns odom_frame_id -> output_frame_id.
    publish_tf: bool = True
    # The first drives the filter between scans and levels it; samples from frames not listed are dropped.
    imus: list[ImuConfig] = Field(default_factory=list)
    # Scans from frames not listed are dropped.
    lidars: list[LidarConfig] = Field(default_factory=list)
    odom_sources: list[SourceConfig] = Field(default_factory=list)
    process_noise: ProcessNoise = Field(default_factory=ProcessNoise)
    # Every this many scans, log the handlers' time and the inputs' age on arrival (lidar clock vs host); 0 never.
    stats_every_scans: int = Field(default=100, ge=0)
    # CPUs the process runs on (Linux); empty leaves it to the scheduler.
    cpus: list[int] = Field(default_factory=list)
    # Scan thinning before fusion: every keep_every-th usable point, then one point per scan_cube m cube (Point-LIO's
    # defaults); the filter's cost is linear in what is left.
    keep_every: int = Field(default=3, ge=1)
    scan_cube: float = Field(default=0.2, gt=0)

    @model_validator(mode="after")
    def _frames_are_distinct(self) -> DimSlamLioConfig:
        for kind, frames in (
            ("imus", [imu.frame_id for imu in self.imus]),
            ("lidars", [lidar.frame_id for lidar in self.lidars]),
        ):
            if len(set(frames)) != len(frames):
                raise ValueError(f"{kind} repeat a frame_id: {frames}")
        return self


class DimSlamLio(NativeModule):
    """Every lidar publishes onto ``lidar`` and every IMU onto ``imu``, told apart by frame_id
    (clouds as Livox drivers write them: float32 x/y/z plus a uint32 ``offset_time``, ns after
    the stamp). Mounts come from tf against ``output_frame_id``. Publishes ``odometry`` and
    odom_frame_id -> output_frame_id on tf."""

    config: DimSlamLioConfig

    lidar: In[PointCloud2]
    imu: In[Imu]
    odom_sources: In[Odometry]

    odometry: Out[Odometry]
    tf: IO[TFMessage]
