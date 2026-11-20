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

"""The M20's odometry from dimSLAM's lidar-inertial filter in place of the vendor LIO: both Airys'
clouds on ``lidar`` and their IMUs on ``imu`` (each in its own lidar frame, placed by the
base_link -> lidar mount tf), out as ``slam_odom`` and map -> base_link on tf."""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.mapping.dim_slam_lio.dim_slam_lio import (
    DimSlamLio,
    ImuConfig,
    LidarConfig,
    ProcessNoise,
)
from dimos.robot.deeprobotics.m20.constants import MAP_FRAME


def m20_dim_slam(
    lidar_frames: tuple[str, ...] = ("front_lidar", "rear_lidar"),
    imu_frames: tuple[str, ...] = ("rear_lidar",),
    # the RK3588's A76 cores (4-7): on its A55s (0-3) the filter takes ~100% of a core and falls seconds behind
    cpus: tuple[int, ...] = (4, 5, 6, 7),
    **overrides: Any,
) -> Blueprint:
    """``imu_frames``: the first drives and levels the filter. The rear Airy's IMU, its gyro bias
    ~0.006 rad/s against the front's ~0.043; both together ran under real time on the GOS RK3588."""
    return DimSlamLio.blueprint(
        odom_frame_id=MAP_FRAME,
        output_frame_id="base_link",
        # m^2: Point-LIO's 0.01 jittered 4 mm per scan at rest on the Airys; 0.05 steadied it to 1.1 mm
        lidars=[LidarConfig(frame_id=frame, point_variance=0.05) for frame in lidar_frames],
        # an Airy IMU at rest (90 s, 200 Hz): gyro variance ~5e-7 rad^2/s^2, accel ~1e-4 m^2/s^4 per axis
        imus=[
            ImuConfig(frame_id=frame, gyro_variance=1e-6, accel_variance=1e-4)
            for frame in imu_frames
        ],
        # a walking dog's body changes motion far slower than Point-LIO's handheld defaults allow
        process_noise=ProcessNoise(velocity=2.0, angular_velocity=100.0, specific_force=50.0),
        cpus=list(cpus),
        **overrides,
    ).remappings([(DimSlamLio, "odometry", "slam_odom")])
