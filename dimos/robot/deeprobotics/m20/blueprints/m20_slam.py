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
from dimos.mapping.dim_slam_lio.dim_slam_lio import DimSlamLio, ImuConfig, LidarConfig
from dimos.robot.deeprobotics.m20.constants import MAP_FRAME


def m20_dim_slam(
    lidar_frames: tuple[str, ...] = ("front_lidar", "rear_lidar"),
    imu_frames: tuple[str, ...] = ("front_lidar",),
    **overrides: Any,
) -> Blueprint:
    """``imu_frames``: the first drives and levels the filter; one lidar IMU ran 3x real time on
    the GOS RK3588, both lidar IMUs together did not keep up and drifted more at rest."""
    return DimSlamLio.blueprint(
        odom_frame_id=MAP_FRAME,
        output_frame_id="base_link",
        lidars=[LidarConfig(frame_id=frame) for frame in lidar_frames],
        imus=[ImuConfig(frame_id=frame) for frame in imu_frames],
        **overrides,
    ).remappings([(DimSlamLio, "odometry", "slam_odom")])
