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

"""Alfred running MLS planning off the Mid-360, with both IMUs feeding dimSLAM.

The Mid-360 runs through the raw Livox driver instead of Point-LIO. The lidar streams to a
single host endpoint, so only one of the two can hold it, and the raw driver is the one that
publishes the sensor's own IMU. That buys the second IMU dimSLAM's fusion needs and costs
Point-LIO's lidar odometry, which dimSLAM is replacing anyway.

    dimos run alfred-mls-nav-lidar
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.dim_slam.dim_slam import ImuConfig
from dimos.robot.diy.alfred.blueprints.alfred_mls_nav import (
    D455_REMAPPINGS,
    d455_stereo,
    jpeg_color,
)
from dimos.robot.diy.alfred.blueprints.vis_nav import D455_IMU, vis_nav
from dimos.robot.diy.alfred.config import MID360_IP
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf

MID360_FRAME = "mid360_link"
MID360_IMU_FRAME = "mid360_imu_link"
"""Both are alfred.urdf links; dimSLAM reads the mount off tf, so the names have to agree."""

MID360_IMU = ImuConfig(
    frame_id=MID360_IMU_FRAME,
    # ICM-40609-D datasheet: rate noise spectral density 0.0045 deg/s/sqrt(Hz) and 100
    # ug/sqrt(Hz), converted to rad/s and m/s^2. The random walks are NOT on the datasheet
    # and are a placeholder of the same shape as the D455's - measure both off an Allan
    # variance of a stationary segment once this blueprint has recorded one.
    gyro_noise_density=7.9e-5,
    gyro_random_walk=2e-6,
    accel_noise_density=9.8e-4,
    accel_random_walk=1e-4,
)
"""The Mid-360's built-in IMU, an order of magnitude quieter than the D455's BMI055, which is
the point of the pairing: the filter keeps a bias pair per IMU, so the good gyro keeps its
say instead of being averaged into the cheap one."""

LIDAR_MAX_RANGE_METERS = 15.0
"""The Mid-360 reaches far past this, but the voxel map raytraces every point from the sensor
origin, so the far returns cost the most and carve the least reliable free space."""

alfred_mls_nav_lidar = (
    autoconnect(
        d455_stereo(),
        Mid360.blueprint(
            lidar_ip=MID360_IP,
            frame_id=MID360_FRAME,
            imu_frame_id=MID360_IMU_FRAME,
        ),
        AlfredMountTf.blueprint(),
        AlfredHighLevel.blueprint(),
        vis_nav(
            imus=[D455_IMU, MID360_IMU],
            map_cloud_topic="lidar",
            map_max_range=LIDAR_MAX_RANGE_METERS,
        ),
    )
    .remappings([*D455_REMAPPINGS, (AlfredHighLevel, "wheel_odometry", "source_odometry")])
    .transports(jpeg_color())
    .global_config(n_workers=12, robot_model="alfred")
)
