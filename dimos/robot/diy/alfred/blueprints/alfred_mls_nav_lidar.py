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
from dimos.hardware.sensors.camera.realsense.constants import IMU_BMI055
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.dim_slam.dim_slam import ImuConfig
from dimos.robot.diy.alfred.blueprints.alfred_mls_nav import alfred_mls_nav
from dimos.robot.diy.alfred.blueprints.vis_nav import vis_nav
from dimos.robot.diy.alfred.config import ALFRED

MID360_IMU_FRAME = "mid360_imu_link"

alfred_mls_nav_lidar = autoconnect(
    alfred_mls_nav,
    Mid360.blueprint(
        lidar_ip=ALFRED.mid360_ip,
        frame_id="mid360_link",
        imu_frame_id=MID360_IMU_FRAME,
    ),
    # Same modules as alfred_mls_nav's vis_nav, so this replaces it: both IMUs fused, and
    # free space carved from the lidar instead of the depth cloud.
    vis_nav(
        imus=[
            # Gyro yaw halved final drift on drive_2026-08-18_23-05-04.db, 2.66 m out to 1.33 m
            # against a 0.59 m floor on the lidar reference's own heading. Datasheet-class
            # figures for the part, not an Allan variance of this particular camera.
            ImuConfig(frame_id="d455_accel_optical_frame", **IMU_BMI055),
            # The Mid-360's built-in IMU, an order of magnitude quieter than the D455's BMI055, which is
            # the point of the pairing: the filter keeps a bias pair per IMU, so the good gyro keeps its
            # say instead of being averaged into the cheap one.
            ImuConfig(
                frame_id=MID360_IMU_FRAME,
                gyro_noise_density=7.9e-5,
                gyro_random_walk=2e-6,
                accel_noise_density=9.8e-4,
                accel_random_walk=1e-4,
            ),
        ],
        map_cloud_topic="lidar",
        # The Mid-360 reaches far past this, but the voxel map raytraces every point from the
        # sensor origin, so the far returns cost the most and carve the least reliable space.
        map_max_range=15.0,
    ),
).global_config(n_workers=12, robot_model="alfred")
