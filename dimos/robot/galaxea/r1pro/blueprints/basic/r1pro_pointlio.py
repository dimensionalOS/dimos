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

"""The R1 Pro placed by Point-LIO on the chassis Mid-360 instead of wheel odometry."""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLioRust
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_visualization,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_LIDAR_HOST_IP,
    R1PRO_CHASSIS_LIDAR_IP,
)
from dimos.robot.galaxea.r1pro.lio import (
    LIDAR_FRAME,
    ODOM_FRAME,
    R1ProLioMountTf,
    R1ProLioOdomPose,
)
from dimos.robot.galaxea.r1pro.vendor_stack import R1ProVendorStack


def r1pro_lidar_odometry() -> Blueprint:
    """Our Mid-360 driver (per-point times, its own IMU) into Point-LIO, plus the mount tf."""
    return autoconnect(
        Mid360.blueprint(
            frame_id=LIDAR_FRAME,
            lidar_ip=R1PRO_CHASSIS_LIDAR_IP,
            host_ip=R1PRO_CHASSIS_LIDAR_HOST_IP,
        ).remappings([(Mid360, "lidar", "lidar_raw")]),
        PointLioRust.blueprint(frame_id=ODOM_FRAME, sensor_frame_id=LIDAR_FRAME).remappings(
            [
                (PointLioRust, "lidar", "lidar"),
                (PointLioRust, "odometry", "pointlio_odometry"),
            ]
        ),
        R1ProLioMountTf.blueprint(),
        R1ProLioOdomPose.blueprint(),
    ).remappings(
        [
            (R1ProLioOdomPose, "odometry", "pointlio_odometry"),
            # The name the planners already read.
            (R1ProLioOdomPose, "pose", "chassis_odom"),
        ]
    )


r1pro_pointlio = autoconnect(
    R1ProVendorStack.blueprint(stop_vendor_lidar=True),
    r1pro_visualization(),
    # Off, so base_link has exactly one parent: Point-LIO's, through the mount.
    r1pro_control(publish_odom=False),
    r1pro_lidar_odometry(),
).global_config(n_workers=4)
