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

"""The R1 Pro placed by lidar-inertial odometry.

``r1pro-coordinator`` with the wheel odometry replaced by Point-LIO on the
chassis Mid-360. Everything downstream that read the chassis pose keeps reading
it under the same name (``chassis_odom``); it now comes from the lidar and the
IMU inside it rather than from the speed the chassis was told to go.

Usage, on the robot::

    dimos run r1pro-pointlio --g.transport lcm

Point-LIO's C++ binary is built on first run (``nix build`` inside
``dimos/hardware/sensors/lidar/pointlio/cpp``; twenty minutes on an Orin). The
lidar's address comes from the vendor's ``MID360_config.json`` unless
``DIMOS_POINTLIO_LIDAR_IP``/``DIMOS_POINTLIO_HOST_IP`` say otherwise. Run on
``lcm``: the C++ estimator's cloud does not register in the Rust voxel map over
zenoh. See the R1 README for what this does to the vendor's own lidar driver,
and how to give the sensor back.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    r1pro_control,
    r1pro_visualization,
)
from dimos.robot.galaxea.r1pro.lio import (
    LIDAR_FRAME,
    ODOM_FRAME,
    R1ProLioMountTf,
    R1ProLioOdomPose,
    R1ProPointLio,
)


def r1pro_lidar_odometry() -> Blueprint:
    """Point-LIO on the chassis Mid-360, and the frames that hang the robot off it.

    Point-LIO reads the sensor itself, beside the vendor driver -- and the vendor
    driver stops receiving once it does: a Livox pushes its data to the host that
    last asked. So the estimator's own cloud goes onto the ``lidar`` bus rather
    than a stream of its own. It is not a second opinion beside the vendor's
    copy, it is the only copy -- and the better one: deskewed, and stamped in
    the frame the estimator is itself tracking.

    Point-LIO needs the Mid-360's *own* IMU -- the one inside the sensor, on the
    same rigid body as the beams, with the extrinsic the estimator assumes.
    ``imu_chassis`` is the robot's body IMU somewhere else on the chassis, and
    feeding that in would hand the filter a lever arm it does not know about.
    """
    return autoconnect(
        R1ProPointLio.blueprint(frame_id=ODOM_FRAME, sensor_frame_id=LIDAR_FRAME).remappings(
            [
                (R1ProPointLio, "lidar", "lidar"),
                (R1ProPointLio, "odometry", "pointlio_odometry"),
            ]
        ),
        R1ProLioMountTf.blueprint(),
        R1ProLioOdomPose.blueprint(),
    ).remappings(
        [
            (R1ProLioOdomPose, "odometry", "pointlio_odometry"),
            # The name the planners already read. It used to carry wheel
            # odometry; it now carries Point-LIO's, and nothing downstream
            # has to know which.
            (R1ProLioOdomPose, "pose", "chassis_odom"),
        ]
    )


r1pro_pointlio = autoconnect(
    r1pro_visualization(),
    # Off, so base_link has exactly one parent: Point-LIO's, through the mount.
    r1pro_control(publish_odom=False),
    r1pro_lidar_odometry(),
).global_config(n_workers=4)
