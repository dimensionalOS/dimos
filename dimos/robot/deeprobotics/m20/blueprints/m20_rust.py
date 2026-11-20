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

"""The M20 on its own Rust drivers, with no vendor DrDDS stack: ``m20`` with dimSLAM in place of the vendor LIO.

AOS (a ``dimos host`` tagged ``board=aos``) runs the hardware: both Airys, the Yesense body IMU, the cameras and,
with ``--choice M20Joints=ecan``, the motor-bus joint state and batteries (needs rl_deploy with the ecan shim).
Everything else runs on the machine that starts the run (GOS): dimSLAM, the ray tracer, planners and relocalization.

Both Airys publish onto one ``lidar`` stream (their IMUs onto ``lidar_imu``), each scan in its own frame (front_lidar,
rear_lidar). The URDF filter cuts the robot's body out of each scan, frame kept, onto ``lidar_filtered``: dimSLAM
tells the lidars apart by frame_id, the ray tracer traces each from its own lidar's origin, and relocalization places
each by tf at its stamp. dimSLAM runs on the front lidar's IMU alone (both ran under real time
on GOS). Each Airy is namespaced but publishes its mount onto the one global tf. The cameras' module replaces ``m20-camera.service``
(``--choice M20Cameras=off`` leaves them to it); ``--choice M20BodyImu=off`` leaves the body IMU to the vendor
yesense_node, which holds its serial port in vendor mode. Nothing commands the legs.
"""

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, ChoiceAtom, autoconnect
from dimos.mapping.dim_slam_lio.dim_slam_lio import DimSlamLio
from dimos.robot.deeprobotics.m20.blueprints._nav import (
    VOXEL_SIZE_M,
    m20_nav_stack,
    m20_relocalization,
)
from dimos.robot.deeprobotics.m20.blueprints.m20_slam import m20_dim_slam
from dimos.robot.deeprobotics.m20.camera.module import M20CameraRelay
from dimos.robot.deeprobotics.m20.ecan.module import M20Ecan
from dimos.robot.deeprobotics.m20.imu.module import M20Imu
from dimos.robot.deeprobotics.m20.lidar import m20_lidars
from dimos.robot.deeprobotics.m20.model import (
    M20_MODEL,
    M20StandingJointState,
    M20StandingJointStateConfig,
)
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

# The `dimos host` tag of the board the lidars, IMU, cameras and motor bus are wired to.
AOS = {"board": "aos"}


def _body_filter(**config: Any) -> Blueprint:
    """The robot's own body cut out of every raw scan, at the joint state nearest its stamp, onto lidar_filtered."""
    return UrdfManagerPointClearing.blueprint(model=M20_MODEL, **config).remappings(
        [
            (UrdfManagerPointClearing, "pointcloud", "lidar"),
            (UrdfManagerPointClearing, "filtered_pointcloud", "lidar_filtered"),
        ]
    )


m20_rust = autoconnect(
    # the lidars voxelize at the ray tracer's voxel, the only thinning its scans get
    m20_lidars(voxel=VOXEL_SIZE_M).hosted(tags=AOS),
    # off while the vendor yesense_node holds the IMU's serial port
    ChoiceAtom(
        name="M20BodyImu",
        default="on",
        choices={"on": M20Imu.blueprint().hosted(tags=AOS), "off": autoconnect()},
    ),
    ChoiceAtom(
        name="M20Cameras",
        default="relay",
        choices={"relay": M20CameraRelay.blueprint().hosted(tags=AOS), "off": autoconnect()},
    ),
    # the body filter takes the joint state nearest each scan, so a fixed pose republished every period matches within it
    ChoiceAtom(
        name="M20Joints",
        default="standing",
        choices={
            "standing": autoconnect(
                M20StandingJointState.blueprint(),
                _body_filter(state_tolerance_s=M20StandingJointStateConfig().publish_period_s),
            ),
            "ecan": autoconnect(
                M20Ecan.blueprint()
                .remappings([(M20Ecan, "joint_state", "coordinator_joint_state")])
                .hosted(tags=AOS),
                _body_filter(),
            ),
        },
    ),
    m20_dim_slam().remappings(
        [(DimSlamLio, "lidar", "lidar_filtered"), (DimSlamLio, "imu", "lidar_imu")]
    ),
    m20_nav_stack(lidar="lidar_filtered", pre_trace_subsample=1, ray_subsample=1),
    m20_relocalization(lidar="lidar_filtered"),
).global_config(
    n_workers=5,
    transport="zenoh",
)
