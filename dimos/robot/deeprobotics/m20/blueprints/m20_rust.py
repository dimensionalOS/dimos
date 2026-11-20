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

The Airys are two independent streams, ``front_lidar`` and ``rear_lidar`` (each in its own lidar frame, with
``front_lidar_imu`` / ``rear_lidar_imu``), and dimSLAM, the ray tracer and relocalization each take both, never merged.
dimSLAM runs on the front lidar's IMU alone (both IMUs ran under real time on GOS). The cameras' module replaces ``m20-camera.service``
(``--choice M20Cameras=off`` leaves them to it). Nothing commands the legs.
"""

from dimos.core.coordination.blueprints import ChoiceAtom, autoconnect
from dimos.mapping.dim_slam_lio.dim_slam_lio import DimSlamLio
from dimos.robot.deeprobotics.m20.blueprints._nav import m20_nav_stack, m20_relocalization
from dimos.robot.deeprobotics.m20.blueprints.m20_slam import m20_dim_slam
from dimos.robot.deeprobotics.m20.camera.module import M20CameraRelay
from dimos.robot.deeprobotics.m20.ecan.module import M20Ecan
from dimos.robot.deeprobotics.m20.imu.module import M20Imu
from dimos.robot.deeprobotics.m20.lidar import m20_airy_lidars
from dimos.robot.deeprobotics.m20.model import M20_MODEL, M20StandingJointState
from dimos.robot.urdf_manager import UrdfManager

# The `dimos host` tag of the board the lidars, IMU, cameras and motor bus are wired to.
AOS = {"board": "aos"}

m20_rust = autoconnect(
    autoconnect(m20_airy_lidars(), M20Imu.blueprint()).hosted(tags=AOS),
    ChoiceAtom(
        name="M20Cameras",
        default="relay",
        choices={"relay": M20CameraRelay.blueprint().hosted(tags=AOS), "off": autoconnect()},
    ),
    ChoiceAtom(
        name="M20Joints",
        default="standing",
        choices={
            "standing": M20StandingJointState.blueprint(),
            "ecan": M20Ecan.blueprint()
            .remappings([(M20Ecan, "joint_state", "coordinator_joint_state")])
            .hosted(tags=AOS),
        },
    ),
    m20_dim_slam().remappings(
        [
            (DimSlamLio, "lidar", "front_lidar"),
            (DimSlamLio, "second_lidar", "rear_lidar"),
            (DimSlamLio, "imu", "front_lidar_imu"),
        ]
    ),
    UrdfManager.blueprint(model=M20_MODEL),
    m20_nav_stack(lidar="front_lidar", second_lidar="rear_lidar"),
    m20_relocalization(lidar="front_lidar", second_lidar="rear_lidar"),
).global_config(
    n_workers=5,
    transport="zenoh",
)
