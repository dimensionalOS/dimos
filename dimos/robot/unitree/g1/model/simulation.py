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

"""G1 device model and sensor mounts; controller selection does not change the robot."""

from dataclasses import replace
from pathlib import Path

from dimos.robot.unitree.g1.model.joints import g1_joints
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
from dimos.sim2.sensors.lidar.spec import Lidar
from dimos.sim2.spec import Joint, RobotConfig
from dimos.utils.data import LfsPath

G1 = RobotConfig(
    model=Path(__file__).parents[1] / "assets" / "g1_29dof.xml",
    meshdir=LfsPath("g1_urdf/meshes"),
    root_body="pelvis",
    floating=True,
    spawn_height=0.793,
    joints=tuple(
        Joint(
            name=name,
            model_name=name.split("/", 1)[1] + "_joint",
            actuator=name.split("/", 1)[1] + "_joint",
        )
        for name in g1_joints
    ),
    sensors=(
        Imu("imu", site="control_imu"),
        Lidar("lidar", "mid360_link", Fibonacci, rate_hz=10.0, maximum_world_elevation=0.0),
        Camera("camera", camera="front_camera"),
    ),
)


def g1_definition(source: str = "g1") -> RobotConfig:
    """Same physical G1 and mounts regardless of which controller drives it."""
    return replace(
        G1,
        joints=tuple(
            replace(joint, name=f"{source}/{joint.name.split('/', 1)[1]}") for joint in G1.joints
        ),
    )
