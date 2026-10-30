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

"""Visual core example: a joint-driven pusher and loose box in DimOS Desktop."""

import math

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import RobotModel
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.module import SimModule
from dimos.sim2.mujoco import MujocoBackend
from dimos.sim2.sensor_module import SimSensorModule
from dimos.sim2.sensors import MujocoSensors
from dimos.sim2.types import Camera, Drive, Imu, Instance, Lidar, World

_source = RobotModel.from_file(DIMOS_PROJECT_ROOT / "assets/sim2_basic/scene.urdf")
_pusher = (
    _source.with_subtree_rooted_at("base")
    .with_fixed_frame(
        "camera", "tool", xyz=(-0.25, 0, 0.6), rpy=(-3 * math.pi / 4, 0, -math.pi / 2)
    )
    .with_fixed_frame("lidar", "tool", xyz=(0, 0, 0.12))
    .with_fixed_frame("imu", "tool")
)
WORLD = World(
    instances=(
        Instance("pusher", _pusher, drives=(Drive("tool_joint"),)),
        Instance("box", _source.with_subtree_rooted_at("box"), xyz=(0.1, 0, 0.075), floating=True),
    ),
    sensors=(Camera("camera", hz=15), Lidar("lidar", rays=720, packets=10), Imu("imu")),
)

sim2_basic = autoconnect(
    SimModule.blueprint(world=WORLD, backend=MujocoBackend),
    SimSensorModule.blueprint(
        world=WORLD, reader=MujocoSensors, camera="camera", lidar="lidar", imu="imu"
    ),
    UrdfManager.blueprint(model=_pusher).remappings(
        [("urdfmanager", "coordinator_joint_state", "joint_state")]
    ),
).global_config(viewer="none", transport="zenoh", n_workers=4)
