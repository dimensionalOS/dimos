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
from dimos.sim2.description import Drive, Instance, SceneConfig
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.physics.simulation import SimulationConfig
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster

_source = RobotModel.from_file(DIMOS_PROJECT_ROOT / "assets/sim2_basic/scene.urdf")
_pusher = (
    _source.with_subtree_rooted_at("base")
    .with_fixed_frame(
        "camera", "tool", xyz=(-0.25, 0, 0.6), rpy=(-3 * math.pi / 4, 0, -math.pi / 2)
    )
    .with_fixed_frame("lidar", "tool", xyz=(0, 0, 0.12))
    .with_fixed_frame("imu", "tool")
)
SCENE = SceneConfig(
    instances=(
        Instance("pusher", drives=(Drive("tool_joint"),)),
        Instance("box", xyz=(0.1, 0, 0.075), floating=True),
    ),
)

sim2_basic = autoconnect(
    UrdfManager.blueprint(models={"pusher": _pusher, "box": _source.with_subtree_rooted_at("box")}),
    SimModule.blueprint(
        scene=SCENE,
        backend=MujocoBackend,
        timing=SimulationConfig(motion_frames=("imu",), joint_ids=("tool_joint",)),
    ),
    SimImuModule.blueprint(sensor=ImuConfig("imu", orientation="ideal")),
    SimJointEncoderModule.blueprint(sensor=EncoderConfig(("tool_joint",))),
    SimCameraModule.blueprint(scene=SCENE, sensor=CameraConfig("camera"), renderer=MujocoRenderer),
    SimLidarModule.blueprint(scene=SCENE, sensor=LidarConfig("lidar"), raycaster=MujocoRaycaster),
).global_config(viewer="none", transport="zenoh", n_workers=5)
