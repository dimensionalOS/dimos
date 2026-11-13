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

"""A joint-driven pusher and box: devices contribute USD, one manager composes."""

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import UsdModel
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.sim2.sensors.tf import SimTfModule

_camera = SimCameraModule.blueprint(
    instance_name="camera",
    sensor=CameraConfig("camera"),
)
_lidar = SimLidarModule.blueprint(
    instance_name="lidar",
    scene_manager="scene",
    sensor=LidarConfig("lidar"),
    raycaster=MujocoRaycaster,
)
_imu = SimImuModule.blueprint(instance_name="imu", sensor=ImuConfig("imu", orientation="ideal"))
_scene = UsdManager.blueprint(
    publish_joint_tf=False,
    instance_name="scene",
    model=UsdModel.from_file(DIMOS_PROJECT_ROOT / "assets/sim2_basic/scene.usda"),
    mounts={"camera_mount": _camera, "lidar_mount": _lidar, "imu_mount": _imu},
)

sim2_basic = autoconnect(
    _camera,
    CameraRenderModule.blueprint(
        scene_manager=_scene, renderer=MujocoRenderer, cameras=("camera",)
    ),
    _lidar,
    _imu,
    _scene,
    SimTfModule.blueprint(),
    SimModule.blueprint(scene_manager=_scene, backend=MujocoBackend, joint_ids=("tool_joint",)),
    SimJointEncoderModule.blueprint(sensor=EncoderConfig(("tool_joint",))),
).global_config(viewer="none", transport="zenoh", n_workers=5)
