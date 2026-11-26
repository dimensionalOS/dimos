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

"""A joint-driven pusher and box, with independently mounted sensor hardware."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.sim2.examples.scene import BasicScene
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

_camera = SimCameraModule.attach(
    BasicScene.mounts.camera_mount, instance_name="camera", sensor=CameraConfig("camera")
)
_lidar = SimLidarModule.attach(
    BasicScene.mounts.lidar_mount,
    instance_name="lidar",
    sensor=LidarConfig("lidar"),
    raycaster=MujocoRaycaster,
)
_imu = SimImuModule.attach(
    BasicScene.mounts.imu_mount, instance_name="imu", sensor=ImuConfig("imu", orientation="ideal")
)

sim2_basic = autoconnect(
    BasicScene.blueprint(instance_name="scene"),
    _camera,
    _lidar,
    _imu,
    CameraRenderModule.blueprint(
        renderer=MujocoRenderer, cameras=tuple(atom.name for atom in _camera.active_blueprints)
    ),
    SimModule.blueprint(backend=MujocoBackend, joint_ids=("world/joints/tool_joint",)),
    SimJointEncoderModule.blueprint(sensor=EncoderConfig(("world/joints/tool_joint",))),
).global_config(viewer="none", transport="zenoh", n_workers=5)
