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

"""K1 locomotion with device-owned sensors and the shared sim2 physics module."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import UsdModel
from dimos.robot.booster.k1.control_config import JOINTS
from dimos.robot.booster.k1.policy_module import K1PolicyModule
from dimos.robot.booster.k1.sim2 import K1SimConnection
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimColorCameraModule, SimCompressedCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.utils.data import LfsPath

_imu = SimImuModule.blueprint(
    instance_name="imu", sensor=ImuConfig("k1/imu", hz=50, orientation="ideal")
)
_camera = SimCompressedCameraModule.blueprint(
    instance_name="camera", sensor=CameraConfig("k1/realsense_optical")
)
_stereo_camera = SimColorCameraModule.blueprint(
    instance_name="stereo_camera", sensor=CameraConfig("k1/stereo_optical", depth=False)
).remappings(
    [
        (SimColorCameraModule, "color_image", "stereo_color_image"),
        (SimColorCameraModule, "camera_info", "stereo_camera_info"),
    ]
)
_scene = UsdManager.blueprint(
    instance_name="scene",
    model=UsdModel.from_file(LfsPath("booster_k1/world.usda")),
    mounts={
        "k1/trunk": _imu,
        "k1/head_realsense_rgb_link": _camera,
        "k1/head_booster_stereo_rgb_link": _stereo_camera,
    },
)

# Keep the 50 Hz policy and scene processing in separate workers.
booster_k1_sim2 = autoconnect(
    _scene,
    _imu,
    _camera,
    _stereo_camera,
    CameraRenderModule.blueprint(
        scene_manager=_scene, renderer=MujocoRenderer, cameras=("camera", "stereo_camera")
    ),
    SimModule.blueprint(
        scene_manager=_scene, backend=MujocoBackend, observation_hz=50, joint_ids=JOINTS
    ),
    SimJointEncoderModule.blueprint(sensor=EncoderConfig(JOINTS, hz=50)),
    K1PolicyModule.blueprint(),
    K1SimConnection.blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=9)
