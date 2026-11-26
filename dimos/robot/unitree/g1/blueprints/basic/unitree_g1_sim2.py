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

"""G1/GR00T and device-owned sensor USDs attached through HardwareModule."""

from functools import partial

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2_policy import JOINTS, G1PolicyModule
from dimos.robot.unitree.g1.sim2_scene import TORSO, G1Desk, G1DeskConfig
from dimos.sim2.description import joint_paths
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.data import LfsPath

IMU = ImuConfig("imu", orientation="ideal")
_MODEL = G1DeskConfig.model_fields["usd"].default
_PATHS = joint_paths(_MODEL)
JOINT_IDS = tuple(_PATHS[name] for name in JOINTS)
ENCODERS = EncoderConfig(JOINT_IDS, hz=50)
CAMERA = CameraConfig("camera")
LIDAR = LidarConfig(
    "mid360/lidar_link",
    pattern=partial(Mid360Scan, LfsPath("mid360_pattern/fourier.npz"), downsample=10),
    min_range=0.16,
    response=Mid360Returns,
    # The vendor head mesh seals the optical window. Until that asset has a
    # separate window, omit only this head link from optical queries; retain
    # every other robot link and all physical collisions.
    excluded_bodies=(
        TORSO + "/head_body",
        TORSO + "/lidar_mount/mid360/lidar_link/lidar_link_body",
    ),
)


_imu = SimImuModule.attach(G1Desk.mounts.imu_mount, instance_name="imu", sensor=IMU).remappings(
    [(SimImuModule, "imu", "imu")]
)
_camera = SimCameraModule.attach(G1Desk.mounts.camera_mount, instance_name="camera", sensor=CAMERA)
_lidar = SimLidarModule.attach(
    G1Desk.mounts.lidar_mount,
    instance_name="lidar",
    sensor=LIDAR,
    raycaster=MujocoRaycaster,
    usd=UsdModel.from_file(DIMOS_PROJECT_ROOT / "assets/g1_desk/mid360.usda"),
).remappings([(SimLidarModule, "lidar", "lidar_raw"), (SimLidarModule, "pointcloud", "pointcloud")])
_scene = G1Desk.blueprint(instance_name="scene")

unitree_g1_sim2 = autoconnect(
    _imu,
    _camera,
    CameraRenderModule.blueprint(
        renderer=MujocoRenderer, cameras=tuple(atom.name for atom in _camera.active_blueprints)
    ),
    _lidar,
    _scene,
    SimModule.blueprint(
        backend=MujocoBackend,
        observation_hz=200,
        joint_ids=JOINT_IDS,
    ),
    SimJointEncoderModule.blueprint(sensor=ENCODERS),
    VoxelGridMapper.blueprint(
        device="CPU:0", carve_columns=False, block_count=100_000, emit_every=5
    ).remappings([(VoxelGridMapper, "lidar", "pointcloud")]),
    G1PolicyModule.blueprint(scene=_MODEL),
).global_config(viewer="none", transport="zenoh", n_workers=8)
