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

"""G1/GR00T, desk and device-owned sensors composed by UsdManager."""

from functools import partial

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.module import mid360_usd
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2_policy import JOINTS, G1PolicyModule
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
from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.data import LfsPath

IMU = ImuConfig("g1/imu_in_pelvis", orientation="ideal")
ENCODERS = EncoderConfig(JOINTS, hz=50)
CAMERA = CameraConfig("g1/camera")
LIDAR = LidarConfig(
    "g1/mid360_link",
    pattern=partial(Mid360Scan, LfsPath("mid360_pattern/fourier.npz"), downsample=10),
    min_range=0.16,
    response=Mid360Returns,
    # The vendor head mesh seals the optical window. Until that asset has a
    # separate window, omit only this head link from optical queries; retain
    # every other robot link and all physical collisions.
    excluded_bodies=("g1/head_link", "g1/mid360_link_body"),
)


# Providers own their device subtrees. The world manager composes all mounts;
# consumers refer only to that manager, never a parallel Python model ledger.
_imu = SimImuModule.blueprint(instance_name="imu", sensor=IMU)
_camera = SimCameraModule.blueprint(instance_name="camera", sensor=CAMERA)
_lidar = SimLidarModule.blueprint(
    instance_name="lidar",
    scene_manager="scene",
    sensor=LIDAR,
    raycaster=MujocoRaycaster,
    model=UsdModel.from_layer(mid360_usd(LIDAR.frame_id, "g1/mid360_imu")),
).remappings([(SimLidarModule, "lidar", "lidar_raw")])


_scene = UsdManager.blueprint(
    instance_name="scene",
    model=UsdModel.from_file(LfsPath("sim2_g1_usd/world.usda")),
    mounts={
        "g1/imu_mount": _imu,
        "g1/camera_mount": _camera,
        "g1/lidar_mount": _lidar,
    },
)

unitree_g1_sim2 = autoconnect(
    _imu,
    _camera,
    CameraRenderModule.blueprint(
        scene_manager=_scene, renderer=MujocoRenderer, cameras=("camera",)
    ),
    _lidar,
    _scene,
    SimModule.blueprint(
        scene_manager=_scene,
        backend=MujocoBackend,
        observation_hz=200,
        joint_ids=JOINTS,
    ),
    SimJointEncoderModule.blueprint(sensor=ENCODERS),
    VoxelGridMapper.blueprint(
        device="CPU:0", carve_columns=False, block_count=100_000, emit_every=5
    ).remappings([(VoxelGridMapper, "lidar", "pointcloud")]),
    G1PolicyModule.blueprint(scene_manager=_scene),
).global_config(viewer="none", transport="zenoh", n_workers=8)
