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

"""Mobile R1 Pro in SF555, with standard sensors and DimOS Desktop controls.

Prepare the R1 Pro model and office assets; see docs/development/sim2-r1pro.md.
"""

from dataclasses import replace
from functools import partial

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.galaxea.r1pro.sim.config import CONTROL, JOINTS, MODEL, SCENE as ROBOT_SCENE
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.description import Instance, SceneConfig
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.scenes.sf555.config import OFFICE_MODEL, OFFICE_RENDERER
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.data import LfsPath

MODELS = {"r1pro": MODEL, "office": OFFICE_MODEL}
SCENE = SceneConfig(
    instances=(replace(ROBOT_SCENE.instances[0], xyz=(16, 1, 0.005)), Instance("office")),
    ground_plane=False,
)
# Use the vendor optical frame. Intrinsics are the existing office demo's
# pinhole approximation, not the hardware stereo calibration.
CAMERA = CameraConfig(
    "r1pro/camera_head_left_link",
    width=640,
    height=480,
    fx=376.72,
    fy=376.72,
    cx=320,
    cy=240,
)
LIDAR = LidarConfig(
    "r1pro/lidar_chassis_left_link",
    # Preserve rolling acquisition times with 1/50 of the hardware rays.
    pattern=partial(Mid360Scan, LfsPath("mid360_pattern/fourier.npz"), downsample=50),
    min_range=0.16,
    response=Mid360Returns,
)

r1pro_sim2 = autoconnect(
    UrdfManager.blueprint(models=MODELS),
    SimModule.blueprint(
        scene=SCENE,
        backend=MujocoBackend,
        dt_ns=5_000_000,
        startup_timeout_s=120,
        observation_hz=200,
        controllers=(CONTROL,),
        joint_ids=JOINTS,
    ),
    SimJointEncoderModule.blueprint(sensor=EncoderConfig(JOINTS, hz=50)),
    SimImuModule.blueprint(
        instance_name="chassis_imu",
        sensor=ImuConfig("r1pro/imu_chassis_link", orientation="ideal"),
    ).remappings([("chassis_imu", "imu", "imu_chassis")]),
    SimImuModule.blueprint(
        instance_name="torso_imu",
        sensor=ImuConfig("r1pro/imu_torso_link", orientation="ideal"),
    ).remappings([("torso_imu", "imu", "imu_torso")]),
    SimCameraModule.blueprint(scene=SCENE, sensor=CAMERA, renderer=OFFICE_RENDERER),
    SimLidarModule.blueprint(scene=SCENE, sensor=LIDAR, raycaster=MujocoRaycaster).remappings(
        [(SimLidarModule, "lidar", "lidar_raw")]
    ),
    VoxelGridMapper.blueprint(
        device="CPU:0", carve_columns=False, block_count=100_000, emit_every=5
    ).remappings([(VoxelGridMapper, "lidar", "pointcloud")]),
    R1ProSimControl.blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=9)
