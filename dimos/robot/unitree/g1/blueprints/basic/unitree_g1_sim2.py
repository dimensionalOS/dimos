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

"""G1/GR00T and a desk, using independently acquired sensor measurements.

Model ledger: vendor G1 URDF, metres/radians, X forward/Y left/Z up.
Pelvis IMU and native Mid360 axes use the existing URDF links. Camera is an
explicit prototype optical frame: X right/Y down/Z forward.
"""

from functools import partial
import math
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.sim2_policy import CONTROL, JOINTS, POLICY, G1PolicyModule
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.description import Drive, Instance, SceneConfig
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.data import LfsPath

G1_MODEL = (
    RobotModel.from_file(
        Path(__file__).parents[2] / "g1.urdf", package_paths={"g1_description": LfsPath("g1_urdf")}
    )
    .with_subtree_rooted_at("pelvis")
    .with_fixed_frame(
        "camera", "torso_link", xyz=(0.05, 0, 0.40), rpy=(-math.pi / 2, 0, -math.pi / 2)
    )
    .with_prefix("g1/")
)
MODELS = {
    "g1": G1_MODEL,
    "desk": RobotModel.from_file(
        DIMOS_PROJECT_ROOT / "assets/g1_desk/desk.urdf",
        package_paths={"g1_desk": LfsPath("g1_desk")},
    ),
    "box": RobotModel.from_file(DIMOS_PROJECT_ROOT / "assets/g1_desk/box.urdf").with_prefix("box/"),
}
SCENE = SceneConfig(
    instances=(
        Instance(
            "g1",
            xyz=(0, 0, 0.793),
            floating=True,
            drives=tuple(
                Drive(n, kp, kd, home, 0.01, 0.1)
                for n, kp, kd, home in zip(
                    JOINTS, G1_GROOT_KP, G1_GROOT_KD, POLICY.default_positions_29, strict=True
                )
            ),
        ),
        Instance("desk", xyz=(2, 0, 0), rpy=(0, 0, math.pi / 2)),
        Instance("box", xyz=(1, 1, 0.05), floating=True),
    ),
)

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
    excluded_bodies=("g1/head_link",),
)


# The mapper is an unchanged downstream consumer of truth-registered scans.
# Robot returns remain in the scans and can accumulate in this preview map.
unitree_g1_sim2 = autoconnect(
    UrdfManager.blueprint(models=MODELS),
    SimModule.blueprint(
        scene=SCENE,
        backend=MujocoBackend,
        observation_hz=200,
        controllers=(CONTROL,),
        joint_ids=JOINTS,
    ),
    SimImuModule.blueprint(sensor=IMU),
    SimJointEncoderModule.blueprint(sensor=ENCODERS),
    SimCameraModule.blueprint(scene=SCENE, sensor=CAMERA, renderer=MujocoRenderer),
    SimLidarModule.blueprint(scene=SCENE, sensor=LIDAR, raycaster=MujocoRaycaster).remappings(
        [(SimLidarModule, "lidar", "lidar_raw")]
    ),
    VoxelGridMapper.blueprint(
        device="CPU:0", carve_columns=False, block_count=100_000, emit_every=5
    ).remappings([(VoxelGridMapper, "lidar", "pointcloud")]),
    G1PolicyModule.blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=8)
