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
explicit prototype optical frame: X right/Y down/Z forward. The independent
Mid360 IMU is deliberately colocated with the laser for the simulated device.
"""

from dataclasses import replace
from functools import partial
import math
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.sim2_policy import CONTROL, JOINTS, POLICY, G1PolicyModule
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing
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

IMU = ImuConfig("g1/imu_in_pelvis", source_id="g1/imu", orientation="ideal")
MID360_IMU = ImuConfig("g1/mid360_link", source_id="g1/mid360/imu")
ENCODERS = EncoderConfig(JOINTS, source_id="g1/encoders")
CAMERA = CameraConfig("g1/camera", source_id="g1/camera")
LIDAR = LidarConfig(
    "g1/mid360_link",
    source_id="g1/mid360",
    pattern=partial(Mid360Scan, LfsPath("mid360_pattern/fourier.npz"), downsample=10),
    min_range=0.16,
    response=Mid360Returns,
    # The vendor head mesh seals the optical window. Until that asset has a
    # separate window, omit only this head link from optical queries; retain
    # every other robot link and all physical collisions.
    excluded_bodies=("g1/head_link",),
)


def _g1_sim2(*, pointlio: bool = False) -> Blueprint:
    localization = (
        (
            PointLio.blueprint(
                sensor_frame_id="g1/mid360_link",
                # The simulated device IMU is colocated with the laser.
                extrinsic_t=[0.0, 0.0, 0.0],
            ).remappings([(PointLio, "lidar_raw", "filtered_pointcloud")]),
        )
        if pointlio
        else ()
    )
    return autoconnect(
        UrdfManagerPointClearing.blueprint(
            models=MODELS,
            padding_m=0.05,
            filter_geometry="visual",
            external_pose_frames={"g1": "g1/mid360_link"} if pointlio else {},
        ).remappings(
            [(UrdfManagerPointClearing, "pointcloud", "lidar_raw" if pointlio else "pointcloud")]
        ),
        SimModule.blueprint(
            scene=SCENE,
            backend=MujocoBackend,
            controllers=(CONTROL,),
            joint_ids=JOINTS,
        ),
        SimImuModule.blueprint(sensor=IMU),
        SimImuModule.blueprint(instance_name="mid360_imu", sensor=MID360_IMU).remappings(
            [("mid360_imu", "imu", "imu_raw")]
        ),
        SimJointEncoderModule.blueprint(sensor=ENCODERS),
        SimCameraModule.blueprint(scene=SCENE, sensor=CAMERA, renderer=MujocoRenderer),
        SimLidarModule.blueprint(
            scene=SCENE,
            sensor=replace(LIDAR, publish_truth=not pointlio),
            raycaster=MujocoRaycaster,
        ).remappings(
            [
                (SimLidarModule, "lidar", "lidar_raw"),
                (SimLidarModule, "pointcloud", "truth_pointcloud" if pointlio else "pointcloud"),
            ]
        ),
        VoxelGridMapper.blueprint(
            device="CPU:0",
            carve_columns=False,
            block_count=100_000,
            frame_id="odom" if pointlio else "world",
        ).remappings(
            [(VoxelGridMapper, "lidar", "pointcloud" if pointlio else "filtered_pointcloud")]
        ),
        G1PolicyModule.blueprint(),
        *localization,
    )


unitree_g1_sim2 = _g1_sim2().global_config(viewer="none", transport="zenoh", n_workers=5)
unitree_g1_sim2_pointlio = _g1_sim2(pointlio=True).global_config(
    viewer="none", transport="zenoh", n_workers=5
)
