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
import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.robot.galaxea.r1pro.sim.camera_module import R1ProSimCameras
from dimos.robot.galaxea.r1pro.sim.cameras import (
    WRIST_MAX_DEPTH_M,
    WRIST_MIN_DEPTH_M,
    head_camera,
    wrist_camera,
)
from dimos.robot.galaxea.r1pro.sim.config import CONTROL, JOINTS, MODEL, SCENE as ROBOT_SCENE
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl
from dimos.robot.galaxea.r1pro.sim.spatial import R1ProSpatialTeleop
from dimos.robot.galaxea.r1pro.sim.teleop import (
    TELEOP_MODEL,
    R1ProTeleopCoordinator,
    arm_tasks,
)
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.description import Instance, SceneConfig
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.scenes.sf555.config import OFFICE_MODEL, OFFICE_RENDERER
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.teleop.spatial.model import SpatialGroup
from dimos.utils.data import LfsPath

MODELS = {"r1pro": MODEL, "office": OFFICE_MODEL}
SCENE = SceneConfig(
    instances=(replace(ROBOT_SCENE.instances[0], xyz=(16, 1, 0.005)), Instance("office")),
    ground_plane=False,
)
# Testing default; opt back into the native office renderer without rewiring streams.
CAMERA_RENDERERS = {"mujoco": MujocoRenderer, "unreal": OFFICE_RENDERER}
CAMERA_RENDERER = CAMERA_RENDERERS[os.environ.get("DIMOS_R1PRO_CAMERA_RENDERER", "mujoco")]
CAMERA = head_camera()
LIDAR = LidarConfig(
    "r1pro/lidar_chassis_left_link",
    # Preserve rolling acquisition times with 1/50 of the hardware rays.
    pattern=partial(Mid360Scan, LfsPath("mid360_pattern/fourier.npz"), downsample=50),
    min_range=0.16,
    response=Mid360Returns,
)

r1pro_sim2 = (
    autoconnect(
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
        R1ProSimCameras.blueprint(
            scene=SCENE,
            head=CAMERA,
            left=wrist_camera("left"),
            right=wrist_camera("right"),
            renderer=CAMERA_RENDERER,
        ),
        SimLidarModule.blueprint(scene=SCENE, sensor=LIDAR, raycaster=MujocoRaycaster).remappings(
            [(SimLidarModule, "lidar", "lidar_raw")]
        ),
        VoxelGridMapper.blueprint(
            device="CPU:0", carve_columns=False, block_count=100_000, emit_every=5
        ).remappings([(VoxelGridMapper, "lidar", "pointcloud")]),
        R1ProSimControl.blueprint(
            upper_command_joints=tuple(
                f"r1pro/{side}_arm_joint{i}" for side in ("left", "right") for i in range(1, 8)
            )
        ),
        R1ProSpatialTeleop.blueprint(
            title="R1 Pro · SF555",
            environment="Simulation",
            joint_jog_available=True,
            robot_model=RobotModelConfig(
                model=MODEL, joint_names=list(JOINTS), base_link="r1pro/base_link"
            ),
            groups={
                side: SpatialGroup(
                    label=f"{side.title()} arm",
                    joint_names=tuple(f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)),
                    tip_frame=f"r1pro/{side}_gripper_link",
                    base_frame="r1pro/base_link",
                )
                for side in ("left", "right")
            },
            cameras={
                "workspace": "Head · RGB-D",
                "left": "Left wrist · RGB-D",
                "right": "Right wrist · RGB-D",
            },
            camera_depth_ranges={
                side: (WRIST_MIN_DEPTH_M, WRIST_MAX_DEPTH_M) for side in ("left", "right")
            },
            camera_max_edge_m={side: 0.035 for side in ("left", "right")},
            base_drive=True,
            max_depth_m=12.0,
            max_edge_m=0.25,
        ),
        R1ProTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=TELEOP_MODEL,
            tasks=arm_tasks(),
            tick_rate=50,
            publish_joint_state=False,
        ),
    )
    .remappings(
        [
            (R1ProSpatialTeleop, "left_controller_output", "left_cartesian_command"),
            (R1ProSpatialTeleop, "right_controller_output", "right_cartesian_command"),
            (R1ProTeleopCoordinator, "r1pro_joint_state", "joint_state"),
            (R1ProSpatialTeleop, "coordinator_joint_state", "joint_state"),
            (R1ProTeleopCoordinator, "r1pro_position_command", "upper_command"),
        ]
    )
    .global_config(viewer="none", transport="zenoh", n_workers=11)
)
