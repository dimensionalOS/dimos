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

"""G1/GR00T and a desk: pose-only camera/lidar, separate robot feedback.

Model ledger: vendor G1 URDF, metres/radians, X forward/Y left/Z up.
Pelvis IMU mount matches the existing GR00T model. Camera/lidar mounts are
explicit prototype placements on torso_link; optical camera X right/Y down/Z
forward. Fixed mount links are massless and share the robot's one model.
"""

import math
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.demo_sim2_control import JOINTS, POLICY, G1PolicyDemo
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.description import Camera, Drive, Imu, Instance, Lidar, World
from dimos.sim2.physics.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.module import SimSensorModule
from dimos.sim2.sensors.mujoco import MujocoSensors
from dimos.utils.data import LfsPath

G1_MODEL = (
    RobotModel.from_file(
        Path(__file__).parents[2] / "g1.urdf", package_paths={"g1_description": LfsPath("g1_urdf")}
    )
    .with_subtree_rooted_at("pelvis")
    .with_fixed_frame("imu", "pelvis", xyz=(0.04525, 0, -0.08339))
    .with_fixed_frame(
        "camera", "torso_link", xyz=(0.05, 0, 0.40), rpy=(-math.pi / 2, 0, -math.pi / 2)
    )
    .with_fixed_frame("lidar", "torso_link", xyz=(0.07, 0, 0.42))
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
WORLD = World(
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
    sensors=(Camera("g1/camera", hz=15), Lidar("g1/lidar", rays=720, packets=10), Imu("g1/imu")),
)

unitree_g1_sim2 = autoconnect(
    UrdfManager.blueprint(models=MODELS).remappings(
        [("urdfmanager", "coordinator_joint_state", "joint_state")]
    ),
    SimModule.blueprint(world=WORLD, backend=MujocoBackend, control_joints=JOINTS),
    G1PolicyDemo.blueprint(),
    SimSensorModule.blueprint(reader=MujocoSensors, camera="g1/camera", lidar="g1/lidar"),
).global_config(viewer="none", transport="zenoh", n_workers=4)
