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

"""unitree-go2-basic in sim2: the walking Go2, its head L1 and front camera, its URDF posed for viewers."""

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    L1_MOUNT,
    L1_XYZ,
    OPTICAL_RPY,
)
from dimos.robot.unitree.go2.sim2 import GO2_MJCF, GO2_SIM_MODEL, Go2Sim
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.l1 import SimL1
from dimos.sim2.module import Simulation
from dimos.visualization.vis_module import vis_module

L1_FRAME = "utlidar_lidar"
CAMERA_FRAME = "front_camera"
CAMERA_OPTICAL_FRAME = "camera_optical"

unitree_go2_sim2_l1 = autoconnect(
    Simulation.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(),
    UrdfManager.blueprint(
        model=GO2_SIM_MODEL.with_fixed_frame(
            L1_FRAME, "base_link", xyz=L1_XYZ, rpy=tuple(math.radians(a) for a in L1_MOUNT)
        )
        .with_fixed_frame(CAMERA_FRAME, "base_link", xyz=CAMERA_XYZ)
        .with_fixed_frame(CAMERA_OPTICAL_FRAME, CAMERA_FRAME, rpy=OPTICAL_RPY),
        formats={"mjcf": GO2_MJCF},
    ).remappings([(UrdfManager, "coordinator_joint_state", "joint_state")]),
    SimL1.blueprint(frame_id=L1_FRAME),
    SimCamera.blueprint(frame_id=CAMERA_OPTICAL_FRAME),
).global_config(n_workers=6, robot_model="unitree_go2")
