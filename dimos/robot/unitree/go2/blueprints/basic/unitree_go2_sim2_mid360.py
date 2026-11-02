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

"""The sim2 Go2 with a Mid-360 on the real rig's mount, its fitted pattern and mount occlusion."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    MID360_PITCH_DOWN,
    MID360_XYZ,
    OPTICAL_RPY,
)
from dimos.robot.unitree.go2.sim2 import GO2_MJCF, GO2_SIM_MODEL, Go2Sim
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.module import Simulation
from dimos.visualization.vis_module import vis_module

# The real rig's frames (go2_mid360_static_transforms.FRAMES).
CAMERA_FRAME = "front_camera"
MID360_FRAME = "mid360_link"
CAMERA_OPTICAL_FRAME = "camera_optical"

unitree_go2_sim2_mid360 = autoconnect(
    Simulation.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(),
    UrdfManager.blueprint(
        model=GO2_SIM_MODEL.with_fixed_frame(CAMERA_FRAME, "base_link", xyz=CAMERA_XYZ)
        .with_fixed_frame(
            MID360_FRAME, CAMERA_FRAME, xyz=MID360_XYZ, rpy=(0.0, MID360_PITCH_DOWN, 0.0)
        )
        .with_fixed_frame(CAMERA_OPTICAL_FRAME, CAMERA_FRAME, rpy=OPTICAL_RPY),
        formats={"mjcf": GO2_MJCF},
    ).remappings([(UrdfManager, "coordinator_joint_state", "joint_state")]),
    LivoxSim.blueprint(frame_id=MID360_FRAME, go2_occlusion=True),
    SimCamera.blueprint(frame_id=CAMERA_OPTICAL_FRAME),
).global_config(n_workers=4, robot_model="unitree_go2")
