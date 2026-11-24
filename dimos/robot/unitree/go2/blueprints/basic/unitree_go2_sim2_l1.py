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


"""A Go2 with an L1 and a camera in the logistics room; `--choice sim=dimsim` swaps MuJoCo for DimSim.

MuJoCo walks the Go2 on its locomotion policy; DimSim moves it as a kinematic base on cmd_vel.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.sim2 import GO2_TF_FRAMES, GO2_USD, Go2Sim
from dimos.sim2.assets.logistics import Logistics
from dimos.sim2.camera import SimCamera
from dimos.sim2.dimsim.kinematic_base import SimKinematicBase
from dimos.sim2.dimsim.sim import DimSimSim
from dimos.sim2.l1 import SimL1
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.sim import sim
from dimos.visualization.vis_module import vis_module

ROBOT = Logistics.mounts.go2
SHARED = {"world_observations", "actions", "cmd_vel"}

unitree_go2_sim2_l1 = autoconnect(
    # Each backend comes with the Go2 it can move: joint-driven in MuJoCo, a kinematic base in DimSim.
    sim(
        {
            "mujoco": autoconnect(
                MujocoSim.blueprint(),
                Go2Sim.blueprint(mount=ROBOT).namespace(ROBOT, shared=SHARED),
            ),
            "dimsim": autoconnect(
                DimSimSim.blueprint(),
                SimKinematicBase.blueprint(usd=GO2_USD, mount=ROBOT, **GO2_TF_FRAMES).namespace(
                    ROBOT, shared=SHARED
                ),
            ),
        },
        default="mujoco",
    ),
    Logistics.blueprint(),
    autoconnect(
        SimL1.mount(Go2Sim.mounts.l1),
        SimCamera.mount(Go2Sim.mounts.camera),
    ).namespace(ROBOT, shared=SHARED - {"actions", "cmd_vel"}),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
).global_config(robot_model="unitree_go2")
