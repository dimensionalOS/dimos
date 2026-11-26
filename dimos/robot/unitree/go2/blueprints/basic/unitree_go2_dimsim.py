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

"""A Go2 as DimSim's kinematic base in DimSim's apartment.

Only DimSim runs this scene: its meshes are MuJoCo-visible (for the sensors) but not MuJoCo-solid.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.dimsim import Go2KinematicBase
from dimos.robot.unitree.go2.sim2 import GO2_ODOM_TF, Go2Sim
from dimos.sim2.camera import SimCamera
from dimos.sim2.dimsim.apartment import DimSimApartment
from dimos.sim2.dimsim.sim import DimSimSim
from dimos.sim2.l1 import SimL1
from dimos.sim2.odom_tf import OdomTf
from dimos.sim2.sim import on_mount, sim
from dimos.visualization.vis_module import vis_module

ROBOT = DimSimApartment.mounts.go2

unitree_go2_dimsim = autoconnect(
    sim({"dimsim": DimSimSim.blueprint()}, default="dimsim"),
    DimSimApartment.blueprint(),
    on_mount(
        autoconnect(
            Go2KinematicBase.blueprint(attached_to=ROBOT.frame),
            OdomTf.blueprint(**GO2_ODOM_TF),
            SimL1.attach(Go2Sim.mounts.l1),
            SimCamera.attach(Go2Sim.mounts.camera),
        ),
        ROBOT,
        shared={"cmd_vel"},
    ),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
).global_config(robot_model="unitree_go2")
