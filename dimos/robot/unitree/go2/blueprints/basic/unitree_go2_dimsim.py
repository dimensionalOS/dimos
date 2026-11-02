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

"""The Go2 in sim2's DimSim backend: a kinematic base the size of the Go2 pushing the props, with its head L1 and front camera."""

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_sim2_l1 import (
    CAMERA_OPTICAL_FRAME,
    GO2_L1_MODEL,
    L1_FRAME,
)
from dimos.robot.unitree.go2.sim2 import GO2_MJCF, GO2_ROBOT_ID
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.kinematic_base import SimKinematicBase
from dimos.sim2.l1 import SimL1
from dimos.sim2.module import Simulation

unitree_go2_dimsim = autoconnect(
    Simulation.blueprint(
        backend="dimsim", scene=Path(__file__).parents[5] / "sim2" / "logistics_props.xml"
    ),
    SimKinematicBase.blueprint(robot_id=GO2_ROBOT_ID, root_body="base"),
    UrdfManager.blueprint(model=GO2_L1_MODEL, formats={"mjcf": GO2_MJCF}),
    SimL1.blueprint(frame_id=L1_FRAME),
    SimCamera.blueprint(frame_id=CAMERA_OPTICAL_FRAME),
).global_config(n_workers=6, robot_model="unitree_go2")
