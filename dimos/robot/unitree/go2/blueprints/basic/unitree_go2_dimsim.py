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
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.kinematic_base import SimKinematicBase
from dimos.sim2.l1 import SimL1
from dimos.sim2.module import Simulation
from dimos.sim2.utils.mjcf_to_urdf import mjcf_to_urdf

unitree_go2_dimsim = autoconnect(
    Simulation.blueprint(
        backend="dimsim", scene=Path(__file__).parents[5] / "sim2" / "logistics_props.xml"
    ),
    SimKinematicBase.blueprint(robot_id="go2", model=Go2Sim.mjcf, root_body="base"),
    UrdfManager.blueprint(model=mjcf_to_urdf(Go2Sim.mjcf, "go2", "base_link")),
    SimL1.blueprint(frame_id=Go2Sim.l1_mount_frame),
    SimCamera.blueprint(frame_id=Go2Sim.camera_mount_frame),
).global_config(n_workers=6, robot_model="unitree_go2")
