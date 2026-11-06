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

"""R1 Pro mobile simulation with browser base and upper-body teleoperation.

Prepare the simulation URDF first with dimos.robot.galaxea.r1pro.sim.model.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.sim.config import CONTROL, JOINTS, MODEL, SCENE
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl
from dimos.robot.galaxea.r1pro.sim.viewer import R1ProSimViewer
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule

r1pro_sim2 = autoconnect(
    UrdfManager.blueprint(models={"r1pro": MODEL}),
    SimModule.blueprint(
        scene=SCENE,
        backend=MujocoBackend,
        observation_hz=50,
        controllers=(CONTROL,),
        joint_ids=JOINTS,
    ),
    SimJointEncoderModule.blueprint(sensor=EncoderConfig(JOINTS, hz=50)),
    R1ProSimControl.blueprint(),
    R1ProSimViewer.blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=5)
