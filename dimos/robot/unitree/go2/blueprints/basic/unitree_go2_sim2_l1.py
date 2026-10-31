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

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.go2_mid360_static_transforms import CAMERA_XYZ, OPTICAL_RPY
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.l1 import SimL1
from dimos.sim2.module import Simulation
from dimos.sim2.spec import Mount
from dimos.visualization.vis_module import vis_module

_GO2_DESCRIPTION = (
    RobotDescriptionSource(
        url="https://github.com/unitreerobotics/unitree_ros",
        ref="38a9b00ecbff0b70e85a7627eb9c65db9a2341f7",
    )
    / "robots"
    / "go2_description"
)


class Go2UrdfRoot(StaticTfPublisher):
    """base_link -> base, the root link of Unitree's go2_description URDF."""

    def transforms(self) -> list[Transform]:
        return [Transform(frame_id="base_link", child_frame_id="base")]


unitree_go2_sim2_l1 = autoconnect(
    Simulation.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(),
    Go2UrdfRoot.blueprint(),
    UrdfManager.blueprint(
        model=RobotModel.from_file(
            _GO2_DESCRIPTION / "urdf" / "go2_description.urdf",
            package_paths={"go2_description": _GO2_DESCRIPTION},
        )
    ).remappings([(UrdfManager, "coordinator_joint_state", "joint_state")]),
    SimL1.blueprint(),
    SimCamera.blueprint(robot_id="go2", mount=Mount("base", CAMERA_XYZ, OPTICAL_RPY)),
).global_config(n_workers=6, robot_model="unitree_go2")
