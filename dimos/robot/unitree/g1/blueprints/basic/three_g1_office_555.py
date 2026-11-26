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

"""Three G1s standing in office 555, each walking under its own GR00T policy.

Only `main` takes `cmd_vel` from outside its namespace, so a joystick drives that one; `left`
and `right` stand under their own policies. A G1 is driven joint by joint, so its physics is MuJoCo,
and its walker runs its policy at 50 Hz of wall time, as on the real robot.
"""

import math

from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.hardware_module import Mounts
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2 import G1_ODOM_TF, G1Sim
from dimos.robot.unitree.g1.sim2_walker import G1SimWalker
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.odom_tf import OdomTf
from dimos.sim2.scene import SimScene, SimSceneConfig
from dimos.sim2.sim import on_mount, sim
from dimos.simulation.scenes.office_555.room import SCENE
from dimos.visualization.vis_module import vis_module

# Where each robot starts, on the floor: a mount puts the robot's USD root there, and its layer lifts the pelvis to standing.
SPOTS = {
    "main": (-0.85, -7.66),
    "left": (0.65, -7.66),
    "right": (-0.85, -6.16),
}
# A room camera behind the robots, as an optical frame (z forward, y down): looking along +y, 25 degrees down.
OVERVIEW = ((-0.1, -10.5, 2.2), (-(math.pi / 2 + math.radians(25)), 0.0, 0.0))
DRIVEN = "main"


class Office555Config(SimSceneConfig):
    usd: UsdModel = UsdModel.from_file(SCENE).with_frames(
        {name: ("office_555", (x, y, 0.0), (0.0, 0.0, 0.0)) for name, (x, y) in SPOTS.items()}
        | {"overview": ("office_555", *OVERVIEW)}
    )


class Office555(SimScene):
    config: Office555Config
    mounts = Mounts(*(f"office_555/{name}" for name in [*SPOTS, "overview"]))


def _g1(spot: str) -> MetaBlueprint:
    mount = getattr(Office555.mounts, spot)
    return on_mount(
        autoconnect(
            G1Sim.blueprint(attached_to=mount.frame),
            OdomTf.blueprint(**G1_ODOM_TF),
            LivoxSim.attach(G1Sim.mounts.lidar),
            G1SimWalker.blueprint(),
        ).remappings([(LivoxSim, "imu", "lidar_imu")]),
        mount,
        shared={"cmd_vel"} if spot == DRIVEN else set(),
    )


three_g1_office_555 = autoconnect(
    sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
    Office555.blueprint(),
    SimCamera.attach(Office555.mounts.overview),
    *(_g1(spot) for spot in SPOTS),
    vis_module(viewer_backend=global_config.viewer),
).global_config(robot_model="unitree_g1")
