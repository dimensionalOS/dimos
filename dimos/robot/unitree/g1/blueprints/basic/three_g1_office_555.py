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

"""Three G1s standing in office_555: one world, three robots, each with its own description.

The room is the physics as well as the picture, and it is the *same file* for both: MuJoCo reads
the converted `scene.usda` directly, so `Simulation` compiles the very layer the viewer draws
(449 bodies, 3925 geoms). A lidar return therefore comes off the furniture that is on screen
rather than off a generated stand-in -- a scan landing outside the walls that are drawn is the
usual sign those two have drifted apart.

Each robot is a `G1Sim` with its own `robot_id` and its own `UsdManager`, under its own
namespace. One manager per robot is not an accident of wiring: a manager answers for exactly one
model, so three robots sharing one would have to pick which of them it describes. Namespacing
gives each its own `joint_state` and its own tf, and `expose` lets the three of them still reach
the single world they are standing in.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.g1.sim2 import G1Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.livox import LivoxSim
from dimos.sim2.module import Simulation
from dimos.simulation.scenes.office_555 import SCENE
from dimos.visualization.vis_module import vis_module

# Clear floor in the room's open plan, measured off the layer rather than guessed: far enough
# apart that no two of them start in contact, close enough that one camera frames all three.
SPOTS = {
    "main": (-0.85, -7.66),
    "left": (0.65, -7.66),
    "right": (-0.85, -6.16),
}
# What each namespace lets through to the shared world: the simulation is one world for all
# three, so its traffic must not be namespaced apart with the rest of the robot's topics.
SHARED = {"world_observations", "actions"}


def _robot(name: str, xy: tuple[float, float]):
    """One G1 in the room: the sim module, the description it serves, and its Mid-360."""
    return autoconnect(
        # The robot id and the namespace are the same word on purpose: what
        # the world calls this robot is what namespace() prefixes its frames with, so a cloud in
        # "left/mid360_link" finds "left/pelvis" above it.
        G1Sim.blueprint(robot_id=name, xy=xy, frame_id=f"{name}/pelvis"),
        UsdManager.blueprint(),
        LivoxSim.blueprint(frame_id=G1Sim.lidar_mount_frame),
    ).namespace(name, expose=SHARED)


three_g1_office_555 = autoconnect(
    Simulation.blueprint(scene=SCENE),
    vis_module(viewer_backend=global_config.viewer),
    *[_robot(name, xy) for name, xy in SPOTS.items()],
).global_config(n_workers=10, robot_model="unitree_g1")
