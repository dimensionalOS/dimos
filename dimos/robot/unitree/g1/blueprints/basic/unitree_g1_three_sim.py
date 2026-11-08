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


"""Three G1s in one simulated world, each with its own controller: a root one and two namespaced.

One `SimG1Many` owns the MuJoCo world and steps it. Three `G1Sim` modules each own one robot's
walking actor and publish that robot's tf, odometry and joint state -- the first under the names
a controller already drives, the other two under their own namespaces, so three copies of the
same robot stay apart on one bus.

The three streams the world and the controllers share are exposed through the namespaces: the
world publishes every robot's state on one pair of topics and routes the targets back by the
prefix on the joint names.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.simulation.g1_sim.sim_module import G1Sim
from dimos.simulation.g1_sim.world import SimG1Many

# Where each robot stands in the generated office of the sim's default seed, 1.5 m apart on
# clear floor around its start. The world refuses to begin with a robot inside the furniture,
# so a scene that does not fit them says so rather than starting wrong.
MAIN, LEFT, RIGHT = "main_", "left_", "right_"
ROBOTS = {
    MAIN: (1.0, 1.0, 0.0),
    LEFT: (2.5, 1.0, 0.0),
    RIGHT: (1.0, 2.5, 0.0),
}
# What crosses a namespace boundary: the world's view of every robot, and the targets going
# back. Everything else a G1Sim publishes is that one robot's and stays inside its namespace.
SHARED = {"sim_odom", "sim_state", "joint_target"}

unitree_g1_three_sim = autoconnect(
    SimG1Many.blueprint(robots=ROBOTS),
    # The root robot takes no namespace, so its frames keep the names a controller already drives.
    G1Sim.blueprint(prefix=MAIN),
    G1Sim.blueprint(prefix=LEFT).namespace("left", expose=SHARED),
    G1Sim.blueprint(prefix=RIGHT).namespace("right", expose=SHARED),
)
