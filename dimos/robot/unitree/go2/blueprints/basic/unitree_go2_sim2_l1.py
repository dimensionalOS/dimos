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

"""unitree-go2-basic in sim2: the walking Go2 in the logistics room, its head L1 and front camera.

Two managers, nested, which is the shape every one of these has. The inner one answers for the
*robot*: the Go2's own layer with each sensor grafted onto the frame it is bolted to. The outer
one answers for the *world*: the room, with the robot grafted onto a spot on its floor. What
that outer one composes is the world -- `get_scene` hands out one layer holding the room, the
robot and the sensors -- and it is what the physics runs and what every sensor renders from, so
a scan lands on the furniture that is drawn.

Nobody describes themselves to the simulation. The blueprint says which robots are in the world,
because the blueprint is where that is already written down, and the robot's gains and the pose
it holds are in its layer.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.camera import SimCamera
from dimos.sim2.l1 import SimL1
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.simulation.go2_legged.robot import CONTROL_DT
from dimos.visualization.vis_module import vis_module

# What the world calls this robot: the entity id its actions are addressed to, and the prefix on
# every frame of it.
ROBOT = "go2"
# Where it stands, said once. The scene's mount frame is the one statement of it: the simulation
# gets this world from the manager and finds the robot already in it, so a second xyz on the
# description could only disagree with the room.
SPOT = (0.0, 0.0, 0.35)
# The namespace the scene's manager lives in, and so the name the things that ask it for the
# scene use. A manager outside every namespace would be a second one answering on the same
# unnamespaced topics as the robot's.
SCENE = "scene"
SCENE_MANAGER = f"{SCENE}/{UsdManager.name}"

# The room, with a spot on its floor for the robot. A mounts dict is keyed by frame, so a frame
# is what a visitor hangs off; the room arrives with frames for its own furniture and none for
# anything standing in it.
_room = UsdModel.from_file(LOGISTICS).with_frames({ROBOT: ("logistics", SPOT, (0.0, 0.0, 0.0))})

# The sensors as their own blueprints, because the manager mounts them by the instance each one
# deploys: a frame_id alone would name a module the manager cannot address. Each asks the scene's
# manager for the world it is in and builds its own copy of it.
_lidar = SimL1.blueprint(frame_id=Go2Sim.l1_mount_frame, scene_manager=SCENE_MANAGER)
_camera = SimCamera.blueprint(frame_id=Go2Sim.camera_mount_frame, scene_manager=SCENE_MANAGER)

# One manager holding the whole robot: the Go2's own layer, with each sensor grafted onto the
# frame it is bolted to. Without the mounts it answers for the robot alone, so the lidar and the
# camera are nowhere in the stage a viewer draws.
_robot = UsdManager.blueprint(
    model=Go2Sim.usd, mounts={Go2Sim.l1_mount_frame: _lidar, Go2Sim.camera_mount_frame: _camera}
).remappings([(UsdManager, "coordinator_joint_state", "joint_state")])

_scene = UsdManager.blueprint(
    model=_room,
    mounts={ROBOT: _robot},
    # It is a scene, not a robot: its frames are the world's and take no prefix.
    frame_id_prefix="",
    # The room's root link is "logistics" and tf has never heard of it: the robot publishes
    # world -> go2/base, so "world" is what everything else is placed against.
    tf_renames={"logistics": "world"},
).namespace(SCENE, expose={"render_file_request", "file_info", "tf"})

unitree_go2_sim2_l1 = autoconnect(
    sim_module(
        scene_manager=SCENE_MANAGER,
        # The one thing a layer cannot say: what the world calls this robot, and how often its
        # controller acts. Its gains and the pose it holds are in the layer.
        robots=[SimulatedRobot(ROBOT, Go2Sim.usd, control_period=CONTROL_DT)],
    ),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(robot_id=ROBOT),
    _scene,
    _robot,
    _lidar,
    _camera,
).global_config(n_workers=8, robot_model="unitree_go2")
