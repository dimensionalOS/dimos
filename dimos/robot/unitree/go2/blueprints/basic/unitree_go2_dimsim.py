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

"""The Go2 in sim2's DimSim backend: a kinematic base the size of the Go2 pushing the props, with its head L1 and front camera.

The same two nested managers as the MuJoCo one: the inner answers for the *robot* (the Go2's own
layer with each sensor grafted onto the frame it is bolted to), the outer for the *world* (the
room, with the robot grafted onto a spot on its floor). What differs is below the contract -- this
engine has no articulation, so the Go2 does not walk here: it slides as one box, driven by cmd_vel
through its base joints.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.assets.logistics_props import LOGISTICS_PROPS
from dimos.sim2.camera import SimCamera
from dimos.sim2.dimsim.kinematic_base import SimKinematicBase
from dimos.sim2.l1 import SimL1
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.visualization.vis_module import vis_module

# What the world calls this robot: the entity id its actions are addressed to, and the prefix on
# every frame of it.
ROBOT = "go2"
# Where it stands, said once, as the scene's mount frame. Here it is where the *box's bottom*
# goes, because that is the whole of this robot.
SPOT = (0.0, 0.0, 0.0)
# The namespace the scene's manager lives in, and so the name the things that ask it for the scene
# use. A manager outside every namespace would be a second one answering on the same unnamespaced
# topics as the robot's.
SCENE = "scene"
SCENE_MANAGER = f"{SCENE}/{UsdManager.name}"

# The room and its props, with a spot on the floor for the robot. A mounts dict is keyed by frame,
# so a frame is what a visitor hangs off, and the room arrives with frames for its own furniture
# and none for anything standing in it.
_room = UsdModel.from_file(LOGISTICS_PROPS).with_frames(
    {ROBOT: ("logistics", SPOT, (0.0, 0.0, 0.0))}
)

# The sensors as their own blueprints, because the manager mounts them by the instance each one
# deploys: a frame_id alone would name a module the manager cannot address.
_lidar = SimL1.blueprint(frame_id=Go2Sim.l1_mount_frame, scene_manager=SCENE_MANAGER)
_camera = SimCamera.blueprint(frame_id=Go2Sim.camera_mount_frame, scene_manager=SCENE_MANAGER)

# Mounted by the instance name each one deploys, not by the blueprint object: a mounts value is
# read again in the worker the manager lands in, by which point a blueprint has been flattened to
# a plain dict and is no longer recognised.
#
# No joint_state remapping, unlike the walking blueprint: this engine has no articulation, so
# nothing here ever says where a leg is and the manager would be fed from nowhere.
_robot = UsdManager.blueprint(
    model=Go2Sim.usd,
    mounts={Go2Sim.l1_mount_frame: SimL1.name, Go2Sim.camera_mount_frame: SimCamera.name},
)

_scene = UsdManager.blueprint(
    model=_room,
    mounts={ROBOT: UsdManager.name},
    # It is a scene, not a robot: its frames are the world's and take no prefix.
    frame_id_prefix="",
    # The room's root link is "logistics" and tf has never heard of it: the base publishes
    # world -> base_link, so "world" is what everything else is placed against.
    tf_renames={"logistics": "world"},
).namespace(SCENE, expose={"render_file_request", "file_info", "tf"})

unitree_go2_dimsim = autoconnect(
    sim_module(
        "dimsim",
        scene_manager=SCENE_MANAGER,
        # What the world calls this robot, and where it stands. Unlike the articulated engines
        # this one does not take the robot out of the composed scene -- it replaces it with a box
        # of its own -- so the spot is said to both, from the one constant, rather than once.
        # No control_period: a kinematic base has no controller for the world to wait on.
        robots=[SimulatedRobot(ROBOT, Go2Sim.usd, xyz=SPOT)],
    ),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    SimKinematicBase.blueprint(robot_id=ROBOT),
    _scene,
    _robot,
    _lidar,
    _camera,
).global_config(n_workers=8, robot_model="unitree_go2")
