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

The room is the physics as well as the picture, and it is the *same layer* for both: the room's
manager composes `scene.usda` with the robots grafted in (449 bodies, 3925 geoms before they
arrive), and that composition is what every part of this reads.

Each robot walks under its own GR00T whole-body controller, the same task and the same ONNX
policy the real G1 runs: the coordinator reads that robot's `motor_states` and `imu`, and writes
its `motor_command`. `cmd_vel` is deliberately *not* namespaced, so one joystick in the
Controller drives all three at once -- three robots walking off one command is what shows the
policy is per-robot rather than a single rig being puppeted.

Each robot is a `G1Sim` with its own `robot_id` and its own `UsdManager`, under its own
namespace. One manager per robot is not an accident of wiring: a manager answers for exactly one
model, so three robots sharing one would have to pick which of them it describes. Namespacing
gives each its own `joint_state` and its own tf, and `expose` lets the three of them still reach
the single world they are standing in.

Above those three sits the room's own manager, and what *it* composes is the world: the office
with all three robots and their lidars grafted into it. That one layer is what the physics runs,
what each lidar ranges against, and what the viewer draws -- which is the point. A scan landing
outside the walls that are drawn is the usual sign those have drifted apart, and now there is
only one of them to drift.
"""

from dimos.control.components import HardwareComponent, HardwareType, make_humanoid_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2 import HOME_HEIGHT, G1Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.livox import LivoxSim
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.simulation.scenes.office_555.room import SCENE
from dimos.utils.data import LfsPath
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
# The namespace the room's manager lives in, and so the name everything that needs the world
# asks it by. Its own namespace because a manager outside every namespace would be a fourth one
# answering on the same unnamespaced topics as the three robots'.
SCENE_NAMESPACE = "scene"
SCENE_MANAGER = f"{SCENE_NAMESPACE}/{UsdManager.name}"
# The one robot the joystick drives. Its cmd_vel is the unnamespaced topic the Controller
# publishes on; the other two keep theirs inside their namespace, where nothing writes, so
# they stand still. One joystick driving all three at once was a demo, not a thing anyone
# would want while trying to drive a robot.
DRIVEN = "main"
# The rate the GR00T policy was trained at, so the tick runs it every tick.
POLICY_HZ = 50.0


class _Walker(TeleopControlCoordinator):
    """The G1's own whole-body coordinator, one per robot.

    A subclass for two reasons. Each robot gets its own class-named instance; and the velocity
    input is named `cmd_vel` rather than the coordinator's own `twist_command`, because the
    Controller decides whether a blueprint can be driven by looking for an input stream whose
    *name* contains "cmd_vel", read from the blueprint catalog rather than from topics. Pointing
    `twist_command` at the cmd_vel topic drives the robots perfectly well from a script and
    still leaves the app saying "nothing to drive", which is no use to someone holding a
    joystick. `stream_bind` on the task below joins the two.

    Nothing else here is sim-specific: `transport_lcm` is the same adapter the real G1 runs
    behind, reading `motor_states` and `imu` and writing `motor_command`, and in a namespace
    those three names are that robot's alone.
    """

    cmd_vel: In[Twist]


def _walker(name: str):
    """The GR00T whole-body controller that keeps one robot upright and walks it."""
    # A component's hardware_id is also the prefix on its joint names, and it is what the
    # transport adapter builds its topics from. Three robots called "g1" would be one set of
    # joints on one set of topics, so each takes its own name all the way down.
    joints = make_humanoid_joints(name)
    return _Walker.blueprint(
        tick_rate=POLICY_HZ,
        hardware=[
            HardwareComponent(
                hardware_id=name,
                hardware_type=HardwareType.WHOLE_BODY,
                joints=joints,
                adapter_type="transport_lcm",
                address="",
                wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
            )
        ],
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="g1_groot_wbc",
                # Legs and waist; the arms are nobody's here.
                joint_names=joints[:15],
                priority=50,
                auto_start=True,
                # The task's own card input is twist_command; read it off the cmd_vel port.
                stream_bind={"twist_command": "cmd_vel"},
                params={
                    "model_path": LfsPath("groot"),
                    "hardware_id": name,
                    # There is nobody to click Activate in a sim, and nothing to protect.
                    "auto_arm": True,
                    "auto_dry_run": False,
                    "default_ramp_seconds": 0.0,
                    # One inference per tick: the tick already runs at the policy's rate.
                    "decimation": 1,
                },
            )
        ],
    )


# The room itself, with a frame per robot added for the scene manager to graft onto. A mounts
# dict is keyed by frame, so three robots need three frames: grafted at one they would all land
# on the same prim and come out as a single G1 with the frame count of three.
#
# The office USD rather than a stub of frames, because it is the one place that says what this
# world is -- its geometry, and its sun, its dome light and its eighteen ceiling fittings. Served
# as a stub, none of that reached the viewer, which then lit the room with a guess of its own and
# showed flat black through its windows. Its meshes are file references with hashes already, the
# same form a robot's are, so the viewer reads them by the path it already has.
# "office_555" is the scene's root link: a converted room is one Xform holding its furniture,
# and the furniture is what carries the frames, so a visitor hangs off the root itself.
# At HOME_HEIGHT rather than on the floor, because this frame is the one statement of where a
# robot stands: the simulation gets this world from the manager and finds its robots already in
# it, so a frame on the floor would be a G1 with its pelvis at ankle height. It used to be said
# twice -- here at z=0 for the picture, and again as G1Sim's xy plus HOME_HEIGHT for the physics
# -- agreeing only because both read SPOTS.
SCENE_WITH_MOUNTS = UsdModel.from_file(SCENE).with_frames(
    {name: ("office_555", (x, y, HOME_HEIGHT), (0.0, 0.0, 0.0)) for name, (x, y) in SPOTS.items()}
)


def _robot(name: str):
    """One G1 in the room: its sim module, the manager that answers for it, and its Mid-360."""
    return (
        autoconnect(
            # The robot id and the namespace are the same word on purpose: what
            # the world calls this robot is what namespace() prefixes its frames with, so a cloud in
            # "left/mid360_link" finds "left/pelvis" above it.
            G1Sim.blueprint(
                robot_id=name,
                # No xy: the scene's mount frame places this robot now.
                frame_id=f"{name}/pelvis",
                # Where it reads the world from, for the IMU in its own pelvis.
                scene_manager=SCENE_MANAGER,
                # No fade-in, to match the controller arming with no ramp. The fade-in exists
                # for a real G1, where something else holds the robot while stiffness comes up.
                # Here nothing does: until the first command the servos' own gains hold the
                # stance, and the first command replaces them -- scaled from zero, so the legs
                # go limp at the one moment they are carrying the robot, and it is on the floor
                # long before the three seconds are up.
                soft_start_seconds=0.0,
            ),
            # The lidar is mounted by the instance name its namespace gives it, not by the
            # blueprint object: a mounts value is read again in the worker the manager lands in,
            # by which point a blueprint has been flattened to a plain dict and is no longer
            # recognised ("mounts values are a Module subclass, a Blueprint or an instance
            # name; got {'blueprints': ...}").
            UsdManager.blueprint(
                model=G1Sim.usd,
                mounts={G1Sim.lidar_mount_frame: f"{name}/{LivoxSim.name}"},
            ),
            LivoxSim.blueprint(frame_id=G1Sim.lidar_mount_frame, scene_manager=SCENE_MANAGER),
            _walker(name),
        )
        .remappings(
            [
                # The Mid-360 has an IMU of its own, and in this namespace it would publish on
                # the same "imu" the whole-body adapter reads. The policy would then be fed a
                # sensor bolted upside down on the torso instead of the one in the pelvis, and
                # would fight a gravity vector that points the wrong way.
                (LivoxSim, "imu", "lidar_imu"),
            ]
        )
        .namespace(name, expose=SHARED | ({"cmd_vel"} if name == DRIVEN else set()))
    )


def _manager_name(blueprint) -> str:
    """The deployed instance name of a robot's UsdManager, for the scene manager to mount.

    The blueprint object itself is the documented form, but a mounts value is read again in the
    worker the manager lands in, by which point the blueprint has been flattened to a plain dict
    and is no longer recognised. The instance name is the other documented form and is just a
    string, so it arrives intact; taking it off the atom keeps it in step with however the
    coordinator names instances.
    """
    return next(atom.name for atom in blueprint.blueprints if atom.module is UsdManager)


_robots = {name: _robot(name) for name in SPOTS}

# The one manager a viewer talks to. The Controller asks a single "*.usd" on the unnamespaced
# /render_file_request and draws whatever one stage comes back; three namespaced managers are on
# three topics it never asks, which is why three robots could be standing there -- three lidars,
# three tf trees -- with no robot geometry drawn at all. This one holds the mounts, so get_scene
# collects all three descriptions into that single stage, each under its own namespace's frames.
scene_manager = UsdManager.blueprint(
    model=SCENE_WITH_MOUNTS,
    # Its own three joints are fixed and sit at the origin, so the viewer places those frames
    # from the stage itself. Nothing sends it a joint state and it poses nothing: each robot's
    # own manager publishes that robot's joints, and G1Sim publishes world -> <robot>/pelvis.
    mounts={name: _manager_name(blueprint) for name, blueprint in _robots.items()},
    # It is a scene, not a robot: its frames are the world's and take no prefix. Set here rather
    # than left out, because namespace() below would otherwise fill it in.
    frame_id_prefix="",
    # The scene's root link is "office_555", and tf has never heard of it: the robots publish
    # world -> <robot>/pelvis, so "world" is the frame everything else is placed against. Under
    # its own name the room has no transform and the viewer says so outright -- "office_555 isn't
    # in the tree ... so nothing places it in the view" -- and draws none of it. Renaming the
    # root on the way out is what joins the room to the frame the robots already stand in.
    tf_renames={"office_555": "world"},
).namespace(
    # A namespace of its own, so it is not a fourth manager answering on the same unnamespaced
    # topics as the three robots'. The topics a viewer reaches it on stay unnamespaced.
    SCENE_NAMESPACE,
    expose={"render_file_request", "file_info", "tf"},
)

three_g1_office_555 = autoconnect(
    sim_module(
        # The world is what the scene's manager composed: the room, with all three robots and
        # their lidars grafted into it. Not the room on its own -- that would be a world the
        # physics ran with nothing standing in it, and three robots publishing into it.
        scene_manager=SCENE_MANAGER,
        # The one thing a layer cannot say: what the world calls each robot. Where each stands is
        # the room's mount frame, and the gains and pose that hold them up are in G1Sim.usd.
        robots=[SimulatedRobot(name, G1Sim.usd) for name in SPOTS],
    ),
    vis_module(viewer_backend=global_config.viewer),
    scene_manager,
    *_robots.values(),
).global_config(n_workers=17, robot_model="unitree_g1")
