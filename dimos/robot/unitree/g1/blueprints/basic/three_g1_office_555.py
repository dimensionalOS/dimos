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
"""

from dimos.control.components import HardwareComponent, HardwareType, make_humanoid_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.sim2 import G1Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.livox import LivoxSim
from dimos.sim2.module import Simulation
from dimos.simulation.scenes.office_555 import SCENE
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
SHARED = {"world_observations", "actions", "cmd_vel"}
# The rate the GR00T policy was trained at, so the tick runs it every tick.
POLICY_HZ = 50.0


class _Walker(TeleopControlCoordinator):
    """The G1's own whole-body coordinator, one per robot.

    A subclass only so each robot gets its own class-named instance; the ports and the task
    registry are the coordinator's. Nothing here is sim-specific -- `transport_lcm` is the same
    adapter the real G1 runs behind, reading `motor_states` and `imu` and writing
    `motor_command`, and in a namespace those three names are that robot's alone.
    """


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


# One link per robot, all fixed to the world, for the scene manager to graft a robot onto. A
# mounts dict is keyed by frame, so three robots need three frames: grafted at one they would all
# land on the same prim and come out as a single G1 with the frame count of three.
MOUNT_URDF = "".join(
    ['<robot name="office_555"><link name="world" />']
    + [
        f'<link name="{name}" />'
        f'<joint name="world_to_{name}" type="fixed">'
        f'<parent link="world" /><child link="{name}" /></joint>'
        for name in SPOTS
    ]
    + ["</robot>"]
)


def _robot(name: str, xy: tuple[float, float]):
    """One G1 in the room: the sim module, the description it serves, and its Mid-360."""
    return (
        autoconnect(
            # The robot id and the namespace are the same word on purpose: what
            # the world calls this robot is what namespace() prefixes its frames with, so a cloud in
            # "left/mid360_link" finds "left/pelvis" above it.
            G1Sim.blueprint(robot_id=name, xy=xy, frame_id=f"{name}/pelvis"),
            UsdManager.blueprint(),
            LivoxSim.blueprint(frame_id=G1Sim.lidar_mount_frame),
            _walker(name),
        )
        .remappings(
            # The manager poses the description from a joint state, and G1Sim publishes that as
            # motor_states. Without this it listens on a topic nobody writes, never poses anything,
            # and the robot's own frames -- the Mid-360's mount among them -- never reach tf.
            [
                (UsdManager, "coordinator_joint_state", "motor_states"),
                # The coordinator's velocity input is named for the task that reads it; what the
                # Controller's joystick publishes is cmd_vel.
                (_Walker, "twist_command", "cmd_vel"),
                # The Mid-360 has an IMU of its own, and in this namespace it would publish on
                # the same "imu" the whole-body adapter reads. The policy would then be fed a
                # sensor bolted upside down on the torso instead of the one in the pelvis, and
                # would fight a gravity vector that points the wrong way.
                (LivoxSim, "imu", "lidar_imu"),
            ]
        )
        .namespace(name, expose=SHARED)
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


_robots = {name: _robot(name, xy) for name, xy in SPOTS.items()}

# The one manager a viewer talks to. The Controller asks a single "*.usd" on the unnamespaced
# /render_file_request and draws whatever one stage comes back; three namespaced managers are on
# three topics it never asks, which is why three robots could be standing there -- three lidars,
# three tf trees -- with no robot geometry drawn at all. This one holds the mounts, so get_scene
# collects all three descriptions into that single stage, each under its own namespace's frames.
scene_manager = UsdManager.blueprint(
    model=RobotModel.from_urdf_string(MOUNT_URDF),
    # Its own three joints are fixed and sit at the origin, so the viewer places those frames
    # from the stage itself. Nothing sends it a joint state and it poses nothing: each robot's
    # own manager publishes that robot's joints, and G1Sim publishes world -> <robot>/pelvis.
    mounts={name: _manager_name(blueprint) for name, blueprint in _robots.items()},
    # It is a scene, not a robot: its frames are the world's and take no prefix. Set here rather
    # than left out, because namespace() below would otherwise fill it in.
    frame_id_prefix="",
).namespace(
    # A namespace of its own, for one reason: a manager outside every namespace hears
    # @shared_config(UsdManager) from all three robots and refuses, having been handed four
    # models when it answers for one. The topics a viewer reaches it on stay unnamespaced.
    "scene",
    expose={"render_file_request", "file_info", "tf"},
)

three_g1_office_555 = autoconnect(
    Simulation.blueprint(scene=SCENE),
    vis_module(viewer_backend=global_config.viewer),
    scene_manager,
    *_robots.values(),
).global_config(n_workers=17, robot_model="unitree_g1")
