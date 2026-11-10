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


from dimos.control.components import HardwareComponent, HardwareType, make_humanoid_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2 import G1Sim
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.livox import LivoxSim
from dimos.sim2.sim import sim_module
from dimos.sim2.spec import SimulatedRobot
from dimos.simulation.scenes.office_555.room import SCENE
from dimos.utils.data import LfsPath
from dimos.visualization.vis_module import vis_module

SPOTS = {
    "main": (-0.85, -7.66),
    "left": (0.65, -7.66),
    "right": (-0.85, -6.16),
}
SHARED = {"world_observations", "actions"}
SCENE_NAMESPACE = "scene"
SCENE_MANAGER = f"{SCENE_NAMESPACE}/{UsdManager.name}"
DRIVEN = "main"
POLICY_HZ = 50.0


class _Walker(TeleopControlCoordinator):
    cmd_vel: In[Twist]


def _walker(name: str) -> Blueprint:
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
                joint_names=joints[:15],
                priority=50,
                auto_start=True,
                stream_bind={"twist_command": "cmd_vel"},
                params={
                    "model_path": LfsPath("groot"),
                    "hardware_id": name,
                    "auto_arm": True,
                    "auto_dry_run": False,
                    "default_ramp_seconds": 0.0,
                    "decimation": 1,
                },
            )
        ],
    )


SCENE_WITH_MOUNTS = UsdModel.from_file(SCENE).with_frames(
    # On the floor. A mount frame in a composed scene says where the robot's *asset* goes, not
    # where its pelvis goes -- `graft_usd` puts the layer's root prim there and the layer's own
    # offsets place everything above it -- so a frame on the floor is a G1 drawn standing, pelvis
    # at the 0.793 its layer carries. HOME_HEIGHT here would add the two and hang it 0.76 m up.
    #
    # It does start 38 mm above where it finishes: 0.793 is the pose the layer is drawn in, every
    # joint at zero, and the home pose bends the knees, which lowers the pelvis to about 0.755
    # without lowering the feet. Nothing is wrong with that -- the drop is 0.09 s of free fall and
    # the policies are already driving by the time the world steps (see `startup_delay`). The old
    # path hid it by adding a keyframe's base height to this frame; there is no keyframe now.
    {name: ("office_555", (x, y, 0.0), (0.0, 0.0, 0.0)) for name, (x, y) in SPOTS.items()}
)


def _robot(name: str) -> Blueprint:
    return (
        autoconnect(
            G1Sim.blueprint(
                robot_id=name,
                frame_id=f"{name}/pelvis",
                scene_manager=SCENE_MANAGER,
                soft_start_seconds=0.0,
            ),
            UsdManager.blueprint(
                model=G1Sim.usd,
                mounts={G1Sim.lidar_mount_frame: f"{name}/{LivoxSim.name}"},
            ),
            LivoxSim.blueprint(frame_id=G1Sim.lidar_mount_frame, scene_manager=SCENE_MANAGER),
            _walker(name),
        )
        .remappings(
            [
                (LivoxSim, "imu", "lidar_imu"),
            ]
        )
        .namespace(name, expose=SHARED | ({"cmd_vel"} if name == DRIVEN else set()))
    )


def _manager_name(blueprint: Blueprint) -> str:
    return str(next(atom.name for atom in blueprint.blueprints if atom.module is UsdManager))


def _walker_name(blueprint: Blueprint) -> str:
    """The deployed instance name of a robot's coordinator, for the world to ask about arming.

    Taken off the atom rather than spelled out, for the reason `_manager_name` is: it stays in
    step with however the coordinator names instances, namespace prefix and all.
    """
    return str(next(atom.name for atom in blueprint.blueprints if atom.module is _Walker))


_robots = {name: _robot(name) for name in SPOTS}

scene_manager = UsdManager.blueprint(
    model=SCENE_WITH_MOUNTS,
    mounts={name: _manager_name(blueprint) for name, blueprint in _robots.items()},
    frame_id_prefix="",
    tf_renames={"office_555": "world"},
).namespace(
    SCENE_NAMESPACE,
    expose={"render_file_request", "file_info", "tf"},
)

three_g1_office_555 = autoconnect(
    sim_module(
        scene_manager=SCENE_MANAGER,
        robots=[SimulatedRobot(name, G1Sim.usd) for name in SPOTS],
        # Who the world waits for: it starts the moment all three coordinators report their
        # GR00T task armed, and `startup_delay` below is only the cap on that wait.
        wait_for=[_walker_name(blueprint) for blueprint in _robots.values()],
        # Long enough for all three policies to have the joints before the world starts. These
        # robots are held up by their policies and not by their servos -- left at the home pose
        # a G1 folds in about a second and a half -- and a policy cannot command one until it
        # has seen its joints, which it reads from this world's own observations. Building this
        # world takes most of twenty seconds, because every module that wants a view of the room
        # composes it again and compiles its own copy, so the controllers used to arm 8.5 to
        # 9.8 s after stepping had already begun, by which time the robots were on the floor.
        startup_delay=20.0,
    ),
    vis_module(viewer_backend=global_config.viewer),
    scene_manager,
    *_robots.values(),
).global_config(n_workers=17, robot_model="unitree_g1")
