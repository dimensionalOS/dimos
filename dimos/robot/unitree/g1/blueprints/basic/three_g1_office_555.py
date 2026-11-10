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
from dimos.robot.unitree.g1.sim2 import HOME_HEIGHT, G1Sim
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
    # At HOME_HEIGHT, which is where a G1's pelvis is when it stands in the pose it starts in.
    # The frame says where the robot stands and nothing else does: `compile_world` zeroes the
    # offset the robot's own layer carries from its root up to its pelvis, in this branch as in
    # the other, so the two no longer add. They did once, and the robot hung 0.76 m in the air:
    #
    #     freejoint on /office_555/main   z = 0.755   (this frame)
    #                  main/pelvis        z = 1.548   (0.755 + the robot's own 0.793)
    #
    # and the fix of putting this frame on the floor only halved the error -- the pelvis then
    # started at 0.793, which is where the layer draws it with every joint at zero, 38 mm above
    # where the same robot's pelvis sits once its knees are bent into the home pose. Zeroing the
    # root is what makes HOME_HEIGHT mean what it says.
    {name: ("office_555", (x, y, HOME_HEIGHT), (0.0, 0.0, 0.0)) for name, (x, y) in SPOTS.items()}
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
    ),
    vis_module(viewer_backend=global_config.viewer),
    scene_manager,
    *_robots.values(),
).global_config(n_workers=17, robot_model="unitree_g1")
