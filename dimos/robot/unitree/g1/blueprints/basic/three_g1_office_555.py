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
and `right` stand under their own policies. A G1 is driven joint by joint, so its physics is MuJoCo.
"""

import math

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_legs_waist
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.hardware_module import Mounts
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2 import G1Sim, G1SimConfig
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.mujoco.sim import MujocoSim
from dimos.sim2.scene import SimScene, SimSceneConfig
from dimos.sim2.sim import sim
from dimos.simulation.scenes.office_555.room import SCENE
from dimos.utils.data import LfsPath
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
WORLD = {"world_observations", "actions"}
# GR00T was trained at 50 Hz.
POLICY_HZ = 50.0


class Office555Config(SimSceneConfig):
    usd: UsdModel | None = UsdModel.from_file(SCENE).with_frames(
        {name: ("office_555", (x, y, 0.0), (0.0, 0.0, 0.0)) for name, (x, y) in SPOTS.items()}
        | {"overview": ("office_555", *OVERVIEW)}
    )


class Office555(SimScene):
    config: Office555Config
    mounts = Mounts(*SPOTS, "overview")


class _G1Walker(TeleopControlCoordinator):
    cmd_vel: In[Twist]
    g1_position_command: Out[JointState]
    g1_joint_state: In[JointState]
    g1_imu: In[Imu]


def _walker() -> Blueprint:
    return _G1Walker.blueprint(
        tick_rate=POLICY_HZ,
        connections=[
            ConnectionConfig(**G1SimConfig().model_dump(include=set(ConnectionConfig.model_fields)))
        ],
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="g1_groot_wbc",
                joint_names=g1_legs_waist,
                priority=50,
                auto_start=True,
                stream_bind={"twist_command": "cmd_vel"},
                params={
                    "model_path": LfsPath("groot"),
                    "hardware_id": "g1",
                    "auto_arm": True,
                    "auto_dry_run": False,
                    "default_ramp_seconds": 0.0,
                    "decimation": 1,
                },
            )
        ],
    )


def _g1(spot: str) -> MetaBlueprint:
    return (
        autoconnect(
            G1Sim.blueprint(_mount=spot),
            LivoxSim.mount(G1Sim.mounts.lidar),
            _walker(),
        )
        .remappings(
            [
                (G1Sim, "position_command", "g1_position_command"),
                (_G1Walker, "g1_joint_state", "joint_state"),
                (_G1Walker, "g1_imu", "imu"),
                (LivoxSim, "imu", "lidar_imu"),
            ]
        )
        .namespace(spot, shared=WORLD | ({"cmd_vel"} if spot == DRIVEN else set()))
    )


three_g1_office_555 = autoconnect(
    sim({"mujoco": MujocoSim.blueprint()}, default="mujoco"),
    Office555.blueprint(),
    SimCamera.mount(Office555.mounts.overview),
    *(_g1(spot) for spot in SPOTS),
    vis_module(viewer_backend=global_config.viewer),
).global_config(robot_model="unitree_g1")
