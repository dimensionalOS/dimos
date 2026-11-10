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

"""A-750 teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.a750.config import a750_model, make_a750_model_config
from dimos.robot.manipulators.a750.connection import A750Connection
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class A750TwistCoordinator(ArmTwistCoordinator):
    """End-effector twists in; one A-750 named "arm" out, driven through ``A750Connection``."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
A750_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (A750Connection, "position_command", "arm_position_command"),
    (A750TwistCoordinator, "arm_joint_state", "joint_state"),
]


def _keyboard_teleop() -> Blueprint:
    """Jog an A-750's end from this computer's keyboard, and plan its motions,
    through ``A750Connection``.

    Wire it up with ``.remappings(A750_REMAPS)``. The serial port is given at
    run time, after the blueprint name (``--a750connection.device-path
    /dev/ttyACM0``) or as ``A750CONNECTION__DEVICE_PATH``.
    """
    model = make_a750_model_config(robot="arm")
    return autoconnect(
        KeyboardTeleopModule.blueprint(),
        A750TwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            tick_rate=100.0,
            publish_joint_state=True,
            joint_state_frame_id="coordinator",
            model=a750_model(robot="arm"),
            tasks=[
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(model.joint_names),
                    priority=10,
                    params={"robot_model": model, "target_frame": "gripper_base"},
                ),
                joint_trajectory_task([*model.joint_names, "arm/finger"]),
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/finger"],
                    priority=20,
                ),
            ],
        ),
        ManipulationModule.blueprint(
            gripper_tasks={"manipulator": "arm_gripper"},
            model=model,
            visualization={"backend": "viser"},
        ),
        A750Connection.blueprint(),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
keyboard_teleop_a750 = _keyboard_teleop().remappings(A750_REMAPS)
