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

"""Galaxea A1Z teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.a1z.config import (
    a1z_hardware,
    a1z_model,
    make_a1z_model_config,
)
from dimos.robot.manipulators.a1z.connection import A1ZConnection
from dimos.robot.manipulators.common.blueprints import (
    eef_twist_task,
    teleop_ik_task,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.urdf_manager import UrdfManager
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_a1z_keyboard_hw = a1z_hardware("arm")
_a1z_model = make_a1z_model_config()

keyboard_teleop_a1z = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_a1z_keyboard_hw],
        tasks=[
            eef_twist_task(
                _a1z_keyboard_hw,
                robot_model=_a1z_model,
                target_frame="gripper_eef_link",
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
            ),
            trajectory_task(_a1z_keyboard_hw, priority=20),
        ],
    ),
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_a1z_model,
        visualization={"backend": "viser"},
    ),
)


_a1z_webxr_hw = a1z_hardware("arm")
_a1z_webxr_model = make_a1z_model_config()

coordinator_teleop_a1z = autoconnect(
    TeleopControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_a1z_webxr_hw],
        tasks=[
            teleop_ik_task(
                _a1z_webxr_hw,
                name="teleop_a1z",
                robot_model=_a1z_webxr_model,
                bindings=[
                    {
                        "hand": "left",
                        "target_frame": "gripper_eef_link",
                    }
                ],
                priority=20,
                params={"max_joint_velocity_rad_s": 2.0},
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=["arm/gripper"],
                priority=20,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            trajectory_task(_a1z_webxr_hw),
        ],
    ),
    ManipulationModule.blueprint(
        gripper_tasks={"manipulator": "arm_gripper"},
        model=_a1z_webxr_model,
        visualization={"backend": "viser"},
    ),
)


class A1ZTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the Desktop Manipulation app;
    one A1Z named "arm" out, driven through ``A1ZConnection`` by joint targets."""

    right_gripper_command: In[Float32]
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
_ARM_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (A1ZConnection, "position_command", "arm_position_command"),
    (A1ZTwistCoordinator, "arm_joint_state", "joint_state"),
]


def _teleop() -> Blueprint:
    """Jog an A1Z's end and open and close its gripper from the Desktop
    Manipulation app, through ``A1ZConnection``. ``UrdfManager`` serves the
    A1Z's URDF and puts its joints on tf, so the app can draw it.

    Wire it up with ``.remappings(_ARM_REMAPS)``. The CAN port is given at run
    time, after the blueprint name (``--a1zconnection.can-port a1zcan``) or as
    ``A1ZCONNECTION__CAN_PORT``.
    """
    ik_model = make_a1z_model_config(robot="arm")
    model = a1z_model(robot="arm")
    return autoconnect(
        A1ZTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=model,
            tasks=[
                TaskConfig(
                    name=EEF_TWIST_TASK_NAME,
                    type="eef_twist",
                    joint_names=list(ik_model.joint_names),
                    priority=10,
                    params={
                        "robot_model": ik_model,
                        "target_frame": "gripper_eef_link",
                        # The app sends twists only while a jog is held: the
                        # end stops 0.3 s after the last one.
                        "timeout": 0.3,
                    },
                ),
                # arm/gripper is open at its upper end, the task's default.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                    stream_bind={"gripper_command": "right_gripper_command"},
                ),
            ],
        ),
        A1ZConnection.blueprint(),
        UrdfManager.blueprint(model=model),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
a1z_teleop = _teleop().remappings(_ARM_REMAPS)
