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

"""Galaxea A1Z teleop blueprints, through ``A1ZConnection``."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.a1z.config import (
    a1z_coordinator_connection,
    a1z_model,
    make_a1z_model_config,
)
from dimos.robot.manipulators.a1z.connection import A1ZConnection
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.urdf_manager import UrdfManager
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


class A1ZTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the Desktop Manipulation app;
    one A1Z named "arm" out, driven through ``A1ZConnection`` by joint targets."""

    right_gripper_command: In[Float32]
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


class A1ZKeyboardCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper openings in, from the keyboard; one A1Z
    named "arm" out, driven through ``A1ZConnection`` by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


class A1ZWebXRCoordinator(TeleopControlCoordinator):
    """VR controller poses and gripper openings in, from WebXR; one A1Z named
    "arm" out, driven through ``A1ZConnection`` by joint targets."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def _remaps(coordinator: type[ModuleBase]) -> list[tuple[type[ModuleBase], str, str]]:
    """The connection's ports, onto ``coordinator``'s ports for the robot named "arm"."""
    return [
        (A1ZConnection, "position_command", "arm_position_command"),
        (coordinator, "arm_joint_state", "joint_state"),
    ]


def _eef_twist_task() -> TaskConfig:
    """Move the A1Z's gripper end by the twists that come in on ``ee_twist_command``."""
    ik_model = make_a1z_model_config(robot="arm")
    return TaskConfig(
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
    )


def _webxr_task() -> TaskConfig:
    """Make the A1Z's gripper end follow the left VR controller."""
    ik_model = make_a1z_model_config(robot="arm")
    return TaskConfig(
        name="teleop_a1z",
        type="teleop_ik",
        joint_names=list(ik_model.joint_names),
        priority=20,
        params={
            "robot_model": ik_model,
            "bindings": [{"hand": "left", "target_frame": "gripper_eef_link"}],
            "max_joint_velocity_rad_s": 2.0,
        },
    )


def _teleop(
    coordinator: type[ControlCoordinator],
    arm_task: TaskConfig,
    gripper_stream: str,
    *inputs: Blueprint,
) -> Blueprint:
    """Move an A1Z's end and open and close its gripper, through ``A1ZConnection``.
    ``UrdfManager`` serves the A1Z's URDF and puts its joints on tf, so it can be drawn.

    Wire it up with ``.remappings(_remaps(coordinator))``. The CAN port is given
    at run time (``--a1zconnection.can-port a1zcan``) or as ``A1ZCONNECTION__CAN_PORT``.

    Args:
        coordinator: The coordinator class, with the input streams the teleop sends on.
        arm_task: The task that moves the arm's joints.
        gripper_stream: The stream that carries gripper openings (0 closed, 1 open).
        inputs: The modules that send the commands, if any.
    """
    model = a1z_model(robot="arm")
    return autoconnect(
        *inputs,
        coordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[a1z_coordinator_connection()],
            tasks=[
                arm_task,
                # arm/gripper is open at its upper end, the task's default.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                    stream_bind={"gripper_command": gripper_stream},
                ),
            ],
        ),
        A1ZConnection.blueprint(),
        UrdfManager.blueprint(model=model),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
# From the Desktop Manipulation app.
a1z_teleop = _teleop(A1ZTwistCoordinator, _eef_twist_task(), "right_gripper_command").remappings(
    _remaps(A1ZTwistCoordinator)
)
# From the keyboard, in a pygame window.
a1z_keyboard_teleop = _teleop(
    A1ZKeyboardCoordinator, _eef_twist_task(), "gripper_command", KeyboardTeleopModule.blueprint()
).remappings(_remaps(A1ZKeyboardCoordinator))
# From a VR headset's left controller; teleop_webxr_a1z adds the WebXR module.
coordinator_teleop_a1z = _teleop(
    A1ZWebXRCoordinator, _webxr_task(), "left_gripper_command"
).remappings(_remaps(A1ZWebXRCoordinator))
