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

"""OpenYAM keyboard and WebXR teleoperation over its connection."""

from copy import deepcopy

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    DamiaoTwistCoordinator,
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.openyam.blueprints.basic import _gripper_task, _trajectory_task
from dimos.robot.manipulators.openyam.config import make_openyam_model_config
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.joints import OPENYAM_GRIPPER_JOINT
from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.webxr.extensions import ArmTeleopModule

_openyam_model = make_openyam_model_config(robot="arm")


def _eef_twist_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(_openyam_model.joint_names),
        priority=priority,
        params={"robot_model": _openyam_model, "target_frame": "gripper_tip"},
    )


def _twist_control(tasks: list[TaskConfig]) -> Blueprint:
    """Twist-driven coordinator named ControlCoordinator over the connection."""
    return autoconnect(
        DamiaoTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(OpenYamConnection)],
            tasks=list(tasks),
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenYamConnection),
    ).remappings(arm_remaps(OpenYamConnection, DamiaoTwistCoordinator))


def _teleop_control(tasks: list[TaskConfig]) -> Blueprint:
    """WebXR coordinator named ControlCoordinator over the connection."""
    return autoconnect(
        DamiaoTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(OpenYamConnection)],
            tasks=list(tasks),
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenYamConnection),
    ).remappings(arm_remaps(OpenYamConnection, DamiaoTeleopCoordinator))


keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    _twist_control([_eef_twist_task(), _gripper_task()]),
    ManipulationModule.blueprint(
        model=_openyam_model,
        gripper_tasks={"manipulator": "arm_gripper"},
        visualization={"backend": "viser"},
    ),
)

OPENYAM_WEBXR_TASK_NAME = "teleop_openyam"
OPENYAM_QUEST_MODEL = _openyam_model
OPENYAM_QUEST_KINEMATICS = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)
_openyam_webxr_task = TaskConfig(
    name=OPENYAM_WEBXR_TASK_NAME,
    type="teleop_ik",
    priority=20,
    joint_names=list(_openyam_model.joint_names),
    params={
        "robot_model": _openyam_model,
        "solver_type": OpenYamPinkPoseTargetSolver,
        "bindings": [{"hand": "right", "target_frame": "gripper_tip"}],
        "pink": OPENYAM_QUEST_KINEMATICS,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_command_filter_cutoff_hz": 30.0,
    },
)


def openyam_quest_tasks() -> list[TaskConfig]:
    """Build fresh canonical Quest tasks, including policy takeover priorities."""
    return [
        deepcopy(_openyam_webxr_task),
        # joint7 is open at its lower end.
        TaskConfig(
            name="arm_gripper",
            type="gripper",
            joint_names=[OPENYAM_GRIPPER_JOINT],
            priority=20,
            params={"hold_duration": 0.1, "open_at": "lower"},
            stream_bind={"gripper_command": "right_gripper_command"},
        ),
        _trajectory_task(priority=10),
    ]


teleop_webxr_openyam = autoconnect(
    ArmTeleopModule.blueprint(zero_button="B"),
    _teleop_control(openyam_quest_tasks()),
    ManipulationModule.blueprint(
        model=_openyam_model,
        gripper_tasks={"manipulator": "arm_gripper"},
        kinematics=OPENYAM_QUEST_KINEMATICS,
        visualization={"backend": "viser"},
    ),
).remappings([(ArmTeleopModule, "right_controller_output", "right_cartesian_command")])

keyboard_teleop_openyam_planner = autoconnect(
    KeyboardTeleopModule.blueprint(),
    planner(model=_openyam_model, gripper_tasks={"manipulator": "arm_gripper"}),
    _twist_control([_eef_twist_task(priority=20), _gripper_task(), _trajectory_task(priority=10)]),
)
