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

"""OpenYAM keyboard and Quest teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    planner,
    teleop_ik_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_HARDWARE_ID,
    OPENYAM_JOINTS,
    make_openyam_model_config,
    openyam_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

_openyam_keyboard_hw = openyam_hardware()
_openyam_model = make_openyam_model_config()


def _eef_twist_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(OPENYAM_ARM_JOINTS),
        priority=priority,
        params={"robot_model": _openyam_model, "target_frame": "yam_hand_tcp"},
    )


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return joint_trajectory_task(OPENYAM_JOINTS, priority=priority)


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name=f"{OPENYAM_HARDWARE_ID}_gripper",
        type="gripper",
        joint_names=[OPENYAM_GRIPPER_JOINT],
        priority=20,
    )


keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_openyam_keyboard_hw],
        tasks=[
            _eef_twist_task(),
            _gripper_task(),
        ],
    ),
    ManipulationModule.blueprint(
        model=_openyam_model,
        visualization={"backend": "viser"},
    ),
)

OPENYAM_QUEST_TASK_NAME = "teleop_openyam"

_openyam_quest_pink = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)
_openyam_quest_hw = openyam_hardware()
_openyam_quest_model = make_openyam_model_config()
_openyam_quest_task = teleop_ik_task(
    _openyam_quest_hw,
    robot_model=_openyam_quest_model,
    name=OPENYAM_QUEST_TASK_NAME,
    joint_names=OPENYAM_ARM_JOINTS,
    priority=10,
    bindings=[
        {
            "hand": "right",
            "target_frame": "yam_hand_tcp",
        }
    ],
    params={
        "pink": _openyam_quest_pink,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_command_filter_cutoff_hz": 30.0,
    },
)

# Single-arm Quest teleop: right controller -> OpenYAM arm
teleop_quest_openyam = autoconnect(
    ArmTeleopModule.blueprint(),
    TeleopControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_openyam_quest_hw],
        tasks=[
            _openyam_quest_task,
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=[OPENYAM_GRIPPER_JOINT],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            _trajectory_task(priority=20),
        ],
    ),
    ManipulationModule.blueprint(
        model=_openyam_quest_model,
        kinematics=_openyam_quest_pink,
        visualization={"backend": "viser"},
    ),
).remappings(
    [
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
        (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
    ]
)

_openyam_keyboard_planner_hw = openyam_hardware()

keyboard_teleop_openyam_planner = autoconnect(
    KeyboardTeleopModule.blueprint(),
    planner(model=_openyam_model),
    coordinator(
        hardware=[_openyam_keyboard_planner_hw],
        tasks=[
            _eef_twist_task(priority=10),
            _gripper_task(),
            _trajectory_task(priority=20),
        ],
    ),
)
