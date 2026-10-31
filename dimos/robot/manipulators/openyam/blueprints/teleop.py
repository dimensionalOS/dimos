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

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.common.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    DamiaoTwistCoordinator,
    connect_damiao,
)
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.openyam.blueprints.basic import _gripper_task, _trajectory_task
from dimos.robot.manipulators.openyam.connection import OpenYamConnection
from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.webxr.extensions import ArmTeleopModule

_openyam = OpenYamConnection.models()
_openyam_model = _openyam.planning


def _eef_twist_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(_openyam.arm_joints),
        priority=priority,
        params={"robot_model": _openyam_model, "target_frame": "gripper_tip"},
    )


keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    connect_damiao(
        OpenYamConnection,
        coordinator=DamiaoTwistCoordinator,
        tasks=[_eef_twist_task(), _gripper_task()],
    ),
    ManipulationModule.blueprint(model=_openyam_model, visualization={"backend": "viser"}),
)

OPENYAM_WEBXR_TASK_NAME = "teleop_openyam"
_openyam_webxr_pink = PinkKinematicsConfig(
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
    joint_names=list(_openyam.arm_joints),
    params={
        "robot_model": _openyam_model,
        "solver_type": OpenYamPinkPoseTargetSolver,
        "bindings": [{"hand": "right", "target_frame": "gripper_tip"}],
        "pink": _openyam_webxr_pink,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_command_filter_cutoff_hz": 30.0,
    },
)
teleop_webxr_openyam = autoconnect(
    ArmTeleopModule.blueprint(zero_button="B"),
    connect_damiao(
        OpenYamConnection,
        coordinator=DamiaoTeleopCoordinator,
        tasks=[
            _openyam_webxr_task,
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=list(_openyam.gripper_joints),
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            _trajectory_task(priority=10),
        ],
    ),
    ManipulationModule.blueprint(
        model=_openyam_model,
        kinematics=_openyam_webxr_pink,
        visualization={"backend": "viser"},
    ),
).remappings([(ArmTeleopModule, "right_controller_output", "right_cartesian_command")])

keyboard_teleop_openyam_planner = autoconnect(
    KeyboardTeleopModule.blueprint(),
    planner(model=_openyam_model),
    connect_damiao(
        OpenYamConnection,
        coordinator=DamiaoTwistCoordinator,
        tasks=[_eef_twist_task(priority=20), _gripper_task(), _trajectory_task(priority=10)],
    ),
)
