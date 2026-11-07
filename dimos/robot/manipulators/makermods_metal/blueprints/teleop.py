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

"""Quest (WebXR) teleoperation of one MakerMods Metal over its connection."""

from copy import deepcopy

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.makermods_metal.config import make_metal_model_config
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.teleop.webxr.extensions import ArmTeleopModule

METAL_WEBXR_TASK_NAME = "teleop_metal"
METAL_QUEST_MODEL = make_metal_model_config(robot="arm")
#: The deployed joints a Quest recording and a policy rollout see, jaw last.
METAL_QUEST_JOINTS = [*METAL_QUEST_MODEL.joint_names, "arm/joint7"]
METAL_QUEST_KINEMATICS = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)
# The URDF has no fingertip frame; gripper_base is the jaw mount the planner tips at.
_metal_webxr_task = TaskConfig(
    name=METAL_WEBXR_TASK_NAME,
    type="teleop_ik",
    priority=20,
    joint_names=list(METAL_QUEST_MODEL.joint_names),
    params={
        "robot_model": METAL_QUEST_MODEL,
        "solver_type": PinkPoseTargetSolver,
        "bindings": [{"hand": "right", "target_frame": "gripper_base"}],
        "pink": METAL_QUEST_KINEMATICS,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_command_filter_cutoff_hz": 30.0,
    },
)


def metal_quest_tasks() -> list[TaskConfig]:
    """Fresh Quest tasks: right-hand IK on the arm, the jaw on joint7, trajectories below."""
    return [
        deepcopy(_metal_webxr_task),
        # joint7 is open at its lower end.
        TaskConfig(
            name="metal_gripper",
            type="gripper",
            joint_names=["arm/joint7"],
            priority=20,
            params={"hold_duration": 0.1, "open_at": "lower"},
            stream_bind={"gripper_command": "right_gripper_command"},
        ),
        joint_trajectory_task(METAL_QUEST_JOINTS, priority=10),
    ]


def metal_teleop_control(
    tasks: list[TaskConfig],
    coordinator: type[ControlCoordinator] = DamiaoTeleopCoordinator,
) -> Blueprint:
    """A coordinator named ControlCoordinator over the Metal connection, WebXR by default.

    Under ``--simulation`` the connection is the echo mock instead.
    """
    return autoconnect(
        coordinator.blueprint(
            instance_name="ControlCoordinator",
            model=METAL_QUEST_MODEL.model,
            tasks=list(tasks),
            zero_stiffness_joints=[],
        ),
        arm_connection(MetalConnection),
    ).remappings(arm_remaps(MetalConnection, coordinator))


def build_metal_webxr() -> Blueprint:
    """Compose one Metal's Quest teleop: arm input, coordinator, connection and planner.

    A function rather than a bare expression so a deployment can rebuild the
    stack, and so tests can compose it after changing ``global_config``, which
    ``arm_connection`` reads when it decides between the Metal and the mock.
    """
    return autoconnect(
        ArmTeleopModule.blueprint(zero_button="X"),
        metal_teleop_control(metal_quest_tasks()),
        ManipulationModule.blueprint(
            model=METAL_QUEST_MODEL,
            gripper_tasks={"arm": "metal_gripper"},
            kinematics=METAL_QUEST_KINEMATICS,
            visualization={"backend": "viser"},
            default_speed_scale=0.2,
        ),
    ).remappings([(ArmTeleopModule, "right_controller_output", "right_cartesian_command")])


teleop_webxr_metal = autoconnect(build_metal_webxr())
