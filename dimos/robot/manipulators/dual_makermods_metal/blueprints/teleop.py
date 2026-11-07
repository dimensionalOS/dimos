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

"""Quest (WebXR) teleoperation of two MakerMods Metals, one per controller."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.dual_makermods_metal.blueprints.basic import (
    DualMetalManipulationModule,
)
from dimos.robot.manipulators.dual_makermods_metal.config import (
    ARM_JOINTS,
    FINGER_JOINTS,
    SIDES,
    dual_metal_model_config,
)
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection
from dimos.teleop.webxr.extensions import ArmTeleopModule

DUAL_METAL_WEBXR_TASK_NAME = "teleop_dual_metal"
DUAL_METAL_QUEST_MODEL = dual_metal_model_config(robot="arm")
#: The deployed joints a Quest recording and a policy rollout see: each arm, then its jaw.
DUAL_METAL_QUEST_JOINTS = [
    f"arm/{name}" for side in SIDES for name in (*ARM_JOINTS[side], FINGER_JOINTS[side][0])
]
DUAL_METAL_QUEST_KINEMATICS = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)


def _teleop_task(side: str) -> TaskConfig:
    # The URDF has no fingertip frame; gripper_base is the jaw mount the planner tips at.
    return TaskConfig(
        name=f"{DUAL_METAL_WEBXR_TASK_NAME}_{side}",
        type="teleop_ik",
        priority=20,
        joint_names=[f"arm/{name}" for name in ARM_JOINTS[side]],
        params={
            "robot_model": DUAL_METAL_QUEST_MODEL,
            "solver_type": PinkPoseTargetSolver,
            "bindings": [{"hand": side, "target_frame": f"{side}_gripper_base"}],
            "pink": DUAL_METAL_QUEST_KINEMATICS,
            "timeout": 0.5,
            "max_command_tracking_error_deg": 10.0,
            "max_joint_velocity_rad_s": 2.0,
            "joint_command_filter_cutoff_hz": 30.0,
        },
    )


def _gripper_task(side: str) -> TaskConfig:
    # Each side's joint7 is open at its lower end.
    return TaskConfig(
        name=f"{side}_arm_gripper",
        type="gripper",
        joint_names=[f"arm/{FINGER_JOINTS[side][0]}"],
        priority=20,
        params={"hold_duration": 0.1, "open_at": "lower"},
        stream_bind={"gripper_command": f"{side}_gripper_command"},
    )


def dual_metal_quest_tasks() -> list[TaskConfig]:
    """Fresh Quest tasks: each hand's IK on its arm, each jaw on its joint7, trajectories below."""
    return [
        *(_teleop_task(side) for side in SIDES),
        *(_gripper_task(side) for side in SIDES),
        joint_trajectory_task(DUAL_METAL_QUEST_JOINTS, priority=10),
    ]


def dual_metal_teleop_control(tasks: list[TaskConfig]) -> Blueprint:
    """The WebXR coordinator, named ControlCoordinator, over the two-bus Metal connection.

    Under ``--simulation`` the connection is the echo mock instead.
    """
    return autoconnect(
        DamiaoTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=DUAL_METAL_QUEST_MODEL.model,
            tasks=list(tasks),
            zero_stiffness_joints=[],
        ),
        arm_connection(DualMetalConnection),
    ).remappings(arm_remaps(DualMetalConnection, DamiaoTeleopCoordinator))


teleop_webxr_dual_metal = autoconnect(
    ArmTeleopModule.blueprint(zero_button="X"),
    dual_metal_teleop_control(dual_metal_quest_tasks()),
    DualMetalManipulationModule.blueprint(
        model=DUAL_METAL_QUEST_MODEL,
        gripper_tasks={"left_arm": "left_arm_gripper", "right_arm": "right_arm_gripper"},
        kinematics=DUAL_METAL_QUEST_KINEMATICS,
        visualization={"backend": "viser"},
        default_speed_scale=0.2,
    ),
).remappings(
    [
        (ArmTeleopModule, "left_controller_output", "left_cartesian_command"),
        (ArmTeleopModule, "left_gripper_command", "left_gripper_command"),
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
        (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
    ]
)
