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

"""WebXR teleoperation for Dual OpenYAM, with each arm engaged by its own controller."""

from collections.abc import Mapping
from typing import Any

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.visualization.config import ManipulationVisualizationConfig
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_SIDES,
    dual_openyam_arm_joints,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.teleop_ik import (
    DualOpenYamPinkPoseTargetSolver,
)
from dimos.teleop.webxr.extensions import ArmTeleopModule

DUAL_OPENYAM_WEBXR_TASK_NAME = "teleop_dual_openyam"

_dual_openyam_webxr_pink = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=1.0,
)
_dual_openyam_webxr_model = dual_openyam_model_config(robot="arm")


def dual_openyam_webxr_tasks(
    model: RobotModelConfig, teleop_params: Mapping[str, Any]
) -> dict[str, TaskConfig]:
    """One pose-following task per side, each engaged by its own controller's grip.

    Args:
        model: The planning model IK runs on.
        teleop_params: Settings that replace the tasks' own.
    """
    return {
        side: TaskConfig(
            type="teleop_ik",
            name=f"{DUAL_OPENYAM_WEBXR_TASK_NAME}_{side}",
            joint_names=[f"arm/{name}" for name in dual_openyam_arm_joints(side)],
            priority=20,
            params={
                "robot_model": model,
                "solver_type": DualOpenYamPinkPoseTargetSolver,
                "bindings": [{"hand": side, "target_frame": f"{side}_grasp_frame"}],
                "pink": _dual_openyam_webxr_pink,
                "timeout": 0.5,
                "max_command_tracking_error_deg": 10.0,
                "max_joint_velocity_rad_s": 2.0,
                "joint_command_filter_cutoff_hz": 30.0,
                **teleop_params,
            },
        )
        for side in DUAL_OPENYAM_SIDES
    }


_dual_openyam_webxr_tasks = dual_openyam_webxr_tasks(_dual_openyam_webxr_model, {})


def build_dual_openyam_webxr(
    *,
    visualization: ManipulationVisualizationConfig = ViserVisualizationConfig(),
    connection: type[DualOpenYamConnection] = DualOpenYamConnection,
    model: RobotModelConfig = _dual_openyam_webxr_model,
    teleop_params: Mapping[str, Any] | None = None,
    zero_button: str | None = "B",
) -> MetaBlueprint:
    """Compose dual-arm teleop with deployment-specific visualization.

    Args:
        visualization: Where the planner draws the arms.
        connection: The connection that owns the two buses.
        model: The planning model the tasks and the planner use.
        teleop_params: Settings that replace the pose-following tasks' own.
        zero_button: The Quest button that sends both arms to the zero pose, or ``None``.
    """
    return autoconnect(
        ArmTeleopModule.blueprint(zero_button=zero_button),
        autoconnect(
            DamiaoTeleopCoordinator.blueprint(
                instance_name="ControlCoordinator",
                connections=[arm_declaration(connection)],
                tasks=[
                    *dual_openyam_webxr_tasks(model, teleop_params or {}).values(),
                    TaskConfig(
                        name="left_arm_gripper",
                        type="gripper",
                        joint_names=["arm/left_gripper"],
                        priority=20,
                        stream_bind={"gripper_command": "left_gripper_command"},
                    ),
                    TaskConfig(
                        name="right_arm_gripper",
                        type="gripper",
                        joint_names=["arm/right_gripper"],
                        priority=20,
                        stream_bind={"gripper_command": "right_gripper_command"},
                    ),
                    dual_openyam_trajectory_task(priority=10),
                ],
                zero_stiffness_joints=[],
            ),
            arm_connection(connection),
        ).remappings(arm_remaps(connection, DamiaoTeleopCoordinator)),
        ManipulationModule.blueprint(
            gripper_tasks={
                "left_manipulator": "left_arm_gripper",
                "right_manipulator": "right_arm_gripper",
            },
            model=model,
            kinematics=_dual_openyam_webxr_pink,
            visualization=visualization,
        ),
    ).remappings(
        [
            (ArmTeleopModule, "left_controller_output", "left_cartesian_command"),
            (ArmTeleopModule, "left_gripper_command", "left_gripper_command"),
            (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
            (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
        ]
    )


teleop_webxr_dual_openyam = autoconnect(build_dual_openyam_webxr())
