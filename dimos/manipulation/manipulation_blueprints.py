# Copyright 2025-2026 Dimensional Inc.
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

"""
Blueprints for manipulation module integration with ControlOrchestrator.

Usage:
    # Start orchestrator first, then planner:
    coordinator = xarm7_planner_orchestrator.build()
    coordinator.loop()

    # Plan and execute via RPC client:
    from dimos.manipulation.planning.examples.manipulation_client import ManipulationClient
    client = ManipulationClient()
    client.plan_to_joints([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    client.execute()
"""

import numpy as np

from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import manipulation_module
from dimos.manipulation.planning.spec import RobotModelConfig
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.data import get_data

# =============================================================================
# URDF Helpers
# =============================================================================


def _get_xarm_urdf_path() -> str:
    """Get path to xarm URDF."""
    return str(get_data("xarm_description") / "urdf/xarm_device.urdf.xacro")


def _get_xarm_package_paths() -> dict[str, str]:
    """Get package paths for xarm xacro resolution."""
    return {"xarm_description": str(get_data("xarm_description"))}


def _get_piper_urdf_path() -> str:
    """Get path to piper URDF."""
    return str(get_data("piper_description") / "urdf/piper_description.xacro")


def _get_piper_package_paths() -> dict[str, str]:
    """Get package paths for piper xacro resolution."""
    return {"piper_description": str(get_data("piper_description"))}


# XArm gripper collision exclusions (parallel linkage mechanism)
XARM_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("right_inner_knuckle", "right_outer_knuckle"),
    ("right_inner_knuckle", "right_finger"),
    ("left_inner_knuckle", "left_outer_knuckle"),
    ("left_inner_knuckle", "left_finger"),
    ("left_finger", "right_finger"),
    ("left_outer_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "right_inner_knuckle"),
]


# =============================================================================
# Robot Configs
# =============================================================================


def _make_xarm6_config(
    name: str = "arm",
    y_offset: float = 0.0,
    joint_prefix: str = "",
    orchestrator_task: str | None = None,
) -> RobotModelConfig:
    """Create XArm6 robot config.

    Args:
        name: Robot name in Drake world
        y_offset: Y-axis offset for base pose (for multi-arm setups)
        joint_prefix: Prefix for joint name mapping (e.g., "left_" or "right_")
        orchestrator_task: Task name for orchestrator RPC execution
    """
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    return RobotModelConfig(
        name=name,
        urdf_path=_get_xarm_urdf_path(),
        base_pose=np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, y_offset],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        ),
        joint_names=joint_names,
        end_effector_link="link_tcp",
        base_link="link_base",
        package_paths=_get_xarm_package_paths(),
        xacro_args={"dof": "6", "limited": "true", "add_gripper": "true"},
        collision_exclusion_pairs=XARM_GRIPPER_COLLISION_EXCLUSIONS,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        orchestrator_task_name=orchestrator_task,
    )


def _make_xarm7_config(
    name: str = "arm",
    joint_prefix: str = "",
    orchestrator_task: str | None = None,
) -> RobotModelConfig:
    """Create XArm7 robot config."""
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    return RobotModelConfig(
        name=name,
        urdf_path=_get_xarm_urdf_path(),
        base_pose=np.eye(4, dtype=np.float64),
        joint_names=joint_names,
        end_effector_link="link7",
        base_link="link_base",
        package_paths=_get_xarm_package_paths(),
        xacro_args={"dof": "7", "limited": "true"},
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        orchestrator_task_name=orchestrator_task,
    )


# =============================================================================
# Blueprints
# =============================================================================


# Single XArm6 planner (standalone, no orchestrator)
xarm6_planner_only = manipulation_module(
    robots=[_make_xarm6_config()],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
    }
)


# Dual XArm6 planner with orchestrator integration
# Usage: Start with orchestrator_dual_mock, then plan/execute via RPC
dual_xarm6_planner = manipulation_module(
    robots=[
        _make_xarm6_config(
            "left_arm", y_offset=0.5, joint_prefix="left_", orchestrator_task="traj_left"
        ),
        _make_xarm6_config(
            "right_arm", y_offset=-0.5, joint_prefix="right_", orchestrator_task="traj_right"
        ),
    ],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)


# Single XArm7 planner for orchestrator-mock
# Usage: dimos run orchestrator-mock, then dimos run xarm7-planner-orchestrator
xarm7_planner_orchestrator = manipulation_module(
    robots=[_make_xarm7_config("arm", joint_prefix="arm_", orchestrator_task="traj_arm")],
    planning_timeout=10.0,
    enable_viz=True,
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)


__all__ = [
    "XARM_GRIPPER_COLLISION_EXCLUSIONS",
    "dual_xarm6_planner",
    "xarm6_planner_only",
    "xarm7_planner_orchestrator",
]
