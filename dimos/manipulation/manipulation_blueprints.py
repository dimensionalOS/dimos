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

from dimos.core.blueprints import autoconnect
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


# Piper gripper collision exclusions (parallel jaw gripper)
# The gripper fingers (link7, link8) can touch each other and gripper_base
PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]


# XArm gripper collision exclusions (parallel linkage mechanism)
# The gripper uses mimic joints where non-adjacent links can overlap legitimately
XARM_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    # Inner knuckle <-> outer knuckle (parallel linkage)
    ("right_inner_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "left_outer_knuckle"),
    # Inner knuckle <-> finger (parallel linkage)
    ("right_inner_knuckle", "right_finger"),
    ("left_inner_knuckle", "left_finger"),
    # Cross-finger pairs (mimic joint symmetry)
    ("left_finger", "right_finger"),
    ("left_outer_knuckle", "right_outer_knuckle"),
    ("left_inner_knuckle", "right_inner_knuckle"),
    # Outer knuckle <-> opposite finger
    ("left_outer_knuckle", "right_finger"),
    ("right_outer_knuckle", "left_finger"),
    # Gripper base <-> all moving parts (can touch at limits)
    ("xarm_gripper_base_link", "left_inner_knuckle"),
    ("xarm_gripper_base_link", "right_inner_knuckle"),
    ("xarm_gripper_base_link", "left_finger"),
    ("xarm_gripper_base_link", "right_finger"),
    # Arm link6 <-> gripper (attached via fixed joint, can touch)
    ("link6", "xarm_gripper_base_link"),
    ("link6", "left_outer_knuckle"),
    ("link6", "right_outer_knuckle"),
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
        # Pass attach_xyz to position each robot; xacro creates world_joint at this offset
        xacro_args={
            "dof": "6",
            "limited": "true",
            "add_gripper": "true",
            "attach_xyz": f"0 {y_offset} 0",
        },
        collision_exclusion_pairs=XARM_GRIPPER_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        orchestrator_task_name=orchestrator_task,
    )


def _make_xarm7_config(
    name: str = "arm",
    y_offset: float = 0.0,
    joint_prefix: str = "",
    orchestrator_task: str | None = None,
    add_gripper: bool = False,
) -> RobotModelConfig:
    """Create XArm7 robot config.

    Args:
        name: Robot name in Drake world
        y_offset: Y-axis offset for base pose (for multi-arm setups)
        joint_prefix: Prefix for joint name mapping (e.g., "left_" or "right_")
        orchestrator_task: Task name for orchestrator RPC execution
        add_gripper: Whether to add the xarm gripper
    """
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    xacro_args: dict[str, str] = {
        "dof": "7",
        "limited": "true",
        "attach_xyz": f"0 {y_offset} 0",
    }
    if add_gripper:
        xacro_args["add_gripper"] = "true"

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
        end_effector_link="link_tcp" if add_gripper else "link7",
        base_link="link_base",
        package_paths=_get_xarm_package_paths(),
        xacro_args=xacro_args,
        collision_exclusion_pairs=XARM_GRIPPER_COLLISION_EXCLUSIONS if add_gripper else [],
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=joint_mapping,
        orchestrator_task_name=orchestrator_task,
    )


def _make_piper_config(
    name: str = "piper",
    y_offset: float = 0.0,
    joint_prefix: str = "",
    orchestrator_task: str | None = None,
) -> RobotModelConfig:
    """Create Piper robot config.

    Args:
        name: Robot name in Drake world
        y_offset: Y-axis offset for base pose (for multi-arm setups)
        joint_prefix: Prefix for joint name mapping (e.g., "piper_")
        orchestrator_task: Task name for orchestrator RPC execution

    Note:
        Piper has 6 revolute joints (joint1-joint6) for the arm and 2 prismatic
        joints (joint7, joint8) for the parallel jaw gripper.
    """
    # Piper arm joints (6-DOF)
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    joint_mapping = {f"{joint_prefix}{j}": j for j in joint_names} if joint_prefix else {}

    return RobotModelConfig(
        name=name,
        urdf_path=_get_piper_urdf_path(),
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
        end_effector_link="gripper_base",  # End of arm, before gripper fingers
        base_link="arm_base",
        package_paths=_get_piper_package_paths(),
        xacro_args={},  # Piper xacro doesn't need special args
        collision_exclusion_pairs=PIPER_GRIPPER_COLLISION_EXCLUSIONS,
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


# =============================================================================
# Combined Blueprints (Orchestrator + Planner in one process)
# =============================================================================


# Combined: Quad mock orchestrator + Quad manipulation planner
# Usage: dimos run quad-mock-with-planner
def _make_quad_mock_with_planner():
    """Create combined quad-arm blueprint (lazy import to avoid circular deps)."""
    from dimos.control.blueprints import orchestrator_quad_mock

    return autoconnect(
        orchestrator_quad_mock,
        manipulation_module(
            robots=[
                _make_xarm6_config(
                    "arm1", y_offset=0.75, joint_prefix="arm1_", orchestrator_task="traj_arm1"
                ),
                _make_xarm6_config(
                    "arm2", y_offset=0.25, joint_prefix="arm2_", orchestrator_task="traj_arm2"
                ),
                _make_xarm6_config(
                    "arm3", y_offset=-0.25, joint_prefix="arm3_", orchestrator_task="traj_arm3"
                ),
                _make_xarm6_config(
                    "arm4", y_offset=-0.75, joint_prefix="arm4_", orchestrator_task="traj_arm4"
                ),
            ],
            planning_timeout=10.0,
            enable_viz=True,
        ),
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
        }
    )


# Lazy-loaded to avoid circular import
quad_mock_with_planner = _make_quad_mock_with_planner()


# Combined: Mixed-arm orchestrator (XArm7 + XArm6 + Piper) + manipulation planner
# Usage: dimos run mixed-mock-with-planner
def _make_mixed_mock_with_planner():
    """Create combined mixed-arm blueprint (XArm7 + XArm6 + Piper).

    Layout (top view, looking down at table):
        y=0.5: XArm7 (7-DOF with gripper)
        y=0.0: XArm6 (6-DOF with gripper)
        y=-0.5: Piper (6-DOF with parallel jaw gripper)
    """
    from dimos.control.blueprints import orchestrator_mixed_mock

    return autoconnect(
        orchestrator_mixed_mock,
        manipulation_module(
            robots=[
                _make_xarm7_config(
                    "xarm7",
                    y_offset=0.5,
                    joint_prefix="xarm7_",
                    orchestrator_task="traj_xarm7",
                    add_gripper=True,
                ),
                _make_xarm6_config(
                    "xarm6",
                    y_offset=0.0,
                    joint_prefix="xarm6_",
                    orchestrator_task="traj_xarm6",
                ),
                _make_piper_config(
                    "piper",
                    y_offset=-0.5,
                    joint_prefix="piper_",
                    orchestrator_task="traj_piper",
                ),
            ],
            planning_timeout=10.0,
            enable_viz=True,
        ),
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
        }
    )


# Lazy-loaded to avoid circular import
mixed_mock_with_planner = _make_mixed_mock_with_planner()


__all__ = [
    "PIPER_GRIPPER_COLLISION_EXCLUSIONS",
    "XARM_GRIPPER_COLLISION_EXCLUSIONS",
    "dual_xarm6_planner",
    "mixed_mock_with_planner",
    "quad_mock_with_planner",
    "xarm6_planner_only",
    "xarm7_planner_orchestrator",
]
