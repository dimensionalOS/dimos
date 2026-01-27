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

"""Pre-configured blueprints for the ControlCoordinator.

This module provides ready-to-use coordinator blueprints for common setups.

Usage:
    # Run via CLI:
    dimos run coordinator-mock           # Mock 7-DOF arm
    dimos run coordinator-xarm7          # XArm7 real hardware
    dimos run coordinator-dual-mock      # Dual mock arms

    # Or programmatically:
    from dimos.control.blueprints import coordinator_mock
    coordinator = coordinator_mock.build()
    coordinator.loop()
"""

from __future__ import annotations

from typing import Any

from dimos.control.components import (
    HardwareComponent,
    HardwareId,
    HardwareType,
    JointName,
    TaskName,
    make_joints,
)
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointState

# =============================================================================
# Helper functions
# =============================================================================


def _joint_names(hardware_id: HardwareId, dof: int) -> list[JointName]:
    """Generate joint names for a hardware component."""
    return [f"{hardware_id}_joint{i + 1}" for i in range(dof)]


def _get_piper_model_path() -> str:
    """Get path to Piper MJCF model for IK solver."""
    from dimos.utils.data import get_data

    piper_path = get_data("piper_description")
    return str(piper_path / "mujoco_model" / "piper_no_gripper_description.xml")


def _standard_transports() -> dict[Any, Any]:
    """Standard transports for coordinator blueprints."""
    return {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }


def _streaming_transports(joint_command_topic: str) -> dict[Any, Any]:
    """Transports for streaming control blueprints (includes joint_command)."""
    return {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport(joint_command_topic, JointState),
    }


def _cartesian_transports() -> dict[Any, Any]:
    """Transports for cartesian IK blueprints (includes cartesian_command)."""
    return {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
    }


# =============================================================================
# Hardware configurations
# =============================================================================


def _mock_arm(hardware_id: HardwareId = "arm", dof: int = 7) -> HardwareComponent:
    """Mock arm for testing."""
    return HardwareComponent(
        hardware_id=hardware_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hardware_id, dof),
        adapter_type="mock",
    )


def _xarm(
    hardware_id: HardwareId = "arm", dof: int = 6, address: str = "192.168.1.210"
) -> HardwareComponent:
    """XArm hardware."""
    return HardwareComponent(
        hardware_id=hardware_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hardware_id, dof),
        adapter_type="xarm",
        address=address,
        auto_enable=True,
    )


def _piper(hardware_id: HardwareId = "arm", address: str = "can0") -> HardwareComponent:
    """Piper arm (6-DOF, CAN bus)."""
    return HardwareComponent(
        hardware_id=hardware_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hardware_id, 6),
        adapter_type="piper",
        address=address,
        auto_enable=True,
    )


# =============================================================================
# Task configurations
# =============================================================================


def _trajectory_task(
    name: TaskName, hardware_id: HardwareId, dof: int, priority: int = 10
) -> TaskConfig:
    """Trajectory tracking task."""
    return TaskConfig(
        name=name,
        type="trajectory",
        joint_names=_joint_names(hardware_id, dof),
        priority=priority,
    )


def _servo_task(
    name: TaskName, hardware_id: HardwareId, dof: int, priority: int = 10
) -> TaskConfig:
    """Streaming position control task."""
    return TaskConfig(
        name=name,
        type="servo",
        joint_names=_joint_names(hardware_id, dof),
        priority=priority,
    )


def _velocity_task(
    name: TaskName, hardware_id: HardwareId, dof: int, priority: int = 10
) -> TaskConfig:
    """Velocity control task."""
    return TaskConfig(
        name=name,
        type="velocity",
        joint_names=_joint_names(hardware_id, dof),
        priority=priority,
    )


def _cartesian_ik_task(
    name: TaskName,
    hardware_id: HardwareId,
    dof: int,
    model_path: str,
    ee_joint_id: int = 6,
    priority: int = 10,
) -> TaskConfig:
    """Cartesian IK task with internal Pinocchio solver."""
    return TaskConfig(
        name=name,
        type="cartesian_ik",
        joint_names=_joint_names(hardware_id, dof),
        priority=priority,
        model_path=model_path,
        ee_joint_id=ee_joint_id,
    )


# =============================================================================
# Single Arm Blueprints
# =============================================================================

# Mock 7-DOF arm (for testing)
coordinator_mock = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_mock_arm("arm", 7)],
    tasks=[_trajectory_task("traj_arm", "arm", 7)],
).transports(_standard_transports())

# XArm7 real hardware
coordinator_xarm7 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_xarm("arm", 7, "192.168.2.235")],
    tasks=[_trajectory_task("traj_arm", "arm", 7)],
).transports(_standard_transports())

# XArm6 real hardware
coordinator_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_xarm("arm", 6, "192.168.1.210")],
    tasks=[_trajectory_task("traj_xarm", "arm", 6)],
).transports(_standard_transports())

# Piper arm (6-DOF, CAN bus)
coordinator_piper = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_piper("arm", "can0")],
    tasks=[_trajectory_task("traj_piper", "arm", 6)],
).transports(_standard_transports())


# =============================================================================
# Dual Arm Blueprints
# =============================================================================

# Dual mock arms (7-DOF left, 6-DOF right)
coordinator_dual_mock = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        _mock_arm("left_arm", 7),
        _mock_arm("right_arm", 6),
    ],
    tasks=[
        _trajectory_task("traj_left", "left_arm", 7),
        _trajectory_task("traj_right", "right_arm", 6),
    ],
).transports(_standard_transports())

# Dual XArm (XArm7 left, XArm6 right)
coordinator_dual_xarm = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        _xarm("left_arm", 7, "192.168.2.235"),
        _xarm("right_arm", 6, "192.168.1.210"),
    ],
    tasks=[
        _trajectory_task("traj_left", "left_arm", 7),
        _trajectory_task("traj_right", "right_arm", 6),
    ],
).transports(_standard_transports())

# Dual arm (XArm6 + Piper)
coordinator_piper_xarm = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        _xarm("xarm_arm", 6, "192.168.1.210"),
        _piper("piper_arm", "can0"),
    ],
    tasks=[
        _trajectory_task("traj_xarm", "xarm_arm", 6),
        _trajectory_task("traj_piper", "piper_arm", 6),
    ],
).transports(_standard_transports())


# =============================================================================
# Streaming Control Blueprints
# =============================================================================

# XArm6 teleop - streaming position control
coordinator_teleop_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_xarm("arm", 6, "192.168.1.210")],
    tasks=[_servo_task("servo_arm", "arm", 6)],
).transports(_streaming_transports("/teleop/joint_command"))

# XArm6 velocity control - streaming velocity for joystick
coordinator_velocity_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_xarm("arm", 6, "192.168.1.210")],
    tasks=[_velocity_task("velocity_arm", "arm", 6)],
).transports(_streaming_transports("/joystick/joint_command"))

# XArm6 combined (servo + velocity tasks)
coordinator_combined_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_xarm("arm", 6, "192.168.1.210")],
    tasks=[
        _servo_task("servo_arm", "arm", 6),
        _velocity_task("velocity_arm", "arm", 6),
    ],
).transports(_streaming_transports("/control/joint_command"))


# =============================================================================
# Cartesian IK Blueprints (internal Pinocchio IK solver)
# =============================================================================

# Mock 6-DOF arm with CartesianIK (run jogger separately)
coordinator_cartesian_ik_mock = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_mock_arm("arm", 6)],
    tasks=[
        _cartesian_ik_task(
            name="cartesian_ik_arm",
            hardware_id="arm",
            dof=6,
            model_path=_get_piper_model_path(),
            ee_joint_id=6,
        ),
    ],
).transports(_cartesian_transports())

# Piper arm with CartesianIK (run jogger separately)
coordinator_cartesian_ik_piper = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[_piper("arm", "can0")],
    tasks=[
        _cartesian_ik_task(
            name="cartesian_ik_arm",
            hardware_id="arm",
            dof=6,
            model_path=_get_piper_model_path(),
            ee_joint_id=6,
        ),
    ],
).transports(_cartesian_transports())


# =============================================================================
# Raw Blueprints (for programmatic setup)
# =============================================================================

coordinator_basic = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
).transports(_standard_transports())


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Raw
    "coordinator_basic",
    # Cartesian IK
    "coordinator_cartesian_ik_mock",
    "coordinator_cartesian_ik_piper",
    "coordinator_combined_xarm6",
    # Dual arm
    "coordinator_dual_mock",
    "coordinator_dual_xarm",
    # Single arm
    "coordinator_mock",
    "coordinator_piper",
    "coordinator_piper_xarm",
    # Streaming control
    "coordinator_teleop_xarm6",
    "coordinator_velocity_xarm6",
    "coordinator_xarm6",
    "coordinator_xarm7",
]
