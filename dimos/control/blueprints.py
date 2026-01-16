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

"""Pre-configured blueprints for the ControlOrchestrator.

This module provides ready-to-use orchestrator blueprints for common setups.

Usage:
    # Run via CLI:
    dimos run orchestrator-mock           # Mock 7-DOF arm
    dimos run orchestrator-xarm7          # XArm7 real hardware
    dimos run orchestrator-dual-mock      # Dual mock arms

    # Or programmatically:
    from dimos.control.blueprints import orchestrator_mock
    coordinator = orchestrator_mock.build()
    coordinator.loop()

Example with trajectory setter:
    # Terminal 1: Run the orchestrator
    dimos run orchestrator-mock

    # Terminal 2: Send trajectories via RPC
    python -m dimos.control.examples.orchestrator_trajectory_setter --task traj_arm
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.orchestrator import (
    ControlOrchestrator,
    TaskConfig,
    control_orchestrator,
)
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import JointState


def _joint_names(hardware_id: str, dof: int) -> list[str]:
    """Generate joint names for a hardware component."""
    return [f"{hardware_id}_joint{i + 1}" for i in range(dof)]


# =============================================================================
# Single Arm Blueprints
# =============================================================================

# Mock 7-DOF arm (for testing)
orchestrator_mock = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 7),
            backend_type="mock",
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=_joint_names("arm", 7),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# XArm7 real hardware (requires IP configuration)
orchestrator_xarm7 = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 7),
            backend_type="xarm",
            address="192.168.2.235",  # Default IP, override via env or config
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=_joint_names("arm", 7),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# XArm6 real hardware
orchestrator_xarm6 = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            backend_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_xarm",
            type="trajectory",
            joint_names=_joint_names("arm", 6),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# Piper arm (6-DOF, CAN bus)
orchestrator_piper = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            backend_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_piper",
            type="trajectory",
            joint_names=_joint_names("arm", 6),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# =============================================================================
# Dual Arm Blueprints
# =============================================================================

# Dual mock arms (7-DOF left, 6-DOF right) for testing
orchestrator_dual_mock = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="left_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("left_arm", 7),
            backend_type="mock",
        ),
        HardwareComponent(
            hardware_id="right_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("right_arm", 6),
            backend_type="mock",
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=_joint_names("left_arm", 7),
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=_joint_names("right_arm", 6),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# Dual XArm setup (XArm7 left, XArm6 right)
orchestrator_dual_xarm = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="left_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("left_arm", 7),
            backend_type="xarm",
            address="192.168.2.235",
            auto_enable=True,
        ),
        HardwareComponent(
            hardware_id="right_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("right_arm", 6),
            backend_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=_joint_names("left_arm", 7),
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=_joint_names("right_arm", 6),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# Dual Arm setup (XArm6 , Piper )
orchestrator_piper_xarm = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="xarm_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("xarm_arm", 6),
            backend_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
        HardwareComponent(
            hardware_id="piper_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("piper_arm", 6),
            backend_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_xarm",
            type="trajectory",
            joint_names=_joint_names("xarm_arm", 6),
            priority=10,
        ),
        TaskConfig(
            name="traj_piper",
            type="trajectory",
            joint_names=_joint_names("piper_arm", 6),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# =============================================================================
# High-frequency Blueprints (200Hz)
# =============================================================================

# High-frequency mock for demanding applications
orchestrator_highfreq_mock = control_orchestrator(
    tick_rate=200.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 7),
            backend_type="mock",
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=_joint_names("arm", 7),
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)

# =============================================================================
# Raw Blueprints (no hardware/tasks configured - for programmatic setup)
# =============================================================================

# Basic orchestrator with transport only (add hardware/tasks programmatically)
orchestrator_basic = control_orchestrator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="orchestrator",
).transports(
    {
        ("joint_state", JointState): LCMTransport("/orchestrator/joint_state", JointState),
    }
)


__all__ = [
    # Raw blueprints (for programmatic setup)
    "orchestrator_basic",
    # Dual arm blueprints
    "orchestrator_dual_mock",
    "orchestrator_dual_xarm",
    # High-frequency blueprints
    "orchestrator_highfreq_mock",
    # Single arm blueprints
    "orchestrator_mock",
    "orchestrator_piper",
    "orchestrator_piper_xarm",
    "orchestrator_xarm6",
    "orchestrator_xarm7",
]
