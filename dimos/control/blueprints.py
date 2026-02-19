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

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointState
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.data import LfsPath

_PIPER_MODEL_PATH = LfsPath("piper_description/mujoco_model/piper_no_gripper_description.xml")
_XARM6_MODEL_PATH = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")

# =============================================================================
# Single Arm Blueprints
# =============================================================================

# Mock 7-DOF arm (for testing)
coordinator_mock = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 7),
            adapter_type="mock",
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm7 real hardware
coordinator_xarm7 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 7),
            adapter_type="xarm",
            address="192.168.2.235",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_arm",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# XArm6 real hardware
coordinator_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_xarm",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Piper arm (6-DOF, CAN bus)
coordinator_piper = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_piper",
            type="trajectory",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# =============================================================================
# Dual Arm Blueprints
# =============================================================================

# Dual mock arms (7-DOF left, 6-DOF right)
coordinator_dual_mock = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="left_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("left_arm", 7),
            adapter_type="mock",
        ),
        HardwareComponent(
            hardware_id="right_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("right_arm", 6),
            adapter_type="mock",
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=[f"left_arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=[f"right_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Dual XArm (XArm7 left, XArm6 right)
coordinator_dual_xarm = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="left_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("left_arm", 7),
            adapter_type="xarm",
            address="192.168.2.235",
            auto_enable=True,
        ),
        HardwareComponent(
            hardware_id="right_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("right_arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_left",
            type="trajectory",
            joint_names=[f"left_arm_joint{i + 1}" for i in range(7)],
            priority=10,
        ),
        TaskConfig(
            name="traj_right",
            type="trajectory",
            joint_names=[f"right_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

# Dual arm (XArm6 + Piper)
coordinator_piper_xarm = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="xarm_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("xarm_arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
        HardwareComponent(
            hardware_id="piper_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("piper_arm", 6),
            adapter_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="traj_xarm",
            type="trajectory",
            joint_names=[f"xarm_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
        TaskConfig(
            name="traj_piper",
            type="trajectory",
            joint_names=[f"piper_arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# =============================================================================
# Streaming Control Blueprints
# =============================================================================

# XArm6 teleop - streaming position control
coordinator_teleop_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/teleop/joint_command", JointState),
    }
)

# XArm6 velocity control - streaming velocity for joystick
coordinator_velocity_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/joystick/joint_command", JointState),
    }
)

# XArm6 combined (servo + velocity tasks)
coordinator_combined_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="servo_arm",
            type="servo",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
        TaskConfig(
            name="velocity_arm",
            type="velocity",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/control/joint_command", JointState),
    }
)


# =============================================================================
# Cartesian IK Blueprints (internal Pinocchio IK solver)
# =============================================================================


# Mock 6-DOF arm with CartesianIK
coordinator_cartesian_ik_mock = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="mock",
        ),
    ],
    tasks=[
        TaskConfig(
            name="cartesian_ik_arm",
            type="cartesian_ik",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
            model_path=_PIPER_MODEL_PATH,
            ee_joint_id=6,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
    }
)

# Piper arm with CartesianIK
coordinator_cartesian_ik_piper = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="cartesian_ik_arm",
            type="cartesian_ik",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
            model_path=_PIPER_MODEL_PATH,
            ee_joint_id=6,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
    }
)


# =============================================================================
# Teleop IK Blueprints (VR teleoperation with internal Pinocchio IK)
# =============================================================================

# Single XArm6 with TeleopIK
coordinator_teleop_xarm6 = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="teleop_xarm",
            type="teleop_ik",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
            model_path=_XARM6_MODEL_PATH,
            ee_joint_id=6,
            hand="right",
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# Single Piper with TeleopIK
coordinator_teleop_piper = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("arm", 6),
            adapter_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="teleop_piper",
            type="teleop_ik",
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
            model_path=_PIPER_MODEL_PATH,
            ee_joint_id=6,
            hand="left",
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# Dual arm teleop: XArm6 + Piper with TeleopIK
coordinator_teleop_dual = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="xarm_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("xarm_arm", 6),
            adapter_type="xarm",
            address="192.168.1.210",
            auto_enable=True,
        ),
        HardwareComponent(
            hardware_id="piper_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=make_joints("piper_arm", 6),
            adapter_type="piper",
            address="can0",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="teleop_xarm",
            type="teleop_ik",
            joint_names=[f"xarm_arm_joint{i + 1}" for i in range(6)],
            priority=10,
            model_path=_XARM6_MODEL_PATH,
            ee_joint_id=6,
            hand="left",
        ),
        TaskConfig(
            name="teleop_piper",
            type="teleop_ik",
            joint_names=[f"piper_arm_joint{i + 1}" for i in range(6)],
            priority=10,
            model_path=_PIPER_MODEL_PATH,
            ee_joint_id=6,
            hand="right",
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("cartesian_command", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# =============================================================================
# Raw Blueprints (for programmatic setup)
# =============================================================================

coordinator_basic = control_coordinator(
    tick_rate=100.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


# =============================================================================
# Exports
# =============================================================================

__all__ = [
    # Raw
    "coordinator_basic",
    # Cartesian IK
    "coordinator_cartesian_ik_mock",
    "coordinator_cartesian_ik_piper",
    # Streaming control
    "coordinator_combined_xarm6",
    # Dual arm
    "coordinator_dual_mock",
    "coordinator_dual_xarm",
    # Single arm
    "coordinator_mock",
    "coordinator_piper",
    "coordinator_piper_xarm",
    # Teleop IK
    "coordinator_teleop_dual",
    "coordinator_teleop_piper",
    "coordinator_teleop_xarm6",
    "coordinator_velocity_xarm6",
    "coordinator_xarm6",
    "coordinator_xarm7",
]
