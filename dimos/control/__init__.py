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

"""ControlCoordinator - Centralized control for multi-arm coordination.

This module provides a centralized control coordinator that replaces
per-driver/per-controller loops with a single deterministic tick-based system.

Features:
- Single tick loop (read -> compute -> arbitrate -> route -> write)
- Per-joint arbitration (highest priority wins)
- Mode conflict detection
- Partial command support (hold last value)
- Aggregated preemption notifications

Example:
    >>> from dimos.control import ControlCoordinator
    >>> from dimos.control.tasks import JointTrajectoryTask, JointTrajectoryTaskConfig
    >>> from dimos.hardware.manipulators.xarm import XArmAdapter
    >>>
    >>> # Create coordinator
    >>> coord = ControlCoordinator(tick_rate=100.0)
    >>>
    >>> # Add hardware
    >>> adapter = XArmAdapter(ip="192.168.1.185", dof=7)
    >>> adapter.connect()
    >>> coord.add_hardware("left_arm", adapter)
    >>>
    >>> # Add task
    >>> joints = [f"left_arm_joint{i+1}" for i in range(7)]
    >>> task = JointTrajectoryTask(
    ...     "traj_left",
    ...     JointTrajectoryTaskConfig(joint_names=joints, priority=10),
    ... )
    >>> coord.add_task(task)
    >>>
    >>> # Start
    >>> coord.start()
"""

import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "components": [
            "HardwareComponent",
            "HardwareId",
            "HardwareType",
            "JointName",
            "JointState",
            "make_joints",
        ],
        "coordinator": [
            "ControlCoordinator",
            "ControlCoordinatorConfig",
            "TaskConfig",
            "control_coordinator",
        ],
        "hardware_interface": ["ConnectedHardware"],
        "task": [
            "ControlMode",
            "ControlTask",
            "CoordinatorState",
            "JointCommandOutput",
            "JointStateSnapshot",
            "ResourceClaim",
        ],
        "tick_loop": ["TickLoop"],
    },
)
