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

"""ControlOrchestrator - Centralized control for multi-arm coordination.

This module provides a centralized control orchestrator that replaces
per-driver/per-controller loops with a single deterministic tick-based system.

Features:
- Single tick loop (read → compute → arbitrate → route → write)
- Per-joint arbitration (highest priority wins)
- Mode conflict detection
- Partial command support (hold last value)
- Aggregated preemption notifications

Example:
    >>> from dimos.control import ControlOrchestrator
    >>> from dimos.control.tasks import JointTrajectoryTask, JointTrajectoryTaskConfig
    >>> from dimos.hardware.manipulators.xarm import XArmBackend
    >>>
    >>> # Create orchestrator
    >>> orch = ControlOrchestrator(tick_rate=100.0)
    >>>
    >>> # Add hardware
    >>> backend = XArmBackend(ip="192.168.1.185", dof=7)
    >>> backend.connect()
    >>> orch.add_hardware("left_arm", backend, joint_prefix="left")
    >>>
    >>> # Add task
    >>> joints = [f"left_joint{i+1}" for i in range(7)]
    >>> task = JointTrajectoryTask(
    ...     "traj_left",
    ...     JointTrajectoryTaskConfig(joint_names=joints, priority=10),
    ... )
    >>> orch.add_task(task)
    >>>
    >>> # Start
    >>> orch.start()
"""

from dimos.control.hardware_interface import (
    BackendHardwareInterface,
    HardwareInterface,
)
from dimos.control.orchestrator import (
    ControlOrchestrator,
    ControlOrchestratorConfig,
    HardwareConfig,
    TaskConfig,
    control_orchestrator,
)
from dimos.control.task import (
    ControlMode,
    ControlTask,
    JointCommandOutput,
    JointStateSnapshot,
    OrchestratorState,
    ResourceClaim,
)
from dimos.control.tick_loop import TickLoop

__all__ = [
    # Hardware interface
    "BackendHardwareInterface",
    "ControlMode",
    # Orchestrator
    "ControlOrchestrator",
    "ControlOrchestratorConfig",
    # Task protocol and types
    "ControlTask",
    "HardwareConfig",
    "HardwareInterface",
    "JointCommandOutput",
    "JointStateSnapshot",
    "OrchestratorState",
    "ResourceClaim",
    "TaskConfig",
    # Tick loop
    "TickLoop",
    "control_orchestrator",
]
