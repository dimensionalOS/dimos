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
Manipulation Skill Container

Agent-facing skills for robotic manipulation. Wraps ManipulationModule
RPC calls with @skill decorators for LLM agent integration.

## Usage

```python
from dimos.manipulation import ManipulationSkillContainer

# Deploy skill container
skills = cluster.deploy(ManipulationSkillContainer)

# Wire to manipulation module and agent
cluster.link(manip.rpc >> skills.ManipulationModule_move_to_pose)
cluster.link(skills.rpc >> agent.register_skills)

# Agent can now call:
# - move_arm_to_pose(x, y, z, roll, pitch, yaw)
# - move_arm_to_joints(j1, j2, j3, j4, j5, j6)
# - get_arm_state()
# - cancel_arm_motion()
```
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill

if TYPE_CHECKING:
    from dimos.core.rpc_client import RpcCall

logger = logging.getLogger(__name__)


class ManipulationSkillContainer(SkillModule):
    """Agent-facing skills for robotic manipulation.

    Provides @skill decorated methods that wrap ManipulationModule RPC calls
    for LLM agent integration. Each skill returns a human-readable string
    describing the action taken or error encountered.

    ## Available Skills

    - move_arm_to_pose: Move end-effector to target pose (xyzrpy)
    - move_arm_to_joints: Move to joint configuration
    - get_arm_state: Query current manipulation state
    - cancel_arm_motion: Cancel active motion
    - get_arm_position: Get current joint positions
    - get_end_effector_pose: Get current EE pose
    """

    # RPC call references (set via wiring)
    _move_to_pose: RpcCall | None = None
    _move_to_joints: RpcCall | None = None
    _get_state: RpcCall | None = None
    _get_state_name: RpcCall | None = None
    _get_error: RpcCall | None = None
    _cancel: RpcCall | None = None
    _reset: RpcCall | None = None
    _get_current_joints: RpcCall | None = None
    _get_ee_pose: RpcCall | None = None

    def __init__(self) -> None:
        super().__init__()

    @rpc
    def start(self) -> None:
        """Start the skill container."""
        super().start()
        logger.info("ManipulationSkillContainer started")

    @rpc
    def stop(self) -> None:
        """Stop the skill container."""
        super().stop()
        logger.info("ManipulationSkillContainer stopped")

    # =========================================================================
    # RPC Wiring Methods (called by cluster.link)
    # =========================================================================

    @rpc
    def set_ManipulationModule_move_to_pose(self, callable: RpcCall) -> None:
        """Wire move_to_pose RPC from ManipulationModule."""
        self._move_to_pose = callable
        self._move_to_pose.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_move_to_joints(self, callable: RpcCall) -> None:
        """Wire move_to_joints RPC from ManipulationModule."""
        self._move_to_joints = callable
        self._move_to_joints.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_get_state(self, callable: RpcCall) -> None:
        """Wire get_state RPC from ManipulationModule."""
        self._get_state = callable
        self._get_state.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_get_state_name(self, callable: RpcCall) -> None:
        """Wire get_state_name RPC from ManipulationModule."""
        self._get_state_name = callable
        self._get_state_name.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_get_error(self, callable: RpcCall) -> None:
        """Wire get_error RPC from ManipulationModule."""
        self._get_error = callable
        self._get_error.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_cancel(self, callable: RpcCall) -> None:
        """Wire cancel RPC from ManipulationModule."""
        self._cancel = callable
        self._cancel.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_reset(self, callable: RpcCall) -> None:
        """Wire reset RPC from ManipulationModule."""
        self._reset = callable
        self._reset.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_get_current_joints(self, callable: RpcCall) -> None:
        """Wire get_current_joints RPC from ManipulationModule."""
        self._get_current_joints = callable
        self._get_current_joints.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_get_ee_pose(self, callable: RpcCall) -> None:
        """Wire get_ee_pose RPC from ManipulationModule."""
        self._get_ee_pose = callable
        self._get_ee_pose.set_rpc(self.rpc)  # type: ignore[arg-type]

    # =========================================================================
    # Agent Skills
    # =========================================================================

    @skill()
    def move_arm_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> str:
        """Move the robot arm end-effector to a target pose.

        The pose is specified in the robot's base frame using position (x, y, z)
        in meters and orientation (roll, pitch, yaw) in radians.

        Args:
            x: X position in meters (forward/backward from base)
            y: Y position in meters (left/right from base)
            z: Z position in meters (up/down from base)
            roll: Rotation about X axis in radians (default: 0.0)
            pitch: Rotation about Y axis in radians (default: 0.0)
            yaw: Rotation about Z axis in radians (default: 0.0)

        Returns:
            Status message describing the result
        """
        if self._move_to_pose is None:
            return "Error: ManipulationModule not connected"

        try:
            success = self._move_to_pose(x, y, z, roll, pitch, yaw)
            if success:
                return (
                    f"Moving arm to pose: position=({x:.3f}, {y:.3f}, {z:.3f})m, "
                    f"orientation=({roll:.2f}, {pitch:.2f}, {yaw:.2f})rad"
                )
            else:
                error = self._get_error() if self._get_error else "unknown error"
                return f"Failed to plan motion to target pose: {error}"
        except Exception as e:
            logger.error(f"move_arm_to_pose failed: {e}")
            return f"Error executing move_arm_to_pose: {e}"

    @skill()
    def move_arm_to_joints(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        j5: float,
        j6: float,
    ) -> str:
        """Move the robot arm to a target joint configuration.

        All joint values are specified in radians.

        Args:
            j1: Joint 1 position in radians
            j2: Joint 2 position in radians
            j3: Joint 3 position in radians
            j4: Joint 4 position in radians
            j5: Joint 5 position in radians
            j6: Joint 6 position in radians

        Returns:
            Status message describing the result
        """
        if self._move_to_joints is None:
            return "Error: ManipulationModule not connected"

        joints = [j1, j2, j3, j4, j5, j6]

        try:
            success = self._move_to_joints(joints)
            if success:
                joint_str = ", ".join(f"{j:.3f}" for j in joints)
                return f"Moving arm to joints: [{joint_str}] rad"
            else:
                error = self._get_error() if self._get_error else "unknown error"
                return f"Failed to plan motion to joint configuration: {error}"
        except Exception as e:
            logger.error(f"move_arm_to_joints failed: {e}")
            return f"Error executing move_arm_to_joints: {e}"

    @skill()
    def get_arm_state(self) -> str:
        """Get the current state of the manipulation system.

        Returns:
            Current state: IDLE, PLANNING, EXECUTING, COMPLETED, or FAULT
        """
        if self._get_state_name is None:
            return "Error: ManipulationModule not connected"

        try:
            state_name = self._get_state_name()
            error = ""
            if state_name == "FAULT" and self._get_error:
                error = self._get_error()
                return f"Arm state: {state_name} - {error}"
            return f"Arm state: {state_name}"
        except Exception as e:
            logger.error(f"get_arm_state failed: {e}")
            return f"Error getting arm state: {e}"

    @skill()
    def cancel_arm_motion(self) -> str:
        """Cancel the currently executing arm motion.

        The arm will stop at its current position.

        Returns:
            Status message describing the result
        """
        if self._cancel is None:
            return "Error: ManipulationModule not connected"

        try:
            success = self._cancel()
            if success:
                return "Arm motion cancelled - arm stopped at current position"
            else:
                return "No active motion to cancel"
        except Exception as e:
            logger.error(f"cancel_arm_motion failed: {e}")
            return f"Error cancelling arm motion: {e}"

    @skill()
    def reset_arm(self) -> str:
        """Reset the manipulation system from FAULT or COMPLETED state.

        Required before executing new motions after a fault or completion.

        Returns:
            Status message describing the result
        """
        if self._reset is None:
            return "Error: ManipulationModule not connected"

        try:
            success = self._reset()
            if success:
                return "Arm controller reset to IDLE state - ready for new commands"
            else:
                return "Cannot reset while executing (cancel first)"
        except Exception as e:
            logger.error(f"reset_arm failed: {e}")
            return f"Error resetting arm: {e}"

    @skill()
    def get_arm_position(self) -> str:
        """Get the current joint positions of the robot arm.

        Returns:
            Current joint positions in radians
        """
        if self._get_current_joints is None:
            return "Error: ManipulationModule not connected"

        try:
            joints = self._get_current_joints()
            if joints is None:
                return "Joint positions not available (no feedback received)"
            joint_str = ", ".join(f"{j:.4f}" for j in joints)
            return f"Current joint positions: [{joint_str}] rad"
        except Exception as e:
            logger.error(f"get_arm_position failed: {e}")
            return f"Error getting arm position: {e}"

    @skill()
    def get_end_effector_pose(self) -> str:
        """Get the current pose of the robot arm end-effector.

        Returns:
            Current pose as (x, y, z, roll, pitch, yaw)
        """
        if self._get_ee_pose is None:
            return "Error: ManipulationModule not connected"

        try:
            pose = self._get_ee_pose()
            if pose is None:
                return "End-effector pose not available (no feedback received)"
            x, y, z, roll, pitch, yaw = pose
            return (
                f"End-effector pose: position=({x:.4f}, {y:.4f}, {z:.4f})m, "
                f"orientation=({roll:.3f}, {pitch:.3f}, {yaw:.3f})rad"
            )
        except Exception as e:
            logger.error(f"get_end_effector_pose failed: {e}")
            return f"Error getting end-effector pose: {e}"


# Expose blueprint for declarative composition
manipulation_skills = ManipulationSkillContainer.blueprint

__all__ = ["ManipulationSkillContainer", "manipulation_skills"]
