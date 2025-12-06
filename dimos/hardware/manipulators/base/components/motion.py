# Copyright 2025 Dimensional Inc.
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

"""Standard motion control component for manipulator drivers."""

import logging
from queue import Queue
import time
from typing import Optional

from ..driver import Command
from ..sdk_interface import BaseManipulatorSDK
from ..spec import ManipulatorCapabilities
from ..utils import SharedState, scale_velocities, validate_joint_limits, validate_velocity_limits


class StandardMotionComponent:
    """Motion control component that works with any SDK wrapper.

    This component provides standard motion control RPC methods that work
    consistently across all manipulator types. It handles:
    - Joint position control
    - Joint velocity control
    - Joint effort/torque control (if supported)
    - Trajectory execution (if supported)
    - Motion safety validation
    """

    def __init__(
        self,
        sdk: BaseManipulatorSDK | None = None,
        shared_state: SharedState | None = None,
        command_queue: Queue | None = None,
        capabilities: ManipulatorCapabilities | None = None,
    ):
        """Initialize the motion component.

        Args:
            sdk: SDK wrapper instance (can be set later)
            shared_state: Shared state instance (can be set later)
            command_queue: Command queue (can be set later)
            capabilities: Manipulator capabilities (can be set later)
        """
        self.sdk = sdk
        self.shared_state = shared_state
        self.command_queue = command_queue
        self.capabilities = capabilities
        self.logger = logging.getLogger(self.__class__.__name__)

        # Motion limits
        self.velocity_scale = 1.0  # Global velocity scaling (0-1)
        self.acceleration_scale = 1.0  # Global acceleration scaling (0-1)

    # ============= Initialization Methods (called by BaseDriver) =============

    def set_sdk(self, sdk: BaseManipulatorSDK):
        """Set the SDK wrapper instance."""
        self.sdk = sdk

    def set_shared_state(self, shared_state: SharedState):
        """Set the shared state instance."""
        self.shared_state = shared_state

    def set_command_queue(self, command_queue: Queue):
        """Set the command queue instance."""
        self.command_queue = command_queue

    def set_capabilities(self, capabilities: ManipulatorCapabilities):
        """Set the capabilities instance."""
        self.capabilities = capabilities

    def initialize(self):
        """Initialize the component after all resources are set."""
        self.logger.debug("Motion component initialized")

    # ============= RPC Methods =============

    def rpc_move_joint(
        self,
        positions: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
        validate: bool = True,
    ) -> dict:
        """Move joints to target positions.

        Args:
            positions: Target joint positions in radians
            velocity: Velocity scaling factor (0-1)
            acceleration: Acceleration scaling factor (0-1)
            wait: If True, block until motion completes
            validate: If True, validate against joint limits

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Validate inputs
            if validate and self.capabilities:
                if len(positions) != self.capabilities.dof:
                    return {
                        "success": False,
                        "error": f"Expected {self.capabilities.dof} positions, got {len(positions)}",
                    }

                # Check joint limits
                if self.capabilities.joint_limits_lower and self.capabilities.joint_limits_upper:
                    valid, error = validate_joint_limits(
                        positions,
                        self.capabilities.joint_limits_lower,
                        self.capabilities.joint_limits_upper,
                    )
                    if not valid:
                        return {"success": False, "error": error}

            # Apply global scaling
            velocity = velocity * self.velocity_scale
            acceleration = acceleration * self.acceleration_scale

            # Queue command for async execution
            if self.command_queue and not wait:
                command = Command(
                    type="position",
                    data={
                        "positions": positions,
                        "velocity": velocity,
                        "acceleration": acceleration,
                        "wait": False,
                    },
                    timestamp=time.time(),
                )
                self.command_queue.put(command)
                return {"success": True, "queued": True}

            # Execute directly (blocking or wait mode)
            success = self.sdk.set_joint_positions(positions, velocity, acceleration, wait)

            if success and self.shared_state:
                self.shared_state.set_target_joints(positions=positions)

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in move_joint: {e}")
            return {"success": False, "error": str(e)}

    def rpc_move_joint_velocity(
        self, velocities: list[float], acceleration: float = 1.0, validate: bool = True
    ) -> dict:
        """Set joint velocities.

        Args:
            velocities: Target joint velocities in rad/s
            acceleration: Acceleration scaling factor (0-1)
            validate: If True, validate against velocity limits

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Validate inputs
            if validate and self.capabilities:
                if len(velocities) != self.capabilities.dof:
                    return {
                        "success": False,
                        "error": f"Expected {self.capabilities.dof} velocities, got {len(velocities)}",
                    }

                # Check velocity limits
                if self.capabilities.max_joint_velocity:
                    valid, error = validate_velocity_limits(
                        velocities, self.capabilities.max_joint_velocity, self.velocity_scale
                    )
                    if not valid:
                        # Scale velocities to stay within limits
                        velocities = scale_velocities(
                            velocities, self.capabilities.max_joint_velocity, self.velocity_scale
                        )
                        self.logger.warning("Velocities scaled to stay within limits")

            # Queue command for async execution
            if self.command_queue:
                command = Command(
                    type="velocity", data={"velocities": velocities}, timestamp=time.time()
                )
                self.command_queue.put(command)
                return {"success": True, "queued": True}

            # Execute directly
            success = self.sdk.set_joint_velocities(velocities)

            if success and self.shared_state:
                self.shared_state.set_target_joints(velocities=velocities)

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in move_joint_velocity: {e}")
            return {"success": False, "error": str(e)}

    def rpc_move_joint_effort(self, efforts: list[float], validate: bool = True) -> dict:
        """Set joint efforts/torques.

        Args:
            efforts: Target joint efforts in Nm
            validate: If True, validate inputs

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Check if effort control is supported
            if not hasattr(self.sdk, "set_joint_efforts"):
                return {"success": False, "error": "Effort control not supported"}

            # Validate inputs
            if validate and self.capabilities:
                if len(efforts) != self.capabilities.dof:
                    return {
                        "success": False,
                        "error": f"Expected {self.capabilities.dof} efforts, got {len(efforts)}",
                    }

            # Queue command for async execution
            if self.command_queue:
                command = Command(type="effort", data={"efforts": efforts}, timestamp=time.time())
                self.command_queue.put(command)
                return {"success": True, "queued": True}

            # Execute directly
            success = self.sdk.set_joint_efforts(efforts)

            if success and self.shared_state:
                self.shared_state.set_target_joints(efforts=efforts)

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in move_joint_effort: {e}")
            return {"success": False, "error": str(e)}

    def rpc_stop_motion(self) -> dict:
        """Stop all ongoing motion immediately.

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Queue stop command with high priority
            if self.command_queue:
                command = Command(type="stop", data={}, timestamp=time.time())
                # Clear queue and add stop command
                while not self.command_queue.empty():
                    try:
                        self.command_queue.get_nowait()
                    except:
                        break
                self.command_queue.put(command)

            # Also execute directly for immediate stop
            success = self.sdk.stop_motion()

            # Clear targets
            if self.shared_state:
                self.shared_state.set_target_joints(positions=None, velocities=None, efforts=None)

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in stop_motion: {e}")
            return {"success": False, "error": str(e)}

    def rpc_get_joint_state(self) -> dict:
        """Get current joint state.

        Returns:
            Dict with joint positions, velocities, efforts, and timestamp
        """
        try:
            if self.shared_state:
                # Get from shared state (updated by reader thread)
                positions, velocities, efforts = self.shared_state.get_joint_state()
            else:
                # Get directly from SDK
                positions = self.sdk.get_joint_positions()
                velocities = self.sdk.get_joint_velocities()
                efforts = self.sdk.get_joint_efforts()

            return {
                "positions": positions,
                "velocities": velocities,
                "efforts": efforts,
                "timestamp": time.time(),
                "success": True,
            }

        except Exception as e:
            self.logger.error(f"Error in get_joint_state: {e}")
            return {"success": False, "error": str(e)}

    def rpc_get_joint_limits(self) -> dict:
        """Get joint position limits.

        Returns:
            Dict with lower and upper limits in radians
        """
        try:
            if self.capabilities:
                return {
                    "lower": self.capabilities.joint_limits_lower,
                    "upper": self.capabilities.joint_limits_upper,
                    "success": True,
                }
            else:
                lower, upper = self.sdk.get_joint_limits()
                return {"lower": lower, "upper": upper, "success": True}

        except Exception as e:
            self.logger.error(f"Error in get_joint_limits: {e}")
            return {"success": False, "error": str(e)}

    def rpc_get_velocity_limits(self) -> dict:
        """Get joint velocity limits.

        Returns:
            Dict with maximum velocities in rad/s
        """
        try:
            if self.capabilities:
                return {"limits": self.capabilities.max_joint_velocity, "success": True}
            else:
                limits = self.sdk.get_velocity_limits()
                return {"limits": limits, "success": True}

        except Exception as e:
            self.logger.error(f"Error in get_velocity_limits: {e}")
            return {"success": False, "error": str(e)}

    def rpc_set_velocity_scale(self, scale: float) -> dict:
        """Set global velocity scaling factor.

        Args:
            scale: Velocity scale factor (0-1)

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if scale <= 0 or scale > 1:
                return {"success": False, "error": f"Invalid scale {scale}, must be in (0, 1]"}

            self.velocity_scale = scale
            return {"success": True, "scale": scale}

        except Exception as e:
            self.logger.error(f"Error in set_velocity_scale: {e}")
            return {"success": False, "error": str(e)}

    def rpc_set_acceleration_scale(self, scale: float) -> dict:
        """Set global acceleration scaling factor.

        Args:
            scale: Acceleration scale factor (0-1)

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            if scale <= 0 or scale > 1:
                return {"success": False, "error": f"Invalid scale {scale}, must be in (0, 1]"}

            self.acceleration_scale = scale
            return {"success": True, "scale": scale}

        except Exception as e:
            self.logger.error(f"Error in set_acceleration_scale: {e}")
            return {"success": False, "error": str(e)}

    # ============= Cartesian Control (Optional) =============

    def rpc_move_cartesian(
        self, pose: dict, velocity: float = 1.0, acceleration: float = 1.0, wait: bool = False
    ) -> dict:
        """Move end-effector to target pose.

        Args:
            pose: Target pose with keys: x, y, z (meters), roll, pitch, yaw (radians)
            velocity: Velocity scaling factor (0-1)
            acceleration: Acceleration scaling factor (0-1)
            wait: If True, block until motion completes

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Check if Cartesian control is supported
            if not self.capabilities or not self.capabilities.has_cartesian_control:
                return {"success": False, "error": "Cartesian control not supported"}

            # Apply global scaling
            velocity = velocity * self.velocity_scale
            acceleration = acceleration * self.acceleration_scale

            # Queue command for async execution
            if self.command_queue and not wait:
                command = Command(
                    type="cartesian",
                    data={
                        "pose": pose,
                        "velocity": velocity,
                        "acceleration": acceleration,
                        "wait": False,
                    },
                    timestamp=time.time(),
                )
                self.command_queue.put(command)
                return {"success": True, "queued": True}

            # Execute directly
            success = self.sdk.set_cartesian_position(pose, velocity, acceleration, wait)

            if success and self.shared_state:
                self.shared_state.target_cartesian_position = pose

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in move_cartesian: {e}")
            return {"success": False, "error": str(e)}

    def rpc_get_cartesian_state(self) -> dict:
        """Get current end-effector pose.

        Returns:
            Dict with pose (x, y, z, roll, pitch, yaw) and timestamp
        """
        try:
            # Check if Cartesian control is supported
            if not self.capabilities or not self.capabilities.has_cartesian_control:
                return {"success": False, "error": "Cartesian control not supported"}

            if self.shared_state and self.shared_state.cartesian_position:
                # Get from shared state
                pose = self.shared_state.cartesian_position
            else:
                # Get directly from SDK
                pose = self.sdk.get_cartesian_position()

            if pose:
                return {"pose": pose, "timestamp": time.time(), "success": True}
            else:
                return {"success": False, "error": "Failed to get Cartesian state"}

        except Exception as e:
            self.logger.error(f"Error in get_cartesian_state: {e}")
            return {"success": False, "error": str(e)}

    # ============= Trajectory Execution (Optional) =============

    def rpc_execute_trajectory(self, trajectory: list[dict], wait: bool = True) -> dict:
        """Execute a joint trajectory.

        Args:
            trajectory: List of waypoints, each with:
                       - 'positions': list[float] in radians
                       - 'velocities': Optional list[float] in rad/s
                       - 'time': float seconds from start
            wait: If True, block until trajectory completes

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Check if trajectory execution is supported
            if not hasattr(self.sdk, "execute_trajectory"):
                return {"success": False, "error": "Trajectory execution not supported"}

            # Validate trajectory if capabilities available
            if self.capabilities:
                from ..utils import validate_trajectory

                valid, error = validate_trajectory(
                    trajectory,
                    self.capabilities.joint_limits_lower,
                    self.capabilities.joint_limits_upper,
                    self.capabilities.max_joint_velocity,
                    self.capabilities.max_joint_acceleration,
                )
                if not valid:
                    return {"success": False, "error": error}

            # Execute trajectory
            success = self.sdk.execute_trajectory(trajectory, wait)

            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in execute_trajectory: {e}")
            return {"success": False, "error": str(e)}

    def rpc_stop_trajectory(self) -> dict:
        """Stop any executing trajectory.

        Returns:
            Dict with 'success' and optional 'error' keys
        """
        try:
            # Check if trajectory execution is supported
            if not hasattr(self.sdk, "stop_trajectory"):
                return {"success": False, "error": "Trajectory execution not supported"}

            success = self.sdk.stop_trajectory()
            return {"success": success}

        except Exception as e:
            self.logger.error(f"Error in stop_trajectory: {e}")
            return {"success": False, "error": str(e)}
