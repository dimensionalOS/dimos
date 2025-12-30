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

"""Piper SDK wrapper implementation."""

import logging
import time
from typing import Any

from ..base.sdk_interface import BaseManipulatorSDK, ManipulatorInfo

# Unit conversion constants
RAD_TO_PIPER = 57295.7795  # radians to Piper units (0.001 degrees)
PIPER_TO_RAD = 1.0 / RAD_TO_PIPER  # Piper units to radians


class PiperSDKWrapper(BaseManipulatorSDK):
    """SDK wrapper for Piper manipulators.

    This wrapper translates Piper's native SDK (which uses radians but 1-indexed joints)
    to our standard interface (0-indexed).
    """

    def __init__(self) -> None:
        """Initialize the Piper SDK wrapper."""
        self.logger = logging.getLogger(self.__class__.__name__)
        self.native_sdk: Any = None
        self.dof = 6  # Piper is always 6-DOF
        self._connected = False
        self._enabled = False

    # ============= Connection Management =============

    def connect(self, config: dict[str, Any]) -> bool:
        """Connect to Piper via CAN bus.

        Args:
            config: Configuration with 'can_port' (e.g., 'can0')

        Returns:
            True if connection successful
        """
        try:
            from piper_sdk import C_PiperInterface_V2  # type: ignore[import-not-found]

            can_port = config.get("can_port", "can0")
            self.logger.info(f"Connecting to Piper via CAN port {can_port}...")

            # Create Piper SDK instance
            self.native_sdk = C_PiperInterface_V2(
                can_name=can_port,
                judge_flag=True,  # Enable safety checks
                can_auto_init=True,  # Let SDK handle CAN initialization
                dh_is_offset=False,
            )

            # Connect to CAN port
            self.native_sdk.ConnectPort(piper_init=True, start_thread=True)

            # Wait for initialization
            time.sleep(0.025)

            # Check connection by trying to get status
            status = self.native_sdk.GetArmStatus()
            if status is not None:
                self._connected = True

                # Get firmware version
                try:
                    version = self.native_sdk.GetPiperFirmwareVersion()
                    self.logger.info(f"Connected to Piper (firmware: {version})")
                except:
                    self.logger.info("Connected to Piper")

                return True
            else:
                self.logger.error("Failed to connect to Piper - no status received")
                return False

        except ImportError:
            self.logger.error("Piper SDK not installed. Please install piper_sdk")
            return False
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from Piper."""
        if self.native_sdk:
            try:
                # Disable arm first
                if self._enabled:
                    self.native_sdk.DisablePiper()
                    self._enabled = False

                # Disconnect
                self.native_sdk.DisconnectPort()
                self._connected = False
                self.logger.info("Disconnected from Piper")
            except:
                pass
            finally:
                self.native_sdk = None

    def is_connected(self) -> bool:
        """Check if connected to Piper.

        Returns:
            True if connected
        """
        if not self._connected or not self.native_sdk:
            return False

        # Try to get status to verify connection
        try:
            status = self.native_sdk.GetArmStatus()
            return status is not None
        except:
            return False

    # ============= Joint State Query =============

    def get_joint_positions(self) -> list[float]:
        """Get current joint positions.

        Returns:
            Joint positions in RADIANS (0-indexed)
        """
        joint_msgs = self.native_sdk.GetArmJointMsgs()
        if not joint_msgs or not joint_msgs.joint_state:
            raise RuntimeError("Failed to get Piper joint positions")

        # Get joint positions from joint_state (values are in Piper units: 0.001 degrees)
        # Convert to radians using PIPER_TO_RAD conversion factor
        joint_state = joint_msgs.joint_state
        positions = [
            joint_state.joint_1 * PIPER_TO_RAD,  # Convert Piper units to radians
            joint_state.joint_2 * PIPER_TO_RAD,
            joint_state.joint_3 * PIPER_TO_RAD,
            joint_state.joint_4 * PIPER_TO_RAD,
            joint_state.joint_5 * PIPER_TO_RAD,
            joint_state.joint_6 * PIPER_TO_RAD,
        ]
        return positions

    def get_joint_velocities(self) -> list[float]:
        """Get current joint velocities.

        Returns:
            Joint velocities in RAD/S (0-indexed)
        """
        # TODO: Get actual velocities from Piper SDK
        # For now return zeros as velocity feedback may not be available
        return [0.0] * self.dof

    def get_joint_efforts(self) -> list[float]:
        """Get current joint efforts/torques.

        Returns:
            Joint efforts in Nm (0-indexed)
        """
        # TODO: Get actual efforts/torques from Piper SDK if available
        # For now return zeros as effort feedback may not be available
        return [0.0] * self.dof

    # ============= Joint Motion Control =============

    def set_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        """Move joints to target positions.

        Args:
            positions: Target positions in RADIANS (0-indexed)
            velocity: Max velocity fraction (0-1)
            acceleration: Max acceleration fraction (0-1)
            wait: If True, block until motion completes

        Returns:
            True if command accepted
        """
        # Convert radians to Piper units (0.001 degrees)
        piper_joints = [round(rad * RAD_TO_PIPER) for rad in positions]

        # Optionally set motion control parameters based on velocity/acceleration
        if velocity < 1.0 or acceleration < 1.0:
            # Scale speed rate based on velocity parameter (0-100)
            speed_rate = int(velocity * 100)
            self.native_sdk.MotionCtrl_2(
                ctrl_mode=0x01,  # CAN control mode
                move_mode=0x01,  # Move mode
                move_spd_rate_ctrl=speed_rate,  # Speed rate
                is_mit_mode=0x00,  # Not MIT mode
            )

        # Send joint control command using JointCtrl with 6 individual parameters
        try:
            self.native_sdk.JointCtrl(
                piper_joints[0],  # Joint 1
                piper_joints[1],  # Joint 2
                piper_joints[2],  # Joint 3
                piper_joints[3],  # Joint 4
                piper_joints[4],  # Joint 5
                piper_joints[5],  # Joint 6
            )
            result = True
        except Exception as e:
            self.logger.error(f"Error setting joint positions: {e}")
            result = False

        # If wait requested, poll until motion completes
        if wait and result:
            start_time = time.time()
            timeout = 30.0  # 30 second timeout

            while time.time() - start_time < timeout:
                try:
                    # Check if reached target (within tolerance)
                    current = self.get_joint_positions()
                    tolerance = 0.01  # radians
                    if all(abs(current[i] - positions[i]) < tolerance for i in range(6)):
                        break
                except:
                    pass  # Continue waiting
                time.sleep(0.01)

        return result

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        """Set joint velocity targets.

        Note: Piper doesn't have native velocity control. The driver should
        implement velocity control via position integration if needed.

        Args:
            velocities: Target velocities in RAD/S (0-indexed)

        Returns:
            False - velocity control not supported at SDK level
        """
        # Piper doesn't have native velocity control
        # The driver layer should implement this via position integration
        self.logger.debug("Velocity control not supported at SDK level - use position integration")
        return False

    def set_joint_efforts(self, efforts: list[float]) -> bool:
        """Set joint effort/torque targets.

        Args:
            efforts: Target efforts in Nm (0-indexed)

        Returns:
            True if command accepted
        """
        # Check if torque control is supported
        if not hasattr(self.native_sdk, "SetJointTorque"):
            self.logger.warning("Torque control not available in this Piper version")
            return False

        # Convert 0-indexed to 1-indexed dict
        torque_dict = {i + 1: torque for i, torque in enumerate(efforts)}

        # Send torque command
        self.native_sdk.SetJointTorque(torque_dict)
        return True

    def stop_motion(self) -> bool:
        """Stop all ongoing motion.

        Returns:
            True if stop successful
        """
        # Piper emergency stop
        if hasattr(self.native_sdk, "EmergencyStop"):
            self.native_sdk.EmergencyStop()
        else:
            # Alternative: set zero velocities
            zero_vel = {i: 0.0 for i in range(1, 7)}
            if hasattr(self.native_sdk, "SetJointSpeed"):
                self.native_sdk.SetJointSpeed(zero_vel)

        return True

    # ============= Servo Control =============

    def enable_servos(self) -> bool:
        """Enable motor control.

        Returns:
            True if servos enabled
        """
        # Enable Piper
        attempts = 0
        max_attempts = 100

        while not self.native_sdk.EnablePiper() and attempts < max_attempts:
            time.sleep(0.01)
            attempts += 1

        if attempts < max_attempts:
            self._enabled = True

            # Set control mode
            self.native_sdk.MotionCtrl_2(
                ctrl_mode=0x01,  # CAN control mode
                move_mode=0x01,  # Move mode
                move_spd_rate_ctrl=30,  # Speed rate
                is_mit_mode=0x00,  # Not MIT mode
            )

            return True

        return False

    def disable_servos(self) -> bool:
        """Disable motor control.

        Returns:
            True if servos disabled
        """
        self.native_sdk.DisablePiper()
        self._enabled = False
        return True

    def are_servos_enabled(self) -> bool:
        """Check if servos are enabled.

        Returns:
            True if enabled
        """
        return self._enabled

    # ============= System State =============

    def get_robot_state(self) -> dict[str, Any]:
        """Get current robot state.

        Returns:
            State dictionary
        """
        status = self.native_sdk.GetArmStatus()

        if status and status.arm_status:
            # Map Piper states to standard states
            # Use the nested arm_status object
            arm_status = status.arm_status

            # Default state mapping
            state = 0  # idle
            mode = 0  # position mode
            error_code = 0

            # Check for error status
            if hasattr(arm_status, "err_code"):
                error_code = arm_status.err_code
                if error_code != 0:
                    state = 2  # error state

            # Check motion status if available
            if hasattr(arm_status, "motion_status"):
                # Could check if moving
                pass

            return {
                "state": state,
                "mode": mode,
                "error_code": error_code,
                "warn_code": 0,  # Piper doesn't have warn codes
                "is_moving": False,  # Would need to track this
                "cmd_num": 0,  # Piper doesn't expose command queue
            }

        return {
            "state": 2,  # Error if can't get status
            "mode": 0,
            "error_code": 999,
            "warn_code": 0,
            "is_moving": False,
            "cmd_num": 0,
        }

    def get_error_code(self) -> int:
        """Get current error code.

        Returns:
            Error code (0 = no error)
        """
        status = self.native_sdk.GetArmStatus()
        if status and hasattr(status, "error_code"):
            return int(status.error_code)
        return 0

    def get_error_message(self) -> str:
        """Get human-readable error message.

        Returns:
            Error message string
        """
        error_code = self.get_error_code()
        if error_code == 0:
            return ""

        # Piper error codes (approximate)
        error_map = {
            1: "Communication error",
            2: "Motor error",
            3: "Encoder error",
            4: "Overtemperature",
            5: "Overcurrent",
            6: "Joint limit error",
            7: "Emergency stop",
            8: "Power error",
        }

        return error_map.get(error_code, f"Unknown error {error_code}")

    def clear_errors(self) -> bool:
        """Clear error states.

        Returns:
            True if errors cleared
        """
        if hasattr(self.native_sdk, "ClearError"):
            self.native_sdk.ClearError()
            return True

        # Alternative: disable and re-enable
        self.disable_servos()
        time.sleep(0.1)
        return self.enable_servos()

    def emergency_stop(self) -> bool:
        """Execute emergency stop.

        Returns:
            True if e-stop executed
        """
        if hasattr(self.native_sdk, "EmergencyStop"):
            self.native_sdk.EmergencyStop()
            return True

        # Alternative: disable servos
        return self.disable_servos()

    # ============= Information =============

    def get_info(self) -> ManipulatorInfo:
        """Get manipulator information.

        Returns:
            ManipulatorInfo object
        """
        firmware_version = None
        try:
            firmware_version = self.native_sdk.GetPiperFirmwareVersion()
        except:
            pass

        return ManipulatorInfo(
            vendor="Agilex",
            model="Piper",
            dof=self.dof,
            firmware_version=firmware_version,
            serial_number=None,  # Piper doesn't expose serial number
        )

    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        """Get joint position limits.

        Returns:
            Tuple of (lower_limits, upper_limits) in RADIANS
        """
        # Piper joint limits (approximate, in radians)
        lower_limits = [-3.14, -2.35, -2.35, -3.14, -2.35, -3.14]
        upper_limits = [3.14, 2.35, 2.35, 3.14, 2.35, 3.14]

        return (lower_limits, upper_limits)

    def get_velocity_limits(self) -> list[float]:
        """Get joint velocity limits.

        Returns:
            Maximum velocities in RAD/S
        """
        # Piper max velocities (approximate)
        max_vel = 3.14  # rad/s
        return [max_vel] * self.dof

    def get_acceleration_limits(self) -> list[float]:
        """Get joint acceleration limits.

        Returns:
            Maximum accelerations in RAD/S²
        """
        # Piper max accelerations (approximate)
        max_acc = 10.0  # rad/s²
        return [max_acc] * self.dof

    # ============= Optional Methods =============

    def get_cartesian_position(self) -> dict[str, float] | None:
        """Get current end-effector pose.

        Returns:
            Pose dict or None if not supported
        """
        if hasattr(self.native_sdk, "GetEndPose"):
            pose = self.native_sdk.GetEndPose()
            if pose:
                return {
                    "x": pose.x,
                    "y": pose.y,
                    "z": pose.z,
                    "roll": pose.roll,
                    "pitch": pose.pitch,
                    "yaw": pose.yaw,
                }
        return None

    def set_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        """Move end-effector to target pose.

        Args:
            pose: Target pose dict
            velocity: Max velocity fraction (0-1)
            acceleration: Max acceleration fraction (0-1)
            wait: Block until complete

        Returns:
            True if command accepted
        """
        if not hasattr(self.native_sdk, "MoveL"):
            self.logger.warning("Cartesian control not available")
            return False

        # Create pose object for Piper
        target = {
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"],
            "roll": pose["roll"],
            "pitch": pose["pitch"],
            "yaw": pose["yaw"],
        }

        # Send Cartesian command
        self.native_sdk.MoveL(target)

        # Wait if requested
        if wait:
            start_time = time.time()
            timeout = 30.0

            while time.time() - start_time < timeout:
                current = self.get_cartesian_position()
                if current:
                    # Check if reached target (within tolerance)
                    tol_pos = 0.005  # 5mm
                    tol_rot = 0.05  # ~3 degrees

                    if (
                        abs(current["x"] - pose["x"]) < tol_pos
                        and abs(current["y"] - pose["y"]) < tol_pos
                        and abs(current["z"] - pose["z"]) < tol_pos
                        and abs(current["roll"] - pose["roll"]) < tol_rot
                        and abs(current["pitch"] - pose["pitch"]) < tol_rot
                        and abs(current["yaw"] - pose["yaw"]) < tol_rot
                    ):
                        break

                time.sleep(0.01)

        return True

    def get_gripper_position(self) -> float | None:
        """Get gripper position.

        Returns:
            Position in meters or None
        """
        if hasattr(self.native_sdk, "GetGripperState"):
            state = self.native_sdk.GetGripperState()
            if state:
                # Piper gripper position is 0-100 (percentage)
                # Convert to meters (assume max opening 0.08m)
                return float(state / 100.0) * 0.08
        return None

    def set_gripper_position(self, position: float, force: float = 1.0) -> bool:
        """Set gripper position.

        Args:
            position: Target position in meters
            force: Force fraction (0-1)

        Returns:
            True if successful
        """
        if not hasattr(self.native_sdk, "GripperCtrl"):
            self.logger.warning("Gripper control not available")
            return False

        # Convert meters to percentage (0-100)
        # Assume max opening 0.08m
        percentage = int((position / 0.08) * 100)
        percentage = max(0, min(100, percentage))

        # Control gripper
        self.native_sdk.GripperCtrl(percentage)
        return True

    def set_control_mode(self, mode: str) -> bool:
        """Set control mode.

        Args:
            mode: 'position', 'velocity', 'torque', or 'impedance'

        Returns:
            True if successful
        """
        # Piper modes via MotionCtrl_2
        # ctrl_mode: 0x01=CAN control
        # move_mode: 0x01=position, 0x02=velocity?

        if not hasattr(self.native_sdk, "MotionCtrl_2"):
            return False

        move_mode = 0x01  # Default position
        if mode == "velocity":
            move_mode = 0x02

        self.native_sdk.MotionCtrl_2(
            ctrl_mode=0x01, move_mode=move_mode, move_spd_rate_ctrl=30, is_mit_mode=0x00
        )

        return True

    def get_control_mode(self) -> str | None:
        """Get current control mode.

        Returns:
            Mode string or None
        """
        status = self.native_sdk.GetArmStatus()
        if status and hasattr(status, "arm_mode"):
            # Map Piper modes
            mode_map = {0x01: "position", 0x02: "velocity"}
            return mode_map.get(status.arm_mode, "unknown")

        return "position"  # Default assumption
