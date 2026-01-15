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

"""Piper backend - implements ManipulatorBackend protocol.

Handles all Piper SDK communication and unit conversion.
"""

import math
import time
from typing import Any

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorBackend,
    ManipulatorInfo,
)

# Unit conversion constants
# Piper uses 0.001 degrees internally
RAD_TO_PIPER = 57295.7795  # radians to Piper units (0.001 degrees)
PIPER_TO_RAD = 1.0 / RAD_TO_PIPER  # Piper units to radians


class PiperBackend(ManipulatorBackend):
    """Piper-specific backend.

    Implements ManipulatorBackend protocol via duck typing.
    No inheritance required - just matching method signatures.

    Unit conversions:
    - Angles: Piper uses 0.001 degrees, we use radians
    - Velocities: Piper uses internal units, we use rad/s
    """

    def __init__(self, can_port: str = "can0", dof: int = 6) -> None:
        if dof != 6:
            raise ValueError(f"PiperBackend only supports 6 DOF (got {dof})")
        self._can_port = can_port
        self._dof = dof
        self._sdk: Any = None
        self._connected: bool = False
        self._enabled: bool = False
        self._control_mode: ControlMode = ControlMode.POSITION

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to Piper via CAN bus."""
        try:
            from piper_sdk import C_PiperInterface_V2

            self._sdk = C_PiperInterface_V2(
                can_name=self._can_port,
                judge_flag=True,  # Enable safety checks
                can_auto_init=True,  # Let SDK handle CAN initialization
                dh_is_offset=False,
            )

            # Connect to CAN port
            self._sdk.ConnectPort(piper_init=True, start_thread=True)

            # Wait for initialization
            time.sleep(0.025)

            # Check connection by trying to get status
            status = self._sdk.GetArmStatus()
            if status is not None:
                self._connected = True
                print(f"Piper connected via CAN port {self._can_port}")
                return True
            else:
                print(f"ERROR: Failed to connect to Piper on {self._can_port} - no status received")
                return False

        except ImportError:
            print("ERROR: Piper SDK not installed. Please install piper_sdk")
            return False
        except Exception as e:
            print(f"ERROR: Failed to connect to Piper on {self._can_port}: {e}")
            return False

    def disconnect(self) -> None:
        """Disconnect from Piper."""
        if self._sdk:
            try:
                if self._enabled:
                    self._sdk.DisablePiper()
                    self._enabled = False
                self._sdk.DisconnectPort()
            except Exception:
                pass
            finally:
                self._sdk = None
                self._connected = False

    def is_connected(self) -> bool:
        """Check if connected to Piper."""
        if not self._connected or not self._sdk:
            return False

        try:
            status = self._sdk.GetArmStatus()
            return status is not None
        except Exception:
            return False

    # =========================================================================
    # Info
    # =========================================================================

    def get_info(self) -> ManipulatorInfo:
        """Get Piper information."""
        firmware_version = None
        if self._sdk:
            try:
                firmware_version = self._sdk.GetPiperFirmwareVersion()
            except Exception:
                pass

        return ManipulatorInfo(
            vendor="Agilex",
            model="Piper",
            dof=self._dof,
            firmware_version=firmware_version,
        )

    def get_dof(self) -> int:
        """Get degrees of freedom."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Get joint limits."""
        # Piper joint limits (approximate, in radians)
        lower = [-3.14, -2.35, -2.35, -3.14, -2.35, -3.14]
        upper = [3.14, 2.35, 2.35, 3.14, 2.35, 3.14]
        max_vel = [math.pi] * self._dof  # ~180 deg/s

        return JointLimits(
            position_lower=lower,
            position_upper=upper,
            velocity_max=max_vel,
        )

    # =========================================================================
    # Control Mode
    # =========================================================================

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set Piper control mode via MotionCtrl_2."""
        if not self._sdk:
            return False

        # Piper move modes: 0x01=position, 0x02=velocity
        # SERVO_POSITION uses position mode for high-freq streaming
        move_mode = 0x01  # Default position mode
        if mode == ControlMode.VELOCITY:
            move_mode = 0x02

        try:
            self._sdk.MotionCtrl_2(
                ctrl_mode=0x01,  # CAN control mode
                move_mode=move_mode,
                move_spd_rate_ctrl=50,  # Speed rate (0-100)
                is_mit_mode=0x00,  # Not MIT mode
            )
            self._control_mode = mode
            return True
        except Exception:
            return False

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_joint_positions(self) -> list[float]:
        """Read joint positions (Piper units -> radians)."""
        if not self._sdk:
            raise RuntimeError("Not connected")

        joint_msgs = self._sdk.GetArmJointMsgs()
        if not joint_msgs or not joint_msgs.joint_state:
            raise RuntimeError("Failed to read joint positions")

        js = joint_msgs.joint_state
        return [
            js.joint_1 * PIPER_TO_RAD,
            js.joint_2 * PIPER_TO_RAD,
            js.joint_3 * PIPER_TO_RAD,
            js.joint_4 * PIPER_TO_RAD,
            js.joint_5 * PIPER_TO_RAD,
            js.joint_6 * PIPER_TO_RAD,
        ]

    def read_joint_velocities(self) -> list[float]:
        """Read joint velocities.

        Note: Piper doesn't provide real-time velocity feedback.
        Returns zeros. For velocity estimation, use finite differences.
        """
        return [0.0] * self._dof

    def read_joint_efforts(self) -> list[float]:
        """Read joint efforts/torques.

        Note: Piper doesn't provide torque feedback by default.
        """
        return [0.0] * self._dof

    def read_state(self) -> dict[str, int]:
        """Read robot state."""
        if not self._sdk:
            return {"state": 0, "mode": 0}

        try:
            status = self._sdk.GetArmStatus()
            if status and status.arm_status:
                arm_status = status.arm_status
                error_code = getattr(arm_status, "err_code", 0)
                state = 2 if error_code != 0 else 0  # 2=error, 0=idle
                return {
                    "state": state,
                    "mode": 0,  # Piper doesn't expose mode
                    "error_code": error_code,
                }
        except Exception:
            pass

        return {"state": 0, "mode": 0}

    def read_error(self) -> tuple[int, str]:
        """Read error code and message."""
        if not self._sdk:
            return 0, ""

        try:
            status = self._sdk.GetArmStatus()
            if status and status.arm_status:
                error_code = getattr(status.arm_status, "err_code", 0)
                if error_code == 0:
                    return 0, ""

                # Piper error codes
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
                return error_code, error_map.get(error_code, f"Unknown error {error_code}")
        except Exception:
            pass

        return 0, ""

    # =========================================================================
    # Motion Control (Joint Space)
    # =========================================================================

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Write joint positions (radians -> Piper units).

        Args:
            positions: Target positions in radians
            velocity: Speed as fraction of max (0-1)
        """
        if not self._sdk:
            return False

        # Convert radians to Piper units (0.001 degrees)
        piper_joints = [round(rad * RAD_TO_PIPER) for rad in positions]

        # Set speed rate if not full speed
        if velocity < 1.0:
            speed_rate = int(velocity * 100)
            try:
                self._sdk.MotionCtrl_2(
                    ctrl_mode=0x01,
                    move_mode=0x01,
                    move_spd_rate_ctrl=speed_rate,
                    is_mit_mode=0x00,
                )
            except Exception:
                pass

        try:
            self._sdk.JointCtrl(
                piper_joints[0],
                piper_joints[1],
                piper_joints[2],
                piper_joints[3],
                piper_joints[4],
                piper_joints[5],
            )
            return True
        except Exception as e:
            print(f"Piper joint control error: {e}")
            return False

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Write joint velocities.

        Note: Piper doesn't have native velocity control at SDK level.
        Returns False - the driver should implement this via position integration.
        """
        return False

    def write_stop(self) -> bool:
        """Emergency stop."""
        if not self._sdk:
            return False

        try:
            if hasattr(self._sdk, "EmergencyStop"):
                self._sdk.EmergencyStop()
                return True
        except Exception:
            pass

        # Fallback: disable arm
        return self.write_enable(False)

    # =========================================================================
    # Servo Control
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos."""
        if not self._sdk:
            return False

        try:
            if enable:
                # Enable with retries (500ms max)
                attempts = 0
                max_attempts = 50
                success = False
                while attempts < max_attempts:
                    if self._sdk.EnablePiper():
                        success = True
                        break
                    time.sleep(0.01)
                    attempts += 1

                if success:
                    self._enabled = True
                    # Set control mode
                    self._sdk.MotionCtrl_2(
                        ctrl_mode=0x01,
                        move_mode=0x01,
                        move_spd_rate_ctrl=30,
                        is_mit_mode=0x00,
                    )
                    return True
                return False
            else:
                self._sdk.DisablePiper()
                self._enabled = False
                return True
        except Exception:
            return False

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        return self._enabled

    def write_clear_errors(self) -> bool:
        """Clear error state."""
        if not self._sdk:
            return False

        try:
            if hasattr(self._sdk, "ClearError"):
                self._sdk.ClearError()
                return True
        except Exception:
            pass

        # Alternative: disable and re-enable
        self.write_enable(False)
        time.sleep(0.1)
        return self.write_enable(True)

    # =========================================================================
    # Cartesian Control (Optional)
    # =========================================================================

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose.

        Note: Piper may not support direct cartesian feedback.
        Returns None if not available.
        """
        if not self._sdk:
            return None

        try:
            if hasattr(self._sdk, "GetArmEndPoseMsgs"):
                pose_msgs = self._sdk.GetArmEndPoseMsgs()
                if pose_msgs and pose_msgs.end_pose:
                    ep = pose_msgs.end_pose
                    return {
                        "x": ep.X_axis / 1000.0,  # mm -> m
                        "y": ep.Y_axis / 1000.0,
                        "z": ep.Z_axis / 1000.0,
                        "roll": ep.RX_axis * PIPER_TO_RAD,
                        "pitch": ep.RY_axis * PIPER_TO_RAD,
                        "yaw": ep.RZ_axis * PIPER_TO_RAD,
                    }
        except Exception:
            pass

        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Write end-effector pose.

        Note: Piper may not support direct cartesian control.
        """
        # Cartesian control not commonly supported in Piper SDK
        return False

    # =========================================================================
    # Gripper (Optional)
    # =========================================================================

    def read_gripper_position(self) -> float | None:
        """Read gripper position (percentage -> meters)."""
        if not self._sdk:
            return None

        try:
            if hasattr(self._sdk, "GetArmGripperMsgs"):
                gripper_msgs = self._sdk.GetArmGripperMsgs()
                if gripper_msgs and gripper_msgs.gripper_state:
                    # Piper gripper position is 0-100 percentage
                    # Convert to meters (assume max opening 0.08m)
                    pos = gripper_msgs.gripper_state.grippers_angle
                    return float(pos / 100.0) * 0.08
        except Exception:
            pass

        return None

    def write_gripper_position(self, position: float) -> bool:
        """Write gripper position (meters -> percentage)."""
        if not self._sdk:
            return False

        try:
            if hasattr(self._sdk, "GripperCtrl"):
                # Convert meters to percentage (0-100)
                # Assume max opening 0.08m
                percentage = int((position / 0.08) * 100)
                percentage = max(0, min(100, percentage))
                self._sdk.GripperCtrl(percentage, 1000, 0x01, 0)
                return True
        except Exception:
            pass

        return False

    # =========================================================================
    # Force/Torque Sensor (Optional)
    # =========================================================================

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data.

        Note: Piper doesn't typically have F/T sensor.
        """
        return None


__all__ = ["PiperBackend"]
