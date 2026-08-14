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

"""Piper adapter - implements ManipulatorAdapter protocol.

SDK Units: angles=0.001 degrees (millidegrees), distance=mm
DimOS Units: angles=radians, distance=meters
"""

from __future__ import annotations

from collections.abc import Callable
import math
import time
from typing import Any

from piper_sdk import C_PiperInterface_V2

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorAdapter,
    ManipulatorInfo,
)
from dimos.utils.logging_config import setup_logger

# Unit conversion constants
# Piper uses 0.001 degrees (millidegrees) for angles
RAD_TO_MILLIDEG = 57295.7795  # radians -> millidegrees
MILLIDEG_TO_RAD = 1.0 / RAD_TO_MILLIDEG  # millidegrees -> radians
MM_TO_M = 0.001  # mm -> meters

# Hardware specs
GRIPPER_MAX_OPENING_M = 0.08  # Max gripper opening in meters
GRIPPER_STROKE_UNITS_PER_M = 1_000_000
SHUTDOWN_POSITION_TOLERANCE = 0.03
SHUTDOWN_POLL_INTERVAL = 0.05
SHUTDOWN_SPEED_RATE = 30
SHUTDOWN_TIMEOUT = 5.0
STARTUP_RESET_WAIT = 0.5
STARTUP_ZERO_WAIT = 1.0
ENABLE_RETRY_COUNT = 50
ENABLE_RETRY_INTERVAL = 0.01

# Default configurable parameters
DEFAULT_GRIPPER_SPEED = 1000
GRIPPER_DISABLE_CODE = 0x02

logger = setup_logger()


class PiperAdapter(ManipulatorAdapter):
    """Piper-specific adapter.

    Implements ManipulatorAdapter protocol via duck typing.
    No inheritance required - just matching method signatures.

    Unit conversions:
    - Angles: Piper uses 0.001 degrees, we use radians
    - Velocities: Piper uses internal units, we use rad/s
    """

    def __init__(
        self,
        address: str = "can0",
        dof: int = 6,
        gripper_speed: int = DEFAULT_GRIPPER_SPEED,
        **_: object,
    ) -> None:
        if dof != 6:
            raise ValueError(f"PiperAdapter only supports 6 DOF (got {dof})")
        self._can_port = address
        self._dof = dof
        self._gripper_speed = gripper_speed
        self._sdk: C_PiperInterface_V2 | None = None
        self._connected: bool = False
        self._enabled: bool = False
        self._gripper_initialized: bool = False
        self._control_mode: ControlMode = ControlMode.POSITION

    def connect(self) -> bool:
        """Connect to Piper via CAN bus."""
        try:
            sdk = C_PiperInterface_V2(
                can_name=self._can_port,
                judge_flag=True,  # Enable safety checks
                can_auto_init=True,  # Let SDK handle CAN initialization
                dh_is_offset=False,
            )
            self._sdk = sdk

            # Connect to CAN port
            sdk.ConnectPort(piper_init=True, start_thread=True)

            # Wait for initialization
            time.sleep(0.025)

            # Check connection by trying to get status
            status = sdk.GetArmStatus()
            if status is not None:
                if not self._initialize_startup_state():
                    self._close_failed_connection()
                    return False
                self._connected = True
                logger.info("Piper connected", can_port=self._can_port)
                return True
            logger.error("Failed to connect to Piper: no status received", can_port=self._can_port)
            return False
        except Exception:
            logger.exception("Failed to connect to Piper", can_port=self._can_port)
            return False

    def _initialize_startup_state(self) -> bool:
        """Run Piper's fixed reset, zero-pose, and startup settle sequence."""
        sdk = self._sdk
        if sdk is None:
            return False

        for reset_number in range(2):
            try:
                sdk.MotionCtrl_1(0x02, 0, 0)
            except Exception:
                logger.exception(f"Piper startup reset {reset_number + 1} failed")
                return False
            time.sleep(STARTUP_RESET_WAIT)

        try:
            sdk.MotionCtrl_2(
                ctrl_mode=0x01,
                move_mode=0x01,
                move_spd_rate_ctrl=SHUTDOWN_SPEED_RATE,
                is_mit_mode=0x00,
            )
            sdk.JointCtrl(0, 0, 0, 0, 0, 0)
        except Exception:
            logger.exception("Failed to command Piper startup zero pose")
            return False

        try:
            sdk.GripperCtrl(0, DEFAULT_GRIPPER_SPEED, 0x01, 0)
            self._gripper_initialized = True
        except Exception:
            logger.warning("Piper gripper startup command failed; continuing arm startup")

        time.sleep(STARTUP_ZERO_WAIT)
        return True

    def _enable_piper(self) -> bool:
        """Enable Piper with the SDK retry policy, without changing mode."""
        sdk = self._sdk
        if sdk is None:
            return False
        try:
            for attempt in range(ENABLE_RETRY_COUNT):
                if sdk.EnablePiper():
                    self._enabled = True
                    return True
                if attempt < ENABLE_RETRY_COUNT - 1:
                    time.sleep(ENABLE_RETRY_INTERVAL)
        except Exception:
            logger.exception("Piper SDK enable command failed")
        return False

    def _close_failed_connection(self) -> None:
        """Release a CAN connection after mandatory startup initialization fails."""
        sdk = self._sdk
        if sdk is not None:
            if self._enabled:
                self._attempt_cleanup_step(
                    sdk.DisablePiper,
                    exception_message="Failed to disable Piper after startup failure",
                )
            self._attempt_cleanup_step(
                sdk.DisconnectPort,
                exception_message="Failed to disconnect Piper after startup failure",
            )
        self._clear_connection_state()

    def _attempt_cleanup_step(
        self,
        action: Callable[[], Any],
        *,
        false_message: str | None = None,
        exception_message: str,
    ) -> None:
        """Run one best-effort shutdown action and retain its failure logging."""
        try:
            if action() is False and false_message is not None:
                logger.error(false_message)
        except Exception:
            logger.exception(exception_message)

    def _clear_connection_state(self) -> None:
        """Clear all local state after a connection attempt ends."""
        self._sdk = None
        self._connected = False
        self._enabled = False
        self._gripper_initialized = False

    def disconnect(self) -> None:
        """Disconnect from Piper."""
        sdk = self._sdk
        if sdk is None:
            self._clear_connection_state()
            return

        self._attempt_cleanup_step(
            self._move_to_zero_position,
            false_message="Piper did not reach its zero position before disconnect",
            exception_message="Error homing Piper before disconnect",
        )
        self._attempt_cleanup_step(
            self._deactivate_gripper,
            false_message="Failed to deactivate Piper gripper",
            exception_message="Error deactivating Piper gripper",
        )
        self._attempt_cleanup_step(
            sdk.DisablePiper,
            exception_message="Error disabling Piper",
        )
        self._attempt_cleanup_step(
            sdk.DisconnectPort,
            exception_message="Error disconnecting Piper CAN port",
        )
        self._clear_connection_state()

    def is_connected(self) -> bool:
        """Check if connected to Piper."""
        if not self._connected or not self._sdk:
            return False

        try:
            status = self._sdk.GetArmStatus()
            return status is not None
        except Exception:
            logger.exception("Piper arm status query failed")
            return False

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Stop motion without disabling servos.

        Servo power must remain on until ``disconnect`` has completed the
        bounded home-to-zero movement.
        """
        stopped = self.write_stop()
        if not stopped:
            logger.error("Failed to stop Piper motion during deactivation")
        return stopped

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
            logger.exception("Failed to set Piper control mode")
            return False

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Read joint positions (Piper units -> radians)."""
        if not self._sdk:
            raise RuntimeError("Not connected")

        joint_msgs = self._sdk.GetArmJointMsgs()
        if not joint_msgs or not joint_msgs.joint_state:
            raise RuntimeError("Failed to read joint positions")

        js = joint_msgs.joint_state
        return [
            js.joint_1 * MILLIDEG_TO_RAD,
            js.joint_2 * MILLIDEG_TO_RAD,
            js.joint_3 * MILLIDEG_TO_RAD,
            js.joint_4 * MILLIDEG_TO_RAD,
            js.joint_5 * MILLIDEG_TO_RAD,
            js.joint_6 * MILLIDEG_TO_RAD,
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
        piper_joints = [round(rad * RAD_TO_MILLIDEG) for rad in positions]

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
                logger.exception("Failed to set Piper motion speed")

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
        except Exception:
            logger.exception("Piper joint control failed")
            return False

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Write joint velocities.

        Note: Piper doesn't have native velocity control at SDK level.
        Returns False - the driver should implement this via position integration.
        """
        return False

    def write_stop(self) -> bool:
        """Gracefully stop Piper motion."""
        if not self._sdk:
            return False

        try:
            self._sdk.MotionCtrl_1(0x01, 0, 0)
            return True
        except Exception:
            logger.exception("Failed to stop Piper motion")
            return False

    def _move_to_zero_position(self) -> bool:
        """Move all arm joints to zero before disabling the servos."""
        if not self._sdk:
            return False

        try:
            self._sdk.MotionCtrl_2(
                ctrl_mode=0x01,
                move_mode=0x01,
                move_spd_rate_ctrl=SHUTDOWN_SPEED_RATE,
                is_mit_mode=0x00,
            )
            self._sdk.JointCtrl(0, 0, 0, 0, 0, 0)
        except Exception:
            logger.exception("Failed to command Piper zero position")
            return False

        deadline = time.monotonic() + SHUTDOWN_TIMEOUT
        while time.monotonic() < deadline:
            try:
                if (
                    max(abs(position) for position in self.read_joint_positions())
                    <= SHUTDOWN_POSITION_TOLERANCE
                ):
                    return True
            except Exception:
                logger.exception("Failed to read Piper position during shutdown")
                return False
            time.sleep(SHUTDOWN_POLL_INTERVAL)
        return False

    def _initialize_gripper(self) -> bool:
        """Initialize the gripper in its enabled, closed position."""
        if self._sdk is None:
            return False
        try:
            self._sdk.GripperCtrl(0, self._gripper_speed, GRIPPER_DISABLE_CODE, 0)
            self._sdk.GripperCtrl(0, self._gripper_speed, 0x01, 0)
            self._gripper_initialized = True
            return True
        except Exception:
            logger.exception("Failed to initialize Piper gripper")
            return False

    def _deactivate_gripper(self) -> bool:
        """Disable gripper control before disconnecting the arm."""
        if self._sdk is None:
            return True
        try:
            self._sdk.GripperCtrl(0, self._gripper_speed, GRIPPER_DISABLE_CODE, 0)
            return True
        except Exception:
            logger.exception("Failed to deactivate Piper gripper")
            return False

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos."""
        if not self._sdk:
            return False

        try:
            if enable:
                if self._enabled:
                    return True
                if not self._enable_piper():
                    return False
                self._sdk.MotionCtrl_2(
                    ctrl_mode=0x01,
                    move_mode=0x01,
                    move_spd_rate_ctrl=30,
                    is_mit_mode=0x00,
                )
                return True
            else:
                self._sdk.DisablePiper()
                self._enabled = False
                return True
        except Exception:
            logger.exception("Failed to change Piper enable state", enable=enable)
            return False

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        return self._enabled

    def write_clear_errors(self) -> bool:
        """Clear error state."""
        if not self._sdk:
            return False

        try:
            self._sdk.ClearError()
            return True
        except Exception:
            logger.exception("Failed to clear Piper errors")

        # Alternative: disable and re-enable
        self.write_enable(False)
        time.sleep(0.1)
        return self.write_enable(True)

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose.

        Note: Piper may not support direct cartesian feedback.
        Returns None if not available.
        """
        if not self._sdk:
            return None

        try:
            pose_msgs = self._sdk.GetArmEndPoseMsgs()
            if pose_msgs and pose_msgs.end_pose:
                ep = pose_msgs.end_pose
                return {
                    "x": ep.X_axis * MM_TO_M,
                    "y": ep.Y_axis * MM_TO_M,
                    "z": ep.Z_axis * MM_TO_M,
                    "roll": ep.RX_axis * MILLIDEG_TO_RAD,
                    "pitch": ep.RY_axis * MILLIDEG_TO_RAD,
                    "yaw": ep.RZ_axis * MILLIDEG_TO_RAD,
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

    def read_gripper_position(self) -> float | None:
        """Read gripper position (percentage -> meters)."""
        if not self._sdk:
            return None

        try:
            gripper_msgs = self._sdk.GetArmGripperMsgs()
            if gripper_msgs and gripper_msgs.gripper_state:
                # Piper gripper position is in 0.001 mm units.
                pos: float = gripper_msgs.gripper_state.grippers_angle
                return min(
                    GRIPPER_MAX_OPENING_M,
                    max(0.0, pos / GRIPPER_STROKE_UNITS_PER_M),
                )
        except Exception:
            pass

        return None

    def write_gripper_position(self, position: float) -> bool:
        """Write gripper position (meters -> 0.001 mm units)."""
        if not self._sdk:
            return False

        try:
            if not self._gripper_initialized and not self._initialize_gripper():
                return False
            gripper_position = round(
                max(0.0, min(GRIPPER_MAX_OPENING_M, position)) * GRIPPER_STROKE_UNITS_PER_M
            )
            self._sdk.GripperCtrl(gripper_position, self._gripper_speed, 0x01, 0)
            return True
        except Exception:
            pass

        return False

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data.

        Note: Piper doesn't typically have F/T sensor.
        """
        return None
