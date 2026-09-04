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

"""XArm adapter - implements ManipulatorAdapter protocol.

SDK Units: angles=degrees, distance=mm, velocity=deg/s
DimOS Units: angles=radians, distance=meters, velocity=rad/s
"""

from __future__ import annotations

import math

from xarm.wrapper import XArmAPI

from dimos.hardware.manipulators.spec import (
    ControlMode,
    ManipulatorAdapter,
    ManipulatorInfo,
)
from dimos.hardware.spec import JointLimits
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Cartesian pose conversions; the gripper does not use these.
MM_TO_M = 0.001
M_TO_MM = 1000.0

# The gripper SDK's own dimensionless scale.
XARM_GRIPPER_MIN = 0.0
XARM_GRIPPER_MAX = 850.0
MAX_CARTESIAN_SPEED_MM = 500.0  # Max cartesian speed in mm/s
_XARM_LIFECYCLE_SPEED_DEG = 20.0
_XARM_LIFECYCLE_ACCEL_DEG = 500.0
_XARM6_INITIAL_JOINTS_DEG = [0.0, -40.0, -50.0, 0.0, 90.0, 0.0]
# TODO (CC): change this once we have 7dof arm setup
_XARM7_INITIAL_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, math.degrees(-0.7), 0.0]

# XArm mode codes
_XARM_MODE_POSITION = 0
_XARM_MODE_SERVO_CARTESIAN = 1
_XARM_MODE_JOINT_VELOCITY = 4
_XARM_MODE_CARTESIAN_VELOCITY = 5
_XARM_MODE_JOINT_TORQUE = 6


class XArmAdapter(ManipulatorAdapter):
    """XArm-specific adapter.

    Implements ManipulatorAdapter protocol via duck typing.
    No inheritance required - just matching method signatures.
    """

    def __init__(self, address: str, dof: int = 6, arm_dof: int | None = None, **_: object) -> None:
        if not address:
            raise ValueError("address (IP) is required for XArmAdapter")
        resolved_arm_dof = dof if arm_dof is None else arm_dof
        extra_dof = dof - resolved_arm_dof
        if resolved_arm_dof not in (6, 7) or extra_dof not in (0, 1):
            raise ValueError(
                "XArmAdapter requires 6 or 7 arm axes and at most one additional joint "
                f"(got dof={dof}, arm_dof={resolved_arm_dof})"
            )
        self._ip = address
        self._dof = dof
        self._arm_dof = resolved_arm_dof
        self._gripper_dof = extra_dof
        self._arm: XArmAPI | None = None
        self._control_mode: ControlMode = ControlMode.POSITION
        self._gripper_enabled: bool = False

    def connect(self) -> bool:
        """Connect to XArm via TCP/IP."""
        try:
            self._arm = XArmAPI(self._ip)
            self._arm.connect()

            if not self._arm.connected:
                logger.error("XArm at %s not reachable (connected=False)", self._ip)
                return False

            # Initialize to servo mode for high-frequency control
            self._arm.set_mode(_XARM_MODE_SERVO_CARTESIAN)  # Mode 1 = servo mode
            self._arm.set_state(0)
            self._control_mode = ControlMode.SERVO_POSITION

            return True
        except Exception as e:
            logger.error("Failed to connect to XArm at %s: %s", self._ip, e)
            return False

    def disconnect(self) -> None:
        """Disconnect from XArm."""
        if self._arm:
            self._arm.disconnect()
            self._arm = None

    def is_connected(self) -> bool:
        """Check if connected to XArm."""
        return self._arm is not None and self._arm.connected

    def get_info(self) -> ManipulatorInfo:
        """Get XArm information."""
        return ManipulatorInfo(
            vendor="UFACTORY",
            model=f"xArm{self._arm_dof}",
            dof=self._dof,
        )

    def get_dof(self) -> int:
        """Total joints owned by this adapter."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Arm limits in radians, then the gripper's own 0-850 scale."""
        # XArm typical joint limits (varies by joint, using conservative values)
        limit = 2 * math.pi
        return JointLimits(
            position_lower=[-limit] * self._arm_dof + [XARM_GRIPPER_MIN] * self._gripper_dof,
            position_upper=[limit] * self._arm_dof + [XARM_GRIPPER_MAX] * self._gripper_dof,
            velocity_max=[math.pi] * self._arm_dof + [0.0] * self._gripper_dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Set XArm control mode.

        Note: XArm is flexible and often accepts commands without explicit
        mode switching, but some operations require the correct mode.
        """
        if not self._arm:
            return False

        mode_map = {
            ControlMode.POSITION: _XARM_MODE_POSITION,
            ControlMode.SERVO_POSITION: _XARM_MODE_SERVO_CARTESIAN,  # Mode 1 for high-freq
            ControlMode.VELOCITY: _XARM_MODE_JOINT_VELOCITY,
            ControlMode.TORQUE: _XARM_MODE_JOINT_TORQUE,
            ControlMode.CARTESIAN: _XARM_MODE_SERVO_CARTESIAN,
            ControlMode.CARTESIAN_VELOCITY: _XARM_MODE_CARTESIAN_VELOCITY,
        }

        xarm_mode = mode_map.get(mode)
        if xarm_mode is None:
            return False

        code = self._arm.set_mode(xarm_mode)
        if code == 0:
            self._arm.set_state(0)
            self._control_mode = mode
            return True
        return False

    def get_control_mode(self) -> ControlMode:
        """Get current control mode."""
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Read joint positions (degrees -> radians)."""
        if not self._arm:
            raise RuntimeError("Not connected")

        _, angles = self._arm.get_servo_angle()
        if not angles:
            raise RuntimeError("Failed to read joint positions")
        positions = [math.radians(a) for a in angles[: self._arm_dof]]
        if self._gripper_dof:
            positions.append(self._read_gripper())
        return positions

    def read_joint_velocities(self) -> list[float]:
        """Read joint velocities.

        Note: XArm doesn't provide real-time velocity feedback directly.
        Returns zeros. For velocity estimation, use finite differences
        on positions in the driver.
        """
        return [0.0] * self._dof

    def read_joint_efforts(self) -> list[float]:
        """Read joint torques in Nm; the gripper reports 0.0 (no feedback)."""
        gripper = [0.0] * self._gripper_dof
        if not self._arm:
            return [0.0] * self._arm_dof + gripper

        code, torques = self._arm.get_joints_torque()
        if code == 0 and torques:
            return list(torques[: self._arm_dof]) + gripper
        return [0.0] * self._arm_dof + gripper

    def read_state(self) -> dict[str, int]:
        """Read robot state."""
        if not self._arm:
            return {"state": 0, "mode": 0}

        return {
            "state": self._arm.state,
            "mode": self._arm.mode,
        }

    def read_error(self) -> tuple[int, str]:
        """Read error code and message."""
        if not self._arm:
            return 0, ""

        code = self._arm.error_code
        if code == 0:
            return 0, ""
        return code, f"XArm error {code}"

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Write joint positions for servo mode (radians -> degrees).

        Uses set_servo_angle_j() for high-frequency servo control.
        Requires mode 1 (servo mode) to be active.

        The trailing gripper entry passes to the SDK unconverted (0-850).

        Args:
            positions: Target positions, arm in radians then gripper native
            velocity: Speed as fraction of max (0-1) - not used in servo mode
        """
        if not self._arm:
            return False

        if len(positions) != self._dof:
            return False
        arm, grip = positions[: self._arm_dof], positions[self._arm_dof :]

        # Convert radians to degrees
        angles = [math.degrees(p) for p in arm]

        # Use set_servo_angle_j for high-frequency servo control (100Hz+)
        # This only executes the last instruction, suitable for real-time control
        code: int = self._arm.set_servo_angle_j(angles, speed=100, mvacc=500)
        ok = code == 0
        if grip:
            ok = self._write_gripper(grip[0]) and ok
        return ok

    def activate(self) -> bool:
        """Enable motion and move the arm to its initial joint pose."""
        if not self._arm:
            return False

        self._prepare_for_position_motion()
        if not self._move_to_initial_pose():
            return False
        return self.set_control_mode(ControlMode.SERVO_POSITION)

    def deactivate(self) -> bool:
        """Move the arm to its initial joint pose and enter stopped state."""
        if not self._arm:
            return False

        self._prepare_for_position_motion()
        homed = self._move_to_initial_pose()
        self._arm.motion_enable(enable=False)
        code: int = self._arm.set_state(4)
        return homed and code == 0

    def _move_to_initial_pose(self) -> bool:
        if not self._arm:
            return False

        joints = self._initial_joints_degrees()
        if joints is None:
            return True

        code: int = self._arm.set_servo_angle(
            angle=joints,
            speed=_XARM_LIFECYCLE_SPEED_DEG,
            mvacc=_XARM_LIFECYCLE_ACCEL_DEG,
            wait=True,
        )
        if code != 0:
            logger.warning(
                "xArm move-to-initial-pose failed: set_servo_angle code=%s "
                "(state=%s mode=%s err=%s warn=%s)",
                code,
                self._arm.state,
                self._arm.mode,
                self._arm.error_code,
                self._arm.warn_code,
            )
        return code == 0

    def _initial_joints_degrees(self) -> list[float] | None:
        if self._arm_dof == 6:
            return _XARM6_INITIAL_JOINTS_DEG
        if self._arm_dof == 7:
            return _XARM7_INITIAL_JOINTS_DEG
        return None

    def _prepare_for_position_motion(self) -> None:
        if not self._arm:
            return

        if self._arm.warn_code != 0:
            self._arm.clean_warn()
        if self._arm.error_code != 0:
            self._arm.clean_error()
        self._arm.motion_enable(enable=True)
        self._arm.set_mode(_XARM_MODE_POSITION)
        self._arm.set_state(0)
        self._control_mode = ControlMode.POSITION

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Write joint velocities (rad/s -> deg/s).

        Note: Requires velocity mode to be active.
        """
        if not self._arm:
            return False
        if len(velocities) != self._dof:
            return False

        # Arm entries only; the xArm gripper has no velocity interface.
        if any(value != 0.0 for value in velocities[self._arm_dof :]):
            return False
        speeds = [math.degrees(v) for v in velocities[: self._arm_dof]]
        code: int = self._arm.vc_set_joint_velocity(speeds)
        return code == 0

    def write_stop(self) -> bool:
        """Emergency stop."""
        if not self._arm:
            return False
        code: int = self._arm.emergency_stop()
        return code == 0

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable servos."""
        if not self._arm:
            return False
        code: int = self._arm.motion_enable(enable=enable)
        return code == 0

    def read_enabled(self) -> bool:
        """Check if servos are enabled."""
        if not self._arm:
            return False
        # XArm state 0 = ready/enabled
        state: int = self._arm.state
        return state == 0

    def write_clear_errors(self) -> bool:
        """Clear error state."""
        if not self._arm:
            return False
        code: int = self._arm.clean_error()
        return code == 0

    def read_cartesian_position(self) -> dict[str, float] | None:
        """Read end-effector pose (mm -> meters, degrees -> radians)."""
        if not self._arm:
            return None

        _, pose = self._arm.get_position()
        if pose and len(pose) >= 6:
            return {
                "x": pose[0] * MM_TO_M,
                "y": pose[1] * MM_TO_M,
                "z": pose[2] * MM_TO_M,
                "roll": math.radians(pose[3]),
                "pitch": math.radians(pose[4]),
                "yaw": math.radians(pose[5]),
            }
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        """Write end-effector pose (meters -> mm, radians -> degrees)."""
        if not self._arm:
            return False

        code: int = self._arm.set_position(
            x=pose.get("x", 0) * M_TO_MM,
            y=pose.get("y", 0) * M_TO_MM,
            z=pose.get("z", 0) * M_TO_MM,
            roll=math.degrees(pose.get("roll", 0)),
            pitch=math.degrees(pose.get("pitch", 0)),
            yaw=math.degrees(pose.get("yaw", 0)),
            speed=velocity * MAX_CARTESIAN_SPEED_MM,
            wait=False,
        )
        return code == 0

    def _read_gripper(self) -> float:
        """Read the gripper position in SDK units (0-850)."""
        if not self._arm:
            return 0.0

        result = self._arm.get_gripper_position()
        code: int = result[0]
        pos: float | None = result[1]
        if code == 0 and pos is not None:
            return float(pos)
        return 0.0

    def _write_gripper(self, position: float) -> bool:
        """Command the gripper in SDK units (0-850)."""
        if not self._arm:
            return False

        if not self._gripper_enabled:
            self._arm.set_gripper_enable(True)
            self._gripper_enabled = True
        code: int = self._arm.set_gripper_position(position, wait=False)
        return code == 0

    def read_force_torque(self) -> list[float] | None:
        """Read F/T sensor data if available."""
        if not self._arm:
            return None

        code, ft = self._arm.get_ft_sensor_data()
        if code == 0 and ft:
            return list(ft)
        return None
