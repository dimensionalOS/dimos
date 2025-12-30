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

"""
State Query Component for PiperDriver.

Provides RPC methods for querying robot state including:
- Joint state
- Robot state
- End-effector pose
- Gripper state
- Motor information
- Firmware version
"""

import threading
from typing import Any

from dimos.core import rpc
from dimos.msgs.sensor_msgs import JointState, RobotState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class StateQueryComponent:
    """
    Component providing state query RPC methods for PiperDriver.

    This component assumes the parent class has:
    - self.piper: C_PiperInterface_V2 instance
    - self.config: PiperDriverConfig instance
    - self._joint_state_lock: threading.Lock
    - self._joint_states_: Optional[JointState]
    - self._robot_state_: Optional[RobotState]
    - PIPER_TO_RAD: conversion constant (0.001 degrees → radians)
    """

    # Type hints for attributes expected from parent class
    piper: Any  # C_PiperInterface_V2 instance
    config: Any  # Config dict accessed as object
    _joint_state_lock: threading.Lock
    _joint_states_: JointState | None
    _robot_state_: RobotState | None
    PIPER_TO_RAD: float

    @rpc
    def get_joint_state(self) -> JointState | None:
        """
        Get the current joint state (RPC method).

        Returns:
            Current JointState or None
        """
        with self._joint_state_lock:
            return self._joint_states_

    @rpc
    def get_robot_state(self) -> RobotState | None:
        """
        Get the current robot state (RPC method).

        Returns:
            Current RobotState or None
        """
        with self._joint_state_lock:
            return self._robot_state_

    @rpc
    def get_arm_status(self) -> tuple[bool, dict[str, Any] | None]:
        """
        Get arm status.

        Returns:
            Tuple of (success, status_dict)
        """
        try:
            status = self.piper.GetArmStatus()

            if status is not None:
                status_dict = {
                    "time_stamp": status.time_stamp,
                    "Hz": status.Hz,
                    "motion_mode": status.arm_status.motion_mode,
                    "mode_feedback": status.arm_status.mode_feedback,
                    "teach_status": status.arm_status.teach_status,
                    "motion_status": status.arm_status.motion_status,
                    "trajectory_num": status.arm_status.trajectory_num,
                }
                return (True, status_dict)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_arm_status failed: {e}")
            return (False, None)

    @rpc
    def get_arm_joint_angles(self) -> tuple[bool, list[float] | None]:
        """
        Get arm joint angles in radians.

        Returns:
            Tuple of (success, joint_angles)
        """
        try:
            arm_joint = self.piper.GetArmJointMsgs()

            if arm_joint is not None:
                # Convert from Piper units (0.001 degrees) to radians
                angles = [
                    arm_joint.joint_state.joint_1 * self.PIPER_TO_RAD,
                    arm_joint.joint_state.joint_2 * self.PIPER_TO_RAD,
                    arm_joint.joint_state.joint_3 * self.PIPER_TO_RAD,
                    arm_joint.joint_state.joint_4 * self.PIPER_TO_RAD,
                    arm_joint.joint_state.joint_5 * self.PIPER_TO_RAD,
                    arm_joint.joint_state.joint_6 * self.PIPER_TO_RAD,
                ]
                return (True, angles)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_arm_joint_angles failed: {e}")
            return (False, None)

    @rpc
    def get_end_pose(self) -> tuple[bool, dict[str, float] | None]:
        """
        Get end-effector pose.

        Returns:
            Tuple of (success, pose_dict) with keys: x, y, z, rx, ry, rz
        """
        try:
            end_pose = self.piper.GetArmEndPoseMsgs()

            if end_pose is not None:
                # Convert from Piper units
                pose_dict = {
                    "x": end_pose.end_pose.end_pose_x * 0.001,  # 0.001 mm → mm
                    "y": end_pose.end_pose.end_pose_y * 0.001,
                    "z": end_pose.end_pose.end_pose_z * 0.001,
                    "rx": end_pose.end_pose.end_pose_rx * 0.001 * (3.14159 / 180.0),  # → rad
                    "ry": end_pose.end_pose.end_pose_ry * 0.001 * (3.14159 / 180.0),
                    "rz": end_pose.end_pose.end_pose_rz * 0.001 * (3.14159 / 180.0),
                    "time_stamp": end_pose.time_stamp,
                    "Hz": end_pose.Hz,
                }
                return (True, pose_dict)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_end_pose failed: {e}")
            return (False, None)

    @rpc
    def get_gripper_state(self) -> tuple[bool, dict[str, Any] | None]:
        """
        Get gripper state.

        Returns:
            Tuple of (success, gripper_dict)
        """
        try:
            gripper = self.piper.GetArmGripperMsgs()

            if gripper is not None:
                gripper_dict = {
                    "gripper_angle": gripper.gripper_state.grippers_angle,
                    "gripper_effort": gripper.gripper_state.grippers_effort,
                    "gripper_enable": gripper.gripper_state.grippers_enabled,
                    "time_stamp": gripper.time_stamp,
                    "Hz": gripper.Hz,
                }
                return (True, gripper_dict)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_gripper_state failed: {e}")
            return (False, None)

    @rpc
    def get_arm_enable_status(self) -> tuple[bool, list[int] | None]:
        """
        Get arm enable status for all joints.

        Returns:
            Tuple of (success, enable_status_list)
        """
        try:
            enable_status = self.piper.GetArmEnableStatus()

            if enable_status is not None:
                return (True, enable_status)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_arm_enable_status failed: {e}")
            return (False, None)

    @rpc
    def get_firmware_version(self) -> tuple[bool, str | None]:
        """
        Get Piper firmware version.

        Returns:
            Tuple of (success, version_string)
        """
        try:
            version = self.piper.GetPiperFirmwareVersion()

            if version is not None:
                return (True, version)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_firmware_version failed: {e}")
            return (False, None)

    @rpc
    def get_sdk_version(self) -> tuple[bool, str | None]:
        """
        Get Piper SDK version.

        Returns:
            Tuple of (success, version_string)
        """
        try:
            version = self.piper.GetCurrentSDKVersion()

            if version is not None:
                return (True, version)
            else:
                return (False, None)

        except Exception:
            return (False, None)

    @rpc
    def get_interface_version(self) -> tuple[bool, str | None]:
        """
        Get Piper interface version.

        Returns:
            Tuple of (success, version_string)
        """
        try:
            version = self.piper.GetCurrentInterfaceVersion()

            if version is not None:
                return (True, version)
            else:
                return (False, None)

        except Exception:
            return (False, None)

    @rpc
    def get_protocol_version(self) -> tuple[bool, str | None]:
        """
        Get Piper protocol version.

        Returns:
            Tuple of (success, version_string)
        """
        try:
            version = self.piper.GetCurrentProtocolVersion()

            if version is not None:
                return (True, version)
            else:
                return (False, None)

        except Exception:
            return (False, None)

    @rpc
    def get_can_fps(self) -> tuple[bool, float | None]:
        """
        Get CAN bus FPS (frames per second).

        Returns:
            Tuple of (success, fps_value)
        """
        try:
            fps = self.piper.GetCanFps()

            if fps is not None:
                return (True, fps)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_can_fps failed: {e}")
            return (False, None)

    @rpc
    def get_motor_max_acc_limit(self) -> tuple[bool, dict[str, Any] | None]:
        """
        Get maximum acceleration limit for all motors.

        Returns:
            Tuple of (success, acc_limit_dict)
        """
        try:
            acc_limit = self.piper.GetCurrentMotorMaxAccLimit()

            if acc_limit is not None:
                acc_dict = {
                    "motor_1": acc_limit.current_motor_max_acc_limit.motor_1_max_acc_limit,
                    "motor_2": acc_limit.current_motor_max_acc_limit.motor_2_max_acc_limit,
                    "motor_3": acc_limit.current_motor_max_acc_limit.motor_3_max_acc_limit,
                    "motor_4": acc_limit.current_motor_max_acc_limit.motor_4_max_acc_limit,
                    "motor_5": acc_limit.current_motor_max_acc_limit.motor_5_max_acc_limit,
                    "motor_6": acc_limit.current_motor_max_acc_limit.motor_6_max_acc_limit,
                    "time_stamp": acc_limit.time_stamp,
                }
                return (True, acc_dict)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_motor_max_acc_limit failed: {e}")
            return (False, None)
