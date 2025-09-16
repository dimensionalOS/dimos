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

# dimos/hardware/piper_arm.py

from typing import Tuple, Optional
from piper_sdk import *  # from the official Piper SDK
import numpy as np
import time
import subprocess
import kinpy as kp
import sys
import termios
import tty
import select
from scipy.spatial.transform import Rotation as R
from dimos.utils.transform_utils import euler_to_quaternion, quaternion_to_euler
from dimos.utils.logging_config import setup_logger

import threading
from reactivex import interval

import pytest

import dimos.core as core
import dimos.protocol.service.lcmservice as lcmservice
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3 as MsgVector3
from dimos.msgs.std_msgs import Header
from dimos.protocol.tf import TF
from dimos.utils.transform_utils import create_transform_from_6dof

logger = setup_logger("dimos.hardware.piper_arm")


class PiperArm:
    def __init__(self, arm_name: str = "arm"):
        self.arm = C_PiperInterface_V2()
        self.arm.ConnectPort()
        self.resetArm()
        time.sleep(0.5)
        self.resetArm()
        time.sleep(0.5)
        self.enable()
        self.enable_gripper()  # Enable gripper after arm is enabled
        self.gotoZero()
        time.sleep(1)

    def enable(self):
        while not self.arm.EnablePiper():
            pass
            time.sleep(0.01)
        logger.info("Arm enabled")
        # self.arm.ModeCtrl(
        #     ctrl_mode=0x01,         # CAN command mode
        #     move_mode=0x01,         # “Move-J”, but ignored in MIT
        #     move_spd_rate_ctrl=100, # doesn’t matter in MIT
        #     is_mit_mode=0xAD        # <-- the magic flag
        # )
        self.arm.MotionCtrl_2(0x01, 0x01, 80, 0xAD)

    def gotoZero(self):
        factor = 1000
        position = [57.0, 0.0, 215.0, 0, 90.0, 0, 0]
        X = round(position[0] * factor)
        Y = round(position[1] * factor)
        Z = round(position[2] * factor)
        RX = round(position[3] * factor)
        RY = round(position[4] * factor)
        RZ = round(position[5] * factor)
        joint_6 = round(position[6] * factor)
        logger.debug(f"Going to zero position: X={X}, Y={Y}, Z={Z}, RX={RX}, RY={RY}, RZ={RZ}")
        self.arm.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.arm.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
        self.arm.GripperCtrl(0, 1000, 0x01, 0)

    def gotoObserve(self, pitch=120.0):
        factor = 1000
        position = [57.0, 0.0, 280.0, 0, pitch, 0, 0]
        X = round(position[0] * factor)
        Y = round(position[1] * factor)
        Z = round(position[2] * factor)
        RX = round(position[3] * factor)
        RY = round(position[4] * factor)
        RZ = round(position[5] * factor)
        joint_6 = round(position[6] * factor)
        logger.debug(f"Going to zero position: X={X}, Y={Y}, Z={Z}, RX={RX}, RY={RY}, RZ={RZ}")
        self.arm.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.arm.EndPoseCtrl(X, Y, Z, RX, RY, RZ)

    def softStop(self):
        self.gotoZero()
        time.sleep(1)
        self.arm.MotionCtrl_2(
            0x01,
            0x00,
            100,
        )
        self.arm.MotionCtrl_1(0x01, 0, 0)
        time.sleep(3)

    def cmd_ee_pose_values(self, x, y, z, r, p, y_, line_mode=False):
        """Command end-effector to target pose in space (position + Euler angles)"""
        factor = 1000
        pose = [
            x * factor * factor,
            y * factor * factor,
            z * factor * factor,
            r * factor,
            p * factor,
            y_ * factor,
        ]
        self.arm.MotionCtrl_2(0x01, 0x02 if line_mode else 0x00, 100, 0x00)
        self.arm.EndPoseCtrl(
            int(pose[0]), int(pose[1]), int(pose[2]), int(pose[3]), int(pose[4]), int(pose[5])
        )

    def cmd_ee_pose(self, pose: Pose, line_mode=False):
        """Command end-effector to target pose using Pose message"""
        # Convert quaternion to euler angles
        euler = quaternion_to_euler(pose.orientation, degrees=True)

        # Command the pose
        self.cmd_ee_pose_values(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            euler.x,
            euler.y,
            euler.z,
            line_mode,
        )

    def get_ee_pose(self):
        """Return the current end-effector pose as Pose message with position in meters and quaternion orientation"""
        pose = self.arm.GetArmEndPoseMsgs()
        factor = 1000.0
        # Extract individual pose values and convert to base units
        # Position values are divided by 1000 to convert from SDK units to meters
        # Rotation values are divided by 1000 to convert from SDK units to radians
        x = pose.end_pose.X_axis / factor / factor  # Convert mm to m
        y = pose.end_pose.Y_axis / factor / factor  # Convert mm to m
        z = pose.end_pose.Z_axis / factor / factor  # Convert mm to m
        rx = pose.end_pose.RX_axis / factor
        ry = pose.end_pose.RY_axis / factor
        rz = pose.end_pose.RZ_axis / factor

        # Create position vector (already in meters)
        position = Vector3(x, y, z)

        orientation = euler_to_quaternion(Vector3(rx, ry, rz), degrees=True)

        return Pose(position, orientation)

    def cmd_gripper_ctrl(self, position, effort=0.25):
        """Command end-effector gripper"""
        factor = 1000
        position = position * factor * factor  # meters
        effort = effort * factor  # N/m

        self.arm.GripperCtrl(abs(round(position)), abs(round(effort)), 0x01, 0)
        logger.debug(f"Commanding gripper position: {position}mm")

    def enable_gripper(self):
        """Enable the gripper using the initialization sequence"""
        logger.info("Enabling gripper...")
        while not self.arm.EnablePiper():
            time.sleep(0.01)
        self.arm.GripperCtrl(0, 1000, 0x02, 0)
        self.arm.GripperCtrl(0, 1000, 0x01, 0)
        logger.info("Gripper enabled")

    def release_gripper(self):
        """Release gripper by opening to 100mm (10cm)"""
        logger.info("Releasing gripper (opening to 100mm)")
        self.cmd_gripper_ctrl(0.1)  # 0.1m = 100mm = 10cm

    def get_gripper_feedback(self) -> Tuple[float, float]:
        """
        Get current gripper feedback.

        Returns:
            Tuple of (angle_degrees, effort) where:
                - angle_degrees: Current gripper angle in degrees
                - effort: Current gripper effort (0.0 to 1.0 range)
        """
        gripper_msg = self.arm.GetArmGripperMsgs()
        angle_degrees = (
            gripper_msg.gripper_state.grippers_angle / 1000.0
        )  # Convert from SDK units to degrees
        effort = gripper_msg.gripper_state.grippers_effort / 1000.0  # Convert from SDK units to N/m
        return angle_degrees, effort

    def close_gripper(self, commanded_effort: float = 0.5) -> None:
        """
        Close the gripper.

        Args:
            commanded_effort: Effort to use when closing gripper (default 0.25 N/m)
        """
        # Command gripper to close (0.0 position)
        self.cmd_gripper_ctrl(0.0, effort=commanded_effort)
        logger.info("Closing gripper")

    def gripper_object_detected(self, commanded_effort: float = 0.25) -> bool:
        """
        Check if an object is detected in the gripper based on effort feedback.

        Args:
            commanded_effort: The effort that was used when closing gripper (default 0.25 N/m)

        Returns:
            True if object is detected in gripper, False otherwise
        """
        # Get gripper feedback
        angle_degrees, actual_effort = self.get_gripper_feedback()

        # Check if object is grasped (effort > 80% of commanded effort)
        effort_threshold = 0.8 * commanded_effort
        object_present = abs(actual_effort) > effort_threshold

        if object_present:
            logger.info(f"Object detected in gripper (effort: {actual_effort:.3f} N/m)")
        else:
            logger.info(f"No object detected (effort: {actual_effort:.3f} N/m)")

        return object_present

    def resetArm(self):
        self.arm.MotionCtrl_1(0x02, 0, 0)
        self.arm.MotionCtrl_2(0, 0, 0, 0x00)
        logger.info("Resetting arm")

    def disable(self):
        self.softStop()

        while self.arm.DisablePiper():
            pass
            time.sleep(0.01)
        self.arm.DisconnectPort()


class PiperArmModule(Module):
    """
    Dimos module for Piper Arm that provides RPC control interface and publishes EE pose.

    Publishes:
        - ee_pose: End-effector pose as PoseStamped

    RPC methods:
        - All PiperArm control methods exposed via RPC
    """

    # LCM outputs
    ee_pose: Out[PoseStamped] = None

    def __init__(
        self,
        publish_rate: float = 30.0,
        base_frame_id: str = "base_link",
        ee_frame_id: str = "ee_link",
        camera_frame_id: str = "camera_link",
        ee_to_camera_6dof: Optional[list] = None,
        **kwargs,
    ):
        """
        Initialize Piper Arm Module.

        Args:
            publish_rate: Rate to publish EE pose and transforms (Hz)
            base_frame_id: TF frame ID for robot base
            ee_frame_id: TF frame ID for end-effector
            camera_frame_id: TF frame ID for camera
            ee_to_camera_6dof: EE to camera transform [x, y, z, rx, ry, rz] in meters and radians
        """
        super().__init__(**kwargs)

        self.publish_rate = publish_rate
        self.base_frame_id = base_frame_id
        self.ee_frame_id = ee_frame_id
        self.camera_frame_id = camera_frame_id
        self.publish_period = 1.0 / publish_rate

        # EE to camera transform
        if ee_to_camera_6dof is None:
            ee_to_camera_6dof = [-0.065, 0.03, -0.095, 0.0, 0, 0.0]
        pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
        rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
        self.T_ee_to_camera = create_transform_from_6dof(pos, rot)

        # Extract translation and rotation for TF
        self.ee_to_camera_translation = Vector3(
            ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2]
        )
        # Convert euler to quaternion for TF
        from dimos.utils.transform_utils import euler_to_quaternion

        self.ee_to_camera_rotation = euler_to_quaternion(rot, degrees=False)

        # Internal PiperArm instance
        self.arm = None

        # Publishing control
        self._running = False
        self._subscription = None
        self._sequence = 0

        # Initialize TF publisher
        self.tf = TF()

        logger.info(f"PiperArmModule initialized, will publish at {publish_rate} Hz")

    @rpc
    def start(self):
        """Start the Piper Arm module and begin publishing EE pose."""
        if self._running:
            logger.warning("Piper Arm module already running")
            return

        # Initialize the actual Piper Arm
        logger.info("Initializing Piper Arm hardware...")
        self.arm = PiperArm()

        # Start publishing EE pose
        self._running = True

        # Use reactivex interval for consistent publishing rate
        self._subscription = interval(self.publish_period).subscribe(
            lambda _: self._publish_ee_pose_and_transforms()
        )

        logger.info("Piper Arm module started successfully")

    @rpc
    def stop(self):
        """Stop the Piper Arm module."""
        if not self._running:
            return

        self._running = False

        # Stop subscription
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        # Disable arm
        if self.arm:
            try:
                self.arm.disable()
            except Exception as e:
                logger.warning(f"Error disabling arm: {e}")

        logger.info("Piper Arm module stopped")

    def _publish_ee_pose_and_transforms(self):
        """Publish current end-effector pose and TF transforms."""
        if not self._running or not self.arm:
            return

        try:
            # Get current EE pose
            pose = self.arm.get_ee_pose()

            if pose:
                # Create header with timestamp
                header = Header(self.base_frame_id)
                self._sequence += 1

                # Publish EE pose as PoseStamped
                msg = PoseStamped(
                    ts=header.ts,
                    position=[pose.position.x, pose.position.y, pose.position.z],
                    orientation=[
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ],
                    frame_id=self.base_frame_id,
                )
                self.ee_pose.publish(msg)

                # Publish TF transforms
                # 1. base_link -> ee_link transform
                ee_transform = Transform(
                    translation=pose.position,
                    rotation=pose.orientation,
                    frame_id=self.base_frame_id,
                    child_frame_id=self.ee_frame_id,
                    ts=header.ts,
                )
                self.tf.publish(ee_transform)

                # 2. ee_link -> camera_link transform (static offset)
                camera_transform = Transform(
                    translation=self.ee_to_camera_translation,
                    rotation=self.ee_to_camera_rotation,
                    frame_id=self.ee_frame_id,
                    child_frame_id=self.camera_frame_id,
                    ts=header.ts,
                )
                self.tf.publish(camera_transform)

        except Exception as e:
            logger.error(f"Error publishing EE pose and transforms: {e}")

    # Expose all PiperArm methods via RPC

    @rpc
    def enable(self):
        """Enable the Piper Arm."""
        if self.arm:
            self.arm.enable()

    @rpc
    def disable(self):
        """Disable the Piper Arm."""
        if self.arm:
            self.arm.disable()

    @rpc
    def goto_zero(self):
        """Move arm to zero position."""
        if self.arm:
            self.arm.gotoZero()

    @rpc
    def goto_observe(self, pitch=120.0):
        """Move arm to observe position."""
        if self.arm:
            self.arm.gotoObserve(pitch=pitch)
        else:
            logger.warning("Cannot go to observe position - arm not initialized yet")

    @rpc
    def soft_stop(self):
        """Perform soft stop."""
        if self.arm:
            self.arm.softStop()

    @rpc
    def cmd_ee_pose(self, pose: Pose, line_mode: bool = False):
        """
        Command end-effector to target pose.

        Args:
            pose: Target pose for end-effector
            line_mode: Whether to use line mode for movement
        """
        if self.arm:
            self.arm.cmd_ee_pose(pose, line_mode)

    @rpc
    def get_ee_pose(self) -> Pose:
        """
        Get current end-effector pose.

        Returns:
            Current EE pose
        """
        if self.arm:
            return self.arm.get_ee_pose()
        # Return a default pose if arm not initialized
        return Pose(
            position=MsgVector3(0.057, 0.0, 0.215), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
        )

    @rpc
    def cmd_gripper_ctrl(self, position: float, effort: float = 0.25):
        """
        Command gripper position and effort.

        Args:
            position: Gripper opening in meters
            effort: Gripper effort (N/m)
        """
        if self.arm:
            self.arm.cmd_gripper_ctrl(position, effort)

    @rpc
    def enable_gripper(self):
        """Enable the gripper."""
        if self.arm:
            self.arm.enable_gripper()

    @rpc
    def release_gripper(self):
        """Release (open) the gripper."""
        if self.arm:
            self.arm.release_gripper()

    @rpc
    def close_gripper(self, commanded_effort: float = 0.5):
        """
        Close the gripper.

        Args:
            commanded_effort: Effort to use when closing
        """
        if self.arm:
            self.arm.close_gripper(commanded_effort)

    @rpc
    def get_gripper_feedback(self) -> Tuple[float, float]:
        """
        Get gripper feedback.

        Returns:
            Tuple of (angle_degrees, effort)
        """
        if self.arm:
            return self.arm.get_gripper_feedback()
        return (0.0, 0.0)

    @rpc
    def gripper_object_detected(self, commanded_effort: float = 0.25) -> bool:
        """
        Check if object is detected in gripper.

        Args:
            commanded_effort: The effort that was used when closing

        Returns:
            True if object is detected
        """
        if self.arm:
            return self.arm.gripper_object_detected(commanded_effort)
        return False

    @rpc
    def reset_arm(self):
        """Reset the arm."""
        if self.arm:
            self.arm.resetArm()

    @rpc
    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()


if __name__ == "__main__":
    # Simple test script to start the PiperArmModule
    logger.info("Starting Piper Arm Robot")

    # Create robot instance
    arm = PiperArm()
    arm.enable()
    arm.gotoObserve()
    time.sleep(2)
    print(arm.get_ee_pose())
    arm.gotoZero()
    time.sleep(2)
    print(arm.get_ee_pose())
    arm.softStop()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        arm.stop()
