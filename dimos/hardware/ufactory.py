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

import os
import sys
import time
import math
import numpy as np
import logging
from typing import Tuple, Optional

from xarm.wrapper import XArmAPI

from dimos.hardware.end_effector import EndEffector

import dimos.core as core
from dimos.core import Module, In, Out, rpc
from dimos.protocol.service.lcmservice import autoconf
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist, PoseStamped, Transform, Quaternion
import dimos.protocol.service.lcmservice as lcmservice
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs import Header
from dimos.protocol.tf import TF
from dimos.utils.transform_utils import (
    invert_transform,
    apply_transform,
    quaternion_to_euler,
    euler_to_quaternion,
    create_transform_from_6dof,
    matrix_to_pose,
    pose_to_matrix,
)

# Import for consistent publishing
from reactivex import interval

logger = logging.getLogger(__name__)


class UFactoryEndEffector(EndEffector):
    def __init__(self, model=None, **kwargs):
        super().__init__(**kwargs)
        self.model = model

    def get_model(self):
        return self.model


class xArm:
    def __init__(self, ip=None, xarm_type="xarm7"):
        if ip is None:
            self.ip = input("Enter the IP address of the xArm: ")
        else:
            self.ip = ip

        if xarm_type is None:
            self.xarm_type = input("Enter the type of xArm: ")
        else:
            self.xarm_type = xarm_type

        ee_to_rep103_frame_rotation = [3.14, 0.0, 0.0]
        pos = Vector3(0, 0, 0)
        rot = Vector3(
            ee_to_rep103_frame_rotation[0],
            ee_to_rep103_frame_rotation[1],
            ee_to_rep103_frame_rotation[2],
        )
        self.T_ee_to_rep103_rotation = create_transform_from_6dof(pos, rot)

        # To be used in future for changing between different xArm types
        # from configparser import ConfigParser
        # parser = ConfigParser()
        # parser.read('../robot.conf')
        # self.arm_length = parser.get(xarm_type, 'arm_length')
        # print(parser)

        self.arm = XArmAPI(self.ip)
        print("initializing arm")

        # Initialize error/warn state tracking
        self.current_code = None
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        # self.gotoZero()

    def get_arm_length(self):
        return self.arm_length

    def enable(self):
        self.arm.motion_enable(enable=True)
        self.arm.set_state(state=0)
        self.arm.arm.set_self_collision_detection(True)
        self.arm.arm.set_collision_sensitivity(1)
        self.enable_gripper()

    def disable(self):
        self.arm.motion_enable(enable=False)
        self.arm.set_state(state=0)

    def disconnect(self):
        self.arm.disconnect()

    def gotoZero(self):
        self.enable_position_mode()
        # self.arm.move_gohome(wait=True)
        # self.arm.set_position(
        #     x=344, y=3.2, z=530, roll=180, pitch=-88.5, yaw=0, speed=50, is_radian=False, wait=True
        # )
        self.cmd_joint_angles([0, -75, 0, 13.5, 0, 0, 0], speed=50, is_radian=False)

    def gotoObserve(self):
        """Move to observation position similar to PiperArm"""
        # xArm API expects mm and degrees
        x, y, z = 400, 0, 300  # mm
        roll, pitch, yaw = 180, -20, 0  # degrees
        logger.debug(
            f"Going to observe position: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}"
        )
        # code = self.arm.set_position(
        #     x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=100, is_radian=False, wait=True
        # )
        self.cmd_joint_angles([0.2, -32.6, -0.6, 42.3, 0.5, 55, 0.1], speed=50, is_radian=False)
        # if code != 0:
        #     logger.error(f"Failed to go to observe position, code: {code}")

    def softStop(self):
        """Soft stop the arm by going to zero position and disabling motion"""
        self.gotoZero()
        time.sleep(1)
        self.arm.emergency_stop()
        self.arm.set_state(state=4)  # Set to STOP state
        time.sleep(3)

    def cmd_joint_angles(self, angles, speed, is_radian=False):
        target = np.array(angles)
        # self.enable_joint_mode()
        # Move to target position
        self.arm.set_servo_angle(angle=target.tolist(), speed=speed, wait=True, is_radian=is_radian)

    def enable_joint_mode(self):
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(0.1)

    def enable_position_mode(self):
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)

    def cmd_ee_pose_values(self, x, y, z, r, p, y_, line_mode=False):
        """Command end-effector to target pose in space (position + Euler angles)"""
        self.enable_position_mode()
        # xArm uses mm and degrees - convert from meters to mm
        pose_mm = [x * 1000, y * 1000, z * 1000, r, p, y_]
        logger.debug(f"Commanding EE pose: {pose_mm}")
        # Use set_position with proper parameters (positions in mm, angles in degrees)
        code = self.arm.set_position(
            x=pose_mm[0],
            y=pose_mm[1],
            z=pose_mm[2],
            roll=pose_mm[3],
            pitch=pose_mm[4],
            yaw=pose_mm[5],
            speed=100,
            is_radian=False,
            wait=False,
            radius=0 if line_mode else 2000,
        )

        self.current_code = code

        if code != 0:
            logger.error(f"Failed to set position, code: {code}")
            return code
        return code
    
    def cmd_ee_pose(self, pose: Pose, line_mode=False):
        """Command end-effector to target pose using Pose message"""
        # Convert quaternion to euler angles
        inv = invert_transform(self.T_ee_to_rep103_rotation)
        post_rot_pose = Pose(Vector3(0, 0, 0), pose.orientation)
        post_rot_pose = apply_transform(post_rot_pose, inv)
        pose = Pose(pose.position, post_rot_pose.orientation)
        euler = quaternion_to_euler(pose.orientation, degrees=True)

        # Command the pose
        code = self.cmd_ee_pose_values(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            euler.x,
            180 - euler.y,
            -euler.z,
            line_mode,
        )
        return code

    def get_ee_pose(self):
        """Return the current end-effector pose as Pose message with position in meters and quaternion orientation"""
        # Get current position from xArm (returns [code, [x, y, z, roll, pitch, yaw]])
        position_data = self.arm.get_position(is_radian=False)
        if position_data[0] != 0:
            logger.error(f"Failed to get arm position, code: {position_data[0]}")
            return None
        pose_values = position_data[1]
        # Convert from mm to meters and degrees to quaternion
        x = pose_values[0] / 1000.0  # Convert mm to m
        y = pose_values[1] / 1000.0  # Convert mm to m
        z = pose_values[2] / 1000.0  # Convert mm to m
        rx = pose_values[3]  # degrees
        ry = 180 - pose_values[4]  # degrees
        rz = -pose_values[5]  # degrees

        # Create position vector (already in meters)
        position = Vector3(x, y, z)

        # Convert Euler angles to quaternion
        orientation = euler_to_quaternion(Vector3(rx, ry, rz), degrees=True)
        pre_rot_pose = Pose(Vector3(0, 0, 0), orientation)
        pre_rot_pose = apply_transform(pre_rot_pose, self.T_ee_to_rep103_rotation)

        return Pose(position, pre_rot_pose.orientation)

    def cmd_gripper_ctrl(self, position, effort=0.25):
        """Command end-effector gripper"""
        # xArm gripper position is 0-850 (0 = closed, 850 = open)
        # Convert from meters to gripper units
        self.enable_gripper()
        gripper_pos = int(position * 8500)  # 0.1m = 850 units
        gripper_pos = max(0, min(850, gripper_pos))  # Clamp to valid range

        # xArm speed parameter (1-5000, higher = faster)
        speed = int(effort * 5000)  # Convert effort to speed
        speed = max(1, min(5000, speed))

        code = self.arm.set_gripper_position(gripper_pos, wait=True, speed=speed)
        if code != 0:
            logger.error(f"Failed to command gripper, code: {code}")
        logger.debug(f"Commanding gripper position: {gripper_pos} units, speed: {speed}")

    def enable_gripper(self):
        """Enable the gripper"""
        logger.info("Enabling gripper...")
        self.arm.motion_enable(True)
        self.arm.clean_error()
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)
        # Set gripper mode to position mode first
        code = self.arm.set_gripper_mode(0)  # 0 = location/position mode
        if code != 0:
            logger.error(f"Failed to set gripper mode, code: {code}")

        # Enable the gripper
        code = self.arm.set_gripper_enable(True)
        if code != 0:
            logger.error(f"Failed to enable gripper, code: {code}")
        else:
            logger.info("Gripper enabled")
        code = self.arm.set_gripper_speed(5000)

    def release_gripper(self):
        """Release gripper by opening to 100mm (10cm)"""
        logger.info("Releasing gripper (opening to 100mm)")
        self.cmd_gripper_ctrl(0.1)  # 0.1m = 100mm = 10cm

    def close_gripper(self, commanded_effort: float = 0.5) -> None:
        """
        Close the gripper.

        Args:
            commanded_effort: Effort to use when closing gripper (default 0.5)
        """
        # Command gripper to close (0.0 position)
        self.cmd_gripper_ctrl(0.0, effort=commanded_effort)
        logger.info("Closing gripper")

    def get_gripper_feedback(self) -> Tuple[float, float]:
        """
        Get current gripper feedback.

        Returns:
            Tuple of (position_meters, current_amperes) where:
                - position_meters: Current gripper position in meters
                - current_amperes: Current gripper current in amperes
        """
        gripper_data = self.arm.get_gripper_position()
        if gripper_data[0] != 0:
            logger.error(f"Failed to get gripper position, code: {gripper_data[0]}")
            return 0.0, 0.0

        gripper_pos = gripper_data[1]
        # Convert gripper position from units (0-850) to meters
        position_meters = gripper_pos / 8500.0  # 850 units = 0.1m

        # Try to get gripper current (if available)
        try:
            # xArm may not have direct current reading, use position as proxy
            current_amperes = 0.0  # Fallback since xArm doesn't provide current directly
        except:
            current_amperes = 0.0

        return position_meters, current_amperes

    def gripper_object_detected(self, commanded_effort: float = 0.25) -> bool:
        """
        Check if an object is detected in the gripper based on position feedback.

        Args:
            commanded_effort: The effort that was used when closing gripper (default 0.25)

        Returns:
            True if object is detected in gripper, False otherwise
        """
        # Get gripper feedback
        position_meters, current_amperes = self.get_gripper_feedback()

        # Check if object is grasped (gripper stopped before fully closed)
        # If gripper position > 0.005m (5mm), assume object is present
        object_present = position_meters > 0.005

        if object_present:
            logger.info(
                f"Object detected in gripper (position: {position_meters:.3f} m, current: {current_amperes:.3f} A)"
            )
        else:
            logger.info(
                f"No object detected (position: {position_meters:.3f} m, current: {current_amperes:.3f} A)"
            )

        return object_present

    def clear_errors(self):
        """Clear arm errors using basic SDK methods."""
        logger.info("Clearing arm errors...")
        self.arm.clean_error()
        self.enable()
        # Reset our stored error codes after clearing
        logger.info("Arm errors cleared")

    def resetArm(self):
        """Reset the arm to initial state"""
        logger.info("Resetting arm...")
        self.arm.reset(wait=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        logger.info("Arm reset complete")



class XArmModule(Module):
    """
    Dimos module for xArm that provides RPC control interface and publishes EE pose.

    Publishes:
        - ee_pose: End-effector pose as PoseStamped

    RPC methods:
        - All xArm control methods exposed via RPC
    """

    # LCM outputs
    ee_pose: Out[PoseStamped] = None

    def __init__(
        self,
        arm_ip: str = None,
        arm_type: str = "xarm7",
        publish_rate: float = 30.0,
        base_frame_id: str = "base_link",
        ee_frame_id: str = "ee_link",
        camera_frame_id: str = "camera_link",
        ee_to_camera_6dof: Optional[list] = None,
        **kwargs,
    ):
        """
        Initialize xArm Module.

        Args:
            arm_ip: IP address of the xArm robot
            arm_type: Type of xArm (e.g., "xarm7")
            publish_rate: Rate to publish EE pose and transforms (Hz)
            base_frame_id: TF frame ID for robot base
            ee_frame_id: TF frame ID for end-effector
            camera_frame_id: TF frame ID for camera
            ee_to_camera_6dof: EE to camera transform [x, y, z, rx, ry, rz] in meters and radians
        """
        super().__init__(**kwargs)

        self.arm_ip = arm_ip
        self.arm_type = arm_type
        self.publish_rate = publish_rate
        self.base_frame_id = base_frame_id
        self.ee_frame_id = ee_frame_id
        self.camera_frame_id = camera_frame_id
        self.publish_period = 1.0 / publish_rate

        # EE to camera transform
        if ee_to_camera_6dof is None:
            ee_to_camera_6dof = [0.115, 0.00, 0.00, 0, -1.57, 0.0]
        pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
        rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
        self.T_ee_to_camera = create_transform_from_6dof(pos, rot)

        # Extract translation and rotation for TF
        self.ee_to_camera_translation = Vector3(
            ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2]
        )
        # Convert euler to quaternion for TF
        self.ee_to_camera_rotation = euler_to_quaternion(rot, degrees=False)

        # Internal xArm instance
        self.arm = None

        # Publishing control
        self._running = False
        self._subscription = None
        self._sequence = 0
        self.last_commanded_pose = None

        # Initialize TF publisher
        self.tf = TF()

        logger.info(f"XArmModule initialized, will publish at {publish_rate} Hz")

    def _reset_command_correction(self):
        """Reset cached command-to-arm correction after direct-arm motions."""
        self._last_command_correction_inverse = None

    @rpc
    def start(self):
        """Start the xArm module and begin publishing EE pose."""
        if self._running:
            logger.warning("xArm module already running")
            return

        # Initialize the actual xArm
        logger.info("Initializing xArm hardware...")
        self.arm = xArm(ip=self.arm_ip, xarm_type=self.arm_type)
        self.arm.enable()
        self.arm.enable_gripper()

        # Start publishing EE pose
        self._running = True

        # Use reactivex interval for consistent publishing rate
        self._subscription = interval(self.publish_period).subscribe(
            lambda _: self._publish_ee_pose_and_transforms()
        )

        logger.info("xArm module started successfully")

    @rpc
    def stop(self):
        """Stop the xArm module."""
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

        logger.info("xArm module stopped")

    def _publish_ee_pose_and_transforms(self):
        """Publish current end-effector pose and TF transforms."""
        if not self._running or not self.arm:
            return

        try:
            # Get current EE pose
            pose = self.get_ee_pose()

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
                if self.last_commanded_pose:
                    target_tf = Transform(
                        translation=self.last_commanded_pose.position,
                        rotation=self.last_commanded_pose.orientation,
                        frame_id=self.base_frame_id,
                        child_frame_id="target_pose",
                        ts=header.ts,
                    )
                    self.tf.publish(target_tf)

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

    # Expose all xArm methods via RPC

    @rpc
    def enable(self):
        """Enable the xArm."""
        if self.arm:
            self.arm.enable()
            self.arm.enable_gripper()

    @rpc
    def disable(self):
        """Disable the xArm."""
        if self.arm:
            self.arm.disable()

    @rpc
    def goto_zero(self):
        """Move arm to zero position."""
        if self.arm:
            self.arm.gotoZero()
            # self._reset_command_correction()

    @rpc
    def goto_observe(self):
        """Move arm to observe position."""
        if self.arm:
            self.arm.gotoObserve()
            # self._reset_command_correction()
        else:
            logger.warning("Cannot go to observe position - arm not initialized yet")

    @rpc
    def soft_stop(self):
        """Perform soft stop."""
        if self.arm:
            self.arm.softStop()
            # self._reset_command_correction()

    @rpc
    def cmd_ee_pose(self, pose: Pose, line_mode: bool = False):
        """
        Command end-effector to target pose.

        Args:
            pose: Target pose for end-effector
            line_mode: Whether to use line mode for movement
        """
        if self.arm:
            self.last_commanded_pose = pose
            code = self.arm.cmd_ee_pose(pose, line_mode)
            return code

    @rpc
    def get_ee_pose(self) -> Pose:
        """
        Get current end-effector pose.

        Returns:
            Current EE pose transformed from arm frame to command frame
        """
        if self.arm:
            return self.arm.get_ee_pose()

    @rpc
    def cmd_gripper_ctrl(self, position: float, effort: float = 0.25):
        """
        Command gripper position and effort.

        Args:
            position: Gripper opening in meters
            effort: Gripper effort (normalized 0-1)
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
            Tuple of (position_meters, current_amperes)
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
            # self._reset_command_correction()

    @rpc
    def check_arm_errors(self) -> bool:
        """Check if arm has any errors.

        Returns:
            True if errors detected, False if arm is normal
        """
        if self.arm:
            return self.arm.current_code
        return True  # Assume error if arm not initialized

    @rpc
    def clear_errors(self):
        """Clear arm errors."""
        if self.arm:
            self.arm.clear_errors()

    @rpc
    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()


class xArmBridge(Module):
    joint_state: In[JointState] = None
    pose_state: Out[JointState] = None
    target_joint_state = None
    arm = None

    def __init__(self, arm_ip: str = None, arm_type: str = "xarm7", *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.arm_ip = arm_ip
        self.arm_type = arm_type
        self.arm = None
        self.target_joint_state = [0, 0, 0, 0, 0, 0, 0]

    @rpc
    def start(self):
        # subscribe to incoming LCM JointState messages
        self.arm = xArm(ip=self.arm_ip, xarm_type=self.arm_type)
        self.arm.enable()
        # print(f"Initialized xArmBridge with arm type: {self.arm.xarm_type}")
        self.joint_state.subscribe(self._on_joint_state)
        # print(f"Subscribed to {self.joint_state}")

    # @rpc
    def command_arm(self):
        print("[xArmBridge] Commanding arm with target joint state:", self.target_joint_state)
        self.arm.cmd_joint_angles(self.target_joint_state, speed=3.14, is_radian=True)

    def _on_joint_state(self, msg: JointState):
        # print(f"[xArmBridge] Received joint state: {msg}")
        if not msg:
            # print("[xArmBridge] No joint names found in message.")
            return

        # Extract joint1-joint7 values from indices 3-9
        if len(msg.position) >= 10:
            joint1 = msg.position[3]
            joint2 = msg.position[4]
            joint3 = msg.position[5]
            joint4 = msg.position[6]
            joint5 = msg.position[7]
            joint6 = msg.position[8]
            joint7 = msg.position[9]

            # print(f"[xArmBridge] Joint values - joint1: {joint1}, joint2: {joint2}, joint3: {joint3}, joint4: {joint4}, joint5: {joint5}, joint6: {joint6}, joint7: {joint7}")
            self.target_joint_state = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
        else:
            print(
                f"[xArmBridge] Insufficient joint data: expected at least 10 joints, got {len(msg.position)}"
            )

    def _reader(self):
        while True:
            print("Reading from arm")
            angles = self.arm.arm.get_servo_angle(is_radian=False)[1]
            print(f"Current angles: {angles}")
            if not angles:
                continue


def TestXarmBridge(arm_ip: str = None, arm_type: str = "xArm7"):
    lcmservice.autoconf()
    dimos = core.start(2)

    armBridge = dimos.deploy(xArmBridge, arm_ip=arm_ip, arm_type=arm_type)

    armBridge.pose_state.transport = core.LCMTransport("/armJointState", JointState)
    armBridge.joint_state.transport = core.LCMTransport("/joint_states", JointState)

    armBridge.start()
    print("xArmBridge started and listening for joint states.")

    while True:
        # print(armBridge.target_joint_state)
        armBridge.command_arm()  # Command the arm  at 100hz with the target joint state
        time.sleep(0.01)


def test_simple_error_clearing():
    """Simple test for basic error clearing functionality."""
    print("=== Simple xArm Error Clearing Test ===")

    arm = xArm(ip="10.0.0.197", xarm_type="xarm7")

    try:
        print("Enabling arm...")
        arm.enable()

        print("Going to observe position...")
        arm.gotoObserve()

        print("Testing error clearing...")
        arm.clear_errors()

        print("Going to zero position...")
        arm.gotoZero()

        print("Test completed successfully!")

    except Exception as e:
        print(f"Test error: {e}")
        print("Attempting to clear errors...")
        arm.clear_errors()


def test_xarm():
    """Test function for xArm - moved to avoid circular imports."""
    arm = xArm(ip="10.0.0.197", xarm_type="xarm7")
    # arm.enable()
    # arm.gotoObserve()
    # print(arm.get_ee_pose())
    # arm.enable_gripper()
    # time.sleep(2)
    arm.cmd_gripper_ctrl(0.1)
    # time.sleep(2)
    # arm.gotoZero()
    # print(arm.get_ee_pose())
    arm.close_gripper()
    arm.arm.set_gripper_enable(False)
    
    
    # arm.disconnect()
    print("disconnected")


if __name__ == "__main__":
    # Choose which test to run:

    # Original basic test
    test_xarm()

    # Simple error clearing test
    # test_simple_error_clearing()

    # Bridge test
    # TestXarmBridge(arm_ip="192.168.1.197", arm_type="xarm7")
