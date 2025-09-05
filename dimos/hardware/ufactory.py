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
from typing import Tuple

from xarm.wrapper import XArmAPI

from dimos.hardware.end_effector import EndEffector

import dimos.core as core
from dimos.core import Module, In, Out, rpc
from dimos.protocol.service.lcmservice import autoconf
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist
import dimos.protocol.service.lcmservice as lcmservice
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import quaternion_to_euler, euler_to_quaternion

logger = logging.getLogger(__name__)


class UFactoryEndEffector(EndEffector):
    def __init__(self, model=None, **kwargs):
        super().__init__(**kwargs)
        self.model = model

    def get_model(self):
        return self.model


class UFactory7DOFArm:
    def __init__(self, ip=None, xarm_type="xarm7"):
        if ip is None:
            self.ip = input("Enter the IP address of the xArm: ")
        else:
            self.ip = ip

        if xarm_type is None:
            self.xarm_type = input("Enter the type of xArm: ")
        else:
            self.xarm_type = xarm_type

        # To be used in future for changing between different xArm types
        # from configparser import ConfigParser
        # parser = ConfigParser()
        # parser.read('../robot.conf')
        # self.arm_length = parser.get(xarm_type, 'arm_length')
        # print(parser)

        self.arm = XArmAPI(self.ip)
        print("initializing arm")
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        # self.gotoZero()

    def get_arm_length(self):
        return self.arm_length

    def enable(self):
        self.arm.motion_enable(enable=True)
        self.arm.set_state(state=0)

    def disable(self):
        self.arm.motion_enable(enable=False)
        self.arm.set_state(state=0)

    def disconnect(self):
        self.arm.disconnect()

    def gotoZero(self):
        self.enable_position_mode()
        self.arm.move_gohome(wait=True)

    def gotoObserve(self):
        """Move to observation position similar to PiperArm"""
        self.enable_position_mode()
        # Position: x=57mm, y=0mm, z=280mm, rx=0°, ry=120°, rz=0°
        # xArm API expects mm and degrees
        x, y, z = 500, 0, 200  # mm
        roll, pitch, yaw = 0, 120, 0  # degrees
        logger.debug(
            f"Going to observe position: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}"
        )
        code = self.arm.set_position(
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=100, is_radian=False, wait=True
        )
        if code != 0:
            logger.error(f"Failed to go to observe position, code: {code}")

    def softStop(self):
        """Soft stop the arm by going to zero position and disabling motion"""
        self.gotoZero()
        time.sleep(1)
        self.arm.emergency_stop()
        self.arm.set_state(state=4)  # Set to STOP state
        time.sleep(3)

    def cmd_joint_angles(self, angles, speed, is_radian=False):
        target = np.array(angles)
        self.enable_joint_mode()
        # Move to target position
        self.arm.set_servo_angle_j(
            angles=target.tolist(), speed=speed, wait=True, is_radian=is_radian
        )
        print(f"Moved to angles: {target}")

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
            wait=True,
        )
        if code != 0:
            logger.error(f"Failed to set position, code: {code}")

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
        ry = pose_values[4]  # degrees
        rz = pose_values[5]  # degrees

        # Create position vector (already in meters)
        position = Vector3(x, y, z)

        # Convert Euler angles to quaternion
        orientation = euler_to_quaternion(Vector3(rx, ry, rz), degrees=True)

        return Pose(position, orientation)

    def cmd_gripper_ctrl(self, position, effort=0.25):
        """Command end-effector gripper"""
        # xArm gripper position is 0-850 (0 = closed, 850 = open)
        # Convert from meters to gripper units
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

    def resetArm(self):
        """Reset the arm to initial state"""
        logger.info("Resetting arm...")
        self.arm.reset(wait=True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        logger.info("Arm reset complete")


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
        self.arm = UFactory7DOFArm(ip=self.arm_ip, xarm_type=self.arm_type)
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


if __name__ == "__main__":
    # TestXarmBridge(arm_ip="192.168.1.197", arm_type="xarm7")

    # # arm.cmd_joint_angles([0, 0, 0, 120, 0, 0, 0], speed=speed)

    # arm.gotoZero()
    arm = UFactory7DOFArm(ip="192.168.1.197", xarm_type="xarm7")
    arm.enable()
    # arm.gotoObserve()
    # time.sleep(2)
    arm.gotoZero()
    # arm.disconnect()
    print("disconnected")
