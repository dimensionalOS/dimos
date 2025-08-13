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

from xarm.wrapper import XArmAPI

from dimos.hardware.end_effector import EndEffector

import dimos.core as core
from dimos.core import Module, In, Out, rpc
from dimos.protocol.service.lcmservice import autoconf
from dimos.msgs.geometry_msgs import Pose, Vector3, Twist
import dimos.protocol.service.lcmservice as lcmservice
from dimos.msgs.sensor_msgs.JointState import JointState


class UFactoryEndEffector(EndEffector):
    def __init__(self, model=None, **kwargs):
        super().__init__(**kwargs)
        self.model = model

    def get_model(self):
        return self.model


class UFactoryArm:
    def __init__(self, ip=None, xarm_type="xarm6"):
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

        # Initialize with proper connection settings - use radians for consistency
        self.arm = XArmAPI(self.ip, do_not_open=False, is_radian=True)
        self.arm.clean_error()  # Clear any existing errors
        self.arm.clean_warn()  # Clear any warnings
        print("initializing arm")
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)  # Set to joint control mode since we're using joint commands
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

    def cmd_joint_angles(self, angles, speed, is_radian=True):
        # Validate that we have the correct number of joints
        expected_joints = 7 if self.xarm_type == "xarm7" else 6
        if len(angles) != expected_joints:
            print(
                f"[xArmBridge] Error: Expected {expected_joints} joint values for {self.xarm_type}, got {len(angles)}"
            )
            return

        # Clear any errors before sending command
        if self.arm.error_code != 0:
            if self.arm.error_code == 9 or self.arm.error_code == 2:
                print(f"[xArmBridge] Error: {self.arm.error_code}")
            else:
                self.arm.clean_error()
                self.arm.set_state(0)

        code = self.arm.set_servo_angle_j(
            angles=list(angles), speed=speed, wait=False, is_radian=is_radian
        )
        if code != 0:
            print(f"[xArmBridge] Warning: Command returned code {code}")
        print(f"Moved to angles: {angles}")

    def enable_joint_mode(self):
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(0.1)

    def enable_position_mode(self):
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)

    def initialize_gripper(self):
        """Initialize the gripper if it exists."""
        try:
            print("[xArmBridge] Initializing gripper...")
            self.arm.set_gripper_mode(0)
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_speed(5000)
            self.arm.clean_gripper_error()
        except Exception as e:
            print(f"[xArmBridge] Error initializing gripper: {e}")
            err = self.arm.get_gripper_err_code()
            print(err)

    def set_gripper_pos(self, gripper_pos):
        """Set the position of the gripper."""
        try:
            print(f"[xArmBridge] Setting gripper position to: {gripper_pos}")
            self.arm.set_gripper_position(gripper_pos)
        except Exception as e:
            print(f"[xArmBridge] Error setting gripper position: {e}")


class xArmBridge(Module):
    joint_state: In[JointState] = None
    pose_state: Out[JointState] = None
    target_joint_state = None
    target_gripper_pos = None
    prev_joint_state = None
    prev_gripper_pos = None
    arm = None
    first_message = True

    def __init__(self, arm_ip: str = None, arm_type: str = "xarm6", *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.arm_ip = arm_ip
        self.arm_type = arm_type
        self.arm = None
        # Initialize with correct number of joints based on arm type
        if arm_type == "xarm7":
            self.target_joint_state = [0, 0, 0, 0, 0, 0, 0]
            self.prev_joint_state = [0, 0, 0, 0, 0, 0, 0]
        else:  # xarm6
            self.target_joint_state = [0, 0, 0, 0, 0, 0]
            self.prev_joint_state = [0, 0, 0, 0, 0, 0]
        self.target_gripper_pos = 850
        self.prev_gripper_pos = 850

    @rpc
    def start(self):
        # subscribe to incoming LCM JointState messages
        self.arm = UFactoryArm(ip=self.arm_ip, xarm_type=self.arm_type)
        self.arm.enable()
        self.arm.initialize_gripper()
        # print(f"Initialized xArmBridge with arm type: {self.arm.xarm_type}")
        self.joint_state.subscribe(self._on_joint_state)
        # print(f"Subscribed to {self.joint_state}")

    # @rpc
    def command_arm(self):
        try:
            abs_diff = [abs(t - p) for t, p in zip(self.target_joint_state, self.prev_joint_state)]
            max_diff = max(abs_diff)
            if max_diff > 0.2:
                print("[xArmBridge] Using position mode due to large difference")
                self.arm.enable_position_mode()
                self.arm.arm.set_servo_angle(
                    angle=self.target_joint_state, speed=3.14, wait=True, is_radian=True
                )
                self.arm.enable_joint_mode()
                return

            print("[xArmBridge] Commanding arm with target joint state:", self.target_joint_state)
            self.arm.cmd_joint_angles(self.target_joint_state, speed=3.14, is_radian=True)
        except Exception as e:
            print(f"[xArmBridge] Error commanding arm: {e}")

    def set_gripper_pos(self, gripper_pos):
        self.arm.set_gripper_pos(gripper_pos)

    def _on_joint_state(self, msg: JointState):
        # print(f"[xArmBridge] Received joint state: {msg}")
        if not msg or not msg.name or not msg.position:
            # print("[xArmBridge] No joint names or positions found in message.")
            return

        # Create a mapping of joint names to positions
        joint_map = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_map[name] = msg.position[i]

        # Determine number of joints based on arm type
        num_joints = 7 if self.arm_type == "xarm7" else 6

        # Extract joint values by name
        joint_values = []
        missing_joints = []

        for i in range(1, num_joints + 1):  # joint1 through joint6/7
            joint_name = f"joint{i}"
            if joint_name in joint_map:
                joint_values.append(joint_map[joint_name])
            else:
                missing_joints.append(joint_name)
                joint_values.append(0.0)  # Default to 0 if joint not found

        if missing_joints:
            print(f"[xArmBridge] Warning: Missing joints in message: {missing_joints}")
            print(f"[xArmBridge] Available joints: {list(joint_map.keys())}")
        
        # Updating gripper position
        self.prev_gripper_pos = self.target_gripper_pos
        self.target_gripper_pos = joint_map["drive_joint"] * 1000

        # Update target joint state
        self.prev_joint_state = self.target_joint_state.copy()
        self.target_joint_state = joint_values
        if self.first_message:
            self.first_message = False
            self.prev_joint_state = self.target_joint_state.copy()
        # print(f"[xArmBridge] Updated target joint state: {self.target_joint_state}")

    def _reader(self):
        while True:
            print("Reading from arm")
            angles = self.arm.arm.get_servo_angle(is_radian=False)[1]
            print(f"Current angles: {angles}")
            if not angles:
                continue


def TestXarmBridge(arm_ip: str = None, arm_type: str = "xArm6"):
    lcmservice.autoconf()
    dimos = core.start(2)

    try:
        armBridge = dimos.deploy(xArmBridge, arm_ip=arm_ip, arm_type=arm_type)

        armBridge.pose_state.transport = core.LCMTransport("/armJointState", JointState)
        armBridge.joint_state.transport = core.LCMTransport("joint_states", JointState)

        armBridge.start()
        print("xArmBridge started and listening for joint states.")

        while True:
            # print(armBridge.target_joint_state)
            if armBridge.target_joint_state != armBridge.prev_joint_state:
                armBridge.command_arm()  # Command the arm  at 50hz with the target joint state
            if armBridge.target_gripper_pos != armBridge.prev_gripper_pos:
                armBridge.set_gripper_pos(armBridge.target_gripper_pos)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if "armBridge" in locals() and armBridge.arm:
            armBridge.arm.disconnect()
        dimos.stop()
        print("Cleanup complete")


if __name__ == "__main__":
    TestXarmBridge(arm_ip="192.168.1.210", arm_type="xarm6")
