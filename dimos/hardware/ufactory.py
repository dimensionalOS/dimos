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

        # Initialize with proper connection settings
        self.arm = XArmAPI(self.ip, do_not_open=False, is_radian=False)
        self.arm.clean_error()  # Clear any existing errors
        self.arm.clean_warn()   # Clear any warnings
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

    def cmd_joint_angles(self, angles, speed, is_radian=False):
        self.arm.set_servo_angle_j(
            angles=angles, speed=speed, wait=False, is_radian=is_radian
        )
        print(f"Moved to angles: {angles}")

    def enable_joint_mode(self):
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(0.1)

    def enable_position_mode(self):
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(0.1)


class xArmBridge(Module):
    joint_state: In[JointState] = None
    pose_state: Out[JointState] = None
    target_joint_state = None
    arm = None

    def __init__(self, arm_ip: str = None, arm_type: str = "xarm6", *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.arm_ip = arm_ip
        self.arm_type = arm_type
        self.arm = None
        self.target_joint_state = [0, 0, 0, 0, 0, 0]

    @rpc
    def start(self):
        # subscribe to incoming LCM JointState messages
        self.arm = UFactoryArm(ip=self.arm_ip, xarm_type=self.arm_type)
        self.arm.enable()
        # print(f"Initialized xArmBridge with arm type: {self.arm.xarm_type}")
        self.joint_state.subscribe(self._on_joint_state)
        # print(f"Subscribed to {self.joint_state}")

    # @rpc
    def command_arm(self):
        print("[xArmBridge] Commanding arm with target joint state:", self.target_joint_state)
        try:
            self.arm.cmd_joint_angles(self.target_joint_state, speed=3.14, is_radian=True)
        except Exception as e:
            print(f"[xArmBridge] Error commanding arm: {e}")

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
            # joint7 = msg.position[9]

            # print(f"[xArmBridge] Joint values - joint1: {joint1}, joint2: {joint2}, joint3: {joint3}, joint4: {joint4}, joint5: {joint5}, joint6: {joint6}, joint7: {joint7}")
            self.target_joint_state = [joint1, joint2, joint3, joint4, joint5, joint6]
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
            armBridge.command_arm()  # Command the arm  at 100hz with the target joint state
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'armBridge' in locals() and armBridge.arm:
            armBridge.arm.disconnect()
        dimos.stop()
        print("Cleanup complete")


if __name__ == "__main__":
    TestXarmBridge(arm_ip="192.168.1.210", arm_type="xarm6")