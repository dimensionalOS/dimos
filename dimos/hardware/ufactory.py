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
        self.gotoZero()

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
        
        step_size = 0.017453292519943295  # rad per step
        current = np.array(self.arm.get_servo_angle(is_radian=True)[1])
        target = np.array(angles)
        error = target - current
        print(f"Initial error: {error}")
        
        steps_needed = np.abs(error) / step_size
        max_steps = int(np.ceil(np.max(steps_needed)))
        print(f"Steps needed: {max_steps}, Step size: {step_size}")

        self.enable_joint_mode()
        
        # Perform incremental movement
        for step in range(max_steps):
            # Calculate current step progress
            progress = (step + 1) / max_steps
            
            # Calculate intermediate target for this step
            intermediate_target = current + (error * progress)
            # print(f"Intermediate target at step {step}: {intermediate_target}")
            # Move to intermediate position
            self.arm.set_servo_angle_j(angles=intermediate_target.tolist(), speed=speed, wait=True, is_radian=is_radian)
            
            # Small delay for smooth motion
            time.sleep(0.01)
    
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

    def __init__ (
            self,
            arm_ip: str = None,
            arm_type: str = "xarm7",
            *args, 
            **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.arm_ip = arm_ip
        self.arm_type = arm_type
        self.arm = None
        

    @rpc
    def start(self):
        # subscribe to incoming LCM JointState messages
        self.arm = UFactory7DOFArm(ip=self.arm_ip, xarm_type=self.arm_type)
        self.arm.enable()
        print(f"Initialized xArmBridge with arm type: {self.arm.xarm_type}")
        self.joint_state.subscribe(self._on_joint_state)
        print(f"Subscribed to {self.joint_state}")

    def _on_joint_state(self, msg: JointState):
        print(f"[xArmBridge] Received joint state: {msg}")
        if not msg:
            print("[xArmBridge] No joint names found in message.")
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
        else:
            print(f"[xArmBridge] Insufficient joint data: expected at least 10 joints, got {len(msg.position)}")

        # Set servo angles for the xArm
        angles = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
        # print(f"[xArmBridge] Setting servo angles: {angles}")
        self.arm.cmd_joint_angles(angles, speed=0.1, is_radian=True)

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
        time.sleep(1)



if __name__ == "__main__":
    # arm = UFactory7DOFArm(ip="192.168.1.197", xarm_type="xarm7")
    # arm.enable()
    # print("enabled")
    # time.sleep(1)
    # speed = 10
    
    TestXarmBridge(arm_ip="192.168.1.197", arm_type="xarm7")

    # # arm.cmd_joint_angles([0, 0, 0, 120, 0, 0, 0], speed=speed)
    
    # arm.gotoZero()
    
    # arm.disconnect()
    # print("disconnected")