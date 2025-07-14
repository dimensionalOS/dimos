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

from typing import (
    Optional,
)
from piper_sdk import *  # from the official Piper SDK
import numpy as np
import time
import subprocess


class PiperArm:
    def __init__(self, arm_name: str = "arm"):
        self.init_can()
        self.arm = C_PiperInterface_V2()
        self.arm.ConnectPort()
        time.sleep(0.1)
        self.resetArm()
        time.sleep(0.1)
        self.enable()
        self.gotoZero()
        time.sleep(1)

    def init_can(self):
        result = subprocess.run(
            ["bash", "dimos/dimos/hardware/can_activate.sh"],       # pass the script path directly if it has a shebang and execute perms
            stdout=subprocess.PIPE,             # capture stdout
            stderr=subprocess.PIPE,             # capture stderr
            text=True                           # return strings instead of bytes
        )

    def enable(self):
        while not self.arm.EnablePiper():
            pass
            time.sleep(0.01)
        print(f"[PiperArm] Enabled")
        self.arm.MotionCtrl_2(0x01, 0x01, 80, 0x00)



    def gotoZero(self):
        factor = 57295.7795 #1000*180/3.1415926
        position = [0,0,0,0,0,0,0]
        
        joint_0 = round(position[0]*factor)
        joint_1 = round(position[1]*factor)
        joint_2 = round(position[2]*factor)
        joint_3 = round(position[3]*factor)
        joint_4 = round(position[4]*factor)
        joint_5 = round(position[5]*factor)
        joint_6 = round(position[6]*1000*1000)
        self.arm.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.arm.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.arm.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        pass


    def softStop(self):
        self.gotoZero()
        time.sleep(1)
        self.arm.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.arm.MotionCtrl_1(0x01, 0, 0)
        time.sleep(5)


    def cmd_EE_pose(self, x, y, z, r, p, y_):
        """Command end-effector to target pose in space (position + Euler angles)"""
        factor = 1000
        pose = [x * factor, y * factor, z * factor, r * factor, p * factor, y_ * factor]
        self.arm.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.arm.EndPoseCtrl(int(pose[0]), int(pose[1]), int(pose[2]), int(pose[3]), int(pose[4]), int(pose[5]))
        print(f"[PiperArm] Moving to pose: {pose}")

    def get_EE_pose(self):
        """Return the current end-effector pose as (x, y, z, r, p, y)"""
        pose = self.arm.GetArmEndPoseMsgs()
        print(f"[PiperArm] Current pose: {pose}")
        return pose

    def cmd_gripper_ctrl(self, position):
        """Command end-effector gripper"""
        position = position * 1000

        self.arm.GripperCtrl(abs(round(position)), 1000, 0x01, 0)
        print(f"[PiperArm] Commanding gripper position: {position}")

    def resetArm(self):
        self.arm.MotionCtrl_1(0x02, 0, 0)
        self.arm.MotionCtrl_2(0, 0, 0, 0x00)
        print(f"[PiperArm] Resetting arm")

    def disable(self):
        self.softStop()
        
        while(self.arm.DisablePiper()):
            pass
            time.sleep(0.01)
        self.arm.DisconnectPort()


if __name__ == "__main__":
    arm = PiperArm()

    print("get_EE_pose")
    arm.get_EE_pose()

    while True:
        arm.cmd_EE_pose(60, 0, 300, 0, 85, 0)
        time.sleep(1)
        arm.cmd_EE_pose(60, 0, 260, 0, 85, 0)
        time.sleep(1)

        user_input = input("Press Enter to repeat, or type 'q' to quit: ")
        if user_input.strip().lower() == 'q':
            arm.disable()
            break
    
