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


class PiperArm:
    def __init__(self, arm_name: str = "arm"):
        self.arm = C_PiperInterface_V2()
        self.arm.ConnectPort()
        time.sleep(0.1)
        while not self.arm.EnablePiper():
            pass
            time.sleep(0.01)
        self.arm.MotionCtrl_1(0x02, 0, 0)
        self.arm.MotionCtrl_2(0, 0, 0, 0x00)
        self.arm.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        print(f"[PiperArm] Connected to {arm_name}")

    def softStop(self):
        self.arm.MotionCtrl_1(0x01, 0, 0)
        time.sleep(0.01)

    def cmd_EE_pose(self, x, y, z, r, p, y_):
        """Command end-effector to target pose in space (position + Euler angles)"""
        factor = 1000
        pose = [x * factor, y * factor, z * factor, r * factor, p * factor, y_ * factor]
        self.arm.EndPoseCtrl(pose)
        print(f"[PiperArm] Moving to pose: {pose}")

    def get_EE_pose(self):
        """Return the current end-effector pose as (x, y, z, r, p, y)"""
        pose = self.arm.getArmEndPoseMsgs()
        print(f"[PiperArm] Current pose: {pose}")
        return tuple(pose)

    def cmd_gripper_ctrl(self, position):
        """Command end-effector gripper"""
        position = position * 1000

        self.arm.GripperCtrl(abs(round(position)), 1000, 0x01, 0)
        print(f"[PiperArm] Commanding gripper position: {position}")

    def resetArm(self):
        self.arm.MotionCtrl_1(0x02, 0, 0)
        self.arm.MotionCtrl_2(0, 0, 0, 0x00)
        print(f"[PiperArm] Resetting arm")


if __name__ == "__main__":
    arm = PiperArm()
    arm.cmd_EE_pose(0, 0, 0, 0, 0, 0)
    time.sleep(1)
    arm.get_EE_pose()
    time.sleep(1)
