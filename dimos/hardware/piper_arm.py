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
import kinpy as kp
import sys
import termios
import tty
import select
from scipy.spatial.transform import Rotation as R
from dimos.utils.transform_utils import euler_to_quaternion, quaternion_to_euler

import random
import threading

import pytest

import dimos.core as core
import dimos.protocol.service.lcmservice as lcmservice
from dimos.core import In, Module, Out, rpc
from dimos_lcm.geometry_msgs import Pose, Vector3, Twist


class PiperArm:
    def __init__(self, arm_name: str = "arm"):
        self.init_can()
        self.arm = C_PiperInterface_V2()
        self.arm.ConnectPort()
        time.sleep(0.1)
        self.resetArm()
        time.sleep(0.1)
        self.enable()
        self.enable_gripper()  # Enable gripper after arm is enabled
        self.gotoZero()
        time.sleep(1)
        self.init_vel_controller()

    def init_can(self):
        result = subprocess.run(
            [
                "bash",
                "dimos/hardware/can_activate.sh",
            ],  # pass the script path directly if it has a shebang and execute perms
            stdout=subprocess.PIPE,  # capture stdout
            stderr=subprocess.PIPE,  # capture stderr
            text=True,  # return strings instead of bytes
        )

    def enable(self):
        while not self.arm.EnablePiper():
            pass
            time.sleep(0.01)
        print(f"[PiperArm] Enabled")
        # self.arm.ModeCtrl(
        #     ctrl_mode=0x01,         # CAN command mode
        #     move_mode=0x01,         # “Move-J”, but ignored in MIT
        #     move_spd_rate_ctrl=100, # doesn’t matter in MIT
        #     is_mit_mode=0xAD        # <-- the magic flag
        # )
        self.arm.MotionCtrl_2(0x01, 0x01, 80, 0xAD)

    def gotoZero(self):
        factor = 1000
        position = [57.0, 0.0, 250.0, 0, 97.0, 0, 0]
        X = round(position[0] * factor)
        Y = round(position[1] * factor)
        Z = round(position[2] * factor)
        RX = round(position[3] * factor)
        RY = round(position[4] * factor)
        RZ = round(position[5] * factor)
        joint_6 = round(position[6] * factor)
        print(X, Y, Z, RX, RY, RZ)
        self.arm.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.arm.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
        self.arm.GripperCtrl(0, 1000, 0x01, 0)

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

    def cmd_gripper_ctrl(self, position):
        """Command end-effector gripper"""
        factor = 1000
        position = position * factor * factor

        self.arm.GripperCtrl(abs(round(position)), 250, 0x01, 0)
        print(f"[PiperArm] Commanding gripper position: {position}")

    def enable_gripper(self):
        """Enable the gripper using the initialization sequence"""
        print("[PiperArm] Enabling gripper...")
        while not self.arm.EnablePiper():
            time.sleep(0.01)
        self.arm.GripperCtrl(0, 1000, 0x02, 0)
        self.arm.GripperCtrl(0, 1000, 0x01, 0)
        print("[PiperArm] Gripper enabled")

    def release_gripper(self):
        """Release gripper by opening to 100mm (10cm)"""
        print("[PiperArm] Releasing gripper (opening to 100mm)...")
        self.cmd_gripper_ctrl(0.1)  # 0.1m = 100mm = 10cm

    def resetArm(self):
        self.arm.MotionCtrl_1(0x02, 0, 0)
        self.arm.MotionCtrl_2(0, 0, 0, 0xAD)
        print(f"[PiperArm] Resetting arm")

    def init_vel_controller(self):
        self.chain = kp.build_serial_chain_from_urdf(
            open("dimos/hardware/piper_description.urdf"), "gripper_base"
        )
        self.J = self.chain.jacobian(np.zeros(6))
        self.J_pinv = np.linalg.pinv(self.J)
        self.dt = 0.01

    def cmd_vel(self, x_dot, y_dot, z_dot, R_dot, P_dot, Y_dot):
        joint_state = self.arm.GetArmJointMsgs().joint_state
        # print(f"[PiperArm] Current Joints (direct): {joint_state}", type(joint_state))
        joint_angles = np.array(
            [
                joint_state.joint_1,
                joint_state.joint_2,
                joint_state.joint_3,
                joint_state.joint_4,
                joint_state.joint_5,
                joint_state.joint_6,
            ]
        )
        # print(f"[PiperArm] Current Joints: {joint_angles}", type(joint_angles))
        factor = 57295.7795  # 1000*180/3.1415926
        joint_angles = joint_angles / factor  # convert to radians

        q = np.array(
            [
                joint_angles[0],
                joint_angles[1],
                joint_angles[2],
                joint_angles[3],
                joint_angles[4],
                joint_angles[5],
            ]
        )
        J = self.chain.jacobian(q)
        self.J_pinv = np.linalg.pinv(J)
        dq = self.J_pinv @ np.array([x_dot, y_dot, z_dot, R_dot, P_dot, Y_dot]) * self.dt
        newq = q + dq

        newq = newq * factor

        self.arm.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
        self.arm.JointCtrl(
            int(round(newq[0])),
            int(round(newq[1])),
            int(round(newq[2])),
            int(round(newq[3])),
            int(round(newq[4])),
            int(round(newq[5])),
        )
        time.sleep(self.dt)
        # print(f"[PiperArm] Moving to Joints to : {newq}")

    def cmd_vel_ee(self, x_dot, y_dot, z_dot, RX_dot, PY_dot, YZ_dot):
        factor = 1000
        x_dot = x_dot * factor
        y_dot = y_dot * factor
        z_dot = z_dot * factor
        RX_dot = RX_dot * factor
        PY_dot = PY_dot * factor
        YZ_dot = YZ_dot * factor

        current_pose_msg = self.get_ee_pose()

        # Convert quaternion to euler angles
        quat = [
            current_pose_msg.orientation.x,
            current_pose_msg.orientation.y,
            current_pose_msg.orientation.z,
            current_pose_msg.orientation.w,
        ]
        rotation = R.from_quat(quat)
        euler = rotation.as_euler("xyz")  # Returns [rx, ry, rz] in radians

        # Create current pose array [x, y, z, rx, ry, rz]
        current_pose = np.array(
            [
                current_pose_msg.position.x,
                current_pose_msg.position.y,
                current_pose_msg.position.z,
                euler[0],
                euler[1],
                euler[2],
            ]
        )

        # Apply velocity increment
        current_pose = current_pose + np.array([x_dot, y_dot, z_dot, R_dot, P_dot, Y_dot]) * self.dt

        self.cmd_ee_pose_values(
            current_pose[0],
            current_pose[1],
            current_pose[2],
            current_pose[3],
            current_pose[4],
            current_pose[5],
        )
        time.sleep(self.dt)

    def disable(self):
        self.softStop()

        while self.arm.DisablePiper():
            pass
            time.sleep(0.01)
        self.arm.DisconnectPort()


class VelocityController(Module):
    cmd_vel: In[Twist] = None

    def __init__(self, arm, period=0.01, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.arm = arm
        self.period = period
        self.latest_cmd = None
        self.last_cmd_time = None

    @rpc
    def start(self):
        self.cmd_vel.subscribe(self.handle_cmd_vel)

        def control_loop():
            while True:
                # Check for timeout (1 second)
                if self.last_cmd_time and (time.time() - self.last_cmd_time) > 1.0:
                    print("No velocity command received for 1 second, stopping control loop")
                    break

                cmd_vel = self.latest_cmd

                joint_state = self.arm.GetArmJointMsgs().joint_state
                # print(f"[PiperArm] Current Joints (direct): {joint_state}", type(joint_state))
                joint_angles = np.array(
                    [
                        joint_state.joint_1,
                        joint_state.joint_2,
                        joint_state.joint_3,
                        joint_state.joint_4,
                        joint_state.joint_5,
                        joint_state.joint_6,
                    ]
                )
                factor = 57295.7795  # 1000*180/3.1415926
                joint_angles = joint_angles / factor  # convert to radians
                q = np.array(
                    [
                        joint_angles[0],
                        joint_angles[1],
                        joint_angles[2],
                        joint_angles[3],
                        joint_angles[4],
                        joint_angles[5],
                    ]
                )

                J = self.chain.jacobian(q)
                self.J_pinv = np.linalg.pinv(J)
                dq = (
                    self.J_pinv
                    @ np.array(
                        [
                            cmd_vel.linear.X,
                            cmd_vel.linear.y,
                            cmd_vel.linear.z,
                            cmd_vel.angular.x,
                            cmd_vel.angular.y,
                            cmd_vel.angular.z,
                        ]
                    )
                    * self.dt
                )
                newq = q + dq

                newq = newq * factor  # convert radians to scaled degree units for joint control

                self.arm.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
                self.arm.JointCtrl(
                    int(round(newq[0])),
                    int(round(newq[1])),
                    int(round(newq[2])),
                    int(round(newq[3])),
                    int(round(newq[4])),
                    int(round(newq[5])),
                )
                time.sleep(self.period)

        thread = threading.Thread(target=control_loop, daemon=True)
        thread.start()

    def handle_cmd_vel(self, cmd_vel: Twist):
        self.latest_cmd = cmd_vel
        self.last_cmd_time = time.time()


@pytest.mark.tool
def run_velocity_controller():
    lcmservice.autoconf()
    dimos = core.start(2)

    velocity_controller = dimos.deploy(VelocityController, arm=arm, period=0.01)
    velocity_controller.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)

    velocity_controller.start()

    print("Velocity controller started")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    arm = PiperArm()

    def get_key(timeout=0.1):
        """Non-blocking key reader for arrow keys."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([fd], [], [], timeout)
            if rlist:
                ch1 = sys.stdin.read(1)
                if ch1 == "\x1b":  # Arrow keys start with ESC
                    ch2 = sys.stdin.read(1)
                    if ch2 == "[":
                        ch3 = sys.stdin.read(1)
                        return ch1 + ch2 + ch3
                else:
                    return ch1
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def teleop_linear_vel(arm):
        print("Use arrow keys to control linear velocity (x/y/z). Press 'q' to quit.")
        print("Up/Down: +x/-x, Left/Right: +y/-y, 'w'/'s': +z/-z")
        x_dot, y_dot, z_dot = 0.0, 0.0, 0.0
        while True:
            key = get_key(timeout=0.1)
            if key == "\x1b[A":  # Up arrow
                x_dot += 0.01
            elif key == "\x1b[B":  # Down arrow
                x_dot -= 0.01
            elif key == "\x1b[C":  # Right arrow
                y_dot += 0.01
            elif key == "\x1b[D":  # Left arrow
                y_dot -= 0.01
            elif key == "w":
                z_dot += 0.01
            elif key == "s":
                z_dot -= 0.01
            elif key == "q":
                print("Exiting teleop.")
                arm.disable()
                break

            # Optionally, clamp velocities to reasonable limits
            x_dot = max(min(x_dot, 0.5), -0.5)
            y_dot = max(min(y_dot, 0.5), -0.5)
            z_dot = max(min(z_dot, 0.5), -0.5)

            # Only linear velocities, angular set to zero
            arm.cmd_vel_ee(x_dot, y_dot, z_dot, 0, 0, 0)
            print(
                f"Current linear velocity: x={x_dot:.3f} m/s, y={y_dot:.3f} m/s, z={z_dot:.3f} m/s"
            )

    run_velocity_controller()
