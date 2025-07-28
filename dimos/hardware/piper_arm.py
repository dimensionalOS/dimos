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
    Tuple,
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
from dimos.utils.logging_config import setup_logger

import threading

import pytest

import dimos.core as core
import dimos.protocol.service.lcmservice as lcmservice
from dimos.core import In, Module, rpc
from dimos_lcm.geometry_msgs import Pose, Vector3, Twist

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
        self.init_vel_controller()

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

    def gotoObserve(self):
        factor = 1000
        position = [57.0, 0.0, 280.0, 0, 120.0, 0, 0]
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
        current_pose = (
            current_pose + np.array([x_dot, y_dot, z_dot, RX_dot, PY_dot, YZ_dot]) * self.dt
        )

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
                    logger.warning(
                        "No velocity command received for 1 second, stopping control loop"
                    )
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

    logger.info("Velocity controller started")
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
                logger.info("Exiting teleop")
                arm.disable()
                break

            # Optionally, clamp velocities to reasonable limits
            x_dot = max(min(x_dot, 0.5), -0.5)
            y_dot = max(min(y_dot, 0.5), -0.5)
            z_dot = max(min(z_dot, 0.5), -0.5)

            # Only linear velocities, angular set to zero
            arm.cmd_vel_ee(x_dot, y_dot, z_dot, 0, 0, 0)
            logger.debug(
                f"Current linear velocity: x={x_dot:.3f} m/s, y={y_dot:.3f} m/s, z={z_dot:.3f} m/s"
            )

    run_velocity_controller()
