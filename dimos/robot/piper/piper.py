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

import math
import threading
import time

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from piper_msgs.msg import PiperEulerPose, PiperStatusMsg, PosCmd
from piper_msgs.srv import Enable, EnableResponse, GoZero, GoZeroResponse, Gripper, GripperResponse
from piper_sdk import *
from piper_sdk import C_PiperInterface
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler

from dimos.core import In, Module, Out, rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.piper")


class Piper(Module):
    joint: Out[JointState] = None
    arm_status: Out[PiperStatusMsg] = None
    end_pose: Out[PoseStamped] = None

    set_pose_in: In[PoseStamped] = None
    set_joint_in: In[JointState] = None

    def __init__(
        self,
        can_port="can0",
        auto_enable=True,
        gripper_exist=True,
        gripper_val_multiple=1,
        rate=10,  # hz
    ) -> None:
        self.can_port = can_port
        self.gripper_exist = gripper_exist
        self.gripper_val_multiple = gripper_val_multiple
        self.gripper_val_mutiple = 1
        self.rate = rate

        # joints
        self.joint_states = JointState()
        self.joint_states.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ]
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7

        # Create piper class and open CAN interface
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()
        self.piper.MotionCtrl_2(0x01, 0x01, 30, 0)
        self.block_ctrl_flag = False
        # Start subscription threads
        sub_pos_th = threading.Thread(target=self.SubPosThread)
        sub_pos_th.daemon = True
        sub_pos_th.start()

    def start(self):
        self.set_pose_in.subscribe(self.set_pose)
        self.set_joint_in.subscribe(self.set_joint)

    def GetEnableFlag(self):
        return self.__enable_flag

    def Pubilsh(self):
        """Robotic arm message publishing"""
        enable_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record time before entering loop
        start_time = time.time()
        elapsed_time_flag = False

        enable_flag = (
            self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
            and self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
            and self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
            and self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
            and self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
            and self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        )
        print("Enable state:", enable_flag)

        while True:
            self.PublishArmState()
            self.PublishArmEndPose()
            self.PublishArmJointAndGripper()
            time.sleep(self.rate)

    def PublishArmState(self):
        # Robotic arm state
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = (
            self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        )
        arm_status.joint_2_angle_limit = (
            self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        )
        arm_status.joint_3_angle_limit = (
            self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        )
        arm_status.joint_4_angle_limit = (
            self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        )
        arm_status.joint_5_angle_limit = (
            self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        )
        arm_status.joint_6_angle_limit = (
            self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        )
        arm_status.communication_status_joint_1 = (
            self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        )
        arm_status.communication_status_joint_2 = (
            self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        )
        arm_status.communication_status_joint_3 = (
            self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        )
        arm_status.communication_status_joint_4 = (
            self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        )
        arm_status.communication_status_joint_5 = (
            self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        )
        arm_status.communication_status_joint_6 = (
            self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        )
        self.arm_status.publish(arm_status)

    def PublishArmJointAndGripper(self):
        # Robotic arm joint angles and gripper position
        # Since the raw data is in degrees multiplied by 1000, to convert to radians we need to divide by 1000 first, then multiply by 3.14/180, then limit decimal places to 5
        joint_0: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort / 1000
        effort_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort / 1000
        effort_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort / 1000
        effort_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort / 1000
        effort_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort / 1000
        effort_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort / 1000
        effort_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort / 1000
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]
        self.joint_states.effort = [
            effort_0,
            effort_1,
            effort_2,
            effort_3,
            effort_4,
            effort_5,
            effort_6,
        ]
        # Publish all messages
        self.joint.publish(self.joint_states)

    def PublishArmEndPose(self):
        # End effector pose
        endpos = PoseStamped()
        endpos.pose.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        endpos.pose.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        endpos.pose.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        endpos.pose.orientation.x = quaternion[0]
        endpos.pose.orientation.y = quaternion[1]
        endpos.pose.orientation.z = quaternion[2]
        endpos.pose.orientation.w = quaternion[3]
        endpos.header.stamp = rospy.Time.now()
        self.end_pose.publish(endpos)

        end_pose_euler = PiperEulerPose()
        end_pose_euler.header.stamp = rospy.Time.now()
        # end_pose_euler.header.seq = endpos.header.seq
        end_pose_euler.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        end_pose_euler.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        end_pose_euler.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        end_pose_euler.roll = roll
        end_pose_euler.pitch = pitch
        end_pose_euler.yaw = yaw
        # self.end_pose_euler_pub.publish(end_pose_euler)

    @rpc
    def set_pose(self, pos_data):
        """Robotic arm end effector pose subscription callback function

        Args:
            pos_data ():
        """
        # logger.info("Received PosCmd:")
        # logger.info("x: %f", pos_data.x)
        # logger.info("y: %f", pos_data.y)
        # logger.info("z: %f", pos_data.z)
        # logger.info("roll: %f", pos_data.roll)
        # logger.info("pitch: %f", pos_data.pitch)
        # logger.info("yaw: %f", pos_data.yaw)
        # logger.info("gripper: %f", pos_data.gripper)
        # logger.info("mode1: %d", pos_data.mode1)
        # logger.info("mode2: %d", pos_data.mode2)
        if not self.block_ctrl_flag:
            factor = 180 / 3.1415926
            x = round(pos_data.x * 1000) * 1000
            y = round(pos_data.y * 1000) * 1000
            z = round(pos_data.z * 1000) * 1000
            rx = round(pos_data.roll * 1000 * factor)
            ry = round(pos_data.pitch * 1000 * factor)
            rz = round(pos_data.yaw * 1000 * factor)
            logger.info("Received PosCmd:")
            logger.info("x: %f", x)
            logger.info("y: %f", y)
            logger.info("z: %f", z)
            logger.info("roll: %f", rx)
            logger.info("pitch: %f", ry)
            logger.info("yaw: %f", rz)
            logger.info("gripper: %f", pos_data.gripper)
            logger.info("mode1: %d", pos_data.mode1)
            logger.info("mode2: %d", pos_data.mode2)
            if self.GetEnableFlag():
                self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
                self.piper.MotionCtrl_2(0x01, 0x00, 50)
                self.piper.EndPoseCtrl(x, y, z, rx, ry, rz)
                gripper = round(pos_data.gripper * 1000 * 1000)
                if pos_data.gripper > 80000:
                    gripper = 80000
                if pos_data.gripper < 0:
                    gripper = 0
                if self.gripper_exist:
                    self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
                self.piper.MotionCtrl_2(0x01, 0x00, 50)

    @rpc
    def set_joint(self, joint_data):
        """Robotic arm joint angle callback function

        Args:
            joint_data ():
        """
        if not self.block_ctrl_flag:
            factor = 57324.840764  # 1000*180/3.14
            factor = 1000 * 180 / np.pi
            # logger.info("Received Joint States:")
            # logger.info("joint_0: %f", joint_data.position[0])
            # logger.info("joint_1: %f", joint_data.position[1])
            # logger.info("joint_2: %f", joint_data.position[2])
            # logger.info("joint_3: %f", joint_data.position[3])
            # logger.info("joint_4: %f", joint_data.position[4])
            # logger.info("joint_5: %f", joint_data.position[5])
            # logger.info("joint_6: %f", joint_data.position[6])
            # print(joint_data.position)
            joint_0 = round(joint_data.position[0] * factor)
            joint_1 = round(joint_data.position[1] * factor)
            joint_2 = round(joint_data.position[2] * factor)
            joint_3 = round(joint_data.position[3] * factor)
            joint_4 = round(joint_data.position[4] * factor)
            joint_5 = round(joint_data.position[5] * factor)
            if len(joint_data.position) >= 7:
                joint_6 = round(joint_data.position[6] * 1000 * 1000)
                joint_6 = joint_6 * self.gripper_val_mutiple
                if joint_6 > 80000:
                    joint_6 = 80000
                if joint_6 < 0:
                    joint_6 = 0
            else:
                joint_6 = None
            if self.GetEnableFlag():
                # Set motor speed
                if joint_data.velocity != []:
                    all_zeros = all(v == 0 for v in joint_data.velocity)
                else:
                    all_zeros = True
                if not all_zeros:
                    lens = len(joint_data.velocity)
                    if lens == 7:
                        vel_all = round(joint_data.velocity[6])
                        if vel_all > 100:
                            vel_all = 100
                        if vel_all < 0:
                            vel_all = 0
                        logger.info("vel_all: %d", vel_all)
                        self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                    # elif(lens == 7):
                    #     # Iterate through speed list
                    #     for i, velocity in enumerate(joint_data.velocity):
                    #         if velocity > 0:  # If speed is positive
                    #             # Set joint speed at specified position to this positive speed
                    #             # self.piper.SearchMotorMaxAngleSpdAccLimit(i+1,0x01)
                    #             # self.piper.MotorAngleLimitMaxSpdSet(i+1)
                    else:
                        self.piper.MotionCtrl_2(0x01, 0x01, 50, 0)
                else:
                    self.piper.MotionCtrl_2(0x01, 0x01, 50, 0)

                # Set joint angle position
                self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                # If end gripper exists, send end gripper control
                if self.gripper_exist and joint_6 is not None:
                    if abs(joint_6) < 200:
                        joint_6 = 0
                    if len(joint_data.effort) >= 7:
                        gripper_effort = joint_data.effort[6]
                        gripper_effort = max(0.5, min(gripper_effort, 3))
                        # logger.info("gripper_effort: %f", gripper_effort)
                        gripper_effort = round(gripper_effort * 1000)
                        self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                    # Default 1N
                    else:
                        self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    @rpc
    def gripper(self, req):
        response = GripperResponse()
        response.code = 15999
        response.status = False
        if self.gripper_exist:
            logger.info(f"-----------------------Gripper---------------------------")
            logger.info(f"Received request:")
            logger.info(f"PS: Piper should be enable.Please ensure piper is enable")
            logger.info(f"gripper_angle:{req.gripper_angle}, range is [0m, 0.07m]")
            logger.info(f"gripper_effort:{req.gripper_effort},range is [0.5N/m, 2N/m]")
            logger.info(
                f"gripper_code:{req.gripper_code}, range is [0, 1, 2, 3]\n \
                            0x00: Disable\n \
                            0x01: Enable\n \
                            0x03/0x02: Enable and clear error / Disable and clear error"
            )
            logger.info(
                f"set_zero:{req.set_zero}, range is [0, 0xAE] \n \
                            0x00: Invalid value \n \
                            0xAE: Set zero point"
            )
            logger.info(f"-----------------------Gripper---------------------------")
            gripper_angle = req.gripper_angle
            gripper_angle = round(max(0, min(req.gripper_angle, 0.07)) * 1e6)
            gripper_effort = req.gripper_effort
            gripper_effort = round(max(0.5, min(req.gripper_effort, 2)) * 1e3)
            if req.gripper_code not in [0x00, 0x01, 0x02, 0x03]:
                logger.warning("gripper_code should be in [0, 1, 2, 3], default val is 1")
                gripper_code = 1
                response.code = 15901
            else:
                gripper_code = req.gripper_code
            if req.set_zero not in [0x00, 0xAE]:
                logger.warning("set_zero should be in [0, 0xAE], default val is 0")
                set_zero = 0
                response.code = 15902
            else:
                set_zero = req.set_zero
            response.code = 15900
            self.piper.GripperCtrl(abs(gripper_angle), gripper_effort, gripper_code, set_zero)
            response.status = True
        else:
            logger.warning("gripper_exist param is False.")
            response.code = 15903
            response.status = False
        logger.info(f"Returning GripperResponse: {response.code}, {response.status}")
        return response

    @rpc
    def enable(self, req):
        logger.info(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # Set timeout (seconds)
        timeout = 5
        # Record time before entering loop
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_list = []
            enable_list.append(
                self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
            )
            enable_list.append(
                self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
            )
            enable_list.append(
                self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
            )
            enable_list.append(
                self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
            )
            enable_list.append(
                self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
            )
            enable_list.append(
                self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            )
            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)
            print("Enable state:", enable_flag)
            self.__enable_flag = enable_flag
            print("--------------------")
            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False
            # Check if timeout exceeded
            if elapsed_time > timeout:
                print("Timeout...")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        response = enable_flag
        logger.info(f"Returning response: {response}")
        return EnableResponse(response)

    @rpc
    def stop(self, req):
        response = TriggerResponse()
        response.success = False
        response.message = "stop piper failed"
        logger.info(f"-----------------------STOP---------------------------")
        logger.info(f"Stop piper.")
        logger.info(f"-----------------------STOP---------------------------")
        self.piper.MotionCtrl_1(0x01, 0, 0)
        response.success = True
        response.message = "stop piper success"
        logger.info(f"Returning StopResponse: {response.success}, {response.message}")
        return response

    @rpc
    def reset(self, req):
        response = TriggerResponse()
        response.success = False
        response.message = "reset piper failed"
        logger.info(f"-----------------------RESET---------------------------")
        logger.info(f"reset piper.")
        logger.info(f"-----------------------RESET---------------------------")
        self.piper.MotionCtrl_1(0x02, 0, 0)  # Restore
        response.success = True
        response.message = "reset piper success"
        logger.info(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    @rpc
    def zero(self, req):
        response = GoZeroResponse()
        response.status = False
        response.code = 151000
        logger.info(f"-----------------------GOZERO---------------------------")
        logger.info(f"piper go zero .")
        logger.info(f"-----------------------GOZERO---------------------------")
        if req.is_mit_mode:
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0xAD)
        else:
            self.piper.MotionCtrl_2(0x01, 0x01, 50, 0)
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        response.status = True
        response.code = 151001
        logger.info(f"Returning GoZeroResponse: {response.status}, {response.code}")
        return response

    @rpc
    def block(self, req):
        response = SetBoolResponse()
        logger.info(f"-----------------------BLOCK_ARM---------------------------")
        if req.data:
            response.success = req.data
            response.message = "You will block arm ctrl msg send"
        else:
            response.success = req.data
            response.message = "You will unblock arm ctrl msg send"
        self.block_ctrl_flag = req.data
        logger.info(f"piper block arm .")
        logger.info(f"Returning BlockArmResponse: {response.success}, {response.message}")
        logger.info(f"-----------------------BLOCK_ARM---------------------------")
        return response
