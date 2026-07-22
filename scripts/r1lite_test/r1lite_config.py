# Copyright 2026 Dimensional Inc.
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

"""R1 Lite hardware constants for the test suite.

Robot: R1-LITE, A1X arms, T0 torso, C1 chassis, stereo head camera (no depth
topic), RealSense wrist cameras. Onboard: x86_64, Ubuntu 22.04, ROS 2 Humble.
Set the robot IP via the R1LITE_IP env var or --robot-ip; no IPs are hardcoded.
"""

ROS_DOMAIN_ID = 2

# Kinematics.
ARM_DOF = 6  # /hdas/feedback_arm_*: 6 joints @ 200 Hz
TORSO_DOF = 4  # /hdas/feedback_torso: 4 joints @ 488 Hz
CHASSIS_FB_DOF = 3  # /hdas/feedback_chassis: 3 joints @ 200 Hz
GRIPPER_DOF = 1  # /hdas/feedback_gripper_*: 1 joint @ 200 Hz

TORSO_HOME_POSE: list[float] = [0.0] * TORSO_DOF

# Topics.
FEEDBACK_ARM = "/hdas/feedback_arm_{side}"  # JointState, 200 Hz
FEEDBACK_TORSO = "/hdas/feedback_torso"  # JointState, 488 Hz
FEEDBACK_CHASSIS = "/hdas/feedback_chassis"  # JointState, 200 Hz
FEEDBACK_GRIPPER = "/hdas/feedback_gripper_{side}"  # JointState, 200 Hz
IMU_CHASSIS = "/hdas/imu_chassis"  # sensor_msgs/Imu
IMU_TORSO = "/hdas/imu_torso"  # sensor_msgs/Imu
CMD_ARM = "/motion_target/target_joint_state_arm_{side}"  # JointState
CMD_TORSO = "/motion_target/target_joint_state_torso"  # JointState
CMD_TORSO_SPEED = "/motion_target/target_speed_torso"  # TwistStamped (new vs R1 Pro)
CMD_GRIPPER = "/motion_target/target_position_gripper_{side}"  # JointState
CMD_CHASSIS_SPEED = "/motion_target/target_speed_chassis"  # TwistStamped
CHASSIS_ACC_LIMIT = "/motion_target/chassis_acc_limit"  # TwistStamped
BRAKE_MODE = "/motion_target/brake_mode"  # std_msgs/Bool
CHASSIS_SPEED_FB = "/motion_control/chassis_speed"  # TwistStamped (Gate-1 analog)
# TODO(recon): R1 Pro's gatekeeper /cmd_vel does NOT exist here, chassis
# command path is CMD_CHASSIS_SPEED directly; 3-gate behavior unverified
# (all three gate suspects exist: /controller topic, brake_mode, acc_limit).
CMD_VEL_GATEKEEPER = CMD_CHASSIS_SPEED

# Perception (verified): wrist cams = full RealSense stacks; head = stereo
# RGB pair, NO depth topic; NO chassis cameras; NO lidar topic in this
# boot profile (livox driver installed but not launched).
HEAD_LEFT_COMPRESSED = "/hdas/camera_head/left_raw/image_raw_color/compressed"
HEAD_RIGHT_COMPRESSED = "/hdas/camera_head/right_raw/image_raw_color/compressed"
WRIST_COLOR_COMPRESSED = "/hdas/camera_wrist_{side}/color/image_raw/compressed"
WRIST_DEPTH_ALIGNED = "/hdas/camera_wrist_{side}/aligned_depth_to_color/image_raw"
ROBOT_DESCRIPTION = "/robot_description"  # URDF published on-topic (pubs=2)

# Topics that must exist for test_01 to pass.
EXPECTED_TOPICS = [
    FEEDBACK_ARM.format(side="left"),
    FEEDBACK_ARM.format(side="right"),
    FEEDBACK_CHASSIS,
    FEEDBACK_TORSO,
    FEEDBACK_GRIPPER.format(side="left"),
    FEEDBACK_GRIPPER.format(side="right"),
    IMU_CHASSIS,
    IMU_TORSO,
    CMD_CHASSIS_SPEED,
    CMD_ARM.format(side="left"),
    CMD_ARM.format(side="right"),
    CMD_TORSO,
]

# Remaining open questions.
# 1. Joint names per topic + /joint_states 25-joint composition.
# 2. Torso safe home pose.
# 3. Chassis gating: does r1_lite_chassis_control_node need subscriber/
#    brake/acc-limit unlocks like the R1 Pro's 3 gates? (test_03 will tell.)
# 4. Head depth: stereo-only? (calib/head_{left,right} suggest onboard
#    stereo depth is possible but not published.)
# 5. Lidar: present on unit? (No topic; livox driver installed.)
