# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Controller feedback. TCP values retain the driver's x/y/z/roll/pitch/yaw convention.
std_msgs/Header header
int32 state
int32 mode
int32 error_code
int32 warn_code
int32 cmdnum
int32 mt_brake
int32 mt_able
float64[] tcp_pose
float64[] tcp_offset
float64[] joints
