#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    prefix = LaunchConfiguration("prefix", default="")
    hw_ns = LaunchConfiguration("hw_ns", default="ufactory")
    limited = LaunchConfiguration("limited", default=False)
    effort_control = LaunchConfiguration("effort_control", default=False)
    velocity_control = LaunchConfiguration("velocity_control", default=False)
    add_gripper = LaunchConfiguration("add_gripper", default=False)
    add_vacuum_gripper = LaunchConfiguration("add_vacuum_gripper", default=False)

    # robot rviz launch
    # xarm_description/launch/_robot_rviz_display.launch.py
    robot_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/_robot_rviz_display.launch.py"]),
        launch_arguments={
            "prefix": prefix,
            "hw_ns": hw_ns,
            "limited": limited,
            "effort_control": effort_control,
            "velocity_control": velocity_control,
            "add_gripper": add_gripper,
            "add_vacuum_gripper": add_vacuum_gripper,
            "dof": "6",
            "robot_type": "lite",
        }.items(),
    )

    return LaunchDescription([robot_rviz_launch])
