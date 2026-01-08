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

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def get_xacro_file_content(
    xacro_file=PathJoinSubstitution(
        [FindPackageShare("xarm_description"), "urdf", "xarm_device.urdf.xacro"]
    ),
    arguments=None,
):
    if arguments is None:
        arguments = {}
    command = [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file, " "]
    if arguments and isinstance(arguments, dict):
        for key, val in arguments.items():
            command.extend([f"{key}:=", val, " "])
    return Command(command)
