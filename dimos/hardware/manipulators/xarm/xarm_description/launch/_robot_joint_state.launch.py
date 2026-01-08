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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix", default="")
    hw_ns = LaunchConfiguration("hw_ns", default="xarm")
    limited = LaunchConfiguration("limited", default=False)
    effort_control = LaunchConfiguration("effort_control", default=False)
    velocity_control = LaunchConfiguration("velocity_control", default=False)
    add_gripper = LaunchConfiguration("add_gripper", default=False)
    add_vacuum_gripper = LaunchConfiguration("add_vacuum_gripper", default=False)
    add_bio_gripper = LaunchConfiguration("add_bio_gripper", default=False)
    dof = LaunchConfiguration("dof", default=7)
    robot_type = LaunchConfiguration("robot_type", default="xarm")
    ros2_control_plugin = LaunchConfiguration(
        "ros2_control_plugin", default="uf_robot_hardware/UFRobotSystemHardware"
    )
    joint_states_remapping = LaunchConfiguration("joint_states_remapping", default="joint_states")

    add_realsense_d435i = LaunchConfiguration("add_realsense_d435i", default=False)
    model1300 = LaunchConfiguration("model1300", default=False)

    add_other_geometry = LaunchConfiguration("add_other_geometry", default=False)
    geometry_type = LaunchConfiguration("geometry_type", default="box")
    geometry_mass = LaunchConfiguration("geometry_mass", default=0.1)
    geometry_height = LaunchConfiguration("geometry_height", default=0.1)
    geometry_radius = LaunchConfiguration("geometry_radius", default=0.1)
    geometry_length = LaunchConfiguration("geometry_length", default=0.1)
    geometry_width = LaunchConfiguration("geometry_width", default=0.1)
    geometry_mesh_filename = LaunchConfiguration("geometry_mesh_filename", default="")
    geometry_mesh_origin_xyz = LaunchConfiguration("geometry_mesh_origin_xyz", default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration("geometry_mesh_origin_rpy", default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration("geometry_mesh_tcp_xyz", default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration("geometry_mesh_tcp_rpy", default='"0 0 0"')

    kinematics_suffix = LaunchConfiguration("kinematics_suffix", default="")

    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("xarm_description"), "launch", "_robot_description.launch.py"]
            )
        ),
        launch_arguments={
            "prefix": prefix,
            "hw_ns": hw_ns,
            "limited": limited,
            "effort_control": effort_control,
            "velocity_control": velocity_control,
            "add_gripper": add_gripper,
            "add_vacuum_gripper": add_vacuum_gripper,
            "add_bio_gripper": add_bio_gripper,
            "dof": dof,
            "robot_type": robot_type,
            "ros2_control_plugin": ros2_control_plugin,
            "joint_states_remapping": joint_states_remapping,
            "add_realsense_d435i": add_realsense_d435i,
            "model1300": model1300,
            "add_other_geometry": add_other_geometry,
            "geometry_type": geometry_type,
            "geometry_mass": geometry_mass,
            "geometry_height": geometry_height,
            "geometry_radius": geometry_radius,
            "geometry_length": geometry_length,
            "geometry_width": geometry_width,
            "geometry_mesh_filename": geometry_mesh_filename,
            "geometry_mesh_origin_xyz": geometry_mesh_origin_xyz,
            "geometry_mesh_origin_rpy": geometry_mesh_origin_rpy,
            "geometry_mesh_tcp_xyz": geometry_mesh_tcp_xyz,
            "geometry_mesh_tcp_rpy": geometry_mesh_tcp_rpy,
            "kinematics_suffix": kinematics_suffix,
        }.items(),
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {"source_list": [f"{prefix.perform(context)}{hw_ns.perform(context)}/joint_states"]}
        ],
    )

    return [
        robot_description_launch,
        joint_state_publisher_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
