# Copyright (c) 2025, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch parameters
    robot_ip = LaunchConfiguration('robot_ip', default=os.getenv(
        'ROBOT_IP', os.getenv('GO2_IP', '')))
    enable_video = LaunchConfiguration('enable_video', default='true')
    urdf_file_name = LaunchConfiguration('urdf_file_name', default='go2.urdf')

    pkg_dir = get_package_share_directory('go2_robot_sdk')
    urdf_path = os.path.join(pkg_dir, 'urdf')

    # Read URDF content before LaunchDescription
    default_urdf_path = os.path.join(urdf_path, 'go2.urdf')
    with open(default_urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_ip',
            default_value=os.getenv('ROBOT_IP', os.getenv('GO2_IP', '')),
            description='IP address of the robot'
        ),
        DeclareLaunchArgument(
            'enable_video',
            default_value='true',
            description='Enable video streaming'
        ),
        DeclareLaunchArgument(
            'urdf_file_name',
            default_value='go2.urdf',
            description='Name of the URDF file'
        ),

        # Robot state publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc
            }],
        ),

        # Go2 driver node with minimal parameters
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{
                'robot_ip': robot_ip,
                'token': '',
                'conn_type': 'webrtc',
                'enable_video': enable_video,
                'decode_lidar': False,
                'publish_raw_voxel': True
            },
                {
                "qos_overrides": {
                    "/camera/image_raw": {
                        "publisher": {
                            "reliability": "reliable",
                            "history": "keep_last",
                            "depth": 1,
                        }
                    }
                }
            }],
            remappings=[
                ('cmd_vel_out', 'cmd_vel'),
            ],
        ),

        # Image compression node
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', 'camera/image_raw'),
                ('out/compressed', 'camera/compressed'),
            ],
        ),
    ])
