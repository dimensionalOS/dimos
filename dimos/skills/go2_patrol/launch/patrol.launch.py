import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory("go2_patrol")

    # Declare launch arguments
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=os.path.join(pkg_dir, "config", "waypoints.yaml"),
        description="Path to waypoints YAML file",
    )

    loop_patrol_arg = DeclareLaunchArgument(
        "loop_patrol",
        default_value="true",
        description="Whether to continuously loop the patrol route",
    )

    stop_duration_arg = DeclareLaunchArgument(
        "stop_duration", default_value="5.0", description="Time in seconds to stop at each waypoint"
    )

    max_retries_arg = DeclareLaunchArgument(
        "max_retries",
        default_value="3",
        description="Maximum number of retries for failed waypoints",
    )

    # Create the patrol node
    patrol_node = Node(
        package="go2_patrol",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[
            {
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                "loop_patrol": LaunchConfiguration("loop_patrol"),
                "stop_duration": LaunchConfiguration("stop_duration"),
                "max_retries": LaunchConfiguration("max_retries"),
            }
        ],
    )

    return LaunchDescription(
        [
            waypoints_file_arg,
            loop_patrol_arg,
            stop_duration_arg,
            max_retries_arg,
            patrol_node,
        ]
    )
