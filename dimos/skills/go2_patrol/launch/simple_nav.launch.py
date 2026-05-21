from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get Nav2 bringup package
    nav2_bringup_dir = FindPackageShare("nav2_bringup")

    # Parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Nav2 params file path (using default Nav2 params)
    params_file = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation clock if true"
            ),
            # Launch Nav2 navigation stack
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([nav2_bringup_dir, "launch", "navigation_launch.py"])]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),
            # RViz for visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([nav2_bringup_dir, "rviz", "nav2_default_view.rviz"]),
                ],
                output="screen",
            ),
        ]
    )
