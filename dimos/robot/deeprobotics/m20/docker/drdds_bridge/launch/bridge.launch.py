from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():
    bridge_lib = os.path.join(
        os.environ.get('DRDDS_BRIDGE_PATH', '/opt/drdds_bridge'),
        'lib', 'drdds_bridge')

    # Process A: drdds receiver (standalone binary, NOT a ROS2 node)
    # Must set LD_LIBRARY_PATH to host drdds libs
    drdds_recv = ExecuteProcess(
        cmd=[os.path.join(bridge_lib, 'drdds_recv')],
        name='drdds_recv',
        output='screen',
        additional_env={
            'LD_LIBRARY_PATH': '/opt/drdds-host/lib:'
            + os.environ.get('LD_LIBRARY_PATH', ''),
        },
    )

    # Process B: ROS2 publisher (standard ROS2 node)
    # Delayed 1s to let drdds_recv create SHM (ros2_pub retries if SHM not ready)
    ros2_pub = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='drdds_bridge',
                executable='ros2_pub',
                name='drdds_bridge',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([drdds_recv, ros2_pub])
