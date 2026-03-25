import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import launch.events
import launch_ros
import lifecycle_msgs.msg

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    # Get robot config from environment variable or use default
    robot_config_env = os.environ.get('ROBOT_CONFIG_PATH', 'mechanum_drive')
    map_path_env = os.environ.get('MAP_PATH', '')

    config_path = get_share_file(
        package_name="arise_slam_mid360",
        file_name="config/livox_mid360.yaml")
    calib_path = get_share_file(
        package_name="arise_slam_mid360",
        file_name="config/livox/livox_mid360_calibration.yaml"
    )

    config_path_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_path,
        description="Path to config file for arise_slam"
    )

    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=robot_config_env,
        description='Robot configuration file name (without .yaml)'
    )
    calib_path_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib_path,
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="integrated_to_init"
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="map",
    )
    world_frame_rot_arg = DeclareLaunchArgument(
        "world_frame_rot",
        default_value="map_rot",
    )
    sensor_frame_arg = DeclareLaunchArgument(
        "sensor_frame",
        default_value="sensor",
    )
    sensor_frame_rot_arg = DeclareLaunchArgument(
        "sensor_frame_rot",
        default_value="sensor_rot",
    )

    # Localization arguments
    local_mode_arg = DeclareLaunchArgument(
        "local_mode",
        default_value='true' if map_path_env else 'false',
        description="Enable localization mode (vs mapping mode). Auto-detected from MAP_DIR env variable."
    )

    relocalization_map_path_arg = DeclareLaunchArgument(
        "relocalization_map_path",
        default_value=map_path_env + '.pcd' if map_path_env else '',
        description="Path to map file for localization (.pcd or .txt). Auto-set from MAP_DIR/map.pcd if MAP_DIR exists."
    )

    init_x_arg = DeclareLaunchArgument("init_x", default_value="0.0")
    init_y_arg = DeclareLaunchArgument("init_y", default_value="0.0")
    init_z_arg = DeclareLaunchArgument("init_z", default_value="0.0")
    init_roll_arg = DeclareLaunchArgument("init_roll", default_value="0.0")
    init_pitch_arg = DeclareLaunchArgument("init_pitch", default_value="0.0")
    init_yaw_arg = DeclareLaunchArgument("init_yaw", default_value="0.0")

    feature_extraction_node = LifecycleNode(
        package="arise_slam_mid360",
        namespace="",
        executable="feature_extraction_node",
        name="feature_extraction_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[
            LaunchConfiguration("config_file"),
            PythonExpression([
                "'", FindPackageShare('local_planner'), "/config/",
                LaunchConfiguration('robot_config'), ".yaml'"
            ]),
            { "calibration_file": LaunchConfiguration("calibration_file"),
        }],
    )

    laser_mapping_node = LifecycleNode(
        package="arise_slam_mid360",
        namespace="",
        executable="laser_mapping_node",
        name="laser_mapping_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[LaunchConfiguration("config_file"),
            {
                "calibration_file": LaunchConfiguration("calibration_file"),
                "local_mode": LaunchConfiguration("local_mode"),
                "relocalization_map_path": LaunchConfiguration("relocalization_map_path"),
                "init_x": LaunchConfiguration("init_x"),
                "init_y": LaunchConfiguration("init_y"),
                "init_z": LaunchConfiguration("init_z"),
                "init_roll": LaunchConfiguration("init_roll"),
                "init_pitch": LaunchConfiguration("init_pitch"),
                "init_yaw": LaunchConfiguration("init_yaw"),
        }],
        remappings=[
            ("laser_odom_to_init", LaunchConfiguration("odom_topic")),
        ]
    )

    imu_preintegration_node = LifecycleNode(
        package="arise_slam_mid360",
        namespace="",
        executable="imu_preintegration_node",
        name="imu_preintegration_node",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[
            LaunchConfiguration("config_file"),
            PythonExpression([
                "'", FindPackageShare('local_planner'), "/config/",
                LaunchConfiguration('robot_config'), ".yaml'"
            ]),
            { "calibration_file": LaunchConfiguration("calibration_file")
        }],
    )

    # Auto-configure after delay (M20 ARM64: DDS discovery needs >5s)
    configure_feature = TimerAction(period=5.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(feature_extraction_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))
    ])

    activate_feature = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=feature_extraction_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(feature_extraction_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))],
        handle_once=True,
    ))

    configure_mapping = TimerAction(period=10.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(laser_mapping_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))
    ])

    activate_mapping = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=laser_mapping_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(laser_mapping_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))],
        handle_once=True,
    ))

    configure_imu = TimerAction(period=15.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(imu_preintegration_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ))
    ])

    activate_imu = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=imu_preintegration_node,
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(imu_preintegration_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))],
        handle_once=True,
    ))

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value='false'),
        config_path_arg,
        robot_config_arg,
        calib_path_arg,
        odom_topic_arg,
        world_frame_arg,
        world_frame_rot_arg,
        sensor_frame_arg,
        sensor_frame_rot_arg,
        local_mode_arg,
        relocalization_map_path_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_roll_arg,
        init_pitch_arg,
        init_yaw_arg,
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
        # Auto-transition event handlers
        activate_feature,
        activate_mapping,
        activate_imu,
        # Timed configure triggers
        configure_feature,
        configure_mapping,
        configure_imu,
    ])
