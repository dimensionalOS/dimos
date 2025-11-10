#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""Blueprint configurations for Unitree G1 humanoid robot.

This module provides pre-configured blueprints for various G1 robot setups,
from basic teleoperation to full autonomous agent configurations.
"""

from dimos_lcm.sensor_msgs import CameraInfo

from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.navigation import navigation_skill
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule, camera_module
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.nav_msgs import Odometry, Path
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.navigation.bt_navigator.navigator import (
    behavior_tree_navigator,
)
from dimos.navigation.frontier_exploration import (
    wavefront_frontier_explorer,
)
from dimos.navigation.global_planner import astar_planner
from dimos.navigation.local_planner.holonomic_local_planner import (
    holonomic_local_planner,
)
from dimos.navigation.rosnav import ros_nav
from dimos.perception.object_tracker import object_tracking
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.foxglove_bridge import foxglove_bridge
from dimos.robot.unitree_webrtc.g1_joystick_module import g1_joystick
from dimos.robot.unitree_webrtc.type.map import mapper
from dimos.robot.unitree_webrtc.unitree_g1 import g1_connection
from dimos.robot.unitree_webrtc.unitree_g1_skill_container import g1_skills
from dimos.utils.monitoring import utilization
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

# Basic configuration with navigation and visualization
_basic_no_nav = (
    autoconnect(
        # Core connection module for G1
        g1_connection(),
        # Camera module
        camera_module(
            transform=Transform(
                translation=Vector3(0.05, 0.0, 0.0),
                rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
                frame_id="sensor",
                child_frame_id="camera_link",
            ),
            hardware=lambda: Webcam(
                camera_index=0,
                frequency=15,
                stereo_slice="left",
                camera_info=zed.CameraInfo.SingleWebcam,
            ),
        ),
        # SLAM and mapping
        mapper(voxel_size=0.5, global_publish_interval=2.5),
        # Navigation stack
        astar_planner(),
        holonomic_local_planner(),
        wavefront_frontier_explorer(),
        # Visualization
        websocket_vis(),
        foxglove_bridge(),
    )
    .global_config(n_dask_workers=4, robot_model="unitree_g1")
    .remappings(
        [
            (CameraModule, "image", "color_image"),
        ]
    )
    .transports(
        {
            # G1 uses Twist for movement commands
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            # State estimation from ROS
            ("state_estimation", Odometry): LCMTransport("/state_estimation", Odometry),
            # Odometry output from ROSNavigationModule
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            # Navigation module topics from nav_bot
            ("goal_req", PoseStamped): LCMTransport("/goal_req", PoseStamped),
            ("goal_active", PoseStamped): LCMTransport("/goal_active", PoseStamped),
            ("path_active", Path): LCMTransport("/path_active", Path),
            ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("global_pointcloud", PointCloud2): LCMTransport("/map", PointCloud2),
            # Original navigation topics for backwards compatibility
            ("goal_pose", PoseStamped): LCMTransport("/goal_pose", PoseStamped),
            ("goal_reached", Bool): LCMTransport("/goal_reached", Bool),
            ("cancel_goal", Bool): LCMTransport("/cancel_goal", Bool),
            # Camera topics (if camera module is added)
            ("color_image", Image): LCMTransport("/g1/color_image", Image),
            ("camera_info", CameraInfo): LCMTransport("/g1/camera_info", CameraInfo),
        }
    )
)

basic_ros = autoconnect(
    _basic_no_nav,
    ros_nav(),
)

basic_bt_nav = autoconnect(
    _basic_no_nav,
    behavior_tree_navigator(),
)

_perception_and_memory = autoconnect(
    spatial_memory(),
    object_tracking(frame_id="camera_link"),
    utilization(),
)

standard = autoconnect(
    basic_ros,
    _perception_and_memory,
).global_config(n_dask_workers=8)

standard_bt_nav = autoconnect(
    basic_bt_nav,
    _perception_and_memory,
).global_config(n_dask_workers=8)

# Optimized configuration using shared memory for images
standard_with_shm = autoconnect(
    standard.transports(
        {
            ("color_image", Image): pSHMTransport(
                "/g1/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
        }
    ),
    foxglove_bridge(
        shm_channels=[
            "/g1/color_image#sensor_msgs.Image",
        ]
    ),
)

_agentic_skills = autoconnect(
    llm_agent(),
    human_input(),
    navigation_skill(),
    g1_skills(),
)

# Full agentic configuration with LLM and skills
agentic = autoconnect(
    standard,
    _agentic_skills,
)

agentic_bt_nav = autoconnect(
    standard_bt_nav,
    _agentic_skills,
)

# Configuration with joystick control for teleoperation
with_joystick = autoconnect(
    basic_ros,
    g1_joystick(),  # Pygame-based joystick control
)

# Full featured configuration with everything
full_featured = autoconnect(
    standard_with_shm,
    _agentic_skills,
    g1_joystick(),
)
