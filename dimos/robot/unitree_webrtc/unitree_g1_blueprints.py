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

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE, DEFAULT_CAPACITY_DEPTH_IMAGE
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.nav_msgs import Odometry
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.msgs.std_msgs import Bool
from dimos.perception.spatial_perception import spatial_memory
from dimos.robot.foxglove_bridge import foxglove_bridge
from dimos.robot.unitree_webrtc.unitree_g1 import connection
from dimos.robot.unitree_webrtc.rosnav import navigation_module
from dimos.utils.monitoring import utilization
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis
from dimos.navigation.global_planner import astar_planner
from dimos.navigation.local_planner.holonomic_local_planner import (
    holonomic_local_planner,
)
from dimos.navigation.bt_navigator.navigator import (
    behavior_tree_navigator,
)
from dimos.navigation.frontier_exploration import (
    wavefront_frontier_explorer,
)
from dimos.robot.unitree_webrtc.type.map import mapper
from dimos.robot.unitree_webrtc.depth_module import depth_module
from dimos.perception.object_tracker import object_tracking
from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.navigation import navigation_skill
from dimos.robot.unitree_webrtc.g1_joystick_module import g1_joystick
from dimos.robot.unitree_webrtc.unitree_g1_skill_container import g1_skills


# Basic configuration with navigation and visualization
basic = (
    autoconnect(
        # Core connection module for G1
        connection(),
        # SLAM and mapping
        mapper(voxel_size=0.5, global_publish_interval=2.5),
        # Navigation stack
        astar_planner(),
        holonomic_local_planner(),
        behavior_tree_navigator(),
        wavefront_frontier_explorer(),
        navigation_module(),  # G1-specific ROS navigation
        # Visualization
        websocket_vis(),
        foxglove_bridge(),
    )
    .with_global_config(n_dask_workers=4)
    .with_transports(
        {
            # G1 uses TwistStamped for movement commands
            ("cmd_vel", TwistStamped): LCMTransport("/cmd_vel", TwistStamped),
            # State estimation from ROS
            ("state_estimation", Odometry): LCMTransport("/state_estimation", Odometry),
            # Odometry output
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
            # Navigation topics
            ("goal_pose", PoseStamped): LCMTransport("/goal_pose", PoseStamped),
            ("goal_reached", Bool): LCMTransport("/goal_reached", Bool),
            ("cancel_goal", Bool): LCMTransport("/cancel_goal", Bool),
            # Camera topics (if camera module is added)
            ("color_image", Image): LCMTransport("/g1/color_image", Image),
            ("camera_info", CameraInfo): LCMTransport("/g1/camera_info", CameraInfo),
        }
    )
)

# Standard configuration with perception and memory
standard = (
    autoconnect(
        basic,
        spatial_memory(),
        object_tracking(frame_id="camera_link"),
        depth_module(),
        utilization(),
    )
    .with_global_config(n_dask_workers=8)
    .with_transports(
        {
            ("depth_image", Image): LCMTransport("/g1/depth_image", Image),
        }
    )
)

# Optimized configuration using shared memory for images
standard_with_shm = autoconnect(
    standard.with_transports(
        {
            ("color_image", Image): pSHMTransport(
                "/g1/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
            ("depth_image", Image): pSHMTransport(
                "/g1/depth_image", default_capacity=DEFAULT_CAPACITY_DEPTH_IMAGE
            ),
        }
    ),
    foxglove_bridge(
        shm_channels=[
            "/g1/color_image#sensor_msgs.Image",
            "/g1/depth_image#sensor_msgs.Image",
        ]
    ),
)

# Full agentic configuration with LLM and skills
agentic = autoconnect(
    standard,
    llm_agent(),
    human_input(),
    navigation_skill(),
    g1_skills(),  # G1-specific arm and movement mode skills
)

# Configuration with joystick control for teleoperation
with_joystick = autoconnect(
    basic,
    g1_joystick(),  # Pygame-based joystick control
)

# Full featured configuration with everything
full_featured = autoconnect(
    standard_with_shm,
    llm_agent(),
    human_input(),
    navigation_skill(),
    g1_skills(),
    g1_joystick(),
)
