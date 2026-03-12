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

"""
AGIbot Navigation Test Blueprint

Uses ROSNav to bridge the AGIbot's ROS navigation stack into DimOS.
ROSNav subscribes to ROS topics via ROSTransport on its ros_* In ports,
bridges data internally, and publishes to DimOS Out ports on LCMTransport.

Flow: ROS topics → ROSTransport → ROSNav → LCMTransport → lcmspy/rerun

Usage:
    dimos run agibot-nav-test
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport, ROSTransport
from dimos.msgs.geometry_msgs import PoseStamped, Twist, TwistStamped
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import Joy, PointCloud2
from dimos.msgs.std_msgs import Bool, Int8
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.rosnav import ros_nav

agibot_nav_test = autoconnect(
    ros_nav(),
).transports(
    {
        # DimOS Out ports → LCM (visible in lcmspy / rerun)
        ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
        ("global_pointcloud", PointCloud2): LCMTransport("/map", PointCloud2),
        ("goal_req", PoseStamped): LCMTransport("/goal_req", PoseStamped),
        ("goal_active", PoseStamped): LCMTransport("/goal_active", PoseStamped),
        ("path_active", Path): LCMTransport("/path_active", Path),
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        # ROS In ports → ROSTransport (subscribe from AGIbot nav stack)
        ("ros_goal_reached", Bool): ROSTransport("/goal_reached", Bool),
        ("ros_cmd_vel", TwistStamped): ROSTransport("/cmd_vel", TwistStamped),
        ("ros_way_point", PoseStamped): ROSTransport("/way_point", PoseStamped),
        ("ros_registered_scan", PointCloud2): ROSTransport("/registered_scan", PointCloud2),
        ("ros_global_pointcloud", PointCloud2): ROSTransport("/terrain_map_ext", PointCloud2),
        ("ros_path", Path): ROSTransport("/path", Path),
        ("ros_tf", TFMessage): ROSTransport("/tf", TFMessage),
        # ROS Out ports → ROSTransport (publish to AGIbot nav stack)
        ("ros_goal_pose", PoseStamped): ROSTransport("/goal_pose", PoseStamped),
        ("ros_cancel_goal", Bool): ROSTransport("/cancel_goal", Bool),
        ("ros_soft_stop", Int8): ROSTransport("/stop", Int8),
        ("ros_joy", Joy): ROSTransport("/joy", Joy),
    }
)

__all__ = ["agibot_nav_test"]
