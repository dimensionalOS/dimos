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

"""
NavBot class for navigation-related functionality.
Encapsulates ROS bridge and topic remapping for Unitree robots.
"""

import logging

from dimos import core
from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped, Transform, Vector3
from dimos.msgs.nav_msgs import Odometry
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf import TF
from dimos.robot.ros_bridge import ROSBridge, BridgeDirection
from dimos.utils.transform_utils import euler_to_quaternion
from geometry_msgs.msg import TwistStamped as ROSTwistStamped
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from nav_msgs.msg import Odometry as ROSOdometry
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2
from tf2_msgs.msg import TFMessage as ROSTFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.nav_bot", level=logging.INFO)


class TopicRemapModule(Module):
    """Module that remaps Odometry to PoseStamped and publishes static transforms."""

    odom: In[Odometry] = None
    odom_pose: Out[PoseStamped] = None

    def __init__(self, sensor_to_base_link_transform=None, *args, **kwargs):
        Module.__init__(self, *args, **kwargs)
        self.tf = TF()
        self.sensor_to_base_link_transform = sensor_to_base_link_transform or [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

    @rpc
    def start(self):
        self.odom.subscribe(self._publish_odom_pose)
        logger.info("TopicRemapModule started")

    def _publish_odom_pose(self, msg: Odometry):
        pose_msg = PoseStamped(
            ts=msg.ts,
            frame_id=msg.frame_id,
            position=msg.pose.pose.position,
            orientation=msg.pose.pose.orientation,
        )
        self.odom_pose.publish(pose_msg)

        # Publish static transform from sensor to base_link
        translation = Vector3(
            self.sensor_to_base_link_transform[0],
            self.sensor_to_base_link_transform[1],
            self.sensor_to_base_link_transform[2],
        )
        euler_angles = Vector3(
            self.sensor_to_base_link_transform[3],
            self.sensor_to_base_link_transform[4],
            self.sensor_to_base_link_transform[5],
        )
        rotation = euler_to_quaternion(euler_angles)

        static_tf = Transform(
            translation=translation,
            rotation=rotation,
            frame_id="sensor",
            child_frame_id="base_link",
            ts=msg.ts,
        )
        self.tf.publish(static_tf)


class NavBot:
    """
    NavBot class for navigation-related functionality.
    Manages ROS bridge and topic remapping for navigation.
    """

    def __init__(self, dimos=None, sensor_to_base_link_transform=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        """
        Initialize NavBot.

        Args:
            dimos: DIMOS instance (creates new one if None)
            sensor_to_base_link_transform: Optional [x, y, z, roll, pitch, yaw] transform from sensor to base_link
        """
        if dimos is None:
            self.dimos = core.start(2)
        else:
            self.dimos = dimos

        self.sensor_to_base_link_transform = sensor_to_base_link_transform
        self.ros_bridge = None
        self.topic_remap_module = None
        self.tf = TF()

    def deploy_navigation_modules(self, bridge_name="nav_bot_ros_bridge"):
        # Deploy topic remap module
        logger.info("Deploying topic remap module...")
        self.topic_remap_module = self.dimos.deploy(
            TopicRemapModule, sensor_to_base_link_transform=self.sensor_to_base_link_transform
        )
        self.topic_remap_module.odom.transport = core.LCMTransport("/odom", Odometry)
        self.topic_remap_module.odom_pose.transport = core.LCMTransport("/odom_pose", PoseStamped)

        # Deploy ROS bridge
        logger.info("Deploying ROS bridge...")
        self.ros_bridge = ROSBridge(bridge_name)

        # Configure ROS topics
        self.ros_bridge.add_topic(
            "/cmd_vel", TwistStamped, ROSTwistStamped, direction=BridgeDirection.ROS_TO_DIMOS
        )
        self.ros_bridge.add_topic(
            "/state_estimation",
            Odometry,
            ROSOdometry,
            direction=BridgeDirection.ROS_TO_DIMOS,
            remap_topic="/odom",
        )
        self.ros_bridge.add_topic(
            "/tf", TFMessage, ROSTFMessage, direction=BridgeDirection.ROS_TO_DIMOS
        )
        self.ros_bridge.add_topic(
            "/registered_scan", PointCloud2, ROSPointCloud2, direction=BridgeDirection.ROS_TO_DIMOS
        )
        self.ros_bridge.add_topic(
            "/odom_pose", PoseStamped, ROSPoseStamped, direction=BridgeDirection.DIMOS_TO_ROS
        )

    def start_navigation_modules(self):
        if self.topic_remap_module:
            self.topic_remap_module.start()
            logger.info("Topic remap module started")

        if self.ros_bridge:
            logger.info("ROS bridge started")

    def shutdown_navigation(self):
        logger.info("Shutting down navigation modules...")

        if self.ros_bridge is not None:
            try:
                self.ros_bridge.shutdown()
                logger.info("ROS bridge shut down successfully")
            except Exception as e:
                logger.error(f"Error shutting down ROS bridge: {e}")
