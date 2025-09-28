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
import time

from dimos import core
from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped, Transform, Vector3
from dimos.msgs.nav_msgs import Odometry
from dimos.msgs.sensor_msgs import PointCloud2, Joy, Image
from dimos.msgs.std_msgs import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf import TF
from dimos.robot.ros_bridge import ROSBridge, BridgeDirection
from dimos.utils.transform_utils import euler_to_quaternion
from geometry_msgs.msg import TwistStamped as ROSTwistStamped
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from nav_msgs.msg import Odometry as ROSOdometry
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2, Joy as ROSJoy, Image as ROSImage
from std_msgs.msg import Bool as ROSBool
from tf2_msgs.msg import TFMessage as ROSTFMessage
from dimos.utils.logging_config import setup_logger
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

logger = setup_logger("dimos.robot.unitree_webrtc.nav_bot", level=logging.INFO)

############################################################
# Navigation Module

# first run unitree_g1.py to start the ROS bridge and webrtc connection and teleop
# python dimos/robot/unitree_webrtc/unitree_g1.py


# then deploy this module in any other run file.
############################################################
class NavigationModule(Module):
    goal_reached: In[Bool] = None

    goal_pose: Out[PoseStamped] = None
    cancel_goal: Out[Bool] = None
    joy: Out[Joy] = None

    def __init__(self, *args, **kwargs):
        """Initialize NavigationModule."""
        Module.__init__(self, *args, **kwargs)
        self.goal_reach = None

    @rpc
    def start(self):
        """Start the navigation module."""
        if self.goal_reached:
            self.goal_reached.subscribe(self._on_goal_reached)
        logger.info("NavigationModule started")

    def _on_goal_reached(self, msg: Bool):
        """Handle goal reached status messages."""
        self.goal_reach = msg.data

    def _set_autonomy_mode(self):
        """
        Set autonomy mode by publishing Joy message.
        """

        joy_msg = Joy(
            frame_id="dimos",
            axes=[
                0.0,  # axis 0
                0.0,  # axis 1
                -1.0,  # axis 2
                0.0,  # axis 3
                1.0,  # axis 4
                1.0,  # axis 5
                0.0,  # axis 6
                0.0,  # axis 7
            ],
            buttons=[
                0,  # button 0
                0,  # button 1
                0,  # button 2
                0,  # button 3
                0,  # button 4
                0,  # button 5
                0,  # button 6
                1,  # button 7 - controls autonomy mode
                0,  # button 8
                0,  # button 9
                0,  # button 10
            ],
        )

        if self.joy:
            self.joy.publish(joy_msg)
            logger.info(f"Setting autonomy mode via Joy message")

    @rpc
    def go_to(self, pose: PoseStamped, timeout: float = 60.0) -> bool:
        """
        Navigate to a target pose by publishing to LCM topics.

        Args:
            pose: Target pose to navigate to
            blocking: If True, block until goal is reached
            timeout: Maximum time to wait for goal (seconds)

        Returns:
            True if navigation was successful (or started if non-blocking)
        """
        logger.info(
            f"Navigating to goal: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )

        self.goal_reach = None
        self._set_autonomy_mode()
        self.goal_pose.publish(pose)

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.goal_reach is not None:
                return self.goal_reach
            time.sleep(0.1)

        self.stop()

        logger.warning(f"Navigation timed out after {timeout} seconds")
        return False

    @rpc
    def stop(self) -> bool:
        """
        Cancel current navigation by publishing to cancel_goal.

        Returns:
            True if cancel command was sent successfully
        """
        logger.info("Cancelling navigation")

        if self.cancel_goal:
            cancel_msg = Bool(data=True)
            self.cancel_goal.publish(cancel_msg)
            return True

        return False


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

        sensor_to_base_link_tf = Transform(
            translation=translation,
            rotation=rotation,
            frame_id="sensor",
            child_frame_id="base_link",
            ts=msg.ts,
        )

        # map to world static transform
        map_to_world_tf = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=euler_to_quaternion(Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
            child_frame_id="world",
            ts=msg.ts,
        )

        self.tf.publish(sensor_to_base_link_tf, map_to_world_tf)


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
        self.lcm = LCM()

    def start(self):
        if self.topic_remap_module:
            self.topic_remap_module.start()
            logger.info("Topic remap module started")

        if self.ros_bridge:
            logger.info("ROS bridge started")

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
        self.ros_bridge.add_topic("/joy", Joy, ROSJoy, direction=BridgeDirection.DIMOS_TO_ROS)
        # Navigation control topics from autonomy stack
        self.ros_bridge.add_topic(
            "/goal_pose", PoseStamped, ROSPoseStamped, direction=BridgeDirection.DIMOS_TO_ROS
        )
        self.ros_bridge.add_topic(
            "/cancel_goal", Bool, ROSBool, direction=BridgeDirection.DIMOS_TO_ROS
        )
        self.ros_bridge.add_topic(
            "/goal_reached", Bool, ROSBool, direction=BridgeDirection.ROS_TO_DIMOS
        )

        # self.ros_bridge.add_topic(
        #     "/camera/image", Image, ROSImage, direction=BridgeDirection.ROS_TO_DIMOS
        # )

    def _set_autonomy_mode(self):
        """
        Set autonomy mode by publishing Joy message.
        """

        joy_msg = Joy(
            frame_id="dimos",
            axes=[
                0.0,  # axis 0
                0.0,  # axis 1
                -1.0,  # axis 2
                0.0,  # axis 3
                1.0,  # axis 4
                1.0,  # axis 5
                0.0,  # axis 6
                0.0,  # axis 7
            ],
            buttons=[
                0,  # button 0
                0,  # button 1
                0,  # button 2
                0,  # button 3
                0,  # button 4
                0,  # button 5
                0,  # button 6
                1,  # button 7 - controls autonomy mode
                0,  # button 8
                0,  # button 9
                0,  # button 10
            ],
        )

        self.lcm.publish(Topic("/joy", Joy), joy_msg)

    def navigate_to_goal(self, pose: PoseStamped, blocking: bool = True, timeout: float = 30.0):
        """Navigate to a target pose using ROS topics.

        Args:
            pose: Target pose to navigate to
            blocking: If True, block until goal is reached. If False, return immediately.
            timeout: Maximum time to wait for goal to be reached (seconds)

        Returns:
            If blocking=True: True if navigation was successful, False otherwise
            If blocking=False: True if goal was sent successfully
        """
        logger.info(
            f"Navigating to goal: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )

        # Publish goal to /goal_pose topic
        self._set_autonomy_mode()
        goal_topic = Topic("/goal_pose", PoseStamped)
        self.lcm.publish(goal_topic, pose)

        if not blocking:
            return True

        # Wait for goal_reached signal
        goal_reached_topic = Topic("/goal_reached", Bool)
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                msg = self.lcm.wait_for_message(goal_reached_topic, timeout=0.5)
                if msg and msg.data:
                    logger.info("Navigation goal reached")
                    return True
                elif msg and not msg.data:
                    logger.info("Navigation was cancelled or failed")
                    return False
            except Exception:
                # Timeout on wait_for_message, continue looping
                pass

        logger.warning(f"Navigation timed out after {timeout} seconds")
        return False

    def cancel_navigation(self) -> bool:
        """Cancel the current navigation goal using ROS topics.

        Returns:
            True if cancel command was sent successfully
        """
        logger.info("Cancelling navigation goal")

        # Publish cancel command to /cancel_goal topic
        cancel_topic = Topic("/cancel_goal", Bool)
        cancel_msg = Bool(data=True)
        self.lcm.publish(cancel_topic, cancel_msg)

        return True

    def shutdown(self):
        logger.info("Shutting down navigation modules...")

        if self.ros_bridge is not None:
            try:
                self.ros_bridge.shutdown()
                logger.info("ROS bridge shut down successfully")
            except Exception as e:
                logger.error(f"Error shutting down ROS bridge: {e}")
