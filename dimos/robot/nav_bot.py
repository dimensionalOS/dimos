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
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from dimos import core
from dimos.core import Module, In, Out, rpc
from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs import PoseStamped, TwistStamped, Transform, Vector3
from dimos.msgs.nav_msgs import Odometry
from dimos.msgs.sensor_msgs import PointCloud2, Joy, Image
from dimos.msgs.std_msgs import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf import TF
from dimos.utils.transform_utils import euler_to_quaternion
from dimos.utils.logging_config import setup_logger

# ROS2 message imports
from geometry_msgs.msg import TwistStamped as ROSTwistStamped
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import PointStamped as ROSPointStamped
from nav_msgs.msg import Odometry as ROSOdometry
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2, Joy as ROSJoy, Image as ROSImage
from std_msgs.msg import Bool as ROSBool
from tf2_msgs.msg import TFMessage as ROSTFMessage
from dimos.utils.logging_config import setup_logger
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from reactivex.disposable import Disposable

logger = setup_logger("dimos.robot.unitree_webrtc.nav_bot", level=logging.INFO)


class NavigationModule(Module, Node):
    """
    Handles navigation control and odometry remapping.
    """

    goal_req: In[PoseStamped] = None
    cancel_goal: In[Bool] = None
    soft_stop: In[Int8] = None

    pointcloud: Out[PointCloud2] = None
    goal_active: Out[PoseStamped] = None
    goal_reached: Out[Bool] = None
    odom: Out[Odometry] = None
    cmd_vel: Out[TwistStamped] = None
    odom_pose: Out[PoseStamped] = None

    def __init__(self, sensor_to_base_link_transform=None, *args, **kwargs):
        """Initialize NavigationModule."""
        Module.__init__(self, *args, **kwargs)
        if not rclpy.ok():
            rclpy.init()
        Node.__init__(self, "navigation_module")

        self.goal_reach = None
        self.tf = TF()
        self.sensor_to_base_link_transform = sensor_to_base_link_transform or [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.spin_thread = None

        # ROS2 Publishers
        self.goal_pose_pub = self.create_publisher(ROSPoseStamped, "/goal_pose", 10)
        self.cancel_goal_pub = self.create_publisher(ROSBool, "/cancel_goal", 10)
        self.soft_stop_pub = self.create_publisher(ROSInt8, "/soft_stop", 10)
        self.joy_pub = self.create_publisher(ROSJoy, "/joy", 10)

        # ROS2 Subscribers
        self.goal_reached_sub = self.create_subscription(
            ROSBool, "/goal_reached", self._on_ros_goal_reached, 10
        )
        self.odom_sub = self.create_subscription(
            ROSOdometry, "/state_estimation", self._on_ros_odom, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            ROSTwistStamped, "/cmd_vel", self._on_ros_cmd_vel, 10
        )
        self.goal_waypoint_sub = self.create_subscription(
            ROSPointStamped, "/way_point", self._on_ros_goal_waypoint, 10
        )
        self.registered_scan_sub = self.create_subscription(
            ROSPointCloud2, "/registered_scan", self._on_ros_registered_scan, 10
        )

        logger.info("NavigationModule initialized with ROS2 node")

    @rpc
    def start(self):
        super().start()
        if self.goal_reached:
            unsub = self.goal_reached.subscribe(self._on_goal_reached)
            self._disposables.add(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()

        logger.info("NavigationModule started with ROS2 spinning")

    def _spin_node(self):
        """Spin the ROS2 node to process callbacks."""
        import rclpy

        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    logger.error(f"ROS2 spin error: {e}")

    def _on_ros_goal_reached(self, msg: ROSBool):
        """Bridge ROS goal_reached to DIMOS."""
        self.goal_reach = msg.data
        dimos_bool = Bool(data=msg.data)
        self.goal_reached.publish(dimos_bool)

    def _on_ros_goal_waypoint(self, msg: ROSPointStamped):
        """Convert PointStamped waypoint to PoseStamped and publish as active goal."""
        dimos_pose = PoseStamped(
            ts=time.time(),
            frame_id=msg.header.frame_id,
            position=Vector3(msg.point.x, msg.point.y, msg.point.z),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        self.goal_active.publish(dimos_pose)

    def _on_ros_cmd_vel(self, msg: ROSTwistStamped):
        """Bridge ROS cmd_vel to DIMOS."""
        dimos_twist = TwistStamped.from_ros_msg(msg)
        self.cmd_vel.publish(dimos_twist)

    def _on_ros_odom(self, msg: ROSOdometry):
        """Bridge ROS odometry to DIMOS and remap to PoseStamped."""
        dimos_odom = Odometry.from_ros_msg(msg)
        self.odom.publish(dimos_odom)

        dimos_pose = PoseStamped(
            ts=dimos_odom.ts,
            frame_id=dimos_odom.frame_id,
            position=dimos_odom.pose.pose.position,
            orientation=dimos_odom.pose.pose.orientation,
        )
        self.odom_pose.publish(dimos_pose)

    def _on_ros_registered_scan(self, msg: ROSPointCloud2):
        dimos_pointcloud = PointCloud2.from_ros_msg(msg)
        self.pointcloud.publish(dimos_pointcloud)

    def _on_ros_tf(self, msg: ROSTFMessage):
        """Bridge ROS TF to DIMOS."""
        ros_tf = TFMessage.from_ros_msg(msg)

        # Publish static transforms
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

        map_to_world_tf = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=euler_to_quaternion(Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
            child_frame_id="world",
            ts=msg.ts,
        )

        self.tf.publish(sensor_to_base_link_tf, map_to_world_tf, ros_tf)

    def _on_goal_pose(self, msg: PoseStamped):
        """Handle DIMOS goal_pose by calling navigate_to."""
        self.navigate_to(msg)

    def _on_cancel_goal(self, msg: Bool):
        """Handle DIMOS cancel_goal by calling stop."""
        if msg.data:
            self.stop()

    def _on_soft_stop(self, msg: Int8):
        """Handle DIMOS soft_stop and publish to ROS."""
        ros_int8 = ROSInt8()
        ros_int8.data = msg.data
        self.soft_stop_pub.publish(ros_int8)

    def _set_autonomy_mode(self):
        """Set autonomy mode by publishing Joy message to ROS."""
        joy_msg = ROSJoy()
        joy_msg.axes = [
            0.0,  # axis 0
            0.0,  # axis 1
            -1.0,  # axis 2
            0.0,  # axis 3
            1.0,  # axis 4
            1.0,  # axis 5
            0.0,  # axis 6
            0.0,  # axis 7
        ]
        joy_msg.buttons = [
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
        ]
        self.joy_pub.publish(joy_msg)
        logger.info("Setting autonomy mode via Joy message")

    @rpc
    def navigate_to(self, pose: PoseStamped, timeout: float = 60.0) -> bool:
        """
        Navigate to a target pose by publishing to ROS topics.

        Args:
            pose: Target pose to navigate to
            timeout: Maximum time to wait for goal (seconds)

        Returns:
            True if navigation was successful
        """
        logger.info(
            f"Navigating to goal: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )

        self.goal_reach = None
        self._set_autonomy_mode()

        # Enable soft stop (0 = enable)
        soft_stop_msg = ROSInt8()
        soft_stop_msg.data = 0
        self.soft_stop_pub.publish(soft_stop_msg)

        ros_pose = pose.to_ros_msg()
        self.goal_pose_pub.publish(ros_pose)

        # Wait for goal to be reached
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.goal_reach is not None:
                soft_stop_msg.data = 2
                self.soft_stop_pub.publish(soft_stop_msg)
                return self.goal_reach
            time.sleep(0.1)

        self.stop_navigation()

        logger.warning(f"Navigation timed out after {timeout} seconds")
        return False

    @rpc
    def stop_navigation(self) -> bool:
        """
        Cancel current navigation by publishing to ROS topics.

        Returns:
            True if cancel command was sent successfully
        """
        logger.info("Cancelling navigation")

        cancel_msg = ROSBool()
        cancel_msg.data = True
        self.cancel_goal_pub.publish(cancel_msg)

        soft_stop_msg = ROSInt8()
        soft_stop_msg.data = 2
        self.soft_stop_pub.publish(soft_stop_msg)

        return True

    def shutdown(self):
        """Cleanup the module's ROS2 resources."""
        try:
            self._running = False
            if self.spin_thread:
                self.spin_thread.join(timeout=1)
            self.destroy_node()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")


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
        super().start()
        unsub = self.odom.subscribe(self._publish_odom_pose)
        self._disposables.add(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()

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


class NavBot(Resource):
    """
    NavBot wrapper that deploys NavigationModule with proper DIMOS/ROS2 integration.
    """

    def __init__(self, dimos=None, sensor_to_base_link_transform=None):
        """
        Initialize NavBot.

        Args:
            dimos: DIMOS instance (creates new one if None)
            sensor_to_base_link_transform: Optional [x, y, z, roll, pitch, yaw] transform
        """
        if dimos is None:
            self.dimos = core.start(2)
        else:
            self.dimos = dimos

        self.sensor_to_base_link_transform = sensor_to_base_link_transform or [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.navigation_module = None

    def start(self):
        super().start()

        if self.topic_remap_module:
            self.topic_remap_module.start()
            logger.info("Topic remap module started")

        if self.ros_bridge:
            logger.info("ROS bridge started")

    def stop(self) -> None:
        logger.info("Shutting down navigation modules...")

        if self.ros_bridge is not None:
            try:
                self.ros_bridge.shutdown()
                logger.info("ROS bridge shut down successfully")
            except Exception as e:
                logger.error(f"Error shutting down ROS bridge: {e}")

        super().stop()

    def deploy_navigation_modules(self, bridge_name="nav_bot_ros_bridge"):
        # Deploy topic remap module
        logger.info("Deploying topic remap module...")
        self.topic_remap_module = self.dimos.deploy(
            TopicRemapModule, sensor_to_base_link_transform=self.sensor_to_base_link_transform
        )

        self.navigation_module.goal_req.transport = core.LCMTransport("/goal", PoseStamped)
        self.navigation_module.cancel_goal.transport = core.LCMTransport("/cancel_goal", Bool)
        self.navigation_module.soft_stop.transport = core.LCMTransport("/soft_stop", Int8)

        self.navigation_module.pointcloud.transport = core.LCMTransport(
            "/pointcloud_map", PointCloud2
        )
        self.navigation_module.goal_active.transport = core.LCMTransport(
            "/goal_active", PoseStamped
        )
        self.navigation_module.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.navigation_module.odom.transport = core.LCMTransport("/odom", Odometry)
        self.navigation_module.cmd_vel.transport = core.LCMTransport("/cmd_vel", TwistStamped)
        self.navigation_module.odom_pose.transport = core.LCMTransport("/odom_pose", PoseStamped)

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

        # Start the navigation module
        if self.navigation_module:
            self.navigation_module.start()
            logger.info("Navigation module started")

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
