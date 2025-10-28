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
import threading
import time

# ROS2 message imports
from geometry_msgs.msg import (
    PointStamped as ROSPointStamped,
    PoseStamped as ROSPoseStamped,
    TwistStamped as ROSTwistStamped,
)
from nav_msgs.msg import Odometry as ROSOdometry, Path as ROSPath
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy as ROSJoy, PointCloud2 as ROSPointCloud2
from std_msgs.msg import Bool as ROSBool, Int8 as ROSInt8
from tf2_msgs.msg import TFMessage as ROSTFMessage

from dimos import core
from dimos.core import In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Twist, Vector3
from dimos.msgs.nav_msgs import Odometry, Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.rosnav import ROSNav
from dimos.protocol import pubsub
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger("dimos.robot.unitree_webrtc.nav_bot", level=logging.INFO)


class ROSNavigationModule(ROSNav):
    """
    Handles navigation control and odometry remapping.
    """

    goal_req: In[PoseStamped] = None
    cancel_goal: In[Bool] = None

    pointcloud: Out[PointCloud2] = None
    global_pointcloud: Out[PointCloud2] = None

    goal_active: Out[PoseStamped] = None
    path_active: Out[Path] = None
    goal_reached: Out[Bool] = None
    odom: Out[Odometry] = None
    cmd_vel: Out[Twist] = None
    odom_pose: Out[PoseStamped] = None

    def __init__(self, sensor_to_base_link_transform=None, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("navigation_module")

        self.goal_reach = None
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
        self.goal_pose_pub = self._node.create_publisher(ROSPoseStamped, "/goal_pose", 10)
        self.cancel_goal_pub = self._node.create_publisher(ROSBool, "/cancel_goal", 10)
        self.soft_stop_pub = self._node.create_publisher(ROSInt8, "/soft_stop", 10)
        self.joy_pub = self._node.create_publisher(ROSJoy, "/joy", 10)

        # ROS2 Subscribers
        self.goal_reached_sub = self._node.create_subscription(
            ROSBool, "/goal_reached", self._on_ros_goal_reached, 10
        )
        self.odom_sub = self._node.create_subscription(
            ROSOdometry, "/state_estimation", self._on_ros_odom, 10
        )
        self.cmd_vel_sub = self._node.create_subscription(
            ROSTwistStamped, "/cmd_vel", self._on_ros_cmd_vel, 10
        )
        self.goal_waypoint_sub = self._node.create_subscription(
            ROSPointStamped, "/way_point", self._on_ros_goal_waypoint, 10
        )
        self.registered_scan_sub = self._node.create_subscription(
            ROSPointCloud2, "/registered_scan", self._on_ros_registered_scan, 10
        )
        self.global_pointcloud_sub = self._node.create_subscription(
            ROSPointCloud2, "/terrain_map_ext", self._on_ros_global_pointcloud, 10
        )
        self.path_sub = self._node.create_subscription(ROSPath, "/path", self._on_ros_path, 10)
        self.tf_sub = self._node.create_subscription(ROSTFMessage, "/tf", self._on_ros_tf, 10)

        logger.info("NavigationModule initialized with ROS2 node")

    @rpc
    def start(self) -> None:
        self._running = True
        self.spin_thread = threading.Thread(target=self._spin_node, daemon=True)
        self.spin_thread.start()

        self.goal_req.subscribe(self._on_goal_pose)
        self.cancel_goal.subscribe(self._on_cancel_goal)

        logger.info("NavigationModule started with ROS2 spinning")

    def _spin_node(self) -> None:
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    logger.error(f"ROS2 spin error: {e}")

    def _on_ros_goal_reached(self, msg: ROSBool) -> None:
        self.goal_reach = msg.data
        dimos_bool = Bool(data=msg.data)
        self.goal_reached.publish(dimos_bool)

    def _on_ros_goal_waypoint(self, msg: ROSPointStamped) -> None:
        dimos_pose = PoseStamped(
            ts=time.time(),
            frame_id=msg.header.frame_id,
            position=Vector3(msg.point.x, msg.point.y, msg.point.z),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        self.goal_active.publish(dimos_pose)

    def _on_ros_cmd_vel(self, msg: ROSTwistStamped) -> None:
        # Extract the twist from the stamped message
        dimos_twist = Twist(
            linear=Vector3(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z),
            angular=Vector3(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z),
        )
        self.cmd_vel.publish(dimos_twist)

    def _on_ros_odom(self, msg: ROSOdometry) -> None:
        dimos_odom = Odometry.from_ros_msg(msg)
        self.odom.publish(dimos_odom)

        dimos_pose = PoseStamped(
            ts=dimos_odom.ts,
            frame_id=dimos_odom.frame_id,
            position=dimos_odom.pose.pose.position,
            orientation=dimos_odom.pose.pose.orientation,
        )
        self.odom_pose.publish(dimos_pose)

    def _on_ros_registered_scan(self, msg: ROSPointCloud2) -> None:
        dimos_pointcloud = PointCloud2.from_ros_msg(msg)
        self.pointcloud.publish(dimos_pointcloud)

    def _on_ros_global_pointcloud(self, msg: ROSPointCloud2) -> None:
        dimos_pointcloud = PointCloud2.from_ros_msg(msg)
        self.global_pointcloud.publish(dimos_pointcloud)

    def _on_ros_path(self, msg: ROSPath) -> None:
        dimos_path = Path.from_ros_msg(msg)
        self.path_active.publish(dimos_path)

    def _on_ros_tf(self, msg: ROSTFMessage) -> None:
        ros_tf = TFMessage.from_ros_msg(msg)

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
            ts=time.time(),
        )

        map_to_world_tf = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=euler_to_quaternion(Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )

        self.tf.publish(sensor_to_base_link_tf, map_to_world_tf, *ros_tf.transforms)

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.navigate_to(msg)

    def _on_cancel_goal(self, msg: Bool) -> None:
        if msg.data:
            self.stop()

    def _set_autonomy_mode(self) -> None:
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
        Stop current navigation by publishing to ROS topics.

        Returns:
            True if stop command was sent successfully
        """
        logger.info("Stopping navigation")

        cancel_msg = ROSBool()
        cancel_msg.data = True
        self.cancel_goal_pub.publish(cancel_msg)

        soft_stop_msg = ROSInt8()
        soft_stop_msg.data = 2
        self.soft_stop_pub.publish(soft_stop_msg)

        return True

    @rpc
    def stop(self) -> None:
        try:
            self._running = False
            if self.spin_thread:
                self.spin_thread.join(timeout=1)
            self._node.destroy_node()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")


class NavBot:
    """
    NavBot wrapper that deploys NavigationModule with proper DIMOS/ROS2 integration.
    """

    def __init__(self, dimos=None, sensor_to_base_link_transform=None) -> None:
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

    def start(self) -> None:
        logger.info("Deploying navigation module...")
        self.navigation_module = self.dimos.deploy(
            ROSNavigationModule, sensor_to_base_link_transform=self.sensor_to_base_link_transform
        )

        self.navigation_module.goal_req.transport = core.LCMTransport("/goal", PoseStamped)
        self.navigation_module.cancel_goal.transport = core.LCMTransport("/cancel_goal", Bool)

        self.navigation_module.pointcloud.transport = core.LCMTransport(
            "/pointcloud_map", PointCloud2
        )
        self.navigation_module.global_pointcloud.transport = core.LCMTransport(
            "/global_pointcloud", PointCloud2
        )
        self.navigation_module.goal_active.transport = core.LCMTransport(
            "/goal_active", PoseStamped
        )
        self.navigation_module.path_active.transport = core.LCMTransport("/path_active", Path)
        self.navigation_module.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.navigation_module.odom.transport = core.LCMTransport("/odom", Odometry)
        self.navigation_module.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)
        self.navigation_module.odom_pose.transport = core.LCMTransport("/odom_pose", PoseStamped)

        self.navigation_module.start()

    def shutdown(self) -> None:
        logger.info("Shutting down NavBot...")

        if self.navigation_module:
            self.navigation_module.stop()

        if rclpy.ok():
            rclpy.shutdown()

        logger.info("NavBot shutdown complete")


def main() -> None:
    pubsub.lcm.autoconf()
    nav_bot = NavBot()
    nav_bot.start()

    logger.info("\nTesting navigation in 2 seconds...")
    time.sleep(2)

    test_pose = PoseStamped(
        ts=time.time(),
        frame_id="map",
        position=Vector3(1.0, 1.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
    )

    logger.info("Sending navigation goal to: (1.0, 1.0, 0.0)")

    if nav_bot.navigation_module:
        success = nav_bot.navigation_module.navigate_to(test_pose, timeout=30.0)
        if success:
            logger.info("✓ Navigation goal reached!")
        else:
            logger.warning("✗ Navigation failed or timed out")

    try:
        logger.info("\nNavBot running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        nav_bot.shutdown()


if __name__ == "__main__":
    main()
