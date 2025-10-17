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
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped as ROSPointStamped
from geometry_msgs.msg import PoseStamped as ROSPoseStamped

# ROS2 message imports
from geometry_msgs.msg import TwistStamped as ROSTwistStamped
from nav_msgs.msg import Path as ROSPath
from rclpy.node import Node
from sensor_msgs.msg import Joy as ROSJoy
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2
from std_msgs.msg import Bool as ROSBool
from std_msgs.msg import Int8 as ROSInt8
from tf2_msgs.msg import TFMessage as ROSTFMessage

from dimos.core import DimosCluster, In, LCMTransport, Module, Out, pSHMTransport, rpc
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    TwistStamped,
    Vector3,
)
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger("dimos.robot.unitree_webrtc.nav_bot", level=logging.INFO)


class ROSNav(Module):
    goal_req: In[PoseStamped] = None

    pointcloud: Out[PointCloud2] = None
    global_pointcloud: Out[PointCloud2] = None

    goal_active: Out[PoseStamped] = None
    path_active: Out[Path] = None
    cmd_vel: Out[TwistStamped] = None

    _local_pointcloud: Optional[ROSPointCloud2] = None
    _global_pointcloud: Optional[ROSPointCloud2] = None

    def __init__(self, sensor_to_base_link_transform=None, *args, **kwargs):
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
        self.soft_stop_pub = self._node.create_publisher(ROSInt8, "/soft_stop", 10)
        self.joy_pub = self._node.create_publisher(ROSJoy, "/joy", 10)

        # ROS2 Subscribers
        self.goal_reached_sub = self._node.create_subscription(
            ROSBool, "/goal_reached", self._on_ros_goal_reached, 10
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
    def start(self):
        self._running = True

        # TODO these should be rxpy streams, rxpy has a way to convert callbacks to streams
        def broadcast_lidar():
            while self._running:
                if self._local_pointcloud:
                    self.pointcloud.publish(
                        PointCloud2.from_ros_msg(self._local_pointcloud),
                    )
                time.sleep(0.5)

        def broadcast_map():
            while self._running:
                if self._global_pointcloud:
                    self.global_pointcloud.publish(
                        PointCloud2.from_ros_msg(self._global_pointcloud)
                    )
                time.sleep(1.0)

        self.map_broadcast_thread = threading.Thread(target=broadcast_map, daemon=True)
        self.lidar_broadcast_thread = threading.Thread(target=broadcast_lidar, daemon=True)
        self.map_broadcast_thread.start()
        self.lidar_broadcast_thread.start()
        self.spin_thread = threading.Thread(target=self._spin_node, daemon=True)
        self.spin_thread.start()

        self.goal_req.subscribe(self._on_goal_pose)
        logger.info("NavigationModule started with ROS2 spinning")

    def _spin_node(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    logger.error(f"ROS2 spin error: {e}")

    def _on_ros_goal_reached(self, msg: ROSBool):
        self.goal_reach = msg.data

    def _on_ros_goal_waypoint(self, msg: ROSPointStamped):
        dimos_pose = PoseStamped(
            ts=time.time(),
            frame_id=msg.header.frame_id,
            position=Vector3(msg.point.x, msg.point.y, msg.point.z),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        self.goal_active.publish(dimos_pose)

    def _on_ros_cmd_vel(self, msg: ROSTwistStamped):
        self.cmd_vel.publish(TwistStamped.from_ros_msg(msg))

    def _on_ros_registered_scan(self, msg: ROSPointCloud2):
        self._local_pointcloud = msg

    def _on_ros_global_pointcloud(self, msg: ROSPointCloud2):
        self._global_pointcloud = msg

    def _on_ros_path(self, msg: ROSPath):
        dimos_path = Path.from_ros_msg(msg)
        dimos_path.frame_id = "base_link"
        self.path_active.publish(dimos_path)

    def _on_ros_tf(self, msg: ROSTFMessage):
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

    def _on_goal_pose(self, msg: PoseStamped):
        self.navigate_to(msg)

    def _on_cancel_goal(self, msg: Bool):
        if msg.data:
            self.stop()

    def _set_autonomy_mode(self):
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

        soft_stop_msg = ROSInt8()
        soft_stop_msg.data = 2
        self.soft_stop_pub.publish(soft_stop_msg)

        return True

    @rpc
    def stop(self):
        try:
            self._running = False
            if self.spin_thread:
                self.spin_thread.join(timeout=1)
            self._node.destroy_node()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")


def deploy(dimos: DimosCluster):
    nav = dimos.deploy(ROSNav)
    nav.pointcloud.transport = pSHMTransport("/lidar")
    nav.global_pointcloud.transport = pSHMTransport("/map")
    # nav.pointcloud.transport = LCMTransport("/lidar", PointCloud2)
    # nav.global_pointcloud.transport = LCMTransport("/map", PointCloud2)

    nav.goal_req.transport = LCMTransport("/goal_req", PoseStamped)
    nav.goal_req.transport = LCMTransport("/goal_req", PoseStamped)
    nav.goal_active.transport = LCMTransport("/goal_active", PoseStamped)
    nav.path_active.transport = LCMTransport("/path_active", Path)
    nav.cmd_vel.transport = LCMTransport("/cmd_vel", TwistStamped)
    nav.start()
    return nav
