#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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
Relay node that publishes Joy message to enable autonomy mode when goal_pose is received.
Mimics the behavior of the goalpoint_rviz_plugin for Foxglove compatibility.
"""

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


class GoalAutonomyRelay(Node):
    def __init__(self):
        super().__init__("goal_autonomy_relay")

        # QoS for goal topics (match foxglove_bridge)
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # Subscribe to goal_pose (PoseStamped from Foxglove)
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_pose_callback, goal_qos
        )

        # Subscribe to way_point (PointStamped from Foxglove)
        self.point_sub = self.create_subscription(
            PointStamped, "/way_point", self.way_point_callback, goal_qos
        )

        # Publisher for Joy message to enable autonomy
        self.joy_pub = self.create_publisher(Joy, "/joy", 5)

        self.get_logger().info(
            "Goal autonomy relay started - will publish Joy to enable autonomy when goals are received"
        )

    def publish_autonomy_joy(self):
        """Publish Joy message that enables autonomy mode (mimics goalpoint_rviz_plugin)"""
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = "goal_autonomy_relay"

        # axes[2] = -1.0 enables autonomy mode in pathFollower
        # axes[4] = 1.0 sets forward speed
        # axes[5] = 1.0 enables obstacle checking
        joy.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]

        # buttons[7] = 1 (same as RViz plugin)
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]

        self.joy_pub.publish(joy)
        self.get_logger().info("Published Joy message to enable autonomy mode")

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received goal_pose at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        self.publish_autonomy_joy()

    def way_point_callback(self, msg: PointStamped):
        self.get_logger().info(f"Received way_point at ({msg.point.x:.2f}, {msg.point.y:.2f})")
        self.publish_autonomy_joy()


def main(args=None):
    rclpy.init(args=args)
    node = GoalAutonomyRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
