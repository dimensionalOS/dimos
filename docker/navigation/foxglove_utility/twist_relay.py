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
Simple relay node that converts geometry_msgs/Twist to geometry_msgs/TwistStamped.
Used for Foxglove Teleop panel which only publishes Twist.
"""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")

        # Declare parameters
        self.declare_parameter("input_topic", "/foxglove_teleop")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("frame_id", "vehicle")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        # QoS for real-time control
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribe to Twist (from Foxglove Teleop)
        self.subscription = self.create_subscription(Twist, input_topic, self.twist_callback, qos)

        # Publish TwistStamped
        self.publisher = self.create_publisher(TwistStamped, output_topic, qos)

        self.get_logger().info(
            f"Twist relay: {input_topic} (Twist) -> {output_topic} (TwistStamped)"
        )

    def twist_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
