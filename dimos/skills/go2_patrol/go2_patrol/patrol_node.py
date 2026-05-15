#!/usr/bin/env python3

import os
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


class PatrolNode(Node):
    """
    Autonomous patrol node for Unitree Go2 robot.

    This node navigates the robot through a series of waypoints in a loop,
    stopping at each waypoint for a configurable duration.
    """

    def __init__(self):
        super().__init__("go2_patrol_node")

        # Declare parameters
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("loop_patrol", True)
        self.declare_parameter("stop_duration", 5.0)  # seconds to stop at each waypoint
        self.declare_parameter("max_retries", 3)

        # Get parameters
        waypoints_file = self.get_parameter("waypoints_file").get_parameter_value().string_value
        self.loop_patrol = self.get_parameter("loop_patrol").get_parameter_value().bool_value
        self.stop_duration = self.get_parameter("stop_duration").get_parameter_value().double_value
        self.max_retries = self.get_parameter("max_retries").get_parameter_value().integer_value

        # Initialize action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Waypoints list
        self.waypoints = []

        # Load waypoints from file or use defaults
        if waypoints_file and os.path.exists(waypoints_file):
            self.load_waypoints_from_file(waypoints_file)
        else:
            self.get_logger().warn(
                "No waypoints file provided or file not found. Using default waypoints."
            )
            self.load_default_waypoints()

        # Current waypoint index
        self.current_waypoint_index = 0
        self.patrol_count = 0

        self.get_logger().info(f"Patrol node initialized with {len(self.waypoints)} waypoints")
        self.get_logger().info(
            f"Loop patrol: {self.loop_patrol}, Stop duration: {self.stop_duration}s"
        )

    def load_waypoints_from_file(self, filepath):
        """Load waypoints from a YAML file."""
        try:
            with open(filepath) as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get("waypoints", [])
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from file: {e}")
            self.load_default_waypoints()

    def load_default_waypoints(self):
        """Load default waypoints for testing."""
        # Example waypoints - modify these based on your environment
        self.waypoints = [
            {"x": 2.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            {"x": 2.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.707, "qw": 0.707},
            {"x": 0.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.707, "qw": 0.707},
        ]
        self.get_logger().info(f"Using {len(self.waypoints)} default waypoints")

    def create_goal_pose(self, waypoint):
        """Create a PoseStamped goal from a waypoint dictionary."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = waypoint["x"]
        goal_pose.pose.position.y = waypoint["y"]
        goal_pose.pose.position.z = waypoint.get("z", 0.0)

        goal_pose.pose.orientation.x = waypoint.get("qx", 0.0)
        goal_pose.pose.orientation.y = waypoint.get("qy", 0.0)
        goal_pose.pose.orientation.z = waypoint.get("qz", 0.0)
        goal_pose.pose.orientation.w = waypoint.get("qw", 1.0)

        return goal_pose

    def send_goal(self, waypoint_index):
        """Send a navigation goal to Nav2."""
        if waypoint_index >= len(self.waypoints):
            self.get_logger().error(f"Invalid waypoint index: {waypoint_index}")
            return False

        waypoint = self.waypoints[waypoint_index]

        self.get_logger().info(
            f"Navigating to waypoint {waypoint_index + 1}/{len(self.waypoints)}: "
            f"x={waypoint['x']:.2f}, y={waypoint['y']:.2f}"
        )

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available!")
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_goal_pose(waypoint)

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback."""
        # You can log distance remaining or other feedback here if needed
        # self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')
        pass

    def goal_response_callback(self, future):
        """Callback when goal is accepted or rejected."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.handle_waypoint_failure()
            return

        self.get_logger().info("Goal accepted by Nav2")

        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback when navigation completes."""
        future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f"Reached waypoint {self.current_waypoint_index + 1}! "
                f"Stopping for {self.stop_duration}s"
            )

            # Stop at waypoint for configured duration
            time.sleep(self.stop_duration)

            # Move to next waypoint
            self.current_waypoint_index += 1

            # Check if we've completed the patrol loop
            if self.current_waypoint_index >= len(self.waypoints):
                self.patrol_count += 1
                self.get_logger().info(f"Completed patrol loop #{self.patrol_count}")

                if self.loop_patrol:
                    self.current_waypoint_index = 0
                    self.get_logger().info("Starting next patrol loop...")
                    self.send_goal(self.current_waypoint_index)
                else:
                    self.get_logger().info("Patrol complete! Shutting down.")
                    rclpy.shutdown()
            else:
                # Continue to next waypoint
                self.send_goal(self.current_waypoint_index)
        else:
            self.get_logger().error(f"Navigation failed with status: {status}")
            self.handle_waypoint_failure()

    def handle_waypoint_failure(self):
        """Handle navigation failure with retry logic."""
        self.get_logger().warn("Retrying current waypoint...")
        # You can implement retry logic here
        time.sleep(2.0)
        self.send_goal(self.current_waypoint_index)

    def start_patrol(self):
        """Start the patrol mission."""
        if len(self.waypoints) == 0:
            self.get_logger().error("No waypoints available for patrol!")
            return False

        self.get_logger().info("Starting patrol mission...")
        return self.send_goal(self.current_waypoint_index)


def main(args=None):
    rclpy.init(args=args)

    patrol_node = PatrolNode()

    # Start patrol
    if patrol_node.start_patrol():
        try:
            rclpy.spin(patrol_node)
        except KeyboardInterrupt:
            patrol_node.get_logger().info("Patrol interrupted by user")
    else:
        patrol_node.get_logger().error("Failed to start patrol")

    patrol_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
