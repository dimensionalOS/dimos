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

import threading
import time
import unittest

import numpy as np
import pytest

try:
    from geometry_msgs.msg import TransformStamped, TwistStamped as ROSTwistStamped
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2 as ROSPointCloud2, PointField
    from tf2_msgs.msg import TFMessage as ROSTFMessage
except ImportError:
    rclpy = None
    Node = None
    ROSTwistStamped = None
    ROSPointCloud2 = None
    PointField = None
    ROSTFMessage = None
    TransformStamped = None

from dimos.msgs.geometry_msgs import TwistStamped
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.robot.ros_bridge import BridgeDirection, ROSBridge


@pytest.mark.ros
class TestROSBridge(unittest.TestCase):
    """Test suite for ROS-DIMOS bridge."""

    def setUp(self) -> None:
        """Set up test fixtures."""
        # Skip if ROS is not available
        if rclpy is None:
            self.skipTest("ROS not available")

        # Initialize ROS if not already done
        if not rclpy.ok():
            rclpy.init()

        # Create test bridge
        self.bridge = ROSBridge("test_ros_bridge")

        # Create test node for publishing/subscribing
        self.test_node = Node("test_node")

        # Track received messages
        self.ros_messages = []
        self.dimos_messages = []
        self.message_timestamps = {"ros": [], "dimos": []}

    def tearDown(self) -> None:
        """Clean up test fixtures."""
        self.test_node.destroy_node()
        self.bridge.stop()
        if rclpy.ok():
            rclpy.try_shutdown()

    def test_ros_to_dimos_twist(self) -> None:
        """Test ROS TwistStamped to DIMOS conversion and transmission."""
        # Set up bridge
        self.bridge.add_topic(
            "/test_twist", TwistStamped, ROSTwistStamped, BridgeDirection.ROS_TO_DIMOS
        )

        # Subscribe to DIMOS side
        lcm = LCM()
        lcm.start()
        topic = Topic("/test_twist", TwistStamped)

        def dimos_callback(msg, _topic) -> None:
            self.dimos_messages.append(msg)
            self.message_timestamps["dimos"].append(time.time())

        lcm.subscribe(topic, dimos_callback)

        # Publish from ROS side
        ros_pub = self.test_node.create_publisher(ROSTwistStamped, "/test_twist", 10)

        # Send test messages
        for i in range(10):
            msg = ROSTwistStamped()
            msg.header.stamp = self.test_node.get_clock().now().to_msg()
            msg.header.frame_id = f"frame_{i}"
            msg.twist.linear.x = float(i)
            msg.twist.linear.y = float(i * 2)
            msg.twist.angular.z = float(i * 0.1)

            ros_pub.publish(msg)
            self.message_timestamps["ros"].append(time.time())
            time.sleep(0.01)  # 100Hz

        # Allow time for processing
        time.sleep(0.5)

        # Verify messages received
        self.assertEqual(len(self.dimos_messages), 10, "Should receive all 10 messages")

        # Verify message content
        for i, msg in enumerate(self.dimos_messages):
            self.assertEqual(msg.frame_id, f"frame_{i}")
            self.assertAlmostEqual(msg.linear.x, float(i), places=5)
            self.assertAlmostEqual(msg.linear.y, float(i * 2), places=5)
            self.assertAlmostEqual(msg.angular.z, float(i * 0.1), places=5)

        lcm.stop()

    def test_dimos_to_ros_twist(self) -> None:
        """Test DIMOS TwistStamped to ROS conversion and transmission."""
        # Set up bridge
        self.bridge.add_topic(
            "/test_twist_reverse", TwistStamped, ROSTwistStamped, BridgeDirection.DIMOS_TO_ROS
        )

        # Subscribe to ROS side
        def ros_callback(msg) -> None:
            self.ros_messages.append(msg)
            self.message_timestamps["ros"].append(time.time())

        self.test_node.create_subscription(ROSTwistStamped, "/test_twist_reverse", ros_callback, 10)

        # Use the bridge's LCM instance for publishing
        topic = Topic("/test_twist_reverse", TwistStamped)

        # Send test messages
        for i in range(10):
            msg = TwistStamped(ts=time.time(), frame_id=f"dimos_frame_{i}")
            msg.linear.x = float(i * 3)
            msg.linear.y = float(i * 4)
            msg.angular.z = float(i * 0.2)

            self.bridge.lcm.publish(topic, msg)
            self.message_timestamps["dimos"].append(time.time())
            time.sleep(0.01)  # 100Hz

        # Allow time for processing and spin the test node
        for _ in range(50):  # Spin for 0.5 seconds
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        # Verify messages received
        self.assertEqual(len(self.ros_messages), 10, "Should receive all 10 messages")

        # Verify message content
        for i, msg in enumerate(self.ros_messages):
            self.assertEqual(msg.header.frame_id, f"dimos_frame_{i}")
            self.assertAlmostEqual(msg.twist.linear.x, float(i * 3), places=5)
            self.assertAlmostEqual(msg.twist.linear.y, float(i * 4), places=5)
            self.assertAlmostEqual(msg.twist.angular.z, float(i * 0.2), places=5)

    def test_frequency_preservation(self) -> None:
        """Test that message frequencies are preserved through the bridge."""
        # Set up bridge
        self.bridge.add_topic(
            "/test_freq", TwistStamped, ROSTwistStamped, BridgeDirection.ROS_TO_DIMOS
        )

        # Subscribe to DIMOS side
        lcm = LCM()
        lcm.start()
        topic = Topic("/test_freq", TwistStamped)

        receive_times = []

        def dimos_callback(_msg, _topic) -> None:
            receive_times.append(time.time())

        lcm.subscribe(topic, dimos_callback)

        # Publish from ROS at specific frequencies
        ros_pub = self.test_node.create_publisher(ROSTwistStamped, "/test_freq", 10)

        # Test different frequencies
        test_frequencies = [10, 50, 100]  # Hz

        for target_freq in test_frequencies:
            receive_times.clear()
            send_times = []
            period = 1.0 / target_freq

            # Send messages at target frequency
            start_time = time.time()
            while time.time() - start_time < 1.0:  # Run for 1 second
                msg = ROSTwistStamped()
                msg.header.stamp = self.test_node.get_clock().now().to_msg()
                msg.twist.linear.x = 1.0

                ros_pub.publish(msg)
                send_times.append(time.time())
                time.sleep(period)

            # Allow processing time
            time.sleep(0.2)

            # Calculate actual frequencies
            if len(send_times) > 1:
                send_intervals = np.diff(send_times)
                send_freq = 1.0 / np.mean(send_intervals)
            else:
                send_freq = 0

            if len(receive_times) > 1:
                receive_intervals = np.diff(receive_times)
                receive_freq = 1.0 / np.mean(receive_intervals)
            else:
                receive_freq = 0

            # Verify frequency preservation (within 10% tolerance)
            self.assertAlmostEqual(
                receive_freq,
                send_freq,
                delta=send_freq * 0.1,
                msg=f"Frequency not preserved for {target_freq}Hz: sent={send_freq:.1f}Hz, received={receive_freq:.1f}Hz",
            )

        lcm.stop()

    def test_pointcloud_conversion(self) -> None:
        """Test PointCloud2 message conversion with numpy optimization."""
        # Set up bridge
        self.bridge.add_topic(
            "/test_cloud", PointCloud2, ROSPointCloud2, BridgeDirection.ROS_TO_DIMOS
        )

        # Subscribe to DIMOS side
        lcm = LCM()
        lcm.start()
        topic = Topic("/test_cloud", PointCloud2)

        received_cloud = []

        def dimos_callback(msg, _topic) -> None:
            received_cloud.append(msg)

        lcm.subscribe(topic, dimos_callback)

        # Create test point cloud
        ros_pub = self.test_node.create_publisher(ROSPointCloud2, "/test_cloud", 10)

        # Generate test points
        num_points = 1000
        points = np.random.randn(num_points, 3).astype(np.float32)

        # Create ROS PointCloud2 message
        msg = ROSPointCloud2()
        msg.header.stamp = self.test_node.get_clock().now().to_msg()
        msg.header.frame_id = "test_frame"
        msg.height = 1
        msg.width = num_points
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        msg.is_dense = True

        # Send point cloud
        ros_pub.publish(msg)

        # Allow processing time
        time.sleep(0.5)

        # Verify reception
        self.assertEqual(len(received_cloud), 1, "Should receive point cloud")

        # Verify point data
        received_points = received_cloud[0].as_numpy()
        self.assertEqual(received_points.shape, points.shape)
        np.testing.assert_array_almost_equal(received_points, points, decimal=5)

        lcm.stop()

    def test_tf_high_frequency(self) -> None:
        """Test TF message handling at high frequency."""
        # Set up bridge
        self.bridge.add_topic("/test_tf", TFMessage, ROSTFMessage, BridgeDirection.ROS_TO_DIMOS)

        # Subscribe to DIMOS side
        lcm = LCM()
        lcm.start()
        topic = Topic("/test_tf", TFMessage)

        received_tfs = []
        receive_times = []

        def dimos_callback(msg, _topic) -> None:
            received_tfs.append(msg)
            receive_times.append(time.time())

        lcm.subscribe(topic, dimos_callback)

        # Publish TF at high frequency (100Hz)
        ros_pub = self.test_node.create_publisher(ROSTFMessage, "/test_tf", 100)

        target_freq = 100  # Hz
        period = 1.0 / target_freq
        num_messages = 100  # 1 second worth

        send_times = []
        for i in range(num_messages):
            msg = ROSTFMessage()
            transform = TransformStamped()
            transform.header.stamp = self.test_node.get_clock().now().to_msg()
            transform.header.frame_id = "world"
            transform.child_frame_id = f"link_{i}"
            transform.transform.translation.x = float(i)
            transform.transform.rotation.w = 1.0
            msg.transforms = [transform]

            ros_pub.publish(msg)
            send_times.append(time.time())
            time.sleep(period)

        # Allow processing time
        time.sleep(0.5)

        # Check message count (allow 5% loss tolerance)
        min_expected = int(num_messages * 0.95)
        self.assertGreaterEqual(
            len(received_tfs),
            min_expected,
            f"Should receive at least {min_expected} of {num_messages} TF messages",
        )

        # Check frequency preservation
        if len(receive_times) > 1:
            receive_intervals = np.diff(receive_times)
            receive_freq = 1.0 / np.mean(receive_intervals)

            # For high frequency, allow 20% tolerance
            self.assertAlmostEqual(
                receive_freq,
                target_freq,
                delta=target_freq * 0.2,
                msg=f"High frequency TF not preserved: expected={target_freq}Hz, got={receive_freq:.1f}Hz",
            )

        lcm.stop()

    def test_bidirectional_bridge(self) -> None:
        """Test simultaneous bidirectional message flow."""
        # Set up bidirectional bridges for same topic type
        self.bridge.add_topic(
            "/ros_to_dimos", TwistStamped, ROSTwistStamped, BridgeDirection.ROS_TO_DIMOS
        )

        self.bridge.add_topic(
            "/dimos_to_ros", TwistStamped, ROSTwistStamped, BridgeDirection.DIMOS_TO_ROS
        )

        dimos_received = []
        ros_received = []

        # DIMOS subscriber - use bridge's LCM
        topic_r2d = Topic("/ros_to_dimos", TwistStamped)
        self.bridge.lcm.subscribe(topic_r2d, lambda msg, _: dimos_received.append(msg))

        # ROS subscriber
        self.test_node.create_subscription(
            ROSTwistStamped, "/dimos_to_ros", lambda msg: ros_received.append(msg), 10
        )

        # Set up publishers
        ros_pub = self.test_node.create_publisher(ROSTwistStamped, "/ros_to_dimos", 10)
        topic_d2r = Topic("/dimos_to_ros", TwistStamped)

        # Keep track of whether threads should continue
        stop_spinning = threading.Event()

        # Spin the test node in background to receive messages
        def spin_test_node() -> None:
            while not stop_spinning.is_set():
                rclpy.spin_once(self.test_node, timeout_sec=0.01)

        spin_thread = threading.Thread(target=spin_test_node, daemon=True)
        spin_thread.start()

        # Send messages in both directions simultaneously
        def send_ros_messages() -> None:
            for i in range(50):
                msg = ROSTwistStamped()
                msg.header.stamp = self.test_node.get_clock().now().to_msg()
                msg.twist.linear.x = float(i)
                ros_pub.publish(msg)
                time.sleep(0.02)  # 50Hz

        def send_dimos_messages() -> None:
            for i in range(50):
                msg = TwistStamped(ts=time.time())
                msg.linear.y = float(i * 2)
                self.bridge.lcm.publish(topic_d2r, msg)
                time.sleep(0.02)  # 50Hz

        # Run both senders in parallel
        ros_thread = threading.Thread(target=send_ros_messages)
        dimos_thread = threading.Thread(target=send_dimos_messages)

        ros_thread.start()
        dimos_thread.start()

        ros_thread.join()
        dimos_thread.join()

        # Allow processing time
        time.sleep(0.5)
        stop_spinning.set()
        spin_thread.join(timeout=1.0)

        # Verify both directions worked
        self.assertGreaterEqual(len(dimos_received), 45, "Should receive most ROS->DIMOS messages")
        self.assertGreaterEqual(len(ros_received), 45, "Should receive most DIMOS->ROS messages")

        # Verify message integrity
        for i, msg in enumerate(dimos_received[:45]):
            self.assertAlmostEqual(msg.linear.x, float(i), places=5)

        for i, msg in enumerate(ros_received[:45]):
            self.assertAlmostEqual(msg.twist.linear.y, float(i * 2), places=5)


if __name__ == "__main__":
    unittest.main()
