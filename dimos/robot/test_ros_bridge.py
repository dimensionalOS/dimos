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

from unittest.mock import MagicMock, patch

import pytest
from geometry_msgs.msg import Twist as ROSTwist
from geometry_msgs.msg import Vector3 as ROSVector3
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import Quaternion as ROSQuaternion
from nav_msgs.msg import Path as ROSPath
from sensor_msgs.msg import LaserScan as ROSLaserScan
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2
from std_msgs.msg import String as ROSString
from std_msgs.msg import Header as ROSHeader

from dimos.msgs.geometry_msgs import Twist, Vector3, PoseStamped, Pose, Quaternion
from dimos.msgs.nav_msgs import Path
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.robot.ros_bridge import ROSBridge, BridgeDirection


@pytest.fixture
def bridge():
    """Create a ROSBridge instance with mocked internals."""
    with (
        patch("dimos.robot.ros_bridge.rclpy") as mock_rclpy,
        patch("dimos.robot.ros_bridge.Node") as mock_node_class,
        patch("dimos.robot.ros_bridge.LCM") as mock_lcm_class,
        patch("dimos.robot.ros_bridge.MultiThreadedExecutor") as mock_executor_class,
    ):
        mock_rclpy.ok.return_value = False
        mock_node = MagicMock()
        mock_node.create_subscription = MagicMock(return_value=MagicMock())
        mock_node.create_publisher = MagicMock(return_value=MagicMock())
        mock_node_class.return_value = mock_node

        mock_lcm = MagicMock()
        mock_lcm.subscribe = MagicMock(return_value=MagicMock())
        mock_lcm.publish = MagicMock()
        mock_lcm_class.return_value = mock_lcm

        mock_executor = MagicMock()
        mock_executor_class.return_value = mock_executor

        bridge = ROSBridge("test_bridge")

        bridge._mock_rclpy = mock_rclpy
        bridge._mock_node_class = mock_node_class
        bridge._mock_lcm_class = mock_lcm_class

        return bridge


def test_bridge_initialization():
    """Test that the bridge initializes correctly with its own instances."""
    with (
        patch("dimos.robot.ros_bridge.rclpy") as mock_rclpy,
        patch("dimos.robot.ros_bridge.Node") as mock_node_class,
        patch("dimos.robot.ros_bridge.LCM") as mock_lcm_class,
        patch("dimos.robot.ros_bridge.MultiThreadedExecutor"),
    ):
        mock_rclpy.ok.return_value = False

        bridge = ROSBridge("test_bridge")

        mock_rclpy.init.assert_called_once()
        mock_node_class.assert_called_once_with("test_bridge")
        mock_lcm_class.assert_called_once()
        bridge.lcm.start.assert_called_once()

        assert bridge._bridges == {}
        assert bridge._qos is not None


def test_add_topic_ros_to_dimos(bridge):
    """Test that add_topic creates ROS subscription for ROS->DIMOS direction."""
    topic_name = "/cmd_vel"

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)

    bridge.node.create_subscription.assert_called_once()
    call_args = bridge.node.create_subscription.call_args
    assert call_args[0][0] == ROSTwist
    assert call_args[0][1] == topic_name

    bridge.node.create_publisher.assert_not_called()
    bridge.lcm.subscribe.assert_not_called()

    assert topic_name in bridge._bridges
    assert "dimos_topic" in bridge._bridges[topic_name]
    assert "dimos_type" in bridge._bridges[topic_name]
    assert "ros_type" in bridge._bridges[topic_name]
    assert bridge._bridges[topic_name]["dimos_type"] == Twist
    assert bridge._bridges[topic_name]["ros_type"] == ROSTwist
    assert bridge._bridges[topic_name]["direction"] == BridgeDirection.ROS_TO_DIMOS


def test_add_topic_dimos_to_ros(bridge):
    """Test that add_topic creates ROS publisher for DIMOS->ROS direction."""
    topic_name = "/cmd_vel"

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.DIMOS_TO_ROS)

    bridge.node.create_subscription.assert_not_called()
    bridge.node.create_publisher.assert_called_once_with(ROSTwist, topic_name, bridge._qos)
    bridge.lcm.subscribe.assert_called_once()

    assert topic_name in bridge._bridges
    assert "dimos_topic" in bridge._bridges[topic_name]
    assert "dimos_type" in bridge._bridges[topic_name]
    assert "ros_type" in bridge._bridges[topic_name]
    assert bridge._bridges[topic_name]["dimos_type"] == Twist
    assert bridge._bridges[topic_name]["ros_type"] == ROSTwist
    assert bridge._bridges[topic_name]["direction"] == BridgeDirection.DIMOS_TO_ROS


def test_ros_to_dimos_conversion(bridge):
    """Test ROS to DIMOS message conversion and publishing."""
    # Create a ROS Twist message
    ros_msg = ROSTwist()
    ros_msg.linear = ROSVector3(x=1.0, y=2.0, z=3.0)
    ros_msg.angular = ROSVector3(x=0.1, y=0.2, z=0.3)

    # Create DIMOS topic
    dimos_topic = Topic("/test", Twist)

    # Call the conversion method with type
    bridge._ros_to_dimos(ros_msg, dimos_topic, Twist, "/test")

    # Verify DIMOS publish was called
    bridge.lcm.publish.assert_called_once()

    # Get the published message
    published_topic, published_msg = bridge.lcm.publish.call_args[0]

    assert published_topic == dimos_topic
    assert isinstance(published_msg, Twist)
    assert published_msg.linear.x == 1.0
    assert published_msg.linear.y == 2.0
    assert published_msg.linear.z == 3.0
    assert published_msg.angular.x == 0.1
    assert published_msg.angular.y == 0.2
    assert published_msg.angular.z == 0.3


def test_dimos_to_ros_conversion(bridge):
    """Test DIMOS to ROS message conversion and publishing."""
    # Create a DIMOS Twist message
    dimos_msg = Twist(linear=Vector3(4.0, 5.0, 6.0), angular=Vector3(0.4, 0.5, 0.6))

    # Create mock ROS publisher
    ros_pub = MagicMock()

    # Call the conversion method
    bridge._dimos_to_ros(dimos_msg, ros_pub, "/test")

    # Verify ROS publish was called
    ros_pub.publish.assert_called_once()

    # Get the published message
    published_msg = ros_pub.publish.call_args[0][0]

    assert isinstance(published_msg, ROSTwist)
    assert published_msg.linear.x == 4.0
    assert published_msg.linear.y == 5.0
    assert published_msg.linear.z == 6.0
    assert published_msg.angular.x == 0.4
    assert published_msg.angular.y == 0.5
    assert published_msg.angular.z == 0.6


def test_unidirectional_flow_ros_to_dimos(bridge):
    """Test that messages flow from ROS to DIMOS when configured."""
    topic_name = "/cmd_vel"

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)

    ros_callback = bridge.node.create_subscription.call_args[0][2]

    ros_msg = ROSTwist()
    ros_msg.linear = ROSVector3(x=1.5, y=2.5, z=3.5)
    ros_msg.angular = ROSVector3(x=0.15, y=0.25, z=0.35)

    ros_callback(ros_msg)

    bridge.lcm.publish.assert_called_once()
    _, published_msg = bridge.lcm.publish.call_args[0]
    assert isinstance(published_msg, Twist)
    assert published_msg.linear.x == 1.5


def test_unidirectional_flow_dimos_to_ros(bridge):
    """Test that messages flow from DIMOS to ROS when configured."""
    topic_name = "/cmd_vel"

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.DIMOS_TO_ROS)

    dimos_callback = bridge.lcm.subscribe.call_args[0][1]

    dimos_msg = Twist(linear=Vector3(7.0, 8.0, 9.0), angular=Vector3(0.7, 0.8, 0.9))

    ros_publisher = bridge.node.create_publisher.return_value

    dimos_callback(dimos_msg, None)

    ros_publisher.publish.assert_called_once()
    published_ros_msg = ros_publisher.publish.call_args[0][0]
    assert isinstance(published_ros_msg, ROSTwist)
    assert published_ros_msg.linear.x == 7.0


def test_multiple_topics(bridge):
    """Test that multiple topics can be bridged simultaneously."""
    topics = [
        ("/cmd_vel", BridgeDirection.ROS_TO_DIMOS),
        ("/teleop", BridgeDirection.DIMOS_TO_ROS),
        ("/nav_cmd", BridgeDirection.ROS_TO_DIMOS),
    ]

    for topic, direction in topics:
        bridge.add_topic(topic, Twist, ROSTwist, direction=direction)

    assert len(bridge._bridges) == 3
    for topic, _ in topics:
        assert topic in bridge._bridges

    assert bridge.node.create_subscription.call_count == 2
    assert bridge.node.create_publisher.call_count == 1
    assert bridge.lcm.subscribe.call_count == 1


def test_stress_ros_to_dimos_100_messages(bridge):
    """Test publishing 100 ROS messages and verify DIMOS receives them all."""
    topic_name = "/stress_test"
    num_messages = 100

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)

    ros_callback = bridge.node.create_subscription.call_args[0][2]

    for i in range(num_messages):
        ros_msg = ROSTwist()
        ros_msg.linear = ROSVector3(x=float(i), y=float(i * 2), z=float(i * 3))
        ros_msg.angular = ROSVector3(x=float(i * 0.1), y=float(i * 0.2), z=float(i * 0.3))

        ros_callback(ros_msg)

    assert bridge.lcm.publish.call_count == num_messages

    last_call = bridge.lcm.publish.call_args_list[-1]
    _, last_msg = last_call[0]
    assert isinstance(last_msg, Twist)
    assert last_msg.linear.x == 99.0
    assert last_msg.linear.y == 198.0
    assert last_msg.linear.z == 297.0
    assert abs(last_msg.angular.x - 9.9) < 0.01
    assert abs(last_msg.angular.y - 19.8) < 0.01
    assert abs(last_msg.angular.z - 29.7) < 0.01


def test_stress_dimos_to_ros_100_messages(bridge):
    """Test publishing 100 DIMOS messages and verify ROS receives them all."""
    topic_name = "/stress_test_reverse"
    num_messages = 100

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.DIMOS_TO_ROS)

    dimos_callback = bridge.lcm.subscribe.call_args[0][1]
    ros_publisher = bridge.node.create_publisher.return_value

    for i in range(num_messages):
        dimos_msg = Twist(
            linear=Vector3(float(i * 10), float(i * 20), float(i * 30)),
            angular=Vector3(float(i * 0.01), float(i * 0.02), float(i * 0.03)),
        )

        dimos_callback(dimos_msg, None)

    assert ros_publisher.publish.call_count == num_messages

    last_call = ros_publisher.publish.call_args_list[-1]
    last_ros_msg = last_call[0][0]
    assert isinstance(last_ros_msg, ROSTwist)
    assert last_ros_msg.linear.x == 990.0
    assert last_ros_msg.linear.y == 1980.0
    assert last_ros_msg.linear.z == 2970.0
    assert abs(last_ros_msg.angular.x - 0.99) < 0.001
    assert abs(last_ros_msg.angular.y - 1.98) < 0.001
    assert abs(last_ros_msg.angular.z - 2.97) < 0.001


def test_two_topics_different_directions(bridge):
    """Test two topics with different directions handling messages."""
    topic_r2d = "/ros_to_dimos"
    topic_d2r = "/dimos_to_ros"

    bridge.add_topic(topic_r2d, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)
    bridge.add_topic(topic_d2r, Twist, ROSTwist, direction=BridgeDirection.DIMOS_TO_ROS)

    ros_callback = bridge.node.create_subscription.call_args[0][2]
    dimos_callback = bridge.lcm.subscribe.call_args[0][1]
    ros_publisher = bridge.node.create_publisher.return_value

    for i in range(50):
        ros_msg = ROSTwist()
        ros_msg.linear = ROSVector3(x=float(i), y=0.0, z=0.0)
        ros_msg.angular = ROSVector3(x=0.0, y=0.0, z=float(i * 0.1))
        ros_callback(ros_msg)

        dimos_msg = Twist(
            linear=Vector3(0.0, float(i), 0.0), angular=Vector3(0.0, 0.0, float(i * 0.2))
        )
        dimos_callback(dimos_msg, None)

    assert bridge.lcm.publish.call_count == 50
    assert ros_publisher.publish.call_count == 50

    last_dimos_call = bridge.lcm.publish.call_args_list[-1]
    _, last_dimos_msg = last_dimos_call[0]
    assert last_dimos_msg.linear.x == 49.0

    last_ros_call = ros_publisher.publish.call_args_list[-1]
    last_ros_msg = last_ros_call[0][0]
    assert last_ros_msg.linear.y == 49.0


def test_high_frequency_burst(bridge):
    """Test handling a burst of 1000 messages to ensure no drops."""
    topic_name = "/burst_test"
    burst_size = 1000

    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)

    ros_callback = bridge.node.create_subscription.call_args[0][2]

    messages_sent = []
    for i in range(burst_size):
        ros_msg = ROSTwist()
        ros_msg.linear = ROSVector3(x=float(i), y=float(i), z=float(i))
        ros_msg.angular = ROSVector3(x=0.0, y=0.0, z=0.0)
        messages_sent.append(i)
        ros_callback(ros_msg)

    assert bridge.lcm.publish.call_count == burst_size

    for idx, call in enumerate(bridge.lcm.publish.call_args_list):
        _, msg = call[0]
        assert msg.linear.x == float(idx)


def test_multiple_topics_with_different_rates(bridge):
    """Test multiple topics receiving messages at different rates."""
    topics = {
        "/fast_topic": 100,  # 100 messages
        "/medium_topic": 50,  # 50 messages
        "/slow_topic": 10,  # 10 messages
    }

    for topic_name in topics:
        bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)

    callbacks = []
    for i in range(3):
        callbacks.append(bridge.node.create_subscription.call_args_list[i][0][2])

    bridge.lcm.publish.reset_mock()

    for topic_idx, (topic_name, msg_count) in enumerate(topics.items()):
        for i in range(msg_count):
            ros_msg = ROSTwist()
            ros_msg.linear = ROSVector3(x=float(topic_idx), y=float(i), z=0.0)
            callbacks[topic_idx](ros_msg)

    total_expected = sum(topics.values())
    assert bridge.lcm.publish.call_count == total_expected


def test_pose_stamped_bridging(bridge):
    """Test bridging PoseStamped messages."""
    topic_name = "/robot_pose"

    # Test ROS to DIMOS
    bridge.add_topic(
        topic_name, PoseStamped, ROSPoseStamped, direction=BridgeDirection.ROS_TO_DIMOS
    )

    ros_callback = bridge.node.create_subscription.call_args[0][2]

    ros_msg = ROSPoseStamped()
    ros_msg.header.frame_id = "map"
    ros_msg.header.stamp.sec = 100
    ros_msg.header.stamp.nanosec = 500000000
    ros_msg.pose.position.x = 10.0
    ros_msg.pose.position.y = 20.0
    ros_msg.pose.position.z = 30.0
    ros_msg.pose.orientation.x = 0.0
    ros_msg.pose.orientation.y = 0.0
    ros_msg.pose.orientation.z = 0.707
    ros_msg.pose.orientation.w = 0.707

    ros_callback(ros_msg)

    bridge.lcm.publish.assert_called_once()
    _, published_msg = bridge.lcm.publish.call_args[0]
    assert hasattr(published_msg, "frame_id")
    assert hasattr(published_msg, "position")
    assert hasattr(published_msg, "orientation")


def test_path_bridging(bridge):
    """Test bridging Path messages."""
    topic_name = "/planned_path"

    # Test DIMOS to ROS
    bridge.add_topic(topic_name, Path, ROSPath, direction=BridgeDirection.DIMOS_TO_ROS)

    dimos_callback = bridge.lcm.subscribe.call_args[0][1]
    ros_publisher = bridge.node.create_publisher.return_value

    # Create a DIMOS Path with multiple poses
    poses = []
    for i in range(5):
        pose = PoseStamped(
            ts=100.0 + i,
            frame_id="map",
            position=Vector3(float(i), float(i * 2), 0.0),
            orientation=Quaternion(0, 0, 0, 1),
        )
        poses.append(pose)

    dimos_path = Path(frame_id="map", poses=poses)

    dimos_callback(dimos_path, None)

    ros_publisher.publish.assert_called_once()
    published_ros_msg = ros_publisher.publish.call_args[0][0]
    assert isinstance(published_ros_msg, ROSPath)


def test_multiple_message_types(bridge):
    """Test bridging multiple different message types simultaneously."""
    topics = [
        ("/cmd_vel", Twist, ROSTwist, BridgeDirection.ROS_TO_DIMOS),
        ("/robot_pose", PoseStamped, ROSPoseStamped, BridgeDirection.DIMOS_TO_ROS),
        ("/global_path", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/local_path", Path, ROSPath, BridgeDirection.ROS_TO_DIMOS),
        ("/teleop_twist", Twist, ROSTwist, BridgeDirection.ROS_TO_DIMOS),
    ]

    for topic_name, dimos_type, ros_type, direction in topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    assert len(bridge._bridges) == 5

    # Count subscriptions and publishers
    ros_to_dimos_count = sum(1 for _, _, _, d in topics if d == BridgeDirection.ROS_TO_DIMOS)
    dimos_to_ros_count = sum(1 for _, _, _, d in topics if d == BridgeDirection.DIMOS_TO_ROS)

    assert bridge.node.create_subscription.call_count == ros_to_dimos_count
    assert bridge.node.create_publisher.call_count == dimos_to_ros_count
    assert bridge.lcm.subscribe.call_count == dimos_to_ros_count


def test_navigation_stack_topics(bridge):
    """Test common navigation stack topics."""
    nav_topics = [
        ("/move_base/goal", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/move_base/global_plan", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/move_base/local_plan", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/cmd_vel", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/odom", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/robot_pose", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
    ]

    for topic_name, dimos_type, ros_type, direction in nav_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    assert len(bridge._bridges) == len(nav_topics)

    # Verify each topic is configured correctly
    for topic_name, dimos_type, ros_type, direction in nav_topics:
        assert topic_name in bridge._bridges
        assert bridge._bridges[topic_name]["dimos_type"] == dimos_type
        assert bridge._bridges[topic_name]["ros_type"] == ros_type
        assert bridge._bridges[topic_name]["direction"] == direction


def test_control_topics(bridge):
    """Test control system topics."""
    control_topics = [
        ("/joint_commands", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/joint_states", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/trajectory", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/feedback", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
    ]

    for topic_name, dimos_type, ros_type, direction in control_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    assert len(bridge._bridges) == len(control_topics)


def test_perception_topics(bridge):
    """Test perception system topics."""
    perception_topics = [
        ("/detected_pose", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/tracked_path", Path, ROSPath, BridgeDirection.ROS_TO_DIMOS),
        ("/vision_pose", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
    ]

    for topic_name, dimos_type, ros_type, direction in perception_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    # All perception topics are ROS to DIMOS
    assert bridge.node.create_subscription.call_count == len(perception_topics)
    assert bridge.node.create_publisher.call_count == 0


def test_mixed_frequency_topics(bridge):
    """Test topics with different expected frequencies."""
    # High frequency (100Hz+)
    high_freq_topics = [
        ("/imu/data", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/joint_states", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
    ]

    # Medium frequency (10-50Hz)
    medium_freq_topics = [
        ("/cmd_vel", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/odom", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
    ]

    # Low frequency (1-5Hz)
    low_freq_topics = [
        ("/global_path", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/goal", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
    ]

    all_topics = high_freq_topics + medium_freq_topics + low_freq_topics

    for topic_name, dimos_type, ros_type, direction in all_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    assert len(bridge._bridges) == len(all_topics)

    # Test high frequency message handling
    for topic_name, _, _, direction in high_freq_topics:
        if direction == BridgeDirection.ROS_TO_DIMOS:
            # Find the callback for this topic
            for i, call in enumerate(bridge.node.create_subscription.call_args_list):
                if call[0][1] == topic_name:
                    callback = call[0][2]
                    # Send 100 messages rapidly
                    for j in range(100):
                        ros_msg = ROSPoseStamped()
                        ros_msg.header.stamp.sec = j
                        callback(ros_msg)
                    break


def test_bidirectional_prevention(bridge):
    """Test that the same topic cannot be added in both directions."""
    topic_name = "/cmd_vel"

    # Add topic in one direction
    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.ROS_TO_DIMOS)

    # Try to add the same topic in opposite direction should not create duplicate
    # The bridge should handle this gracefully
    initial_bridges = len(bridge._bridges)
    bridge.add_topic(topic_name, Twist, ROSTwist, direction=BridgeDirection.DIMOS_TO_ROS)

    # Should still have the same number of bridges (topic gets reconfigured, not duplicated)
    assert len(bridge._bridges) == initial_bridges


def test_robot_arm_topics(bridge):
    """Test robot arm control topics."""
    arm_topics = [
        ("/arm/joint_trajectory", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/arm/joint_states", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/arm/end_effector_pose", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/arm/gripper_cmd", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/arm/cartesian_trajectory", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
    ]

    for topic_name, dimos_type, ros_type, direction in arm_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    assert len(bridge._bridges) == len(arm_topics)

    # Check that arm control commands go from DIMOS to ROS
    dimos_to_ros = [t for t in arm_topics if t[3] == BridgeDirection.DIMOS_TO_ROS]
    ros_to_dimos = [t for t in arm_topics if t[3] == BridgeDirection.ROS_TO_DIMOS]

    assert bridge.node.create_publisher.call_count == len(dimos_to_ros)
    assert bridge.node.create_subscription.call_count == len(ros_to_dimos)


def test_mobile_base_topics(bridge):
    """Test mobile robot base topics."""
    base_topics = [
        ("/base/cmd_vel", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/base/odom", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/base/global_pose", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/base/path", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/base/local_plan", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
    ]

    for topic_name, dimos_type, ros_type, direction in base_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    # Verify topics are properly categorized
    for topic_name, dimos_type, ros_type, direction in base_topics:
        bridge_info = bridge._bridges[topic_name]
        assert bridge_info["direction"] == direction
        assert bridge_info["dimos_type"] == dimos_type
        assert bridge_info["ros_type"] == ros_type


def test_autonomous_vehicle_topics(bridge):
    """Test autonomous vehicle topics."""
    av_topics = [
        ("/vehicle/steering_cmd", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/vehicle/throttle_cmd", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/vehicle/brake_cmd", Twist, ROSTwist, BridgeDirection.DIMOS_TO_ROS),
        ("/vehicle/pose", PoseStamped, ROSPoseStamped, BridgeDirection.ROS_TO_DIMOS),
        ("/vehicle/planned_trajectory", Path, ROSPath, BridgeDirection.DIMOS_TO_ROS),
        ("/vehicle/current_path", Path, ROSPath, BridgeDirection.ROS_TO_DIMOS),
    ]

    for topic_name, dimos_type, ros_type, direction in av_topics:
        bridge.add_topic(topic_name, dimos_type, ros_type, direction=direction)

    assert len(bridge._bridges) == len(av_topics)

    # Count control vs feedback topics
    control_topics = [t for t in av_topics if "cmd" in t[0] or "planned" in t[0]]
    feedback_topics = [t for t in av_topics if "pose" in t[0] or "current" in t[0]]

    assert len(control_topics) == 4  # steering, throttle, brake, planned_trajectory
    assert len(feedback_topics) == 2  # pose, current_path
