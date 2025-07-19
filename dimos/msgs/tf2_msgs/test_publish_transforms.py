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

import time

import numpy as np
import pytest
from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic


def test_publish_transforms_with_new_types():
    """Test publishing transforms using our new Transform and TFMessage types."""
    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []
    topic = Topic(topic="/tf_fancy", lcm_type=LCMTFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Create a robot kinematic chain using our new types
    current_time = time.time()

    # 1. World to base_link transform (robot at position)
    world_to_base = Transform(
        translation=Vector3(2.0, 3.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.382683, 0.923880),  # 45 degrees around Z
        frame_id="world",
        ts=current_time,
    )

    # 2. Base to arm transform (arm lifted up)
    base_to_arm = Transform(
        translation=Vector3(0.2, 0.0, 0.5),
        rotation=Quaternion(0.0, 0.258819, 0.0, 0.965926),  # 30 degrees around Y
        frame_id="base_link",
        ts=current_time,
    )

    # 3. Arm to gripper transform (gripper extended)
    arm_to_gripper = Transform(
        translation=Vector3(0.3, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation
        frame_id="arm_link",
        ts=current_time,
    )

    # Create TFMessage with all transforms
    tf_msg = TFMessage(world_to_base, base_to_arm, arm_to_gripper)

    # Encode to LCM bytes with proper child frame IDs
    encoded = tf_msg.lcm_encode(child_frame_ids=["base_link", "arm_link", "gripper_link"])

    # Decode back to LCM TFMessage and publish
    lcm_msg = LCMTFMessage.lcm_decode(encoded)
    lcm.publish(topic, lcm_msg)

    # Wait for reception
    time.sleep(0.1)

    # Verify we received the message
    assert len(received_messages) == 1
    received_msg, received_topic = received_messages[0]

    # Verify it's an LCM TFMessage
    assert isinstance(received_msg, LCMTFMessage)
    assert received_topic == topic

    # Verify content
    assert received_msg.transforms_length == 3

    # Check world to base transform
    tf0 = received_msg.transforms[0]
    assert tf0.header.frame_id == "world"
    assert tf0.child_frame_id == "base_link"
    assert np.isclose(tf0.transform.translation.x, 2.0, atol=1e-10)
    assert np.isclose(tf0.transform.translation.y, 3.0, atol=1e-10)
    assert np.isclose(tf0.transform.rotation.z, 0.382683, atol=1e-6)
    assert np.isclose(tf0.transform.rotation.w, 0.923880, atol=1e-6)

    # Check base to arm transform
    tf1 = received_msg.transforms[1]
    assert tf1.header.frame_id == "base_link"
    assert tf1.child_frame_id == "arm_link"
    assert np.isclose(tf1.transform.translation.x, 0.2, atol=1e-10)
    assert np.isclose(tf1.transform.translation.z, 0.5, atol=1e-10)
    assert np.isclose(tf1.transform.rotation.y, 0.258819, atol=1e-6)
    assert np.isclose(tf1.transform.rotation.w, 0.965926, atol=1e-6)

    # Check arm to gripper transform
    tf2 = received_msg.transforms[2]
    assert tf2.header.frame_id == "arm_link"
    assert tf2.child_frame_id == "gripper_link"
    assert np.isclose(tf2.transform.translation.x, 0.3, atol=1e-10)
    assert tf2.transform.rotation.w == 1.0

    # Verify timestamps are preserved
    for tf in received_msg.transforms:
        received_ts = tf.header.stamp.sec + (tf.header.stamp.nsec / 1_000_000_000)
        assert abs(received_ts - current_time) < 1e-6


def test_dynamic_robot_movement():
    """Test publishing dynamic transforms as robot moves."""
    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []
    topic = Topic(topic="/tf_movement", lcm_type=LCMTFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Simulate robot moving forward and turning
    num_steps = 4
    for i in range(num_steps):
        t = i * 0.1  # Time progression

        # Robot moves forward and turns
        x = t * 2.0  # Move 2 m/s forward
        y = 0.5 * np.sin(t * np.pi)  # Slight sinusoidal motion
        angle = t * (np.pi / 4)  # Turn 45 degrees per second

        # Create transform for current robot pose
        robot_pose = Transform(
            translation=Vector3(x, y, 0.0),
            rotation=Quaternion(0.0, 0.0, np.sin(angle / 2), np.cos(angle / 2)),
            frame_id="odom",
            ts=time.time(),
        )

        # Robot has a sensor that's always 0.3m above base
        sensor_transform = Transform(
            translation=Vector3(0.0, 0.0, 0.3),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            ts=robot_pose.ts,  # Same timestamp
        )

        # Create and publish TFMessage
        tf_msg = TFMessage(robot_pose, sensor_transform)
        encoded = tf_msg.lcm_encode(child_frame_ids=["base_link", "sensor_link"])

        lcm_msg = LCMTFMessage.lcm_decode(encoded)
        lcm.publish(topic, lcm_msg)
        time.sleep(0.05)

    # Wait for all messages
    time.sleep(0.1)

    # Should have received all updates
    assert len(received_messages) == num_steps

    # Verify motion progression
    for i, (msg, _) in enumerate(received_messages):
        t = i * 0.1
        expected_x = t * 2.0
        expected_y = 0.5 * np.sin(t * np.pi)

        # Check robot transform
        robot_tf = msg.transforms[0]
        assert robot_tf.header.frame_id == "odom"
        assert robot_tf.child_frame_id == "base_link"
        assert np.isclose(robot_tf.transform.translation.x, expected_x, atol=1e-10)
        assert np.isclose(robot_tf.transform.translation.y, expected_y, atol=1e-10)

        # Check sensor is always 0.3m above base
        sensor_tf = msg.transforms[1]
        assert sensor_tf.header.frame_id == "base_link"
        assert sensor_tf.child_frame_id == "sensor_link"
        assert np.isclose(sensor_tf.transform.translation.z, 0.3, atol=1e-10)


def test_roundtrip_transform_types():
    """Test that our types can roundtrip through LCM."""
    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []
    topic = Topic(topic="/tf_roundtrip", lcm_type=LCMTFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Create transforms with various configurations
    tf1 = Transform(
        translation=Vector3(1.5, -2.5, 3.5),
        rotation=Quaternion(0.1, 0.2, 0.3, 0.9045),  # Normalized quaternion
        frame_id="sensor1",
        ts=1234.5678,
    )

    tf2 = Transform(
        translation=Vector3(-10, 20, -30),
        rotation=Quaternion(0.5, 0.5, 0.5, 0.5),  # All equal components
        frame_id="sensor2",
        ts=2345.6789,
    )

    # Create TFMessage
    original = TFMessage(tf1, tf2)

    # Encode and publish
    encoded = original.lcm_encode(child_frame_ids=["child1", "child2"])
    lcm_msg = LCMTFMessage.lcm_decode(encoded)
    lcm.publish(topic, lcm_msg)

    time.sleep(0.1)

    # Decode received message back to our types
    assert len(received_messages) == 1
    received_lcm_msg, _ = received_messages[0]

    # Decode back to our TFMessage
    decoded = TFMessage.lcm_decode(received_lcm_msg.lcm_encode())

    # Verify transforms match
    assert len(decoded) == 2

    # Check first transform
    assert decoded[0].translation.x == 1.5
    assert decoded[0].translation.y == -2.5
    assert decoded[0].translation.z == 3.5
    assert np.isclose(decoded[0].rotation.x, 0.1, atol=1e-10)
    assert np.isclose(decoded[0].rotation.y, 0.2, atol=1e-10)
    assert np.isclose(decoded[0].rotation.z, 0.3, atol=1e-10)
    assert np.isclose(decoded[0].rotation.w, 0.9045, atol=1e-10)
    assert decoded[0].frame_id == "sensor1"
    assert abs(decoded[0].ts - 1234.5678) < 1e-6

    # Check second transform
    assert decoded[1].translation.x == -10
    assert decoded[1].translation.y == 20
    assert decoded[1].translation.z == -30
    assert decoded[1].rotation.x == 0.5
    assert decoded[1].rotation.y == 0.5
    assert decoded[1].rotation.z == 0.5
    assert decoded[1].rotation.w == 0.5
    assert decoded[1].frame_id == "sensor2"
    assert abs(decoded[1].ts - 2345.6789) < 1e-6
