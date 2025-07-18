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
from dimos_lcm.geometry_msgs import TransformStamped
from dimos_lcm.std_msgs import Header, Time
from dimos_lcm.tf2_msgs import TFMessage

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic


def test_tfmessage_single_transform():
    """Test sending TFMessage with a single transform through LCM."""
    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []

    # Create topic for TFMessage
    topic = Topic(topic="/tf", lcm_type=TFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Create a transform from world to base_link
    header = Header(seq=1, stamp=Time(sec=123, nsec=456789), frame_id="world")

    # Create the transform - translation and 90 degree rotation around Z
    transform = Transform(
        translation=Vector3(2.0, 1.0, 0.5),
        rotation=Quaternion(0.0, 0.0, 0.707107, 0.707107),  # 90 degrees around Z
    )

    # Create TransformStamped
    transform_stamped = TransformStamped(
        header=header, child_frame_id="base_link", transform=transform
    )

    # Create TFMessage with one transform
    tf_message = TFMessage(transforms_length=1, transforms=[transform_stamped])

    # Publish the TF message
    lcm.publish(topic, tf_message)

    # Wait for message to be received
    time.sleep(0.1)

    # Verify reception
    assert len(received_messages) == 1

    received_tf_msg, received_topic = received_messages[0]

    # Verify it's a TFMessage
    assert isinstance(received_tf_msg, TFMessage)
    assert received_topic == topic

    # Verify the content
    assert received_tf_msg.transforms_length == 1

    # Check the transform
    received_transform = received_tf_msg.transforms[0]
    assert received_transform.header.seq == 1
    assert received_transform.header.stamp.sec == 123
    assert received_transform.header.stamp.nsec == 456789
    assert received_transform.header.frame_id == "world"
    assert received_transform.child_frame_id == "base_link"

    # Check transform values
    assert np.isclose(received_transform.transform.translation.x, 2.0, atol=1e-10)
    assert np.isclose(received_transform.transform.translation.y, 1.0, atol=1e-10)
    assert np.isclose(received_transform.transform.translation.z, 0.5, atol=1e-10)

    assert np.isclose(received_transform.transform.rotation.x, 0.0, atol=1e-10)
    assert np.isclose(received_transform.transform.rotation.y, 0.0, atol=1e-10)
    assert np.isclose(received_transform.transform.rotation.z, 0.707107, atol=1e-10)
    assert np.isclose(received_transform.transform.rotation.w, 0.707107, atol=1e-10)


def test_tfmessage_multiple_transforms():
    """Test TFMessage with multiple transforms (robot kinematic chain)."""
    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []
    topic = Topic(topic="/tf_multi", lcm_type=TFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Create a kinematic chain: world -> base_link -> torso -> head
    current_time = Time(sec=456, nsec=789012)

    transforms = []

    # 1. world -> base_link (robot at position (1,2,0))
    transforms.append(
        TransformStamped(
            header=Header(seq=1, stamp=current_time, frame_id="world"),
            child_frame_id="base_link",
            transform=Transform(
                translation=Vector3(1.0, 2.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.382683, 0.923880),  # 45 degrees around Z
            ),
        )
    )

    # 2. base_link -> torso (torso 0.5m up from base)
    transforms.append(
        TransformStamped(
            header=Header(seq=2, stamp=current_time, frame_id="base_link"),
            child_frame_id="torso",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.5),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation
            ),
        )
    )

    # 3. torso -> head (head 0.3m up and tilted down)
    angle = np.pi / 6  # 30 degrees
    transforms.append(
        TransformStamped(
            header=Header(seq=3, stamp=current_time, frame_id="torso"),
            child_frame_id="head",
            transform=Transform(
                translation=Vector3(0.0, 0.0, 0.3),
                rotation=Quaternion(
                    0.0, np.sin(angle / 2), 0.0, np.cos(angle / 2)
                ),  # 30 degrees around Y
            ),
        )
    )

    # Create TFMessage with multiple transforms
    tf_message = TFMessage(transforms_length=len(transforms), transforms=transforms)

    # Publish
    lcm.publish(topic, tf_message)
    time.sleep(0.1)

    # Verify
    assert len(received_messages) == 1
    received_tf_msg, _ = received_messages[0]

    assert received_tf_msg.transforms_length == 3

    # Verify each transform
    for i, (sent, received) in enumerate(zip(transforms, received_tf_msg.transforms)):
        assert received.header.seq == sent.header.seq
        assert received.header.frame_id == sent.header.frame_id
        assert received.child_frame_id == sent.child_frame_id

        # Check translation
        assert np.isclose(
            received.transform.translation.x, sent.transform.translation.x, atol=1e-10
        )
        assert np.isclose(
            received.transform.translation.y, sent.transform.translation.y, atol=1e-10
        )
        assert np.isclose(
            received.transform.translation.z, sent.transform.translation.z, atol=1e-10
        )

        # Check rotation
        assert np.isclose(received.transform.rotation.x, sent.transform.rotation.x, atol=1e-10)
        assert np.isclose(received.transform.rotation.y, sent.transform.rotation.y, atol=1e-10)
        assert np.isclose(received.transform.rotation.z, sent.transform.rotation.z, atol=1e-10)
        assert np.isclose(received.transform.rotation.w, sent.transform.rotation.w, atol=1e-10)


def test_tfmessage_dynamic_updates():
    """Test sending dynamic TF updates (moving robot)."""
    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []
    topic = Topic(topic="/tf_dynamic", lcm_type=TFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Simulate a robot moving in a circle
    num_steps = 5
    radius = 2.0

    for step in range(num_steps):
        angle = (2 * np.pi * step) / num_steps

        # Calculate position on circle
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)

        # Robot orientation faces tangent to circle
        robot_angle = angle + np.pi / 2

        # Create transform
        transform_stamped = TransformStamped(
            header=Header(seq=step, stamp=Time(sec=100 + step, nsec=0), frame_id="odom"),
            child_frame_id="base_link",
            transform=Transform(
                translation=Vector3(x, y, 0.0),
                rotation=Quaternion(0.0, 0.0, np.sin(robot_angle / 2), np.cos(robot_angle / 2)),
            ),
        )

        # Send TF update
        tf_message = TFMessage(transforms_length=1, transforms=[transform_stamped])

        lcm.publish(topic, tf_message)
        time.sleep(0.05)  # Small delay between updates

    # Wait for all messages
    time.sleep(0.1)

    # Should have received all updates
    assert len(received_messages) == num_steps

    # Verify each position update
    for i, (msg, _) in enumerate(received_messages):
        transform = msg.transforms[0]
        angle = (2 * np.pi * i) / num_steps
        expected_x = radius * np.cos(angle)
        expected_y = radius * np.sin(angle)

        assert transform.header.seq == i
        assert np.isclose(transform.transform.translation.x, expected_x, atol=1e-10)
        assert np.isclose(transform.transform.translation.y, expected_y, atol=1e-10)
        assert transform.child_frame_id == "base_link"
        assert transform.header.frame_id == "odom"


def test_tfmessage_with_dimos_transforms():
    """Test creating TFMessage using our Transform wrapper classes."""
    from dimos.msgs.geometry_msgs import Transform as DimosTransform

    lcm = LCM(autoconf=True)
    lcm.start()

    received_messages = []
    topic = Topic(topic="/tf_dimos", lcm_type=TFMessage)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)

    # Create transform using Dimos wrapper
    dimos_transform = DimosTransform(
        translation=Vector3(3.0, 4.0, 5.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )

    # Convert to LCM format
    lcm_transform = dimos_transform  # Our wrapper inherits from LCM type

    # Create TransformStamped
    transform_stamped = TransformStamped(
        header=Header(seq=99, stamp=Time(sec=999, nsec=888777), frame_id="map"),
        child_frame_id="robot",
        transform=lcm_transform,
    )

    # Create and send TFMessage
    tf_message = TFMessage(transforms_length=1, transforms=[transform_stamped])

    lcm.publish(topic, tf_message)
    time.sleep(0.1)

    # Verify
    assert len(received_messages) == 1
    received_tf_msg, _ = received_messages[0]

    received_transform = received_tf_msg.transforms[0]
    assert received_transform.header.seq == 99
    assert received_transform.header.frame_id == "map"
    assert received_transform.child_frame_id == "robot"

    # Verify transform values match
    assert np.isclose(received_transform.transform.translation.x, 3.0, atol=1e-10)
    assert np.isclose(received_transform.transform.translation.y, 4.0, atol=1e-10)
    assert np.isclose(received_transform.transform.translation.z, 5.0, atol=1e-10)
