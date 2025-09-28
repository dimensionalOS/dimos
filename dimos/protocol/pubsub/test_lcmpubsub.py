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

import pytest

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.protocol.pubsub.lcmpubsub import (
    LCM,
    LCMPubSubBase,
    PickleLCM,
    Topic,
)


@pytest.fixture
def lcm_pub_sub_base():
    lcm = LCMPubSubBase(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()


@pytest.fixture
def pickle_lcm():
    lcm = PickleLCM(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()


@pytest.fixture
def lcm():
    lcm = LCM(autoconf=True)
    lcm.start()
    yield lcm
    lcm.stop()


class MockLCMMessage:
    """Mock LCM message for testing"""

    msg_name = "geometry_msgs.Mock"

    def __init__(self, data):
        self.data = data

    def lcm_encode(self) -> bytes:
        return str(self.data).encode("utf-8")

    @classmethod
    def lcm_decode(cls, data: bytes) -> "MockLCMMessage":
        return cls(data.decode("utf-8"))

    def __eq__(self, other):
        return isinstance(other, MockLCMMessage) and self.data == other.data


def test_LCMPubSubBase_pubsub(lcm_pub_sub_base):
    lcm = lcm_pub_sub_base

    received_messages = []

    topic = Topic(topic="/test_topic", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("test_data")

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message.lcm_encode())
    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, bytes)
    assert received_data.decode() == "test_data"

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


def test_lcm_autodecoder_pubsub(lcm):
    received_messages = []

    topic = Topic(topic="/test_topic", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("test_data")

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message)
    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, MockLCMMessage)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic


test_msgs = [
    (Vector3(1, 2, 3)),
    (Quaternion(1, 2, 3, 4)),
    (Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))),
]


# passes some geometry types through LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_pubsub(test_message, lcm):
    received_messages = []

    topic = Topic(topic="/test_topic", lcm_type=test_message.__class__)

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message)

    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, test_message.__class__)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic

    print(test_message, topic)


# passes some geometry types through pickle LCM
@pytest.mark.parametrize("test_message", test_msgs)
def test_lcm_geometry_msgs_autopickle_pubsub(test_message, pickle_lcm):
    lcm = pickle_lcm
    received_messages = []

    topic = Topic(topic="/test_topic")

    def callback(msg, topic):
        received_messages.append((msg, topic))

    lcm.subscribe(topic, callback)
    lcm.publish(topic, test_message)

    time.sleep(0.1)

    assert len(received_messages) == 1

    received_data = received_messages[0][0]
    received_topic = received_messages[0][1]

    print(f"Received data: {received_data}, Topic: {received_topic}")

    assert isinstance(received_data, test_message.__class__)
    assert received_data == test_message

    assert isinstance(received_topic, Topic)
    assert received_topic == topic

    print(test_message, topic)


def test_wait_for_message_basic(lcm):
    """Test basic wait_for_message functionality - message arrives before timeout."""
    topic = Topic(topic="/test_wait", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("wait_test_data")

    # Publish message after a short delay in another thread
    def publish_delayed():
        time.sleep(0.1)
        lcm.publish(topic, test_message)

    publisher_thread = threading.Thread(target=publish_delayed)
    publisher_thread.start()

    # Wait for message with 1 second timeout
    start_time = time.time()
    received_msg = lcm.wait_for_message(topic, timeout=1.0)
    elapsed_time = time.time() - start_time

    publisher_thread.join()

    # Check that we received the message
    assert received_msg is not None
    assert isinstance(received_msg, MockLCMMessage)
    assert received_msg.data == "wait_test_data"

    # Check that we didn't wait the full timeout
    assert elapsed_time < 0.5  # Should receive message in ~0.1 seconds


def test_wait_for_message_timeout(lcm):
    """Test wait_for_message timeout - no message published."""
    topic = Topic(topic="/test_timeout", lcm_type=MockLCMMessage)

    # Wait for message that will never come
    start_time = time.time()
    received_msg = lcm.wait_for_message(topic, timeout=0.5)
    elapsed_time = time.time() - start_time

    # Check that we got None (timeout)
    assert received_msg is None

    # Check that we waited approximately the timeout duration
    assert 0.4 < elapsed_time < 0.7  # Allow some tolerance


def test_wait_for_message_immediate(lcm):
    """Test wait_for_message with message published immediately after subscription."""
    topic = Topic(topic="/test_immediate", lcm_type=MockLCMMessage)
    test_message = MockLCMMessage("immediate_data")

    # Start waiting in a thread
    received_msg = None

    def wait_for_msg():
        nonlocal received_msg
        received_msg = lcm.wait_for_message(topic, timeout=1.0)

    wait_thread = threading.Thread(target=wait_for_msg)
    wait_thread.start()

    # Give a tiny bit of time for subscription to be established
    time.sleep(0.01)

    # Now publish the message
    start_time = time.time()
    lcm.publish(topic, test_message)

    # Wait for the thread to complete
    wait_thread.join()
    elapsed_time = time.time() - start_time

    # Check that we received the message quickly
    assert received_msg is not None
    assert isinstance(received_msg, MockLCMMessage)
    assert received_msg.data == "immediate_data"
    assert elapsed_time < 0.2  # Should be nearly immediate


def test_wait_for_message_multiple_sequential(lcm):
    """Test multiple sequential wait_for_message calls."""
    topic = Topic(topic="/test_sequential", lcm_type=MockLCMMessage)

    # Test multiple messages in sequence
    messages = ["msg1", "msg2", "msg3"]

    for msg_data in messages:
        test_message = MockLCMMessage(msg_data)

        # Publish in background
        def publish_delayed(msg=test_message):
            time.sleep(0.05)
            lcm.publish(topic, msg)

        publisher_thread = threading.Thread(target=publish_delayed)
        publisher_thread.start()

        # Wait and verify
        received_msg = lcm.wait_for_message(topic, timeout=1.0)
        assert received_msg is not None
        assert received_msg.data == msg_data

        publisher_thread.join()


def test_wait_for_message_concurrent(lcm):
    """Test concurrent wait_for_message calls on different topics."""
    topic1 = Topic(topic="/test_concurrent1", lcm_type=MockLCMMessage)
    topic2 = Topic(topic="/test_concurrent2", lcm_type=MockLCMMessage)

    message1 = MockLCMMessage("concurrent1")
    message2 = MockLCMMessage("concurrent2")

    received_messages = {}

    def wait_for_topic(topic_name, topic):
        msg = lcm.wait_for_message(topic, timeout=2.0)
        received_messages[topic_name] = msg

    # Start waiting on both topics
    thread1 = threading.Thread(target=wait_for_topic, args=("topic1", topic1))
    thread2 = threading.Thread(target=wait_for_topic, args=("topic2", topic2))

    thread1.start()
    thread2.start()

    # Publish to both topics after a delay
    time.sleep(0.1)
    lcm.publish(topic1, message1)
    lcm.publish(topic2, message2)

    # Wait for both threads to complete
    thread1.join(timeout=3.0)
    thread2.join(timeout=3.0)

    # Verify both messages were received
    assert "topic1" in received_messages
    assert "topic2" in received_messages
    assert received_messages["topic1"].data == "concurrent1"
    assert received_messages["topic2"].data == "concurrent2"


def test_wait_for_message_wrong_topic(lcm):
    """Test wait_for_message doesn't receive messages from wrong topic."""
    topic_correct = Topic(topic="/test_correct", lcm_type=MockLCMMessage)
    topic_wrong = Topic(topic="/test_wrong", lcm_type=MockLCMMessage)

    message = MockLCMMessage("wrong_topic_data")

    # Publish to wrong topic
    lcm.publish(topic_wrong, message)

    # Wait on correct topic
    received_msg = lcm.wait_for_message(topic_correct, timeout=0.3)

    # Should timeout and return None
    assert received_msg is None


def test_wait_for_message_pickle(pickle_lcm):
    """Test wait_for_message with PickleLCM."""
    lcm = pickle_lcm
    topic = Topic(topic="/test_pickle")
    test_obj = {"key": "value", "number": 42}

    # Publish after delay
    def publish_delayed():
        time.sleep(0.1)
        lcm.publish(topic, test_obj)

    publisher_thread = threading.Thread(target=publish_delayed)
    publisher_thread.start()

    # Wait for message
    received_msg = lcm.wait_for_message(topic, timeout=1.0)

    publisher_thread.join()

    # Verify received object
    assert received_msg is not None
    # PickleLCM's wait_for_message returns the pickled bytes, need to decode
    import pickle

    decoded_msg = pickle.loads(received_msg)
    assert decoded_msg == test_obj
    assert decoded_msg["key"] == "value"
    assert decoded_msg["number"] == 42
