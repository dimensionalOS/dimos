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

import json

from dimos.protocol.pubsub.memory import Memory, MemoryWithJSONEncoder


def test_json_encoded_pubsub() -> None:
    """Test memory pubsub with JSON encoding."""
    pubsub = MemoryWithJSONEncoder()
    received_messages = []

    def callback(message, topic) -> None:
        received_messages.append(message)

    # Subscribe to a topic
    pubsub.subscribe("json_topic", callback)

    # Publish various types of messages
    test_messages = [
        "hello world",
        42,
        3.14,
        True,
        None,
        {"name": "Alice", "age": 30, "active": True},
        [1, 2, 3, "four", {"five": 5}],
        {"nested": {"data": [1, 2, {"deep": True}]}},
    ]

    for msg in test_messages:
        pubsub.publish("json_topic", msg)

    # Verify all messages were received and properly decoded
    assert len(received_messages) == len(test_messages)
    for original, received in zip(test_messages, received_messages, strict=False):
        assert original == received


def test_json_encoding_edge_cases() -> None:
    """Test edge cases for JSON encoding."""
    pubsub = MemoryWithJSONEncoder()
    received_messages = []

    def callback(message, topic) -> None:
        received_messages.append(message)

    pubsub.subscribe("edge_cases", callback)

    # Test edge cases
    edge_cases = [
        "",  # empty string
        [],  # empty list
        {},  # empty dict
        0,  # zero
        False,  # False boolean
        [None, None, None],  # list with None values
        {"": "empty_key", "null": None, "empty_list": [], "empty_dict": {}},
    ]

    for case in edge_cases:
        pubsub.publish("edge_cases", case)

    assert received_messages == edge_cases


def test_multiple_subscribers_with_encoding() -> None:
    """Test that multiple subscribers work with encoding."""
    pubsub = MemoryWithJSONEncoder()
    received_messages_1 = []
    received_messages_2 = []

    def callback_1(message, topic) -> None:
        received_messages_1.append(message)

    def callback_2(message, topic) -> None:
        received_messages_2.append(f"callback_2: {message}")

    pubsub.subscribe("json_topic", callback_1)
    pubsub.subscribe("json_topic", callback_2)
    pubsub.publish("json_topic", {"multi": "subscriber test"})

    # Both callbacks should receive the message
    assert received_messages_1[-1] == {"multi": "subscriber test"}
    assert received_messages_2[-1] == "callback_2: {'multi': 'subscriber test'}"


# def test_unsubscribe_with_encoding():
#     """Test unsubscribe works correctly with encoded callbacks."""
#     pubsub = MemoryWithJSONEncoder()
#     received_messages_1 = []
#     received_messages_2 = []

#     def callback_1(message):
#         received_messages_1.append(message)

#     def callback_2(message):
#         received_messages_2.append(message)

#     pubsub.subscribe("json_topic", callback_1)
#     pubsub.subscribe("json_topic", callback_2)

#     # Unsubscribe first callback
#     pubsub.unsubscribe("json_topic", callback_1)
#     pubsub.publish("json_topic", "only callback_2 should get this")

#     # Only callback_2 should receive the message
#     assert len(received_messages_1) == 0
#     assert received_messages_2 == ["only callback_2 should get this"]


def test_data_actually_encoded_in_transit() -> None:
    """Validate that data is actually encoded in transit by intercepting raw bytes."""

    # Create a spy memory that captures what actually gets published
    class SpyMemory(Memory):
        def __init__(self) -> None:
            super().__init__()
            self.raw_messages_received = []

        def publish(self, topic: str, message) -> None:
            # Capture what actually gets published
            self.raw_messages_received.append((topic, message, type(message)))
            super().publish(topic, message)

    # Create encoder that uses our spy memory
    class SpyMemoryWithJSON(MemoryWithJSONEncoder, SpyMemory):
        pass

    pubsub = SpyMemoryWithJSON()
    received_decoded = []

    def callback(message, topic) -> None:
        received_decoded.append(message)

    pubsub.subscribe("test_topic", callback)

    # Publish a complex object
    original_message = {"name": "Alice", "age": 30, "items": [1, 2, 3]}
    pubsub.publish("test_topic", original_message)

    # Verify the message was received and decoded correctly
    assert len(received_decoded) == 1
    assert received_decoded[0] == original_message

    # Verify the underlying transport actually received JSON bytes, not the original object
    assert len(pubsub.raw_messages_received) == 1
    topic, raw_message, raw_type = pubsub.raw_messages_received[0]

    assert topic == "test_topic"
    assert raw_type == bytes  # Should be bytes, not dict
    assert isinstance(raw_message, bytes)

    # Verify it's actually JSON
    decoded_raw = json.loads(raw_message.decode("utf-8"))
    assert decoded_raw == original_message
