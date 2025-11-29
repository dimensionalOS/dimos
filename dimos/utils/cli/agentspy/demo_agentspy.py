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

"""Demo script to test agent message publishing and agentspy reception."""

import time

from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)

from dimos.protocol.pubsub import lcm  # type: ignore[attr-defined]
from dimos.protocol.pubsub.lcmpubsub import PickleLCM


def test_publish_messages() -> None:
    """Publish test messages to verify agentspy is working."""
    print("Starting agent message publisher demo...")

    # Create transport
    transport = PickleLCM()
    topic = lcm.Topic("/agent")

    print(f"Publishing to topic: {topic}")

    # Test messages
    messages = [
        SystemMessage("System initialized for testing"),
        HumanMessage("Hello agent, can you help me?"),
        AIMessage(
            "Of course! I'm here to help.",
            tool_calls=[{"name": "get_info", "args": {"query": "test"}, "id": "1"}],
        ),
        ToolMessage(name="get_info", content="Test result: success", tool_call_id="1"),
        AIMessage("The test was successful!", metadata={"state": True}),
    ]

    # Publish messages with delays
    for i, msg in enumerate(messages):
        print(f"\nPublishing message {i + 1}: {type(msg).__name__}")
        print(f"Content: {msg.content if hasattr(msg, 'content') else msg}")

        transport.publish(topic, msg)
        time.sleep(1)  # Wait 1 second between messages

    print("\nAll messages published! Check agentspy to see if they were received.")
    print("Keeping publisher alive for 10 more seconds...")
    time.sleep(10)


if __name__ == "__main__":
    test_publish_messages()
