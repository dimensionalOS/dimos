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

"""Test BaseAgent with AgentMessage and video streams."""

import asyncio
import os
import pickle

from dotenv import load_dotenv
import pytest
from reactivex import operators as ops

from dimos import core
from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay

logger = setup_logger("test_agent_message_streams")


class VideoMessageSender(Module):
    """Module that sends AgentMessage with video frames every 2 seconds."""

    message_out: Out[AgentMessage] = None

    def __init__(self, video_path: str) -> None:
        super().__init__()
        self.video_path = video_path
        self._subscription = None
        self._frame_count = 0

    @rpc
    def start(self) -> None:
        """Start sending video messages."""
        # Use TimedSensorReplay to replay video frames
        video_replay = TimedSensorReplay(self.video_path, autocast=Image.from_numpy)

        # Send AgentMessage with frame every 3 seconds (give agent more time to process)
        self._subscription = (
            video_replay.stream()
            .pipe(
                ops.sample(3.0),  # Every 3 seconds
                ops.take(3),  # Only send 3 frames total
                ops.map(self._create_message),
            )
            .subscribe(
                on_next=lambda msg: self._send_message(msg),
                on_error=lambda e: logger.error(f"Video stream error: {e}"),
                on_completed=lambda: logger.info("Video stream completed"),
            )
        )

        logger.info("Video message streaming started (every 3 seconds, max 3 frames)")

    def _create_message(self, frame: Image) -> AgentMessage:
        """Create AgentMessage with frame and query."""
        self._frame_count += 1

        msg = AgentMessage()
        msg.add_text(f"What do you see in frame {self._frame_count}? Describe in one sentence.")
        msg.add_image(frame)

        logger.info(f"Created message with frame {self._frame_count}")
        return msg

    def _send_message(self, msg: AgentMessage) -> None:
        """Send the message and test pickling."""
        # Test that message can be pickled (for module communication)
        try:
            pickled = pickle.dumps(msg)
            pickle.loads(pickled)
            logger.info(f"Message pickling test passed - size: {len(pickled)} bytes")
        except Exception as e:
            logger.error(f"Message pickling failed: {e}")

        self.message_out.publish(msg)

    @rpc
    def stop(self) -> None:
        """Stop streaming."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None


class MultiImageMessageSender(Module):
    """Send AgentMessage with multiple images."""

    message_out: Out[AgentMessage] = None

    def __init__(self, video_path: str) -> None:
        super().__init__()
        self.video_path = video_path
        self.frames = []

    @rpc
    def start(self) -> None:
        """Collect some frames."""
        video_replay = TimedSensorReplay(self.video_path, autocast=Image.from_numpy)

        # Collect first 3 frames
        video_replay.stream().pipe(ops.take(3)).subscribe(
            on_next=lambda frame: self.frames.append(frame),
            on_completed=self._send_multi_image_query,
        )

    def _send_multi_image_query(self) -> None:
        """Send query with multiple images."""
        if len(self.frames) >= 2:
            msg = AgentMessage()
            msg.add_text("Compare these images and describe what changed between them.")

            for _i, frame in enumerate(self.frames[:2]):
                msg.add_image(frame)

            logger.info(f"Sending multi-image message with {len(msg.images)} images")

            # Test pickling
            try:
                pickled = pickle.dumps(msg)
                logger.info(f"Multi-image message pickle size: {len(pickled)} bytes")
            except Exception as e:
                logger.error(f"Multi-image pickling failed: {e}")

            self.message_out.publish(msg)


class ResponseCollector(Module):
    """Collect responses."""

    response_in: In[AgentResponse] = None

    def __init__(self) -> None:
        super().__init__()
        self.responses = []

    @rpc
    def start(self) -> None:
        self.response_in.subscribe(self._on_response)

    def _on_response(self, resp: AgentResponse) -> None:
        logger.info(f"Collected response: {resp.content[:100] if resp.content else 'None'}...")
        self.responses.append(resp)

    @rpc
    def get_responses(self):
        return self.responses


@pytest.mark.tofix
@pytest.mark.module
@pytest.mark.asyncio
async def test_agent_message_video_stream() -> None:
    """Test BaseAgentModule with AgentMessage containing video frames."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    pubsub.lcm.autoconf()

    logger.info("Testing BaseAgentModule with AgentMessage video stream...")
    dimos = core.start(4)

    try:
        # Get test video
        data_path = get_data("unitree_office_walk")
        video_path = os.path.join(data_path, "video")

        logger.info(f"Using video from: {video_path}")

        # Deploy modules
        video_sender = dimos.deploy(VideoMessageSender, video_path)
        video_sender.message_out.transport = core.pLCMTransport("/agent/message")

        agent = dimos.deploy(
            BaseAgentModule,
            model="openai::gpt-4o-mini",
            system_prompt="You are a vision assistant. Describe what you see concisely.",
            temperature=0.0,
        )
        agent.response_out.transport = core.pLCMTransport("/agent/response")

        collector = dimos.deploy(ResponseCollector)

        # Connect modules
        agent.message_in.connect(video_sender.message_out)
        collector.response_in.connect(agent.response_out)

        # Start modules
        agent.start()
        collector.start()
        video_sender.start()

        logger.info("All modules started, streaming video messages...")

        # Wait for 3 messages to be sent (3 frames * 3 seconds = 9 seconds)
        # Plus processing time, wait 12 seconds total
        await asyncio.sleep(12)

        # Stop video stream
        video_sender.stop()

        # Get all responses
        responses = collector.get_responses()
        logger.info(f"\nCollected {len(responses)} responses:")
        for i, resp in enumerate(responses):
            logger.info(
                f"\nResponse {i + 1}: {resp.content if isinstance(resp, AgentResponse) else resp}"
            )

        # Verify we got at least 2 responses (sometimes the 3rd frame doesn't get processed in time)
        assert len(responses) >= 2, f"Expected at least 2 responses, got {len(responses)}"

        # Verify responses describe actual scene
        all_responses = " ".join(
            resp.content if isinstance(resp, AgentResponse) else resp for resp in responses
        ).lower()
        assert any(
            word in all_responses
            for word in ["office", "room", "hallway", "corridor", "door", "wall", "floor", "frame"]
        ), "Responses should describe the office environment"

        logger.info("\n✅ AgentMessage video stream test PASSED!")

        # Stop agent
        agent.stop()

    finally:
        dimos.close()
        dimos.shutdown()


@pytest.mark.tofix
@pytest.mark.module
@pytest.mark.asyncio
async def test_agent_message_multi_image() -> None:
    """Test BaseAgentModule with AgentMessage containing multiple images."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    pubsub.lcm.autoconf()

    logger.info("Testing BaseAgentModule with multi-image AgentMessage...")
    dimos = core.start(4)

    try:
        # Get test video
        data_path = get_data("unitree_office_walk")
        video_path = os.path.join(data_path, "video")

        # Deploy modules
        multi_sender = dimos.deploy(MultiImageMessageSender, video_path)
        multi_sender.message_out.transport = core.pLCMTransport("/agent/multi_message")

        agent = dimos.deploy(
            BaseAgentModule,
            model="openai::gpt-4o-mini",
            system_prompt="You are a vision assistant that compares images.",
            temperature=0.0,
        )
        agent.response_out.transport = core.pLCMTransport("/agent/multi_response")

        collector = dimos.deploy(ResponseCollector)

        # Connect modules
        agent.message_in.connect(multi_sender.message_out)
        collector.response_in.connect(agent.response_out)

        # Start modules
        agent.start()
        collector.start()
        multi_sender.start()

        logger.info("Modules started, sending multi-image query...")

        # Wait for response
        await asyncio.sleep(8)

        # Get responses
        responses = collector.get_responses()
        logger.info(f"\nCollected {len(responses)} responses:")
        for i, resp in enumerate(responses):
            logger.info(
                f"\nResponse {i + 1}: {resp.content if isinstance(resp, AgentResponse) else resp}"
            )

        # Verify we got a response
        assert len(responses) >= 1, f"Expected at least 1 response, got {len(responses)}"

        # Response should mention comparison or multiple images
        response_text = (
            responses[0].content if isinstance(responses[0], AgentResponse) else responses[0]
        ).lower()
        assert any(
            word in response_text
            for word in ["both", "first", "second", "change", "different", "similar", "compare"]
        ), "Response should indicate comparison of multiple images"

        logger.info("\n✅ Multi-image AgentMessage test PASSED!")

        # Stop agent
        agent.stop()

    finally:
        dimos.close()
        dimos.shutdown()


@pytest.mark.tofix
def test_agent_message_text_only() -> None:
    """Test BaseAgent with text-only AgentMessage."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    from dimos.agents.modules.base import BaseAgent

    # Create agent
    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant. Answer in 10 words or less.",
        temperature=0.0,
        seed=42,
    )

    # Test with text-only AgentMessage
    msg = AgentMessage()
    msg.add_text("What is")
    msg.add_text("the capital")
    msg.add_text("of France?")

    response = agent.query(msg)
    assert "Paris" in response.content, "Expected 'Paris' in response"

    # Test pickling of AgentMessage
    pickled = pickle.dumps(msg)
    unpickled = pickle.loads(pickled)
    assert unpickled.get_combined_text() == "What is the capital of France?"

    # Verify multiple text messages were combined properly
    assert len(msg.messages) == 3
    assert msg.messages[0] == "What is"
    assert msg.messages[1] == "the capital"
    assert msg.messages[2] == "of France?"

    logger.info("✅ Text-only AgentMessage test PASSED!")

    # Clean up
    agent.dispose()


if __name__ == "__main__":
    logger.info("Running AgentMessage stream tests...")

    # Run text-only test first
    test_agent_message_text_only()
    print("\n" + "=" * 60 + "\n")

    # Run async tests
    asyncio.run(test_agent_message_video_stream())
    print("\n" + "=" * 60 + "\n")
    asyncio.run(test_agent_message_multi_image())

    logger.info("\n✅ All AgentMessage tests completed!")
